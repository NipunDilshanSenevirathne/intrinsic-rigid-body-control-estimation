/*
 * ============================================================
 *  Intrinsic Rigid-Body IMU — ESP32 + BNO055 Firmware
 *  Author : Dinuja Karunarathne
 *  Math   : Based on D.H.S. Maithripala's Lie-group AGLES-PID
 *           and Discrete Intrinsic EKF (DEKF) on SO(3)
 * ============================================================
 *
 *  Hardware wiring (I2C default):
 *    BNO055 VIN  -> ESP32 3.3V
 *    BNO055 GND  -> ESP32 GND
 *    BNO055 SDA  -> ESP32 GPIO 21
 *    BNO055 SCL  -> ESP32 GPIO 22
 *    BNO055 PS1  -> GND  (I2C mode)
 *    BNO055 PS0  -> GND  (I2C mode)
 *    BNO055 ADR  -> GND  (I2C address 0x28)
 *
 *  Libraries required (install via Arduino Library Manager):
 *    - Adafruit BNO055       (by Adafruit)
 *    - Adafruit Unified Sensor
 *    - ArduinoJson           (v6+)
 *    - WebSockets            (by Markus Sattler / Links2004)
 *
 *  Flash target: ESP32 Dev Module, 240 MHz, 4 MB flash
 * ============================================================
 */

#include <Wire.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ─── WiFi credentials ───────────────────────────────────────
const char* WIFI_SSID     = "testnet1";     // <-- change
const char* WIFI_PASSWORD = "123456789"; // <-- change

// ─── WebSocket server on port 81 ────────────────────────────
WebSocketsServer webSocket(81);

// ─── BNO055 sensor (I2C address 0x28) ───────────────────────
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// ─── Timing ─────────────────────────────────────────────────
#define SAMPLE_RATE_HZ  50          // 50 Hz streaming
unsigned long lastSampleMs = 0;

// ─── Rotation matrix (SO(3)) from quaternion ────────────────
// R = (I + 2q0*[q_vec]× + 2[q_vec]×²)  (Rodrigues-like)
// We store as flat 9-element array row-major
void quaternionToSO3(float qw, float qx, float qy, float qz, float R[9]) {
  float n = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  qw /= n; qx /= n; qy /= n; qz /= n;

  R[0] = 1 - 2*(qy*qy + qz*qz);
  R[1] =     2*(qx*qy - qz*qw);
  R[2] =     2*(qx*qz + qy*qw);
  R[3] =     2*(qx*qy + qz*qw);
  R[4] = 1 - 2*(qx*qx + qz*qz);
  R[5] =     2*(qy*qz - qx*qw);
  R[6] =     2*(qx*qz - qy*qw);
  R[7] =     2*(qy*qz + qx*qw);
  R[8] = 1 - 2*(qx*qx + qy*qy);
}

// ─── Euler angles from rotation matrix ──────────────────────
// Roll=φ, Pitch=θ, Yaw=ψ  (ZYX / aerospace convention)
void SO3toEuler(float R[9], float &roll, float &pitch, float &yaw) {
  pitch = asin(-R[6]);
  roll  = atan2(R[7], R[8]);
  yaw   = atan2(R[3], R[0]);
}

// ─── Skew-symmetric map: ω → [ω]× ──────────────────────────
// Returns Lie algebra element as string for JSON
void skewString(float wx, float wy, float wz, char* buf, size_t sz) {
  snprintf(buf, sz,
    "[[0,%.4f,%.4f],[%.4f,0,%.4f],[%.4f,%.4f,0]]",
    -wz, wy, wz, -wx, -wy, wx);
}

// ─── WebSocket event handler ─────────────────────────────────
void onWebSocketEvent(uint8_t num, WStype_t type,
                      uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.printf("[WS] Client %u connected\n", num);
  } else if (type == WStype_DISCONNECTED) {
    Serial.printf("[WS] Client %u disconnected\n", num);
  }
}

// ─── Setup ───────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== Intrinsic Rigid-Body IMU ===");

  // I2C on default SDA=21, SCL=22
  Wire.begin(21, 22);

  // BNO055 init
  if (!bno.begin()) {
    Serial.println("[ERROR] BNO055 not detected. Check wiring!");
    while (true) { delay(500); Serial.print("."); }
  }
  bno.setExtCrystalUse(true);
  Serial.println("[OK] BNO055 initialised (NDOF fusion mode)");

  // WiFi
  Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\n[WiFi] Connected — IP: %s\n",
                WiFi.localIP().toString().c_str());

  // WebSocket
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
  Serial.println("[WS] WebSocket server started on port 81");
  Serial.println("[INFO] Open the dashboard and point it to ws://");
  Serial.println(WiFi.localIP().toString() + ":81");
}

// ─── Loop ────────────────────────────────────────────────────
void loop() {
  webSocket.loop();

  unsigned long now = millis();
  if (now - lastSampleMs < (1000 / SAMPLE_RATE_HZ)) return;
  lastSampleMs = now;

  // ── Quaternion (sensor fusion, NDOF mode) ──────────────────
  imu::Quaternion q = bno.getQuat();
  float qw = (float)q.w(), qx = (float)q.x(),
        qy = (float)q.y(), qz = (float)q.z();

  // ── SO(3) rotation matrix ──────────────────────────────────
  float R[9];
  quaternionToSO3(qw, qx, qy, qz, R);

  // ── Euler angles (rad → deg) ───────────────────────────────
  float roll, pitch, yaw;
  SO3toEuler(R, roll, pitch, yaw);
  roll  *= 180.0 / M_PI;
  pitch *= 180.0 / M_PI;
  yaw   *= 180.0 / M_PI;

  // ── Angular velocity (Lie algebra element ω in body frame) ─
  sensors_event_t ev;
  bno.getEvent(&ev, Adafruit_BNO055::VECTOR_GYROSCOPE);
  float wx = ev.gyro.x, wy = ev.gyro.y, wz = ev.gyro.z;

  // ── Linear acceleration ────────────────────────────────────
  bno.getEvent(&ev, Adafruit_BNO055::VECTOR_LINEARACCEL);
  float ax = ev.acceleration.x,
        ay = ev.acceleration.y,
        az = ev.acceleration.z;

  // ── Gravity vector (body frame, from sensor fusion) ────────
  bno.getEvent(&ev, Adafruit_BNO055::VECTOR_GRAVITY);
  float gx = ev.acceleration.x,
        gy = ev.acceleration.y,
        gz = ev.acceleration.z;

  // ── Calibration status ─────────────────────────────────────
  uint8_t sysCal, gyroCal, accelCal, magCal;
  bno.getCalibration(&sysCal, &gyroCal, &accelCal, &magCal);

  // ── Angular momentum magnitude |π| = |I ω| ─────────────────
  // Approximate body inertia as unit sphere (I = diag(1,1,1))
  float pi_mag = sqrt(wx*wx + wy*wy + wz*wz);

  // ── Config error trace (identity means no error) ───────────
  // e_trace = tr(I - R^T R_ref) / 2  → 0 when aligned
  // Here R_ref = I (upright reference), so error = (3 - tr(R))/2
  float tr_R   = R[0] + R[4] + R[8];
  float e_conf = (3.0f - tr_R) / 2.0f;  // ∈ [0,2]

  // ── JSON payload ───────────────────────────────────────────
  StaticJsonDocument<1024> doc;
  doc["ts"]  = now;

  // Quaternion
  JsonObject qObj = doc.createNestedObject("q");
  qObj["w"] = qw; qObj["x"] = qx;
  qObj["y"] = qy; qObj["z"] = qz;

  // SO(3) matrix (row-major)
  JsonArray rArr = doc.createNestedArray("R");
  for (int i = 0; i < 9; i++) rArr.add(R[i]);

  // Euler
  JsonObject euler = doc.createNestedObject("euler");
  euler["roll"] = roll; euler["pitch"] = pitch; euler["yaw"] = yaw;

  // Angular velocity (Lie algebra)
  JsonObject omega = doc.createNestedObject("omega");
  omega["x"] = wx; omega["y"] = wy; omega["z"] = wz;

  // Linear acceleration
  JsonObject accel = doc.createNestedObject("accel");
  accel["x"] = ax; accel["y"] = ay; accel["z"] = az;

  // Gravity
  JsonObject grav = doc.createNestedObject("gravity");
  grav["x"] = gx; grav["y"] = gy; grav["z"] = gz;

  // Geometric scalars
  doc["pi_mag"]  = pi_mag;
  doc["e_conf"]  = e_conf;

  // Calibration
  JsonObject cal = doc.createNestedObject("cal");
  cal["sys"]   = sysCal;  cal["gyro"]  = gyroCal;
  cal["accel"] = accelCal; cal["mag"]   = magCal;

  // Serialise and broadcast
  char buf[1024];
  serializeJson(doc, buf);
  webSocket.broadcastTXT(buf);
}
