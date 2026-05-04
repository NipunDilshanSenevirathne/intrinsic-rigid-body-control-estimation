# 🌀 Intrinsic Rigid-Body IMU Dashboard

**Author:** Dinuja Karunarathne

> A full-stack hardware + web system for **real-time geometric rigid-body motion visualisation** using an **ESP32** and **Adafruit BNO055 9-DOF IMU** — grounded in the mathematics of **Lie groups, SO(3), AGLES-PID control, and Discrete Intrinsic Extended Kalman Filters (DEKF)**.

Mathematical foundation: [mugalan/intrinsic-rigid-body-control-estimation](https://github.com/mugalan/intrinsic-rigid-body-control-estimation) by D.H.S. Maithripala, University of Peradeniya.

---

## 📋 Table of Contents

1. [System Overview](#system-overview)
2. [Hardware You Need](#hardware-you-need)
3. [Step 1 — Wire ESP32 to BNO055](#step-1--wire-esp32-to-bno055)
4. [Step 2 — Flash the ESP32 Firmware](#step-2--flash-the-esp32-firmware)
5. [Step 3 — Connect ESP32 to Home WiFi](#step-3--connect-esp32-to-home-wifi)
6. [Step 4 — Open the Dashboard](#step-4--open-the-dashboard)
7. [Step 5 — Shake the Module and Watch the Dashboard](#step-5--shake-the-module-and-watch-the-dashboard)
8. [Running the Python Simulator (No Hardware)](#running-the-python-simulator-no-hardware)
9. [Mathematical Background](#mathematical-background)
10. [Project File Structure](#project-file-structure)
11. [Troubleshooting](#troubleshooting)
12. [References](#references)

---

## System Overview

```
┌──────────────────────────────────────────────────────────┐
│  BNO055 sensor  ──I2C──►  ESP32  ──WebSocket──►  Browser │
│  (9-axis IMU)             (WiFi)                (Dashboard│
│                                                  HTML)   │
└──────────────────────────────────────────────────────────┘
```

The ESP32 reads fused orientation data from the BNO055 at **50 Hz**, computes the **SO(3) rotation matrix**, **Euler angles**, **angular momentum magnitude**, and the **Morse-type configuration error** `Ψ(e) = (3 − tr R) / 2` — all concepts from the Lie-group control paper. It streams this as JSON over WebSocket to any connected browser on your local network.

The dashboard updates charts, the 3D cube, the matrix display, and shake detection in real time.

---

## Hardware You Need

| Component | Details |
|-----------|---------|
| ESP32 Dev Module | ESP-WROOM-32 (as pictured) |
| Adafruit BNO055 | Breakout board (blue, as pictured) |
| Breadboard or wires | Male-to-female jumper wires (4 wires total) |
| USB cable | Micro-USB for ESP32 |
| Computer | Windows / Mac / Linux with Arduino IDE |

---

## Step 1 — Wire ESP32 to BNO055

Connect **4 wires** between the boards:

```
BNO055 Pin    →    ESP32 Pin
──────────────────────────────
VIN           →    3V3  (3.3 V power)
GND           →    GND
SDA           →    GPIO 21  (I2C data)
SCL           →    GPIO 22  (I2C clock)
```

> **Important:** Also make sure:
> - `PS1` on BNO055 → `GND` (selects I2C mode; on the Adafruit breakout this is often done with a solder jumper — check your board)
> - `PS0` on BNO055 → `GND` (I2C mode)
> - `ADR` on BNO055 → `GND` (I2C address 0x28; if floating it may be 0x29)

### Wiring Diagram (text form)

```
ESP32                     BNO055
  ┌──────┐              ┌────────┐
  │  3V3 ├──────────────┤ VIN    │
  │  GND ├──────────────┤ GND    │
  │  G21 ├──────────────┤ SDA    │
  │  G22 ├──────────────┤ SCL    │
  └──────┘              │ PS0→GND│
                        │ PS1→GND│
                        │ ADR→GND│
                        └────────┘
```

> ⚠️ **Never** connect BNO055 VIN to ESP32 **5V** — the BNO055 Adafruit breakout is 3.3 V logic. The onboard regulator accepts 3.3 V–5 V input on VIN, so 3V3 is fine and safest.

---

## Step 2 — Flash the ESP32 Firmware

### 2a. Install Arduino IDE
Download from [arduino.cc/en/software](https://www.arduino.cc/en/software) and install it.

### 2b. Add ESP32 board support
1. In Arduino IDE: **File → Preferences**
2. In "Additional Board Manager URLs" add:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. **Tools → Board → Boards Manager**, search `esp32`, install **esp32 by Espressif Systems**.

### 2c. Install required libraries
In **Tools → Manage Libraries** search and install each:

| Library | Author |
|---------|--------|
| Adafruit BNO055 | Adafruit |
| Adafruit Unified Sensor | Adafruit |
| ArduinoJson | Benoit Blanchon (install v6+) |
| WebSockets | Markus Sattler / Links2004 |

### 2d. Open and configure the sketch
1. Open `esp32/esp32_bno055_ws.ino` in Arduino IDE.
2. Find these two lines near the top and **fill in your WiFi credentials**:
   ```cpp
   const char* WIFI_SSID     = "YOUR_WIFI_SSID";
   const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
   ```

### 2e. Select board and port
- **Tools → Board → ESP32 Dev Module**
- **Tools → Port** → select the COM port (Windows) or `/dev/ttyUSB0` (Linux/Mac)

### 2f. Upload
Press the **Upload** button (→). When you see `Connecting...` hold the **BOOT** button on the ESP32 briefly if it doesn't connect automatically.

---

## Step 3 — Connect ESP32 to Home WiFi

Once uploaded, open **Tools → Serial Monitor** (baud: **115200**).

You will see:

```
=== Intrinsic Rigid-Body IMU ===
[OK] BNO055 initialised (NDOF fusion mode)
[WiFi] Connecting to MyHomeWiFi...........
[WiFi] Connected — IP: 192.168.1.105
[WS] WebSocket server started on port 81
[INFO] Open the dashboard and point it to ws://
192.168.1.105:81
```

> **Write down the IP address** — you'll need it for the dashboard. Your computer and ESP32 must be on **the same WiFi network**.

If BNO055 is not detected you'll see:
```
[ERROR] BNO055 not detected. Check wiring!
```
→ Recheck your SDA/SCL wires and the VIN/GND.

---

## Step 4 — Open the Dashboard

1. Open `web/dashboard.html` in any modern browser (Chrome, Firefox, Edge).
   - **No server needed** — it's a pure static HTML file.
2. In the top-right corner, change the WebSocket URL from the default to:
   ```
   ws://192.168.1.105:81
   ```
   (replace with your ESP32's actual IP)
3. Click **Connect**.
4. The green dot will appear and all charts will begin updating live.

> **Tip:** You can bookmark the file and just change the IP once. The IP is remembered as long as you don't reload.

---

## Step 5 — Shake the Module and Watch the Dashboard

With the system connected:

- **Hold** the ESP32+BNO055 assembly together (they can be on a breadboard).
- **Shake** it rapidly in any direction.
- You will immediately see:
  - The **3D cube** rotate and spin.
  - **Euler angle** charts spike with roll/pitch/yaw changes.
  - **Angular velocity** `|ω|` chart spikes.
  - The **momentum bar** fills red.
  - The **Shake Event Detector** counter increments and shows the time.
  - The **configuration error** `Ψ(e) = (3 − tr R)/2` approaches 1–2 (far from identity / reference).
  - KPI cards flash briefly on each detected shake.
- When you **hold still**, the cube stabilises, `Ψ(e)` drops back toward 0, and `|ω|` returns to near-zero.

### What each panel shows

| Panel | Mathematical meaning |
|-------|---------------------|
| Roll φ, Pitch θ, Yaw ψ | ZYX Euler angles extracted from R ∈ SO(3) |
| Config Error Ψ(e) | Morse-type error function: `(3 − tr R)/2` — 0 = upright, 2 = flipped |
| SO(3) Rotation Matrix | The 3×3 rotation matrix computed from BNO055 quaternion output |
| Angular Velocity ω | Body-frame angular velocity — Lie algebra so(3) element |
| Momentum \|π\| | `‖ω‖` (unit inertia approximation) — spikes sharply on shakes |
| 3D Cube | CSS 3D transform driven by the SO(3) matrix directly |
| Calibration | BNO055 internal NDOF fusion calibration (0–3 per sensor) |

---

## Running the Python Simulator (No Hardware)

If you don't have the hardware yet, you can test the full dashboard with a **mathematical SO(3) simulator**:

### Install dependencies
```bash
cd server
pip install -r requirements.txt
```

### Run the simulator
```bash
python3 server.py --simulate
```

You'll see:
```
==========================================================
  Intrinsic Rigid-Body IMU — Python Bridge
  Author: Dinuja Karunarathne
==========================================================
[WS] WebSocket server listening on ws://0.0.0.0:8765
[SIM] Simulator running — SO(3) rigid body at 50 Hz
```

Open `web/dashboard.html`, set the URL to `ws://localhost:8765`, click **Connect**.

The simulator models a **free rigid body on SO(3)** with:
- Smooth precessing motion (slow sinusoidal `ω`)
- **Automatic shake bursts every ~8 seconds** (high angular velocity injection)
- Rodrigues-formula integration `R(t+dt) = R(t) · exp([ω]× dt)`
- SVD re-orthonormalisation to keep `R ∈ SO(3)`

### Bridge mode (ESP32 via serial, no WiFi)
```bash
python3 server.py --port /dev/ttyUSB0
```
This forwards raw JSON lines from the ESP32's serial port to WebSocket clients, useful for debugging without WiFi.

---

## Mathematical Background

This project implements the measurement side of the **Intrinsic AGLES-PID controller** theory from [Maithripala & Berg, Automatica 2015].

### Configuration space SO(3)

Rigid body attitude lives on the **special orthogonal group**:

```
SO(3) = { R ∈ ℝ³ˣ³ : Rᵀ R = I,  det R = +1 }
```

Its **Lie algebra** so(3) is the space of skew-symmetric matrices, isomorphic to ℝ³ via the hat map:

```
      [  0  -ωz  ωy ]
ω̂  = [ ωz   0  -ωx ]   ←→   ω = (ωx, ωy, ωz)
      [-ωy  ωx   0  ]
```

### Kinematics

```
Ṙ = ω̂ · R    (body angular velocity ω in body frame)
```

### Configuration error (Morse function)

The **right-invariant configuration error**:

```
e = Rr R⁻¹   (Rr = desired reference attitude)
```

The **Morse-type error function**:

```
Ψ(e) = (3 − tr e) / 2  ∈ [0, 2]
```

- `Ψ = 0`: body perfectly aligned with reference
- `Ψ = 2`: body attitude is the antipodal point (180° rotated)
- During shaking: `Ψ` rises toward 1–2 as attitude departs from rest

### AGLES-PID control law

```
fᵘ = Ad†_{e⁻¹} π̇r + ad†_ωₑ Ad†_{e⁻¹} πr − fᵉ − kₚ πₑ − kd π̇ₑ − kᵢ πᵢ
```

where:
- `πₑ = Ad†_{e⁻¹} πr − π` is the momentum error
- `π̇ᵢ = πₑ` is the integral
- Closed-loop dynamics are **linear in momentum errors**

### Discrete Invariant EKF (DEKF)

The DEKF propagates uncertainty on the Lie algebra, maintaining **SO(3) consistency**:

```
Prediction:  R̂ₖ₊₁ = R̂ₖ · exp(ω̂ Δt)
Update:      R̂ₖ  ← R̂ₖ · exp( K ξ )
```

where `K` is the Kalman gain and `ξ` is the innovation in the Lie algebra.

---

## Project File Structure

```
rigid-body-imu/
├── esp32/
│   └── esp32_bno055_ws.ino      # Arduino firmware for ESP32
│
├── server/
│   ├── server.py                # Python WebSocket bridge + simulator
│   └── requirements.txt
│
├── web/
│   └── dashboard.html           # Single-file dashboard (no server needed)
│
└── README.md                    # This file
```

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| BNO055 not detected | Check SDA→G21, SCL→G22, VIN→3V3, GND→GND. Check ADR pin is GND. |
| ESP32 won't upload | Hold BOOT button during "Connecting..." phase |
| WiFi won't connect | Check SSID/password case sensitivity. Try 2.4 GHz band (not 5 GHz) |
| Dashboard shows "Disconnected" | Verify IP in Serial Monitor. Computer and ESP32 on same network? |
| Charts frozen but connected | Firewall blocking port 81? Try disabling briefly |
| BNO055 calibration stuck at 0 | Move sensor in figure-8 pattern for magnetometer; rotate for gyro |
| Python simulator install fails | Try `pip3 install websockets numpy` (numpy is core requirement) |
| `serial_asyncio` not found | Only needed for bridge mode: `pip install pyserial-asyncio` |

---

## References

1. **D.H.S. Maithripala & J.M. Berg**, *An intrinsic PID controller for mechanical systems on Lie groups*, Automatica, 54:189–200, 2015.

2. **D.H.S. Maithripala, J.M. Berg, W.P. Dayawansa**, *Almost-global tracking of simple mechanical systems on Lie groups*, IEEE Transactions on Automatic Control, 51(2):216–225, 2006.

3. **R.S. Chandrasekaran et al.**, *Geometric PID controller for nonholonomic systems on Lie groups*, Automatica, 165:111658, 2024.

4. **GitHub repo**: [mugalan/intrinsic-rigid-body-control-estimation](https://github.com/mugalan/intrinsic-rigid-body-control-estimation)

5. **ETH Zürich dataset**: [bcaf173e-5dac-484b-bc37-faf97a594f1f](https://www.research-collection.ethz.ch/entities/researchdata/bcaf173e-5dac-484b-bc37-faf97a594f1f)

6. **Adafruit BNO055**: [learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor)

---

*Author: Dinuja Karunarathne — Intrinsic Rigid-Body IMU Dashboard — MIT License*
