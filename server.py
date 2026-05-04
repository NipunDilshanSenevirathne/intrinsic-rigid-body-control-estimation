#!/usr/bin/env python3
"""
=============================================================
 Intrinsic Rigid-Body IMU — Python WebSocket Bridge / Simulator
 Author : Dinuja Karunarathne
 Purpose: Either bridge a serial ESP32 to WebSocket clients,
          OR run a pure math simulator (no hardware needed).
=============================================================

Usage:
  # Simulator mode (no ESP32 needed):
  python3 server.py --simulate

  # Bridge mode (ESP32 on /dev/ttyUSB0):
  python3 server.py --port /dev/ttyUSB0

  Open dashboard.html and connect to ws://localhost:8765
"""

import asyncio, json, math, time, argparse
import numpy as np
import websockets

# ─── SO(3) utilities ─────────────────────────────────────────

def hat(v):
    """Lie algebra map: R³ → so(3)  (skew-symmetric matrix)"""
    return np.array([
        [0,    -v[2],  v[1]],
        [v[2],  0,    -v[0]],
        [-v[1], v[0],  0   ]])

def vee(S):
    """Inverse hat: so(3) → R³"""
    return np.array([S[2,1], S[0,2], S[1,0]])

def exp_SO3(omega, dt=1.0):
    """Rodrigues formula: integrate angular velocity → SO(3)"""
    theta = np.linalg.norm(omega) * dt
    if theta < 1e-9:
        return np.eye(3)
    axis = omega / np.linalg.norm(omega)
    K = hat(axis)
    return np.eye(3) + math.sin(theta)*K + (1 - math.cos(theta))*(K @ K)

def SO3_to_euler(R):
    """ZYX Euler angles from rotation matrix"""
    pitch = math.asin(max(-1, min(1, -R[2,0])))
    roll  = math.atan2(R[2,1], R[2,2])
    yaw   = math.atan2(R[1,0], R[0,0])
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

def SO3_to_quat(R):
    """Rotation matrix → unit quaternion (w,x,y,z)"""
    t = R[0,0] + R[1,1] + R[2,2]
    if t > 0:
        s = 0.5 / math.sqrt(t + 1)
        return (0.25/s, (R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s, (R[1,0]-R[0,1])*s)
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2 * math.sqrt(1 + R[0,0] - R[1,1] - R[2,2])
        return ((R[2,1]-R[1,2])/s, 0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s)
    elif R[1,1] > R[2,2]:
        s = 2 * math.sqrt(1 + R[1,1] - R[0,0] - R[2,2])
        return ((R[0,2]-R[2,0])/s, (R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s)
    else:
        s = 2 * math.sqrt(1 + R[2,2] - R[0,0] - R[1,1])
        return ((R[1,0]-R[0,1])/s, (R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s)

def config_error(R):
    """Morse-type configuration error: Ψ(e) = (3 − tr R) / 2"""
    return (3.0 - np.trace(R)) / 2.0

# ─── Simulated rigid body ─────────────────────────────────────

class RigidBodySimulator:
    """
    Simple forward-Euler integration of a free rigid body on SO(3).
    Angular velocity follows a slow sinusoidal pattern with
    injected 'shake' bursts to mimic physical shaking.
    """
    def __init__(self):
        self.R     = np.eye(3)         # attitude ∈ SO(3)
        self.omega = np.zeros(3)       # body angular velocity
        self.t     = 0.0
        self.dt    = 0.02              # 50 Hz

    def step(self):
        t = self.t
        # Smooth precession + shake component
        ox = 0.3*math.sin(0.4*t) + 0.1*math.sin(1.7*t + 0.5)
        oy = 0.25*math.cos(0.3*t + 0.8) + 0.08*math.sin(2.1*t)
        oz = 0.15*math.sin(0.5*t + 1.2)

        # Shake burst every ~8 seconds
        shake_phase = (t % 8.0) / 8.0
        if 0.1 < shake_phase < 0.25:
            amp = 4.0 * math.sin(math.pi * (shake_phase - 0.1) / 0.15)
            ox += amp * math.sin(20*t)
            oy += amp * math.cos(20*t)
            oz += amp * 0.5 * math.sin(25*t)

        self.omega = np.array([ox, oy, oz])
        dR = exp_SO3(self.omega, self.dt)
        self.R = self.R @ dR

        # Re-orthonormalise periodically (numerical drift)
        U, _, Vt = np.linalg.svd(self.R)
        self.R = U @ Vt

        self.t += self.dt
        return self.build_packet()

    def build_packet(self):
        R = self.R
        omega = self.omega

        roll, pitch, yaw = SO3_to_euler(R)
        qw, qx, qy, qz   = SO3_to_quat(R)
        e_conf            = config_error(R)
        pi_mag            = float(np.linalg.norm(omega))

        # Simulated accelerometer: gravity in body frame = R^T [0,0,-9.81]
        g_world = np.array([0, 0, -9.81])
        g_body  = R.T @ g_world
        accel   = np.random.randn(3) * 0.05  # small noise

        return json.dumps({
            "ts": int(time.time() * 1000),
            "q": {"w": round(qw,5), "x": round(qx,5),
                  "y": round(qy,5), "z": round(qz,5)},
            "R": [round(float(v),5) for v in R.flatten()],
            "euler": {"roll": round(roll,3),
                      "pitch": round(pitch,3),
                      "yaw":   round(yaw,3)},
            "omega": {"x": round(float(omega[0]),4),
                      "y": round(float(omega[1]),4),
                      "z": round(float(omega[2]),4)},
            "accel": {"x": round(float(accel[0]),4),
                      "y": round(float(accel[1]),4),
                      "z": round(float(accel[2]),4)},
            "gravity": {"x": round(float(g_body[0]),4),
                        "y": round(float(g_body[1]),4),
                        "z": round(float(g_body[2]),4)},
            "pi_mag":  round(pi_mag, 4),
            "e_conf":  round(float(e_conf), 5),
            "cal": {"sys":3,"gyro":3,"accel":3,"mag":3}
        })

# ─── WebSocket server ─────────────────────────────────────────

clients = set()

async def register(ws):
    clients.add(ws)
    try:
        await ws.wait_closed()
    finally:
        clients.discard(ws)

async def broadcast(msg):
    if clients:
        await asyncio.gather(*[c.send(msg) for c in clients],
                             return_exceptions=True)

async def simulate_loop():
    sim = RigidBodySimulator()
    print("[SIM] Simulator running — SO(3) rigid body at 50 Hz")
    while True:
        pkt = sim.step()
        await broadcast(pkt)
        await asyncio.sleep(sim.dt)

async def serial_bridge_loop(port, baud=115200):
    import serial_asyncio
    reader, _ = await serial_asyncio.open_serial_connection(
        url=port, baudrate=baud)
    print(f"[SERIAL] Bridge active on {port}")
    while True:
        line = await reader.readline()
        await broadcast(line.decode().strip())

async def main(args):
    stop = asyncio.get_event_loop().create_future()

    async def handler(ws):
        await register(ws)

    server = await websockets.serve(handler, "0.0.0.0", 8765)
    print("[WS] WebSocket server listening on ws://0.0.0.0:8765")
    print("[WS] Open dashboard.html → set URL to ws://localhost:8765")

    if args.simulate:
        await asyncio.gather(server.wait_closed(), simulate_loop())
    else:
        await asyncio.gather(server.wait_closed(),
                             serial_bridge_loop(args.port))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Rigid-Body IMU WebSocket Bridge / Simulator")
    parser.add_argument("--simulate", action="store_true",
                        help="Run SO(3) simulation instead of real hardware")
    parser.add_argument("--port", default="/dev/ttyUSB0",
                        help="Serial port of ESP32 (bridge mode)")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    print("=" * 58)
    print("  Intrinsic Rigid-Body IMU — Python Bridge")
    print("  Author: Dinuja Karunarathne")
    print("=" * 58)
    asyncio.run(main(args))
