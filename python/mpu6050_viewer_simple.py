import math
import socket
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, IO

from vpython import *

# --- 設定 ---
HOST = "0.0.0.0"
PORT = 5000
LOOP_RATE = 20
DT = 0.05


@dataclass
class SensorState:
    roll: float = 0.0
    pitch: float = 0.0
    dist_cm: Optional[float] = None


def start_server(host: str, port: int) -> tuple[socket.socket, socket.socket, IO[str]]:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    print(f"等待 ESP32 連線，埠 {port} ...")
    conn, addr = srv.accept()
    print(f"已連線：{addr}")
    return srv, conn, conn.makefile("r")


def build_scene():
    # 隱藏 3D 畫布，僅使用文字顯示
    scene = canvas(title="MPU6050 Angle & Distance", width=1, height=1, background=color.white, visible=False)
    scene.range = 1

    angle_text = wtext(text=format_tilt_text(0.0, 0.0))
    dist_text = wtext(text=format_distance_text(None))
    return {"scene": scene, "angle_text": angle_text, "dist_text": dist_text}


def format_tilt_text(roll_deg: float, pitch_deg: float) -> str:
    roll_i = round(roll_deg)
    pitch_i = round(pitch_deg)
    return f"<b><span style='font-size:36px; color:#000000'>角度 Roll: {roll_i}°，Pitch: {pitch_i}°</span></b><br>"


def format_distance_text(dist_cm: Optional[float]) -> str:
    if dist_cm is None:
        return "<b><span style='font-size:36px; color:#000000'>超音波距離: -- cm</span></b><br>"
    dist_i = round(dist_cm)
    return f"<b><span style='font-size:36px; color:#000000'>超音波距離: {dist_i} cm</span></b><br>"


def parse_distance(line: str) -> Optional[float]:
    if not line.startswith("distance:"):
        return None
    parts = line.split()
    if len(parts) < 2:
        return None
    try:
        return float(parts[1])
    except ValueError:
        return None


def parse_mpu(line: str) -> Optional[Dict[str, float]]:
    if "ax:" not in line or "gx:" not in line:
        return None
    vals: Dict[str, float] = {}
    for tok in line.replace(",", " ").split():
        if ":" not in tok:
            continue
        key, value = tok.split(":", 1)
        try:
            vals[key] = float(value)
        except ValueError:
            continue
    required = {"ax", "ay", "az", "gx", "gy", "gz"}
    if not required.issubset(vals):
        return None
    return vals


def update_orientation(state: SensorState, vals: Dict[str, float]) -> None:
    accel_roll = math.atan2(vals["ay"], vals["az"])
    accel_pitch = math.atan2(-vals["ax"], math.sqrt(vals["ay"] * vals["ay"] + vals["az"] * vals["az"]))
    gyro_x_rate = vals["gx"] / 131.0 * (math.pi / 180.0)
    gyro_y_rate = vals["gy"] / 131.0 * (math.pi / 180.0)
    state.roll = 0.95 * (state.roll + gyro_x_rate * DT) + 0.05 * accel_roll
    state.pitch = 0.95 * (state.pitch + gyro_y_rate * DT) + 0.05 * accel_pitch


def update_view(state: SensorState, view: Dict[str, object]) -> None:
    view["angle_text"].text = format_tilt_text(math.degrees(state.roll), math.degrees(state.pitch))
    view["dist_text"].text = format_distance_text(state.dist_cm)
    if state.dist_cm is not None:
        view["scene"].title = f"Angle & Distance | {state.dist_cm:.2f} cm"
    else:
        view["scene"].title = "Angle & Distance"


def run_loop(conn_file, state: SensorState, view: Dict[str, object]) -> None:
    print("開始讀取數據...按 Ctrl+C 停止")
    while True:
        rate(LOOP_RATE)
        try:
            line = conn_file.readline()
            if not line:
                print("連線中斷")
                break
            line = line.strip()
            if not line:
                continue

            dist_val = parse_distance(line)
            if dist_val is not None:
                state.dist_cm = dist_val
                update_view(state, view)
                continue

            vals = parse_mpu(line)
            if vals is None:
                continue

            update_orientation(state, vals)
            update_view(state, view)
        except Exception as e:
            print(f"Error: {e}")
            time.sleep(0.05)


def main():
    srv = conn = conn_file = None
    try:
        srv, conn, conn_file = start_server(HOST, PORT)
        view = build_scene()
        state = SensorState()
        run_loop(conn_file, state, view)
    except KeyboardInterrupt:
        print("使用者中止")
    finally:
        if conn_file:
            conn_file.close()
        if conn:
            conn.close()
        if srv:
            srv.close()
        print("連線已關閉")


if __name__ == "__main__":
    main()

