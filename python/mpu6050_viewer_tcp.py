import socket
import time
import math
import json
import os
import webbrowser
from dataclasses import dataclass, field
from typing import Dict, Optional
import statistics as stats
from http.server import HTTPServer, BaseHTTPRequestHandler
from threading import Thread
import threading

# --- 設定 ---
TCP_HOST = "0.0.0.0"  # 監聽所有介面
TCP_PORT = 5001       # TCP 伺服器端口（ESP32 連接的端口）
LOOP_RATE = 20       # 更新頻率 (Hz)
DT = 0.05            # 互補濾波時間步長
DIST_WINDOW = 5      # 距離中值視窗
DIST_JUMP_LIMIT = 30.0  # 單步最大允許變化 (cm)，超過則限幅
EVENT_SHOW_SEC = 3.0    # 按鈕事件顯示秒數
WEB_PORT = 5000      # Web 伺服器端口


@dataclass
class SensorState:
    roll: float = 0.0
    dist_cm: Optional[float] = None
    dist_comp_cm: Optional[float] = None  # 角度補償後的距離
    t: float = 0.0
    dist_history: list[float] = field(default_factory=list)
    event_msg: Optional[str] = None
    event_until: float = 0.0
    mpu_vals: Optional[Dict[str, float]] = None
    button_pressed: bool = False  # PB12 按鈕狀態
    button_press_count: int = 0  # 按鈕按下次數
    last_button_event: float = 0.0  # 最後一次按鈕事件時間
    alarm_distance_cm: float = 50.0  # 警報距離（初始值50，按下按鈕時會加上當前距離）
    zero_roll_deg: float = 45.0  # Roll 角度基準（按下按鈕時會設為當前角度）
    obstacle_hit_count: int = 0  # 障礙物檢測連續次數（STM32 需要連續2次）
    last_distance_for_obstacle: Optional[float] = None  # 用於檢測障礙物的基準距離
    lock: threading.Lock = field(default_factory=threading.Lock)


def parse_distance(line: str) -> Optional[float]:
    """解析距離行: distance: 10.10 cm"""
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
    """解析 MPU 數據行，回傳包含 ax/ay/az/gx/gy/gz 的字典。"""
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
    """套用互補濾波更新 roll。"""
    accel_roll = math.atan2(vals["ay"], vals["az"])

    gyro_x_rate = vals["gx"] / 131.0 * (math.pi / 180.0)

    state.roll = 0.95 * (state.roll + gyro_x_rate * DT) + 0.05 * accel_roll


def smooth_distance(state: SensorState, raw_dist: float) -> float:
    """對距離做中值濾波並限幅，減少左右擺動造成的跳動。"""
    state.dist_history.append(raw_dist)
    if len(state.dist_history) > DIST_WINDOW:
        state.dist_history.pop(0)

    median_dist = stats.median(state.dist_history)
    if state.dist_cm is None:
        return median_dist

    delta = median_dist - state.dist_cm
    if abs(delta) > DIST_JUMP_LIMIT:
        # 限幅避免瞬間大跳
        return state.dist_cm + (DIST_JUMP_LIMIT if delta > 0 else -DIST_JUMP_LIMIT)
    return median_dist


def print_status(state: SensorState, vals: Optional[Dict[str, float]] = None) -> None:
    """純文字輸出當前狀態。"""
    # 清除上一行（簡單的覆蓋效果）
    # print("\r" + " " * 100, end="", flush=True)  # 已禁用終端輸出
    pass
    
    # 事件訊息
    now = time.time()
    if state.event_msg and now < state.event_until:
        event_info = f"[事件] {state.event_msg} | "
    else:
        event_info = ""
        if state.event_msg:
            state.event_msg = None
    
    # 距離資訊
    if state.dist_cm is not None:
        dist_info = f"距離: {state.dist_cm:6.2f} cm | "
    else:
        dist_info = "距離: -- cm | "
    
    # 角度資訊
    roll_deg = math.degrees(state.roll)
    angle_info = f"手杖傾斜角度: {roll_deg:6.2f}° | "
    
    # MPU 原始值（如果有）
    if vals:
        mpu_info = f"MPU: ax={vals['ax']:6.0f} ay={vals['ay']:6.0f} az={vals['az']:6.0f} | "
        mpu_info += f"gx={vals['gx']:6.0f} gy={vals['gy']:6.0f} gz={vals['gz']:6.0f}"
    else:
        mpu_info = ""
    
    output = f"\r{event_info}{dist_info}{angle_info}{mpu_info}"
    # print(output, end="", flush=True)  # 已禁用終端輸出


# --- 全局狀態 ---
state = SensorState()

# HTML 網頁內容
HTML_CONTENT = """<!DOCTYPE html>
<html lang="zh-TW">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>MPU6050 感測器監控 (TCP)</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Microsoft JhengHei', 'Segoe UI', Arial, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        .container {
            max-width: 1400px;
            margin: 0 auto;
            background: white;
            border-radius: 24px;
            box-shadow: 0 20px 60px rgba(0,0,0,0.3);
            padding: 40px;
        }
        .main-content {
            display: grid;
            grid-template-columns: 1fr 400px;
            gap: 30px;
            align-items: start;
        }
        .left-column {
            display: flex;
            flex-direction: column;
            gap: 0;
        }
        h1 {
            text-align: center;
            color: #2c3e50;
            margin-bottom: 40px;
            font-size: 2.8em;
            font-weight: 700;
            letter-spacing: 1px;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.1);
        }
        .status-grid {
            display: grid;
            grid-template-columns: 1fr;
            grid-template-rows: auto auto auto;
            gap: 24px;
            margin-bottom: 35px;
        }
        .status-row {
            display: grid;
            gap: 24px;
        }
        .status-row.single {
            grid-template-columns: 1fr;
        }
        .status-row.double {
            grid-template-columns: 1fr 1fr;
        }
        .status-card {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 28px;
            border-radius: 18px;
            box-shadow: 0 8px 20px rgba(0,0,0,0.15);
            transition: transform 0.3s ease, box-shadow 0.3s ease;
            position: relative;
            overflow: hidden;
        }
        .status-card::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            height: 4px;
            background: rgba(255,255,255,0.3);
        }
        .status-card:hover {
            transform: translateY(-4px);
            box-shadow: 0 12px 28px rgba(0,0,0,0.2);
        }
        .status-card h2 {
            font-size: 1.3em;
            margin-bottom: 18px;
            opacity: 0.95;
            font-weight: 600;
            letter-spacing: 0.5px;
            text-align: center;
        }
        .status-value {
            font-size: 2.8em;
            font-weight: 700;
            margin: 12px 0;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.2);
            line-height: 1.2;
            text-align: center;
        }
        .status-unit {
            font-size: 1.05em;
            opacity: 0.85;
            font-weight: 500;
            margin-top: 8px;
            text-align: center;
        }
        .event-banner {
            background: linear-gradient(135deg, #ff6b6b 0%, #ee5a6f 100%);
            color: white;
            padding: 24px;
            border-radius: 12px;
            text-align: center;
            font-size: 1.6em;
            font-weight: bold;
            margin-bottom: 30px;
            display: none;
            animation: pulse 1.5s infinite;
            box-shadow: 0 4px 15px rgba(255,107,107,0.4);
        }
        @keyframes pulse {
            0%, 100% { opacity: 1; transform: scale(1); }
            50% { opacity: 0.85; transform: scale(1.02); }
        }
        .mpu-data {
            background: linear-gradient(to bottom, #f8f9fa 0%, #e9ecef 100%);
            padding: 28px;
            border-radius: 16px;
            box-shadow: 0 4px 12px rgba(0,0,0,0.08);
            border: 1px solid #dee2e6;
            position: sticky;
            top: 20px;
        }
        .mpu-data h3 {
            color: #2c3e50;
            margin-bottom: 20px;
            font-size: 1.5em;
            font-weight: 600;
            text-align: center;
        }
        .data-row {
            display: flex;
            justify-content: center;
            align-items: center;
            padding: 14px 16px;
            border-bottom: 1px solid #dee2e6;
            transition: background-color 0.2s ease;
            gap: 20px;
        }
        .data-row:hover {
            background-color: rgba(102, 126, 234, 0.05);
        }
        .data-row:last-child {
            border-bottom: none;
        }
        .data-label {
            font-weight: 600;
            color: #495057;
            font-size: 1.05em;
            text-align: center;
        }
        .data-value {
            color: #2c3e50;
            font-family: 'Courier New', 'Consolas', monospace;
            font-size: 1.1em;
            font-weight: 600;
            text-align: center;
        }
        .timestamp {
            text-align: center;
            color: #6c757d;
            margin-top: 30px;
            font-size: 0.95em;
            font-weight: 500;
            padding-top: 20px;
            border-top: 2px solid #e9ecef;
        }
        .button-section {
            background: linear-gradient(135deg, #95a5a6 0%, #7f8c8d 100%);
            border-radius: 20px;
            padding: 40px;
            margin: 35px 0;
            box-shadow: 0 8px 24px rgba(0,0,0,0.15);
            text-align: center;
            transition: all 0.3s ease;
        }
        .button-section:hover {
            box-shadow: 0 12px 32px rgba(0,0,0,0.2);
            transform: translateY(-2px);
        }
        .button-section h2 {
            color: white;
            font-size: 2em;
            margin-bottom: 30px;
            font-weight: 700;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.2);
        }
        .button-status-display {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 20px;
        }
        .button-status-value {
            font-size: 3.5em;
            font-weight: 700;
            color: white;
            text-shadow: 2px 2px 6px rgba(0,0,0,0.3);
            margin: 15px 0;
            text-align: center;
        }
        .button-count {
            font-size: 1.4em;
            color: rgba(255,255,255,0.9);
            font-weight: 600;
            margin-top: 10px;
        }
        .button-pressed {
            background: linear-gradient(135deg, #e74c3c 0%, #c0392b 100%) !important;
            animation: buttonPulse 0.6s ease-in-out;
        }
        @keyframes buttonPulse {
            0%, 100% { transform: scale(1); }
            50% { transform: scale(1.05); }
        }
        @media (max-width: 768px) {
            .container {
                padding: 20px;
            }
            h1 {
                font-size: 2em;
                margin-bottom: 25px;
            }
            .status-grid {
                gap: 16px;
            }
            .status-row.double {
                grid-template-columns: 1fr;
            }
            .status-card {
                padding: 20px;
            }
            .status-value {
                font-size: 2.2em;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>適應性平面校準手杖 (TCP)</h1>
        <div class="event-banner" id="eventBanner"></div>
        <div class="main-content">
            <div class="left-column">
                <div class="status-grid">
            <!-- 第一排：超音波距離 -->
            <div class="status-row single">
                <div class="status-card">
                    <h2>📏 超音波距離</h2>
                    <div class="status-value" id="distance">--</div>
                    <div class="status-unit">公分 (cm)</div>
                </div>
            </div>
            <!-- 第二排：手杖傾斜角度 和 傾斜補償距離 -->
            <div class="status-row double">
                <div class="status-card">
                    <h2>📐 手杖傾斜角度</h2>
                    <div class="status-value" id="roll">0.00</div>
                    <div class="status-unit">度 (°)</div>
                </div>
                <div class="status-card" style="background: linear-gradient(135deg, #16a085 0%, #27ae60 100%);">
                    <h2>📐 傾斜補償距離</h2>
                    <div class="status-value" id="distanceComp">0.00</div>
                    <div class="status-unit">公分 (cm) ±</div>
                </div>
            </div>
            <!-- 第三排：警報距離 -->
            <div class="status-row single">
                <div class="status-card" style="background: linear-gradient(135deg, #9b59b6 0%, #8e44ad 100%);">
                    <h2>⚠️ 警報距離</h2>
                    <div class="status-value" id="alarmDistance" style="font-size: 1.8em;">50.00</div>
                    <div class="status-unit">公分 (cm)</div>
                </div>
            </div>
                </div>
                <div class="button-section" id="buttonSection">
                    <h2>🔘 PB12 按鈕狀態</h2>
                    <div class="button-status-display">
                        <div class="button-status-value" id="buttonStatus">未按下</div>
                    </div>
                </div>
            </div>
            <div class="mpu-data">
                <h3>📊 MPU6050 原始數據</h3>
            <div class="data-row">
                <span class="data-label">加速度 X (ax):</span>
                <span class="data-value" id="ax">--</span>
            </div>
            <div class="data-row">
                <span class="data-label">加速度 Y (ay):</span>
                <span class="data-value" id="ay">--</span>
            </div>
            <div class="data-row">
                <span class="data-label">加速度 Z (az):</span>
                <span class="data-value" id="az">--</span>
            </div>
            <div class="data-row">
                <span class="data-label">陀螺儀 X (gx):</span>
                <span class="data-value" id="gx">--</span>
            </div>
            <div class="data-row">
                <span class="data-label">陀螺儀 Y (gy):</span>
                <span class="data-value" id="gy">--</span>
            </div>
            <div class="data-row">
                <span class="data-label">陀螺儀 Z (gz):</span>
                <span class="data-value" id="gz">--</span>
            </div>
            </div>
        </div>
        <div class="timestamp" id="timestamp">最後更新: --</div>
    </div>
    <script>
        function updateData() {
            fetch('/api/data')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('distance').textContent = 
                        data.dist_cm !== null ? data.dist_cm.toFixed(2) : '--';
                    document.getElementById('roll').textContent = 
                        (data.roll * 180 / Math.PI).toFixed(2);
                    if (data.mpu_vals) {
                        document.getElementById('ax').textContent = data.mpu_vals.ax.toFixed(0);
                        document.getElementById('ay').textContent = data.mpu_vals.ay.toFixed(0);
                        document.getElementById('az').textContent = data.mpu_vals.az.toFixed(0);
                        document.getElementById('gx').textContent = data.mpu_vals.gx.toFixed(0);
                        document.getElementById('gy').textContent = data.mpu_vals.gy.toFixed(0);
                        document.getElementById('gz').textContent = data.mpu_vals.gz.toFixed(0);
                    }
                    const now = new Date();
                    document.getElementById('timestamp').textContent = 
                        '最後更新: ' + now.toLocaleTimeString('zh-TW');
                    const eventBanner = document.getElementById('eventBanner');
                    if (data.event_msg && data.event_until > Date.now() / 1000) {
                        eventBanner.textContent = '🔔 ' + data.event_msg;
                        eventBanner.style.display = 'block';
                    } else {
                        eventBanner.style.display = 'none';
                    }
                    
                    // 更新按鈕狀態
                    const buttonSection = document.getElementById('buttonSection');
                    const buttonStatus = document.getElementById('buttonStatus');
                    
                    if (data.button_pressed) {
                        buttonStatus.textContent = '✅ 已按下';
                        buttonSection.classList.add('button-pressed');
                    } else {
                        buttonStatus.textContent = '未按下';
                        buttonSection.classList.remove('button-pressed');
                    }
                    
                    // 更新警報距離（初始值50，按下按鈕時會加上當前距離）
                    const alarmDistance = document.getElementById('alarmDistance');
                    alarmDistance.textContent = data.alarm_distance_cm.toFixed(2);
                    
                    // 更新角度補償值（只顯示需要增加或減少的數值）
                    const distanceComp = document.getElementById('distanceComp');
                    const roll_deg = data.roll * 180 / Math.PI;
                    
                    // 當角度超過 80 度時，顯示無限大
                    if (roll_deg > 80.0) {
                        distanceComp.textContent = '∞';
                    } else if (data.dist_comp_cm !== null && data.dist_comp_cm !== undefined) {
                        // 檢查是否為無限大（Infinity）
                        if (!isFinite(data.dist_comp_cm)) {
                            distanceComp.textContent = '∞';
                        } else {
                            // 顯示正負號，正值表示需要增加，負值表示需要減少
                            const sign = data.dist_comp_cm >= 0 ? '+' : '';
                            distanceComp.textContent = sign + data.dist_comp_cm.toFixed(2);
                        }
                    } else {
                        distanceComp.textContent = '0.00';
                    }
                })
                .catch(error => console.error('更新數據失敗:', error));
        }
        setInterval(updateData, 100);
        updateData();
    </script>
</body>
</html>"""


class WebHandler(BaseHTTPRequestHandler):
    """簡單的 HTTP 請求處理器"""
    
    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML_CONTENT.encode('utf-8'))
        elif self.path == '/api/data':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            
            with state.lock:
                now = time.time()
                event_msg = None
                if state.event_msg and now < state.event_until:
                    event_msg = state.event_msg
                
                data = {
                    'roll': state.roll,
                    'dist_cm': state.dist_cm,
                    'dist_comp_cm': state.dist_comp_cm,
                    'mpu_vals': state.mpu_vals,
                    'event_msg': event_msg,
                    'event_until': state.event_until,
                    'timestamp': now,
                    'button_pressed': state.button_pressed,
                    'button_press_count': state.button_press_count,
                    'alarm_distance_cm': state.alarm_distance_cm
                }
            
            self.wfile.write(json.dumps(data).encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()
    
    def log_message(self, format, *args):
        pass  # 不顯示日誌訊息


def run_web_server():
    """在背景執行簡單的 Web 伺服器"""
    server = HTTPServer(('0.0.0.0', WEB_PORT), WebHandler)
    # print(f"Web 伺服器已啟動: http://localhost:{WEB_PORT}")  # 已禁用終端輸出
    # 等待服务器启动后自动打开浏览器
    time.sleep(1)  # 给服务器一点时间启动
    webbrowser.open(f'http://localhost:{WEB_PORT}')
    server.serve_forever()


def read_tcp_data():
    """在背景執行 TCP 數據讀取"""
    # --- 初始化 TCP Server ---
    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tcp_socket.bind((TCP_HOST, TCP_PORT))
    tcp_socket.listen(1)
    # print(f"等待 ESP32 連線，TCP 端口 {TCP_PORT} ...")  # 已禁用終端輸出
    
    conn = None
    conn_file = None
    
    try:
        while True:
            try:
                # 等待 ESP32 連接
                if conn is None:
                    conn, addr = tcp_socket.accept()
                    conn.settimeout(1.0)  # 設置超時
                    conn_file = conn.makefile("r", encoding="utf-8", errors="ignore", newline=None)
                    # print(f"ESP32 已連線：{addr}")  # 已禁用終端輸出
                
                time.sleep(1.0 / LOOP_RATE)  # 控制更新頻率
                
                try:
                    # 從 TCP 連接讀取一行數據
                    line = conn_file.readline()
                    if not line:
                        # 連接中斷，重置連接
                        # print("ESP32 連線中斷，等待重新連接...")  # 已禁用終端輸出
                        conn_file.close()
                        conn.close()
                        conn = None
                        conn_file = None
                        continue
                    
                    line = line.strip()
                    if not line:
                        continue

                    with state.lock:
                        # PB12 按鈕檢測：檢測多種可能的按鈕消息
                        line_lower = line.lower()
                        button_detected = False
                        
                        # 檢測各種按鈕相關消息
                        if "calibrated" in line_lower and "pb12" in line_lower:
                            button_detected = True
                            state.event_msg = "PB12 已校準零點"
                            state.button_pressed = True
                            state.button_press_count += 1
                            state.last_button_event = time.time()
                            state.event_until = time.time() + EVENT_SHOW_SEC
                            # 更新警報距離：50 + 當前距離
                            if state.dist_cm is not None:
                                state.alarm_distance_cm = 50.0 + state.dist_cm
                            # 設定當前 Roll 角度為基準角度，補償值歸零
                            state.zero_roll_deg = math.degrees(state.roll)
                            state.dist_comp_cm = 0.0  # 補償值歸零
                            print_status(state)
                            continue
                        elif "button pb12 pressed" in line_lower or "pb12 pressed" in line_lower:
                            button_detected = True
                            state.event_msg = "PB12 按鈕被按下"
                            state.button_pressed = True
                            state.button_press_count += 1
                            state.last_button_event = time.time()
                            state.event_until = time.time() + EVENT_SHOW_SEC
                            # 更新警報距離：50 + 當前距離
                            if state.dist_cm is not None:
                                state.alarm_distance_cm = 50.0 + state.dist_cm
                            # 設定當前 Roll 角度為基準角度，補償值歸零
                            state.zero_roll_deg = math.degrees(state.roll)
                            state.dist_comp_cm = 0.0  # 補償值歸零
                            print_status(state)
                            continue

                        elif "calibrated:" in line_lower:
                            # 通用校準消息（可能包含 PB12）
                            button_detected = True
                            state.event_msg = "校準事件觸發"
                            state.button_pressed = True
                            state.button_press_count += 1
                            state.last_button_event = time.time()
                            state.event_until = time.time() + EVENT_SHOW_SEC
                            # 更新警報距離：50 + 當前距離
                            if state.dist_cm is not None:
                                state.alarm_distance_cm = 50.0 + state.dist_cm
                            # 設定當前 Roll 角度為基準角度，補償值歸零
                            state.zero_roll_deg = math.degrees(state.roll)
                            state.dist_comp_cm = 0.0  # 補償值歸零
                            print_status(state)
                            continue

                        # 如果沒有檢測到按鈕事件，檢查按鈕是否已釋放
                        if not button_detected and state.button_pressed:
                            # 如果距離上次按鈕事件超過一定時間，認為按鈕已釋放
                            if time.time() - state.last_button_event > 0.5:
                                state.button_pressed = False

                        # 解析距離
                        dist_val = parse_distance(line)
                        if dist_val is not None:
                            prev_dist = state.dist_cm
                            state.dist_cm = smooth_distance(state, dist_val)
                            
                            # 計算角度補償值（只與 Roll 角度有關，與距離無關）
                            # 補償值 = Roll角度 - 基準角度
                            # 當角度超過 80 度時，補償值顯示為無限大
                            roll_deg = math.degrees(state.roll)
                            if roll_deg > 80.0:
                                state.dist_comp_cm = None  # 無限大（由前端根據角度判斷顯示）
                            else:
                                state.dist_comp_cm = roll_deg - state.zero_roll_deg
                            
                            # 檢測障礙物：當距離突然變大超過閾值時（STM32 的 safety_margin_cm = 50cm）
                            # STM32 邏輯：distance_comp_rel > 50cm 且連續檢測到 2 次才觸發
                            if prev_dist is not None and state.dist_cm is not None:
                                # 計算距離變化（相對於平滑前的原始值，模擬 STM32 的 distance_comp_rel）
                                # 這裡簡化為直接比較距離變化
                                dist_change = state.dist_cm - prev_dist
                                safety_margin_cm = 50.0  # 與 STM32 一致（50cm）
                                
                                # 如果距離突然變大超過閾值，可能是障礙物/坑洞
                                if dist_change > safety_margin_cm:
                                    state.obstacle_hit_count += 1
                                    # STM32 需要連續 2 次才觸發（hit_need = 2）
                                    # 注意：蜂鳴器狀態已移除，這裡只保留障礙物檢測計數
                                else:
                                    # 距離變化正常，重置計數
                                    state.obstacle_hit_count = 0
                            
                            print_status(state)
                            continue

                        # 解析 MPU 數據
                        vals = parse_mpu(line)
                        if vals is None:
                            continue

                        update_orientation(state, vals)
                        state.mpu_vals = vals
                        
                        # 計算角度補償值（只與 Roll 角度有關，與距離無關）
                        # 補償值 = Roll角度 - 基準角度
                        # 當角度超過 80 度時，補償值顯示為無限大
                        roll_deg = math.degrees(state.roll)
                        if roll_deg > 80.0:
                            state.dist_comp_cm = None  # 無限大（由前端根據角度判斷顯示）
                        else:
                            state.dist_comp_cm = roll_deg - state.zero_roll_deg
                        
                        print_status(state, vals)
                        state.t += DT

                except socket.timeout:
                    # 超時是正常的，繼續循環
                    continue
                except ValueError:
                    pass
                except Exception as e:
                    # print(f"\nError: {e}")  # 已禁用終端輸出
                    pass
                    
            except socket.error as e:
                # 連接錯誤，重置連接
                # print(f"TCP 連接錯誤: {e}")  # 已禁用終端輸出
                if conn_file:
                    try:
                        conn_file.close()
                    except:
                        pass
                if conn:
                    try:
                        conn.close()
                    except:
                        pass
                conn = None
                conn_file = None
                time.sleep(1)  # 等待後重試

    except KeyboardInterrupt:
        # print("\n\n使用者中止")  # 已禁用終端輸出
        pass
    finally:
        if conn_file:
            try:
                conn_file.close()
            except:
                pass
        if conn:
            try:
                conn.close()
            except:
                pass
        if tcp_socket:
            try:
                tcp_socket.close()
            except:
                pass
        # print("TCP 連線已關閉")  # 已禁用終端輸出


def main():
    """主函數：啟動 Web 伺服器和 TCP 數據讀取"""
    # 啟動 Web 伺服器（背景執行）
    web_thread = Thread(target=run_web_server, daemon=True)
    web_thread.start()
    
    # print(f"在瀏覽器中打開 http://localhost:{WEB_PORT} 查看即時數據")  # 已禁用終端輸出
    # print(f"等待 ESP32 連線到 TCP 端口 {TCP_PORT}...")  # 已禁用終端輸出
    # print("=" * 80)  # 已禁用終端輸出
    
    # 在主線程執行 TCP 數據讀取
    read_tcp_data()


if __name__ == "__main__":
    main()
