# 適應性平面校準手杖 (Adaptive Ground Calibration Cane)

一個基於 STM32 和 ESP32 的智能手杖系統，整合 MPU6050 陀螺儀/加速度計和 HC-SR04 超音波感測器，用於檢測障礙物和手杖傾斜角度，並提供即時監控介面。

## 📋 專案簡介

本專案實現了一個智能手杖系統，主要功能包括：

- **超音波距離檢測**：使用 HC-SR04 超音波感測器檢測前方障礙物距離
- **角度檢測**：使用 MPU6050 感測器檢測手杖傾斜角度（Roll/Pitch）
- **角度補償**：根據手杖傾斜角度自動補償距離測量值
- **障礙物警示**：當檢測到障礙物或坑洞時，觸發蜂鳴器和馬達震動提醒
- **按鈕校準**：通過 PB12 按鈕進行零點校準
- **即時監控**：提供 Web 介面即時顯示感測器數據

## 🏗️ 系統架構

```
STM32 Black Pill (主控制器)
    ├── MPU6050 (陀螺儀/加速度計)
    ├── HC-SR04 (超音波感測器)
    ├── PB12 (校準按鈕)
    ├── 蜂鳴器 (警示)
    └── 馬達 (震動提醒)
         │
         │ (串口通訊)
         │
ESP32 (無線傳輸模組)
    ├── WiFi 連接
    └── TCP 轉發
         │
         │ (TCP/IP)
         │
PC (監控端)
    └── Python Web 伺服器
        └── 瀏覽器介面
```

## 📁 專案結構

```
.
├── README.md                      # 本文件
├── requirements.txt               # Python 依賴套件
├── .gitignore                     # Git 忽略文件
│
├── stm32/                         # STM32 相關代碼
│   └── STM32F_hcsr04_mpu6050.cpp # STM32 主程序
│
├── esp32/                         # ESP32 相關代碼
│   ├── esp32.cpp                  # ESP32 串口透傳
│   └── esp32_wifi_tcp.cpp        # ESP32 WiFi TCP 轉發
│
├── python/                        # Python 監控程式
│   ├── mpu6050_viewer.py         # 串口直接讀取版本
│   ├── mpu6050_viewer_tcp.py    # TCP 接收版本
│   ├── mpu6050_viewer_wifi.py   # WiFi TCP 版本（改進版）
│   └── mpu6050_viewer_simple.py # 簡化版本（使用 vpython）
│
└── docs/                          # 文檔和圖片
    ├── Pinout-Diagram.png         # 接線圖
    └── featured_*.jpg            # 專案圖片
```

## 🔧 硬體需求

### STM32 Black Pill (STM32F401CCU6)
- **MPU6050**：I2C 連接（SDA: PB7, SCL: PB6）
- **HC-SR04**：
  - Trig: PB8
  - Echo: PA0（需分壓至 3.3V）
- **按鈕**：PB12（PullUp，按下接地）
- **蜂鳴器**：PB4
- **馬達**：PB5（需經 MOSFET/三極體驅動）
- **心跳 LED**：PC13
- **串口**：PA2 (TX), PA3 (RX)，9600 baud

### ESP32
- **串口連接 STM32**：RX=16, TX=17，9600 baud
- **WiFi 連接**：需配置 SSID 和密碼

## 📦 軟體需求

### STM32 開發環境
- **Mbed OS** 或 **STM32CubeIDE**
- 本專案使用 Mbed OS 開發

### ESP32 開發環境
- **Arduino IDE** 或 **PlatformIO**
- 需安裝 ESP32 開發板支援

### Python 環境
- Python 3.7+
- 所需套件見 `requirements.txt`

## 🚀 安裝與使用

### 1. 安裝 Python 依賴

```bash
pip install -r requirements.txt
```

### 2. 配置 ESP32 WiFi

編輯 `esp32/esp32_wifi_tcp.cpp`，修改以下設定：

```cpp
const char *WIFI_SSID     = "你的WiFi名稱";
const char *WIFI_PASSWORD = "你的WiFi密碼";
const char *SERVER_IP     = "192.168.1.104"; // PC 的 IP 地址
const uint16_t SERVER_PORT = 5001;           // TCP 端口
```

### 3. 上傳程式到硬體

#### STM32
1. 使用 Mbed Online Compiler 或本地 Mbed CLI
2. 編譯並上傳 `STM32F_hcsr04_mpu6050.cpp`

#### ESP32
1. 使用 Arduino IDE 或 PlatformIO
2. 上傳 `esp32_wifi_tcp.cpp`（WiFi 版本）或 `esp32.cpp`（串口透傳版本）

### 4. 運行 Python 監控程式

根據你的連接方式選擇對應的程式：

#### 方式 1：串口直接連接（STM32 → PC）
```bash
python python/mpu6050_viewer.py
```
**注意**：需修改程式中的 `COM_PORT` 為你的串口編號（Windows: COMx, Linux/Mac: /dev/ttyUSBx）

#### 方式 2：TCP 連接（STM32 → ESP32 → PC）
```bash
python python/mpu6050_viewer_tcp.py
```
或使用改進版本：
```bash
python python/mpu6050_viewer_wifi.py
```

#### 方式 3：簡化版本（使用 vpython 3D 顯示）
```bash
python python/mpu6050_viewer_simple.py
```

程式會自動在瀏覽器中打開監控介面（預設 `http://localhost:5000`）

## 📊 功能說明

### 感測器數據
- **超音波距離**：即時顯示前方障礙物距離（單位：cm）
- **手杖傾斜角度**：顯示 Roll 角度（單位：度）
- **傾斜補償距離**：根據角度補償後的距離值
- **警報距離**：可設定的警示距離閾值

### 按鈕功能
- **PB12 按鈕**：
  - 按下時進行零點校準
  - 設定當前角度為基準角度
  - 更新警報距離為「50cm + 當前距離」

### 警示系統
- **障礙物檢測**：當距離突然變大超過 50cm 時，連續檢測 2 次後觸發
- **蜂鳴器**：觸發時發出「響 0.1s → 停 0.1s → 響 0.1s → 停 0.7s」的循環警示音
- **馬達震動**：觸發時震動「0.1s → 停 0.1s → 0.1s」，並有 5 秒冷卻期

### Web 介面功能
- 即時顯示所有感測器數據
- 顯示 MPU6050 原始數據（加速度和陀螺儀）
- 按鈕狀態監控
- 事件提示橫幅
- 響應式設計，支援手機瀏覽

## 🔌 接線說明

### STM32 接線

| 元件 | STM32 腳位 | 說明 |
|------|-----------|------|
| MPU6050 SDA | PB7 | I2C 數據線 |
| MPU6050 SCL | PB6 | I2C 時鐘線 |
| HC-SR04 Trig | PB8 | 觸發腳 |
| HC-SR04 Echo | PA0 | 回波腳（需分壓至 3.3V） |
| 按鈕 | PB12 | 校準按鈕（PullUp，按下接地） |
| 蜂鳴器 | PB4 | 警示蜂鳴器 |
| 馬達 | PB5 | 震動馬達（需驅動電路） |
| LED | PC13 | 心跳指示燈 |
| UART TX | PA2 | 串口發送 |
| UART RX | PA3 | 串口接收 |

### ESP32 接線

| 功能 | ESP32 腳位 | 連接至 |
|------|-----------|--------|
| RX | GPIO 16 | STM32 PA2 (TX) |
| TX | GPIO 17 | STM32 PA3 (RX) |

**注意**：ESP32 為 3.3V 邏輯，STM32 也是 3.3V，可直接連接。

## ⚙️ 參數調整

### STM32 參數（在 `STM32F_hcsr04_mpu6050.cpp` 中）

```cpp
const float safety_margin_cm = 50.0f;  // 障礙物檢測閾值（cm）
const int hit_need = 2;                // 連續檢測次數
const int motor_cooldown_ms = 5000;    // 馬達冷卻期（毫秒）
```

### Python 參數（在各 viewer 程式中）

```python
LOOP_RATE = 20          # 更新頻率 (Hz)
DT = 0.05              # 互補濾波時間步長
DIST_WINDOW = 5        # 距離中值濾波視窗
DIST_JUMP_LIMIT = 30.0 # 距離變化限幅 (cm)
WEB_PORT = 5000        # Web 伺服器端口
TCP_PORT = 5001        # TCP 伺服器端口（TCP 版本）
```

## 🐛 故障排除

### STM32 無法讀取 MPU6050
- 檢查 I2C 接線（SDA/SCL）
- 確認 MPU6050 電源（3.3V 或 5V）
- 檢查 I2C 地址是否為 0x68

### 超音波距離讀數異常
- 確認 Echo 腳已分壓至 3.3V（ESP32/STM32 不支援 5V 輸入）
- 檢查 Trig 和 Echo 接線
- 確認感測器供電正常

### ESP32 無法連接 WiFi
- 檢查 SSID 和密碼是否正確
- 確認 WiFi 頻段為 2.4GHz（ESP32 不支援 5GHz）
- 檢查路由器是否允許新設備連接

### Python 程式無法連接
- **串口版本**：確認 COM 端口號碼正確，且未被其他程式佔用
- **TCP 版本**：確認 PC IP 地址正確，防火牆允許端口 5001
- 檢查 ESP32 和 PC 是否在同一網路

### Web 介面無法顯示數據
- 確認 Python 程式正在運行
- 檢查瀏覽器控制台是否有錯誤
- 確認端口 5000 未被佔用

## 📝 開發日誌

- 實現 MPU6050 和 HC-SR04 感測器讀取
- 實現角度補償算法
- 實現障礙物檢測和警示系統
- 實現按鈕校準功能
- 開發 Web 監控介面
- 實現 ESP32 WiFi 轉發功能
- 優化數據平滑和濾波算法

## 📄 授權

本專案採用 MIT 授權條款。

## 👥 貢獻

歡迎提交 Issue 和 Pull Request！

## 📧 聯絡方式

如有問題或建議，請透過 GitHub Issues 聯繫。

---

**注意**：使用本專案時請注意安全，本系統僅作為輔助工具，不能完全替代人工判斷。
