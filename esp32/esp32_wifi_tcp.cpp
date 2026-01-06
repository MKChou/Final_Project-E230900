// ESP32 透過 Wi‑Fi 將 Black Pill (MPU6050/超音波) 的字串資料轉送到 PC
// 請依需求修改：Wi‑Fi SSID、密碼、PC IP、Port 以及硬體串口腳位

#include <WiFi.h>

// ===== 請填寫網路與伺服器設定 =====
const char *WIFI_SSID     = "TP-Link_B3FC";
const char *WIFI_PASSWORD = "99879921";
const char *SERVER_IP     = "192.168.1.104"; // PC 的 IP
const uint16_t SERVER_PORT = 5001;           // PC 監聽的 TCP Port（Web 伺服器使用 5000）

// ===== 硬體串口：接到 Black Pill PA2/PA3 =====
// 預設 RX=16, TX=17，可自行調整
HardwareSerial bp(2);
const int BP_RX = 16;
const int BP_TX = 17;
const uint32_t BP_BAUD = 9600;

WiFiClient client;
unsigned long lastRetryMs = 0;

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("連線 Wi-Fi 中");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWi-Fi 已連線");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
}

bool connectServer() {
  Serial.printf("連線伺服器 %s:%u ...\n", SERVER_IP, SERVER_PORT);
  if (client.connect(SERVER_IP, SERVER_PORT)) {
    Serial.println("伺服器連線成功");
    return true;
  }
  Serial.println("伺服器連線失敗");
  return false;
}

void setup() {
  Serial.begin(9600);                   // ESP32 USB debug 改為 9600
  bp.begin(BP_BAUD, SERIAL_8N1, BP_RX, BP_TX); // 與 Black Pill 連接

  connectWiFi();
  connectServer();
}

void loop() {
  // 若連線掉了，定期嘗試重連
  if (!client.connected()) {
    unsigned long now = millis();
    if (now - lastRetryMs > 3000) {
      lastRetryMs = now;
      client.stop();
      connectServer();
    }
    delay(10);
    return;
  }

  // 從 Black Pill 讀資料 -> 傳到 PC
  while (bp.available()) {
    int c = bp.read();
    client.write((uint8_t)c);
  }

  // 如需將 PC 指令回傳給 Black Pill，可取消註解以下
  // while (client.available()) {
  //   int c = client.read();
  //   bp.write((uint8_t)c);
  // }

  // 讓循環稍作休息
  delay(2);
}

