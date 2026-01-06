// ESP32: RX=16, TX=17 可改成你喜歡的硬體串口腳位
HardwareSerial bp(2);

void setup() {
  Serial.begin(9600);                  // ESP32 USB 給 Tera Term
  bp.begin(9600, SERIAL_8N1, 16, 17);  // 連 Black Pill PA2/PA3
}

void loop() {
  while (bp.available()) Serial.write(bp.read());    // Black Pill -> PC
  while (Serial.available()) bp.write(Serial.read()); // PC -> Black Pill (可選)
}