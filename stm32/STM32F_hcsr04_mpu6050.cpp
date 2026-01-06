#include "mbed.h"

// 明確指定 UART 走 PA_2/PA_3，預設 9600
static Serial pc(PA_2, PA_3);

// 超音波腳位（Echo 請分壓到 3.3V）
DigitalOut led_hb(PC_13);        // 心跳燈（PC13 低電位亮）
DigitalOut buzzer(PB_4);         // 蜂鳴器
DigitalOut motor(PB_5);          // 馬達驅動（請經 MOSFET/三極體）
DigitalIn btn(PB_12);            // 按鈕（按下校準零點，使用輪詢方式）
DigitalOut trig(PB_8);
InterruptIn echo(PA_0);          // Echo 請確保分壓到 3.3V`
Timer t;

volatile uint32_t echo_us = 0;   // microseconds
bool calibrate = false;          // 按鈕觸發校準（輪詢方式，不需要volatile）
float zero_pitch_deg = 0.0f;     // 手杖角度零點
float zero_distance_cm = 0.0f;   // 距離零點

// MPU6050 腳位與地址
I2C i2c(PB_7, PB_6);             // SDA, SCL我的MPU6050 測是起來
const int MPU_ADDR = 0x68 << 1;  // mbed 使用 8-bit 地址

void echo_rise() {
    t.reset();
    t.start();
}

void echo_fall() {
    t.stop();
    echo_us = t.read_us();
}

// 按鈕已改為輪詢方式，不再使用中斷

void mpu_write(uint8_t reg, uint8_t val) {
    char d[2] = {(char)reg, (char)val};
    i2c.write(MPU_ADDR, d, 2);
}

bool mpu_read(uint8_t reg, char *buf, int len) {
    char r = reg;
    if (i2c.write(MPU_ADDR, &r, 1, true) != 0) return false;
    if (i2c.read(MPU_ADDR, buf, len) != 0) return false;
    return true;
}

bool mpu_init() {
    i2c.frequency(400000); // 400kHz
    mpu_write(0x6B, 0x00); // PWR_MGMT_1: 退出睡眠
    thread_sleep_for(100);
    char whoami = 0;
    if (!mpu_read(0x75, &whoami, 1)) return false;
    return (whoami == 0x68);
}

int main() {
    pc.baud(9600);
    pc.format(8, SerialBase::None, 1);
    pc.printf("HC-SR04 + MPU6050 demo\r\n");

    // 初始化蜂鳴器和馬達為低電平，確保不會有雜音
    buzzer = 0;
    motor = 0;
    pc.printf("Buzzer and motor initialized to OFF\r\n");

    // 超音波設定
    echo.mode(PullDown);    // 防止 Echo 浮空
    echo.rise(&echo_rise);
    echo.fall(&echo_fall);
    btn.mode(PullUp);      // 上拉模式：PB12 > 按鈕 > GND（按下為0，未按下為1）
    pc.printf("Button PB12 initialized (PullUp, polling mode)\r\n");
    pc.printf("Button connection: PB12 > Button > GND\r\n");
    pc.printf("Button logic: Pressed=0 (GND), Released=1 (PullUp)\r\n");

    // MPU6050 初始化
    bool mpu_ok = mpu_init();
    pc.printf("MPU6050 init: %s\r\n", mpu_ok ? "OK" : "FAIL");

    // 按鈕狀態變數
    static bool btn_was_pressed = false;  // 上次是否按下
        
    while (true) {
        // 簡單檢測：PB12是否接到GND（按下為0，未按下為1）
        bool btn_current = btn.read();
        
        // 檢測按下：PB12接地（讀數為0）
        if (btn_current == 0 && !btn_was_pressed && !calibrate) {
            // 按鈕剛按下，觸發校準
            calibrate = true;
            buzzer = 1;
            thread_sleep_for(100);  // 響100ms確認
            buzzer = 0;
            pc.printf("[DEBUG] Button PB12 pressed, calibrate flag set\r\n");
        }
        
        btn_was_pressed = (btn_current == 0);  // 記錄當前狀態
        
        // --- 超音波測距 ---
        echo_us = 0;
        trig = 0;
        thread_sleep_for(2);
        trig = 1;
        wait_us(10);
        trig = 0;

        thread_sleep_for(50);  // 等待回波

        float distance_cm = echo_us * 0.017f;  // 340 m/s -> us to cm
        pc.printf("distance: %.2f cm\r\n", distance_cm);

        // --- 讀 MPU6050 ---
        if (mpu_ok) {
            char buf[14] = {0};
            if (mpu_read(0x3B, buf, 14)) {
                int16_t ax = (buf[0] << 8) | buf[1];
                int16_t ay = (buf[2] << 8) | buf[3];
                int16_t az = (buf[4] << 8) | buf[5];
                int16_t gx = (buf[8] << 8) | buf[9];
                int16_t gy = (buf[10] << 8) | buf[11];
                int16_t gz = (buf[12] << 8) | buf[13];
                // 以加速度估計手杖傾斜角（俯仰/翻滾），單位度數
                float roll = atan2f((float)ay, (float)az) * 57.2958f;
                float pitch = atanf(-(float)ax / sqrtf((float)ay * ay + (float)az * az)) * 57.2958f;
                float pitch_rad = pitch * 0.0174533f;
                float distance_comp = distance_cm * cosf(pitch_rad);  // 補償傾斜

                // 按鈕校準：將當前角度與距離設定為零點
                if (calibrate) {
                    pc.printf("[DEBUG] Calibrate flag detected in main loop\r\n");
                    pc.printf("Calibrated: PB12 pressed\r\n");
                    zero_pitch_deg = pitch;
                    zero_distance_cm = distance_cm;
                    calibrate = false;
                    pc.printf("Calibrated: zero_pitch=%.2f deg, zero_distance=%.2f cm\r\n",
                              zero_pitch_deg, zero_distance_cm);
                }

                float pitch_rel = pitch - zero_pitch_deg;
                float distance_rel = distance_cm - zero_distance_cm;
                float distance_comp_rel = distance_comp - zero_distance_cm; // 以校準零點作為基準

                // 警示邏輯：偵測高低差（距離變大）
                const float safety_margin_cm = 50.0f; // 大於此差值視為下陷/坑洞，可調（50cm）
                static int hit_count = 0;
                const int hit_need = 2;            // 連續次數
                const int buzzer_pulse_ms = 100;  // 蜂鳴器單次響的時間（0.1秒）
                const int buzzer_short_rest_ms = 100;  // 蜂鳴器短休息時間（0.1秒，兩聲之間）
                const int buzzer_long_rest_ms = 700;   // 蜂鳴器長休息時間（0.7秒，周期結束前）
                static bool buzzer_state = false; // 當前蜂鳴器狀態（true=響，false=停）
                static int buzzer_pulse_count = 0; // 蜂鳴器脈衝計數（0=第一聲，1=短休息，2=第二聲，3=長休息）
                static Timer buzzer_timer;        // 蜂鳴器狀態切換計時器
                static bool buzzer_timer_started = false; // 計時器是否已啟動
                const int motor_pulse_ms = 100;   // 馬達單次脈衝時間（0.1秒）
                const int motor_rest_ms = 100;     // 馬達休息時間（0.1秒）
                static Timer motor_timer;          // 馬達計時器
                static bool motor_triggered = false; // 馬達是否已觸發
                static bool motor_timer_started = false; // 馬達計時器是否已啟動
                static int motor_pulse_count = 0;  // 馬達脈衝計數（0=第一次響，1=休息，2=第二次響）
                const int motor_cooldown_ms = 5000; // 馬達冷卻期（5秒），避免連續觸發
                static Timer motor_cooldown_timer; // 馬達冷卻計時器
                static bool motor_in_cooldown = false; // 馬達是否在冷卻期

                // 偵測高低差：距離「變大」才觸發
                bool hit = (echo_us > 0) && (distance_comp_rel > safety_margin_cm);
                if (hit) {
                    if (++hit_count >= hit_need) {
                        // 首次觸發時啟動蜂鳴器計時器
                        if (!buzzer_timer_started) {
                            buzzer_timer.start();
                            buzzer_timer.reset();
                            buzzer_timer_started = true;
                            buzzer_state = true;  // 開始時先響第一聲
                            buzzer_pulse_count = 0;  // 從第一聲開始
                            buzzer = 1;
                            
                            // 馬達獨立觸發：檢查是否在冷卻期
                            if (!motor_in_cooldown) {
                                // 不在冷卻期，可以觸發馬達
                                if (!motor_triggered) {
                                    motor = 1;  // 第一次響
                                    motor_pulse_count = 0;  // 從第一次響開始
                                    motor_timer.start();
                                    motor_timer.reset();
                                    motor_timer_started = true;
                                    motor_triggered = true;
                                    
                                    // 啟動冷卻期計時器
                                    motor_cooldown_timer.start();
                                    motor_cooldown_timer.reset();
                                    motor_in_cooldown = true;
                                }
                            }
                        } else {
                            // 檢查蜂鳴器是否需要切換狀態（響0.1秒 → 停0.1秒 → 響0.1秒 → 停0.7秒，周期1秒）
                            int elapsed_ms = buzzer_timer.read_ms();
                            
                            if (buzzer_pulse_count == 0) {
                                // 第一聲：響0.1秒
                                buzzer = 1;
                                if (elapsed_ms >= buzzer_pulse_ms) {
                                    buzzer_pulse_count = 1;  // 切換到短休息
                                    buzzer = 0;
                                    buzzer_timer.reset();
                                    buzzer_timer.start();
                                }
                            } else if (buzzer_pulse_count == 1) {
                                // 短休息：停0.1秒（兩聲之間）
                                buzzer = 0;
                                if (elapsed_ms >= buzzer_short_rest_ms) {
                                    buzzer_pulse_count = 2;  // 切換到第二聲
                                    buzzer = 1;
                                    buzzer_timer.reset();
                                    buzzer_timer.start();
                                }
                            } else if (buzzer_pulse_count == 2) {
                                // 第二聲：響0.1秒
                                buzzer = 1;
                                if (elapsed_ms >= buzzer_pulse_ms) {
                                    buzzer_pulse_count = 3;  // 切換到長休息
                                    buzzer = 0;
                                    buzzer_timer.reset();
                                    buzzer_timer.start();
                                }
                            } else if (buzzer_pulse_count == 3) {
                                // 長休息：停0.7秒（完成一個周期，回到第一聲）
                                buzzer = 0;
                                if (elapsed_ms >= buzzer_long_rest_ms) {
                                    buzzer_pulse_count = 0;  // 循環回到第一聲
                                    buzzer = 1;
                                    buzzer_timer.reset();
                                    buzzer_timer.start();
                                }
                            }
                        }
                    }
                } else {
                    // 沒有檢測到障礙物，重置所有狀態
                    hit_count = 0;
                    buzzer = 0;
                    motor = 0;
                    buzzer_state = false;
                    buzzer_pulse_count = 0;
                    buzzer_timer.stop();
                    buzzer_timer.reset();
                    buzzer_timer_started = false;
                    motor_timer.stop();
                    motor_timer.reset();
                    motor_timer_started = false;
                    motor_triggered = false;
                    motor_pulse_count = 0;
                    // 注意：冷卻期計時器繼續運行，直到時間到
                }
                
                // 檢查馬達脈衝序列：響0.1秒 → 休息0.1秒 → 再響0.1秒
                if (motor_timer_started) {
                    int elapsed_ms = motor_timer.read_ms();
                    
                    if (motor_pulse_count == 0) {
                        // 第一次響：0.1秒
                        motor = 1;
                        if (elapsed_ms >= motor_pulse_ms) {
                            motor_pulse_count = 1;  // 切換到休息
                            motor = 0;
                            motor_timer.reset();
                            motor_timer.start();
                        }
                    } else if (motor_pulse_count == 1) {
                        // 休息：0.1秒
                        motor = 0;
                        if (elapsed_ms >= motor_rest_ms) {
                            motor_pulse_count = 2;  // 切換到第二次響
                            motor = 1;
                            motor_timer.reset();
                            motor_timer.start();
                        }
                    } else if (motor_pulse_count == 2) {
                        // 第二次響：0.1秒
                        motor = 1;
                        if (elapsed_ms >= motor_pulse_ms) {
                            motor = 0;
                            motor_timer.stop();
                            motor_timer.reset();
                            motor_timer_started = false;
                            motor_pulse_count = 0;
                            // 完成脈衝序列，在冷卻期內不再開啟
                        }
                    }
                }
                
                // 檢查冷卻期是否結束
                if (motor_in_cooldown) {
                    if (motor_cooldown_timer.read_ms() >= motor_cooldown_ms) {
                        motor_in_cooldown = false;
                        motor_cooldown_timer.stop();
                        motor_cooldown_timer.reset();
                        // 冷卻期結束，如果障礙物消失，重置觸發標記
                        if (!hit) {
                            motor_triggered = false;
                        }
                    }
                }

                pc.printf("MPU ax:%d ay:%d az:%d gx:%d gy:%d gz:%d roll:%.2f pitch:%.2f pitch_rel:%.2f\r\n",
                          ax, ay, az, gx, gy, gz, roll, pitch, pitch_rel);
                pc.printf("distance_comp: %.2f cm distance_rel: %.2f cm distance_comp_rel: %.2f cm\r\n",
                          distance_comp, distance_rel, distance_comp_rel);
            } else {
                pc.printf("MPU read fail\r\n");
                if (calibrate) {
                    calibrate = false;
                    pc.printf("Calibrated: PB12 pressed but MPU read fail\r\n");
                }
            }
        }
        // 若 MPU 未就緒，但按鈕被按下，仍傳送事件
        if (!mpu_ok && calibrate) {
            pc.printf("[DEBUG] Calibrate flag detected but MPU not ready\r\n");
            calibrate = false;
            pc.printf("Calibrated: PB12 pressed (MPU not ready)\r\n");
        }
        
        // 檢查按鈕狀態（用於調試）
        static int btn_check_counter = 0;
        if (++btn_check_counter >= 20) {  // 每20次循環檢查一次（約1秒）
            btn_check_counter = 0;
            // 讀取按鈕狀態（PullUp模式下，按下為0，未按下為1）
            // 注意：InterruptIn 不直接支持讀取，但可以通過其他方式檢查
        }

        // 控制數據採集速度：5-10 FPS（每秒5-10次）
        // 計算延遲時間：10 FPS = 100ms, 5 FPS = 200ms
        // 使用中間值約 7.5 FPS = 133ms
        const int target_fps = 8;  // 目標 8 FPS
        const int loop_delay_ms = 1000 / target_fps;  // 約 125ms
        
        // 心跳閃燈：閃0.1秒 暗0.1秒 閃0.1秒 暗0.1秒 閃0.3秒 暗0.3秒（循環）
        static Timer led_timer;
        static int led_state = 0;  // 0=閃0.1, 1=暗0.1, 2=閃0.1, 3=暗0.1, 4=閃0.3, 5=暗0.3
        static bool led_timer_started = false;
        
        if (!led_timer_started) {
            led_timer.start();
            led_timer_started = true;
            led_hb = 0;  // 開始時亮（PC13低電位亮）
        }
        
        int elapsed_ms = led_timer.read_ms();
        int state_duration_ms = 0;
        bool should_be_on = false;
        
        switch (led_state) {
            case 0: // 閃0.1秒
                state_duration_ms = 100;
                should_be_on = true;
                break;
            case 1: // 暗0.1秒
                state_duration_ms = 100;
                should_be_on = false;
                break;
            case 2: // 閃0.1秒
                state_duration_ms = 100;
                should_be_on = true;
                break;
            case 3: // 暗0.1秒
                state_duration_ms = 100;
                should_be_on = false;
                break;
            case 4: // 閃0.3秒
                state_duration_ms = 300;
                should_be_on = true;
                break;
            case 5: // 暗0.3秒
                state_duration_ms = 300;
                should_be_on = false;
                break;
        }
        
        // 設置LED狀態（PC13低電位亮，所以應該取反）
        led_hb = should_be_on ? 0 : 1;
        
        // 檢查是否需要切換到下一個狀態
        if (elapsed_ms >= state_duration_ms) {
            led_state = (led_state + 1) % 6;  // 循環 0-5
            led_timer.reset();
            led_timer.start();
        }
    }
}


