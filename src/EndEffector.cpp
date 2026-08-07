#include "EndEffector.h"

#define SERVO_PIN PB10 

const int GRIPPER_OPEN = 1020;   
const int GRIPPER_CLOSE = 1920; 

int gripper_current_us = GRIPPER_OPEN; 
int gripper_target_us = GRIPPER_OPEN;  
unsigned long servo_last_time = 0; 

// === [修改物理極速] ===
int servo_step_size = 45;  // 每次微調 45 微秒 (原本是 90)
int servo_step_delay = 15; // 每 15 毫秒更新一次 (維持 15)
// ==================================

bool ee_cmd_waiting = false;
int pending_target_us = GRIPPER_OPEN;
bool waiting_for_servo_arrival = false; 

void initEndEffector() {
    pinMode(SERVO_PIN, OUTPUT);
    
    // ==========================================
    // 啟用 STM32 原生硬體 PWM (免疫中斷抖動)
    // ==========================================
    analogWriteResolution(16);     // 設定解析度為 16-bit (範圍 0~65535)
    analogWriteFrequency(50);      // 設定伺服馬達標準頻率 50Hz (週期 20ms = 20000us)
    
    // 將微秒轉換為 16-bit 的佔空比並輸出
    int pwm_val = (gripper_current_us * 65535) / 20000;
    analogWrite(SERVO_PIN, pwm_val);
    
    Serial.println("[System] Gripper initialized in Native Hardware PWM mode.");
}

void setGripperTarget(int percent) {
    percent = constrain(percent, 0, 100);
    pending_target_us = map(percent, 0, 100, GRIPPER_OPEN, GRIPPER_CLOSE);
    ee_cmd_waiting = true;
}

void updateEndEffector(bool isArmIdle) {
    // 1. 等待手臂停穩後，才把任務交給夾爪
    if (ee_cmd_waiting && isArmIdle) {
        gripper_target_us = pending_target_us;
        ee_cmd_waiting = false;
        waiting_for_servo_arrival = true; 
    }

    // 2. 夾爪平滑移動邏輯
    if (gripper_current_us != gripper_target_us) {
        if (millis() - servo_last_time >= servo_step_delay) {
            int distance = gripper_target_us - gripper_current_us;

            if (abs(distance) > servo_step_size) {
                if (distance > 0) gripper_current_us += servo_step_size;
                else gripper_current_us -= servo_step_size;
            } else {
                gripper_current_us = gripper_target_us;
            }
            
            // ==========================================
            // 更新硬體 PWM (CPU 丟給暫存器後就不管了，絕對穩定)
            // ==========================================
            int pwm_val = (gripper_current_us * 65535) / 20000;
            analogWrite(SERVO_PIN, pwm_val);
            
            servo_last_time = millis();
        }
    } else {
        if (waiting_for_servo_arrival) {
            Serial.println("<EE_DONE>"); 
            waiting_for_servo_arrival = false;
        }
    }
}