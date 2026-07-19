#include "Comms.h"
#include "Globals.h"
#include "Config.h"
#include "MotionEngine.h" 
#include <stdint.h>

#pragma pack(push, 1)
struct MotionPacket {
    uint16_t header;       // 2 bytes (0x55AA)
    int32_t  targets[6];   // 24 bytes
    float    speedFactor;  // 4 bytes
    uint8_t  moveMode;     // 1 byte
    uint8_t  checksum;     // 1 byte
};
#pragma pack(pop)

MotionPacket currentPacket;

enum RxState { WAIT_HEADER_1, WAIT_HEADER_2, READ_PAYLOAD };
RxState rxState = WAIT_HEADER_1;
uint8_t rxBuffer[sizeof(MotionPacket)];
int rxIndex = 0;

// ==========================================
// 環狀暫存佇列 (取代單一 pendingMode1)
// ==========================================
const int PENDING_BUF_SIZE = 10;
struct PendingPoint {
    long     t[6];
    uint32_t interval;
};
PendingPoint pendingBuf[PENDING_BUF_SIZE];
int pendingHead = 0;
int pendingTail = 0;
int pendingCount = 0;

// ==========================================
// 工業標準 CRC-8 計算引擎 (多項式 0x07)
// ==========================================
uint8_t calculateCRC8(const uint8_t *data, int len) {
    uint8_t crc = 0x00;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}

// ==========================================
// 核心：純二進制指令執行引擎
// ==========================================
void executeBinaryCommand() {
    long targets[6];
    int moveMode = currentPacket.moveMode;
    float param7 = currentPacket.speedFactor;

    for(int i = 0; i < 6; i++) {
        targets[i] = currentPacket.targets[i];
        if (moveMode == 0 || moveMode == 1 || moveMode == 2) {
            receivedSteps[i] = targets[i]; 
        }
    }

    // ------------------------------------------
    // Mode 4：急停 (E-STOP)
    // ------------------------------------------
    if (moveMode == 4) {
        for(int i = 0; i < 6; i++) homingState[i] = 0;  
        emergencyStopEngine(); 
        
        // 瞬間清空所有排隊中的點位！
        pendingHead = 0;
        pendingTail = 0;
        pendingCount = 0; 
        
        normalMoveActive = false;
        is_estop_latched = true; 
        Serial.println("!!! E-STOP TRIGGERED & LATCHED !!!");
        return;
    }

    // ------------------------------------------
    // Mode 9：明確的人工解除急停復歸 (E-STOP RESET)
    // ------------------------------------------
    if (moveMode == 9) {
        if (is_estop_latched) {
            is_estop_latched = false;
            Serial.println("SYS: E-STOP RESET SUCCESS. SYSTEM READY.");
        } else {
            Serial.println("SYS: SYSTEM ALREADY OPRATIONAL.");
        }
        return;
    }

    // 核心防禦：如果目前處於急停鎖存中，無條件彈回所有後續運動指令！
    if (is_estop_latched) {
        Serial.println("ERR: COMMAND REJECTED. SYSTEM LATCHED IN E-STOP!");
        return;
    }

    // ------------------------------------------
    // Mode 5：夾爪 (Gripper)
    // ------------------------------------------
    if (moveMode == 5) {
        // ... 夾爪控制邏輯 ...
        Serial.println("<EE_DONE>");
        return;
    }

    // ------------------------------------------
    // Mode 3：歸零 (Homing)
    // ------------------------------------------
    if (moveMode == 3) {
        bool group1_req = (targets[0] == 999999 || targets[1] == 999999 || targets[2] == 999999);
        bool j4_req = (targets[3] == 999999);
        bool j6_req = (targets[5] == 999999);
        bool homingTriggered = false;

        for (int i = 0; i < 6; i++) {
            bool targetIsHome = (targets[i] == 999999);
            if (targetIsHome && JOINT_PINS[i].limitPin != 0 && HOMING_CFG[i].homingSpeed != 0) {
                if (homingState[i] == 0) {
                    if (i >= 3 && group1_req) { homingState[i] = 20; homingTriggered = true; } 
                    else if (i == 4 && j6_req) { homingState[i] = 10; homingTriggered = true; } 
                    else if (i == 5 && j4_req) { homingState[i] = 10; homingTriggered = true; } 
                    else {
                        homingState[i] = 1; 
                        long homingSpdSec = abs(HOMING_CFG[i].homingSpeed);
                        // 尋點第一下，套用 0.2f 的柔和緩啟動加速度！
                        jogAxis(i, (HOMING_CFG[i].homingSpeed > 0) ? 1 : -1, (float)homingSpdSec, getAxisAccel(i, 0.2f), true); 
                        homingTriggered = true;
                    }
                } 
            }
        }
        
        if (homingTriggered) {
            Serial.println("OK");
        }
        return; 
    }
        
    // ------------------------------------------
    // 一般運動邏輯 (Mode 0, Mode 1, Mode 2)
    // ------------------------------------------
    if (!isAnyHoming()) {
        
        // ==========================================
        // Mode 1: PVT 串流
        // ==========================================
        if (moveMode == 1) {
            // 防線 1：如果發生緩衝區溢位，軌跡已損毀，直接觸發急停！
            if (pendingCount >= PENDING_BUF_SIZE) {
                Serial.println("ERR: PENDING BUF OVERFLOW! LATCHING E-STOP!");
                emergencyStopEngine();
                is_estop_latched = true;
                pendingHead = 0; pendingTail = 0; pendingCount = 0;
                return;
            }
            
            for(int i = 0; i < 6; i++) {
                long target_val = (targets[i] != 999999) ? targets[i] : getAxisPosition(i);
                
                // 防線 2：PVT 點位入列前的「絕對軟限位檢查」！
                if (JOINT_PINS[i].limitPin != 0) {
                    if (target_val > AXIS_MAX_LIMIT[i] || target_val < AXIS_MIN_LIMIT[i]) {
                        Serial.println("ERR: OUT OF BOUNDS! LATCHING E-STOP!");
                        emergencyStopEngine();
                        is_estop_latched = true;
                        pendingHead = 0; pendingTail = 0; pendingCount = 0;
                        return; // 拒絕收錄這個點，並全系統鎖死
                    }
                }
                pendingBuf[pendingHead].t[i] = target_val; 
            }
            
            pendingBuf[pendingHead].interval = 10000; 
            
            pendingHead = (pendingHead + 1) % PENDING_BUF_SIZE;
            pendingCount++;
            return; 
        }
        
        // ==========================================
        // Mode 0：手動 / 點動模式 (PTP)
        // ==========================================
        else if (moveMode == 0) {
            float speedFactor = (param7 <= 0.0) ? 1.0 : param7;
            long t[6];
            long delta[6];
            float maxTime = 0.0;
            bool needsMove = false;

            for (int i = 0; i < 6; i++) {
                long currentPos = getAxisPosition(i);
                t[i] = (targets[i] != 999999) ? targets[i] : currentPos;
                delta[i] = t[i] - currentPos;
                
                if (delta[i] != 0) {
                    needsMove = true;
                    float v_max = SPEED_CFG[i].controlSpeed * speedFactor;
                    if (v_max > 0.0f) {
                        float t_needed = (float)abs(delta[i]) / v_max;
                        if (t_needed > maxTime) maxTime = t_needed;
                    }
                }
            }

            if (!needsMove) {
                Serial.println("OK");
                return;
            }

            for (int i = 0; i < 6; i++) {
                if (delta[i] != 0) {
                    float coordinatedSpeed = (maxTime > 0.0f) ? ((float)abs(delta[i]) / maxTime) : 1.0f;
                    if (coordinatedSpeed < 0.1f) coordinatedSpeed = 0.1f; 
                    
                    float max_v_this_axis = SPEED_CFG[i].controlSpeed * speedFactor;
                    float scaleRatio = (max_v_this_axis > 0.0f) ? (coordinatedSpeed / max_v_this_axis) : 1.0f;
                    
                    float baseAccel = getAxisAccel(i);
                    float coordinatedAccel = baseAccel * scaleRatio;
                    if (coordinatedAccel < 10.0f) coordinatedAccel = 10.0f; 

                    moveAxisIndependent(i, delta[i], coordinatedSpeed, coordinatedAccel);
                }
            }

            normalMoveActive = true;
            Serial.println("OK"); 
        }
        
        // ==========================================
        // Mode 2: 連續寸動 (Joint Jogging)
        // ==========================================
        else if (moveMode == 2) {
            int axis = targets[0]; 
            int dir = targets[1];  
            float speedFactor = param7;  

            if (axis >= 0 && axis < 6) {
                float absoluteMaxSpeedSec = SPEED_CFG[axis].controlSpeed;
                float jogSpeedSec = absoluteMaxSpeedSec * speedFactor;
                if (jogSpeedSec < 10.0f) jogSpeedSec = 10.0f;
                float accel = getAxisAccel(axis);

                if (dir == 0) jogAxis(axis, 0, 0.0f, accel, false); 
                else jogAxis(axis, dir, jogSpeedSec, accel, false);
            }
            normalMoveActive = true;
            Serial.println("OK");
        }
    }
}

// ==========================================
// 二進制接收狀態機 (搭載逾時防死鎖與 CRC8)
// ==========================================
void receiveBinaryLoop() {
    // 1. 嘗試推送卡在排隊區的 Mode 1 點位 (能推多少推多少)
    while (pendingCount > 0) {
        if (pushMotionPoint(pendingBuf[pendingTail].t[0], pendingBuf[pendingTail].t[1], 
                            pendingBuf[pendingTail].t[2], pendingBuf[pendingTail].t[3], 
                            pendingBuf[pendingTail].t[4], pendingBuf[pendingTail].t[5], 
                            pendingBuf[pendingTail].interval)) {
            
            pendingTail = (pendingTail + 1) % PENDING_BUF_SIZE;
            pendingCount--;
            
            normalMoveActive = true;
            Serial.println("OK"); // 確定推進主引擎後，才發送 OK 給 Python
        } else {
            break; // 引擎水池還是滿的，立刻跳出，等下一圈 UART 迴圈再試
        }
    }

    // 2. 逾時保護機制
    static uint32_t lastRxTime = 0;
    if (rxState != WAIT_HEADER_1 && (millis() - lastRxTime > 50)) {
        Serial.println("WARN: RX Timeout! Dropping incomplete packet.");
        rxState = WAIT_HEADER_1;
        rxIndex = 0;
    }

    // 3. 正常讀取 UART
    while (Serial.available() > 0) {
        uint8_t c = Serial.read();
        lastRxTime = millis(); // 只要有資料進來，就更新最後心跳時間

        switch (rxState) {
            case WAIT_HEADER_1:
                if (c == 0xAA) { 
                    rxBuffer[0] = c;
                    rxState = WAIT_HEADER_2;
                }
                break;
                
            case WAIT_HEADER_2:
                if (c == 0x55) { 
                    rxBuffer[1] = c;
                    rxIndex = 2;
                    rxState = READ_PAYLOAD;
                } else {
                    rxState = WAIT_HEADER_1; 
                }
                break;
                
            case READ_PAYLOAD:
                rxBuffer[rxIndex++] = c;
                
                // 收滿 32 Bytes
                if (rxIndex >= sizeof(MotionPacket)) {
                    
                    // CRC8 校驗 (計算前 31 Bytes，與最後 1 Byte 比對)
                    uint8_t calculatedCRC = calculateCRC8(rxBuffer, sizeof(MotionPacket) - 1);
                    
                    if (calculatedCRC == rxBuffer[sizeof(MotionPacket) - 1]) {
                        memcpy(&currentPacket, rxBuffer, sizeof(MotionPacket));
                        executeBinaryCommand(); 
                    } else {
                        Serial.println("ERR: CRC8 failed. Packet corrupted.");
                    }
                    
                    // 正常收完一包，重置狀態機
                    rxState = WAIT_HEADER_1;
                    rxIndex = 0;
                }
                break;
        }
    }
}