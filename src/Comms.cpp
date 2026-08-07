#include "Comms.h"
#include "Globals.h"
#include "EndEffector.h"

#include "TMC_RawSPI.h" 
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
// 環狀暫存佇列
// ==========================================
const int PENDING_BUF_SIZE = 50;
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
                        jogAxis(i, (HOMING_CFG[i].homingSpeed > 0) ? 1 : -1, (float)homingSpdSec, getAxisAccel(i, 0.5f), true); 
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
    // Mode 4：智慧停止 (E-STOP or Program Abort)
    // ------------------------------------------
    if (moveMode == 4) {
        if (is_paused || (pendingCount == 0 && !normalMoveActive)) {
            // 軟取消 (Abort)：不觸發硬體鎖死，保留歸零狀態
            pendingHead = 0;
            pendingTail = 0;
            pendingCount = 0;
            is_paused = false;      
            normalMoveActive = false;
            
            Serial.println("ABORTED"); 
        } else {
            // 硬急停 (Hard E-Stop)
            for(int i = 0; i < 6; i++) homingState[i] = 0;  
            emergencyStopEngine();                          
            
            pendingHead = 0;
            pendingTail = 0;
            pendingCount = 0; 
            normalMoveActive = false;
            is_estop_latched = true; 
            
            Serial.println("LATCHED"); // 對應 Python 攔截關鍵字
        }
        return;
    }

    // Mode 7: 暫停 (Pause) 
    if (moveMode == 7) {
        is_paused = true;
        return;
    }
        
    // Mode 8: 繼續 (Resume)
    if (moveMode == 8) {
        is_paused = false;
        return;
    }

    // ------------------------------------------
    // Mode 9：解除急停復歸 (E-STOP RESET)
    // ------------------------------------------
    if (moveMode == 9) {
        if (is_estop_latched) {
            is_estop_latched = false;
            
            // 強制將歸零狀態機「重置為閒置 (0)」
            // 這樣 isAnyHoming() 才會回傳 false，正式解開全系統的運動封印！
            for(int i = 0; i < 6; i++) {
                homingState[i] = 0; 
            }

            Serial.println("RESET SUCCESS"); // 對應 Python 攔截關鍵字
        }
        return;
    }

    // Mode 10: UI 主動請求溫度狀態
    if (moveMode == 10) {
        reportSystemTemperatures();
        return;
    }

    // 核心防禦：如果處於急停鎖存中，拒絕所有運動指令
    if (is_estop_latched) {
        Serial.println("ERR: LATCHED");
        return;
    }

    // ------------------------------------------
    // Mode 5：夾爪 (Gripper)
    // ------------------------------------------
    if (moveMode == 5) {
        // 將 UI 傳來的 0~100 數值 (targets[0]) 送給夾爪 API
        setGripperTarget(targets[0]);
        
        // 絕對要刪除這裡的 Serial.println("<EE_DONE>");
        // 讓 updateEndEffector() 去決定什麼時候印出完成暗號
        return;
    }
    
    // ------------------------------------------
    // Mode 6：UI 主動請求真實物理座標 (步數)
    // ------------------------------------------
    if (moveMode == 6) {
        Serial.print("[POS] ");
        for (int i = 0; i < 6; i++) {
            // 呼叫 MotionEngine 提供的 API 取得各軸當前的真實步數
            Serial.print(getAxisPosition(i)); 
            if (i < 5) Serial.print(",");
        }
        Serial.println();
        return; 
    }
        
    // ------------------------------------------
    // 一般運動邏輯 (Mode 0, Mode 1, Mode 2)
    // ------------------------------------------
    if (!isAnyHoming()) {
        
        // Mode 1: PVT 串流
        if (moveMode == 1) {
            if (pendingCount >= PENDING_BUF_SIZE) {
                Serial.println("ERR: OVERFLOW");
                emergencyStopEngine();
                is_estop_latched = true;
                pendingHead = 0; pendingTail = 0; pendingCount = 0;
                return;
            }
            
            for(int i = 0; i < 6; i++) {
                long target_val = (targets[i] != 999999) ? targets[i] : getAxisPosition(i);
                
                if (JOINT_PINS[i].limitPin != 0) {
                    if (target_val > AXIS_MAX_LIMIT[i] || target_val < AXIS_MIN_LIMIT[i]) {
                        Serial.println("ERR: BOUNDS");
                        emergencyStopEngine();
                        is_estop_latched = true;
                        pendingHead = 0; pendingTail = 0; pendingCount = 0;
                        return;
                    }
                }
                pendingBuf[pendingHead].t[i] = target_val; 
            }
            
            pendingBuf[pendingHead].interval = 10000; 
            pendingHead = (pendingHead + 1) % PENDING_BUF_SIZE;
            pendingCount++;
            return; 
        }
        
        // ------------------------------------------
        // Mode 0：獨立絕對座標追蹤 (UI 滑桿專用)
        // ------------------------------------------
        else if (moveMode == 0) {
            float speedFactor = (param7 <= 0.0) ? 1.0 : param7;
            
            // 直接交給底層獨立更新，不干涉未變動的軸
            updateAbsoluteTargets(targets, speedFactor);
            
            normalMoveActive = true;
            Serial.println("OK"); 
        }
        
        // Mode 2: 連續寸動 (Joint Jogging)
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
    // 1. 推送排隊中的 Mode 1 點位
    while (pendingCount > 0) {
        if (is_paused) {
            if (pause_multiplier < 5.0f) {
                pause_multiplier += 0.5f; 
            } else {
                break; 
            }
        } else {
            if (pause_multiplier > 1.0f) {
                pause_multiplier -= 0.5f;
            } else {
                pause_multiplier = 1.0f;
            }
        }

        uint32_t current_interval = (uint32_t)(pendingBuf[pendingTail].interval * pause_multiplier);

        if (pushMotionPoint(pendingBuf[pendingTail].t[0], pendingBuf[pendingTail].t[1], 
                            pendingBuf[pendingTail].t[2], pendingBuf[pendingTail].t[3], 
                            pendingBuf[pendingTail].t[4], pendingBuf[pendingTail].t[5], 
                            current_interval)) {
            
            pendingTail = (pendingTail + 1) % PENDING_BUF_SIZE;
            pendingCount--;
            normalMoveActive = true;
            Serial.println("OK"); 
        } else {
            break; 
        }
    }

    // 2. 逾時保護機制
    static uint32_t lastRxTime = 0;
    if (rxState != WAIT_HEADER_1 && (millis() - lastRxTime > 50)) {
        rxState = WAIT_HEADER_1;
        rxIndex = 0;
    }

    // 3. 正常讀取 UART
    while (Serial.available() > 0) {
        uint8_t c = Serial.read();
        lastRxTime = millis(); 

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
                
                if (rxIndex >= sizeof(MotionPacket)) {
                    uint8_t calculatedCRC = calculateCRC8(rxBuffer, sizeof(MotionPacket) - 1);
                    
                    if (calculatedCRC == rxBuffer[sizeof(MotionPacket) - 1]) {
                        memcpy(&currentPacket, rxBuffer, sizeof(MotionPacket));
                        executeBinaryCommand(); 
                    } else {
                        Serial.println("ERR: CRC");
                    }
                    
                    rxState = WAIT_HEADER_1;
                    rxIndex = 0;
                }
                break;
        }
    }
}