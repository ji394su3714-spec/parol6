/*
 * PAROL6 Controller - Fysetc F6 V1.4
 * 特色：J1~J3 同步歸零 ➡️ J4,J6,J5 序列避障歸零 ➡️ LIN/PTP/JOG 智慧切換
 */

#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TMCStepper.h>
#include <MobaTools.h>

// ==========================================
// 1. 關節配置與硬體參數
// ==========================================
struct JointConfig {
    byte stepPin; byte dirPin; byte enPin; byte limitPin; bool limitActiveState;
    float homingSpeed; float homingPos; long maxSpeedSteps10; int rampSteps;
    uint16_t runCurrent_mA; float holdCurrentRatio;        
};

#define R_SENSE_2209 0.11f  
#define R_SENSE_5160 0.075f

// 綜合配置表 (請務必確認 J4, J5, J6 的 homingPos 與你的實際機構相符)
const JointConfig JOINTS[6] = {
    {54, 55, 38,  2,  LOW,   250,  -30, 15000, 500, 1000, 0.25f}, // J1 (2209)
    {60, 61, 56, 12,  HIGH, -600,   50, 18000, 500,  950,  0.5f}, // J2 (5160)
    {43, 48, 58, 14,  HIGH,  750,  -70, 17000, 500,  850,  0.5f}, // J3 (2209)
    {26, 28, 24, 15,  LOW,   800, -145, 14000, 400,  850, 0.25f}, // J4 (2209)
    {36, 34, 30, 63,  HIGH, 800, -125, 14000, 400,  850, 0.25f}, // J5 (2209)
    {59, 57, 40, 64,  LOW,  1200,    0, 20000, 300,  680, 0.25f}  // J6 (A4988 替代，電流依靠硬體旋鈕)
};

// ==========================================
// 📡 2. 通訊腳位與驅動物件
// ==========================================
#define Y_CS_PIN 39

SoftwareSerial serial_J1(71, 72);
SoftwareSerial serial_J3(78, 79);
SoftwareSerial serial_J4(76, 77);
SoftwareSerial serial_J5(80, 81);
// [A4988 替換] J6 不使用 UART，故註解
// SoftwareSerial serial_J6(82, 22);

TMC2209Stepper driver_J1(&serial_J1, R_SENSE_2209, 0);
TMC5160Stepper driver_J2(Y_CS_PIN, R_SENSE_5160);       
TMC2209Stepper driver_J3(&serial_J3, R_SENSE_2209, 0);
TMC2209Stepper driver_J4(&serial_J4, R_SENSE_2209, 0);
TMC2209Stepper driver_J5(&serial_J5, R_SENSE_2209, 0);
// [A4988 替換] TMC2209Stepper driver_J6(&serial_J6, R_SENSE_2209, 0);

// ==========================================
// 3. MobaTools 與全域變數
// ==========================================
const byte LED_PIN = 13;
const float MICROSTEPS = 8.0;
const float MOTOR_STEPS = 200.0; 
const float GEAR_RATIOS[6] = {6.4, 20.0, 18.1, 4.0, 4.0, 10.0};

MoToStepper stepper_J1(1600, STEPDIR);
MoToStepper stepper_J2(1600, STEPDIR);
MoToStepper stepper_J3(1600, STEPDIR);
MoToStepper stepper_J4(1600, STEPDIR);
MoToStepper stepper_J5(1600, STEPDIR);
MoToStepper stepper_J6(1600, STEPDIR); // A4988 同樣接受 STEP/DIR 控制

MoToStepper* steppers[6] = {&stepper_J1, &stepper_J2, &stepper_J3, &stepper_J4, &stepper_J5, &stepper_J6};

byte homingState[6] = {0, 0, 0, 0, 0, 0}; 
bool normalMoveActive = false;

const byte numChars = 128;
char receivedChars[numChars];
char tempChars[numChars];
float receivedAngles[6] = {0.0};
boolean newData = false;

// --- 輔助函式 ---
float getStepsPerDeg(int axis) {
    return (MOTOR_STEPS * MICROSTEPS * GEAR_RATIOS[axis]) / 360.0;
}

bool isAnyHoming() {
    for(int i = 0; i < 6; i++) {
        if(homingState[i] != 0) return true;
    }
    return false;
}

// ==========================================
// 🎯 核心：歸零狀態機 (完美融合版)
// ==========================================
void updateHomingLogic() {
    // ⚠️ 這裡定義 J6 碰到開關後，要先退回多少度的「預備姿態」
    const float J6_PREP_ANGLE = 90.0; 
    
    for (int i = 0; i < 6; i++) {
        
        // 狀態 1：正在尋找開關
        if (homingState[i] == 1 && JOINTS[i].limitPin != 0) {
            if (digitalRead(JOINTS[i].limitPin) == JOINTS[i].limitActiveState) {
                delay(3); // Debounce
                if (digitalRead(JOINTS[i].limitPin) == JOINTS[i].limitActiveState) {
                    steppers[i]->setRampLen(0);
                    steppers[i]->stop(); 
                    homingState[i] = 2; 
                    Serial.print(">>> J"); Serial.print(i + 1); Serial.println(" Hit! Waiting...");
                }
            } 
        }
        
        // 狀態 2：急停完畢，互相等待與策略分配
        else if (homingState[i] == 2) {
            if (!steppers[i]->moving()) {
                bool readyToOffset = true;
                
                // Group 1 (J1, J2, J3) 的互相等待邏輯
                if (i <= 2) {
                    for (int j = 0; j < 3; j++) {
                        if (homingState[j] == 1 || (homingState[j] == 2 && steppers[j]->moving())) {
                            readyToOffset = false;
                            break;
                        }
                    }
                }

                // 🌟 J6 獨立策略：退到 90 度預備姿態
                if (i == 5) {
                    readyToOffset = false; 
                    long prepSteps = J6_PREP_ANGLE * getStepsPerDeg(i);
                    steppers[i]->setRampLen(JOINTS[i].rampSteps);
                    steppers[i]->setSpeedSteps(JOINTS[i].maxSpeedSteps10);
                    steppers[i]->doSteps(prepSteps); 
                    homingState[i] = 4; // 前往狀態 4
                }
                
                // 🌟 J5 獨立策略：原地等 J6
                if (i == 4) {
                    readyToOffset = false;
                    homingState[i] = 6; // 前往狀態 6 監測 J6
                }

                // J1~J4 放行退回 Offset
                if (readyToOffset) {
                    // 【修正】：使用極短的歸零專屬 Ramp (20步) 與中等退回速度 (搜尋速度的2倍)
                    steppers[i]->setRampLen(100); 
                    steppers[i]->setSpeedSteps(abs(JOINTS[i].homingSpeed) * 20); 
                    long offsetSteps = JOINTS[i].homingPos * getStepsPerDeg(i);
                    steppers[i]->doSteps(offsetSteps); 
                    homingState[i] = 3; 
                }
            }
        }
        
        // 狀態 3：Offset 退回完畢 (在此處真正設為 0)
        else if (homingState[i] == 3) {
            if (!steppers[i]->moving()) {
                steppers[i]->setZero(0); 
                steppers[i]->writeSteps(0); 
                homingState[i] = 0;
                Serial.print(">>> Axis "); Serial.print(i + 1); Serial.println(" Homing Done (At True Zero) <<<");

                // 🌟 【大群組連鎖】：Group 1 叫醒 Group 2
                if (i <= 2) {
                    if (homingState[0] == 0 && homingState[1] == 0 && homingState[2] == 0) {
                        bool j4_waiting = (homingState[3] == 20);
                        bool j6_waiting = (homingState[5] == 20);
                        
                        for (int k = 3; k < 6; k++) {
                            if (homingState[k] == 20) {
                                if (k == 4 && j6_waiting) {
                                    homingState[k] = 10; // J5 拿號碼牌等 J6
                                } else if (k == 5 && j4_waiting) {
                                    homingState[k] = 10; // J6 拿號碼牌等 J4
                                } else {
                                    // 喚醒出發 (通常是 J4)
                                    homingState[k] = 1;  
                                    steppers[k]->setRampLen(0);
                                    steppers[k]->setSpeedSteps(abs(JOINTS[k].homingSpeed) * 10);
                                    steppers[k]->rotate((JOINTS[k].homingSpeed > 0) ? 1 : -1);
                                    Serial.print(">>> Homing Start: J"); Serial.println(k + 1);
                                }
                            }
                        }
                    }
                }

                // 🌟 【腕部內部連鎖】：J4 叫醒 J6
                if (i == 3 && homingState[5] == 10) {
                    homingState[5] = 1;
                    steppers[5]->setRampLen(0);
                    steppers[5]->setSpeedSteps(abs(JOINTS[5].homingSpeed) * 10);
                    steppers[5]->rotate((JOINTS[5].homingSpeed > 0) ? 1 : -1);
                    Serial.println(">>> Homing Start: J6 (Triggered by J4)");
                }
            }
        }

        // 狀態 4：J6 正在前往 90 度預備姿態
        else if (homingState[i] == 4 && i == 5) {
            if (!steppers[i]->moving()) {
                homingState[i] = 5; // 進入狀態 5 鎖定
                Serial.println(">>> J6 Reached 90 Deg Prep Position");

                // 🌟 【腕部內部連鎖】：J6 叫醒 J5
                if (homingState[4] == 10) {
                    homingState[4] = 1;
                    steppers[4]->setRampLen(0);
                    steppers[4]->setSpeedSteps(abs(JOINTS[4].homingSpeed) * 10);
                    steppers[4]->rotate((JOINTS[4].homingSpeed > 0) ? 1 : -1);
                    Serial.println(">>> Homing Start: J5 (Triggered by J6)");
                }
            }
        }

        // 狀態 6：J5 撞到開關了，等待 J6 準備好
        else if (homingState[i] == 6 && i == 4) {
            if (homingState[5] == 5) {
                Serial.println(">>> J5 & J6 Synchronized Final Offset to Zero!");

                // J5 開始退回專屬 Offset
                steppers[4]->setRampLen(100); // 【修正】極短 Ramp
                steppers[4]->setSpeedSteps(abs(JOINTS[4].homingSpeed) * 10); // 【修正】中等速度
                long offsetJ5 = JOINTS[4].homingPos * getStepsPerDeg(4);
                steppers[4]->doSteps(offsetJ5);
                homingState[4] = 3; 

                // J6 開始退回剩餘的 Offset
                steppers[5]->setRampLen(100); // 【修正】極短 Ramp
                steppers[5]->setSpeedSteps(abs(JOINTS[5].homingSpeed) * 10); // 【修正】中等速度
                float remainingAngle = JOINTS[5].homingPos - J6_PREP_ANGLE;
                long offsetJ6 = remainingAngle * getStepsPerDeg(5);
                steppers[5]->doSteps(offsetJ6);
                homingState[5] = 3; 
            }
        }
    }

    static bool wasHoming = false;
    bool stillHoming = isAnyHoming();
    if (wasHoming && !stillHoming) {
        Serial.println("HomingDone");  
    }
    wasHoming = stillHoming;
}

// ==========================================
// 🚀 初始化 Setup
// ==========================================
void setup() {
    Serial.begin(250000); 
    SPI.begin();
    pinMode(LED_PIN, OUTPUT);

    serial_J1.begin(115200);
    serial_J3.begin(115200);
    serial_J4.begin(115200);
    serial_J5.begin(115200);

    // --- TMC2209/5160 底層設定 ---
    auto setupTMC2209 = [](TMC2209Stepper &drv, uint16_t mA, float hold_ratio) {
        drv.begin();
        drv.pdn_disable(true);     
        drv.I_scale_analog(false); 
        drv.toff(5);               
        drv.rms_current(mA, hold_ratio); 
        drv.microsteps(8);        
        drv.en_spreadCycle(false); 
        drv.pwm_autoscale(true);   
        drv.TCOOLTHRS(0); 
    };

    setupTMC2209(driver_J1, JOINTS[0].runCurrent_mA, JOINTS[0].holdCurrentRatio);
    setupTMC2209(driver_J3, JOINTS[2].runCurrent_mA, JOINTS[2].holdCurrentRatio);
    setupTMC2209(driver_J4, JOINTS[3].runCurrent_mA, JOINTS[3].holdCurrentRatio);
    setupTMC2209(driver_J5, JOINTS[4].runCurrent_mA, JOINTS[4].holdCurrentRatio);

    driver_J2.begin();
    driver_J2.toff(5);
    driver_J2.rms_current(JOINTS[1].runCurrent_mA, JOINTS[1].holdCurrentRatio); 
    driver_J2.microsteps(8);
    driver_J2.en_pwm_mode(true);
    driver_J2.pwm_autoscale(true);

    // --- MobaTools 與腳位綁定 ---
    for (int i = 0; i < 6; i++) {
        steppers[i]->attach(JOINTS[i].stepPin, JOINTS[i].dirPin);
        pinMode(JOINTS[i].enPin, OUTPUT);
        digitalWrite(JOINTS[i].enPin, LOW); 
        if (JOINTS[i].limitPin != 0) {
            pinMode(JOINTS[i].limitPin, INPUT_PULLUP);
        }
        steppers[i]->setSpeedSteps(JOINTS[i].maxSpeedSteps10);
        steppers[i]->setRampLen(JOINTS[i].rampSteps); 
    }
    Serial.println("<F6 Unified (Hybrid TMC + Sync Homing) Ready>");
}

// ==========================================
// 📨 指令處理與封包解析
// ==========================================
void processCommand() {
    if (strncmp(tempChars, "STOP", 4) == 0) {
        for(int i = 0; i < 6; i++) {
            homingState[i] = 0;  
            steppers[i]->doSteps(0);
        }
        normalMoveActive = false;
        Serial.println("!!! E-STOP TRIGGERED !!!");
        return;
    }

    float tempParsed[8] = {0.0};
    int parseCount = 0; 

    char * strtokIndx = strtok(tempChars, ",");
    while (strtokIndx != NULL && parseCount < 8) {
        tempParsed[parseCount] = atof(strtokIndx);
        parseCount++;
        strtokIndx = strtok(NULL, ",");
    }

    if (parseCount < 6) {
        Serial.print("[HW Error] Dropped corrupted packet: ");
        Serial.println(tempChars);
        return; 
    }

    for(int i = 0; i < 6; i++) {
        receivedAngles[i] = tempParsed[i];
    }

    float param7 = (parseCount >= 7) ? tempParsed[6] : 1.0;
    int moveMode = (parseCount >= 8) ? (int)tempParsed[7] : 0; 
    
    // ==========================================
    // 3. 歸零觸發 (大群組序列: J1~J3 先，再 J4~J6)
    // ==========================================
    bool group1_req = (receivedAngles[0] == 999.0 || receivedAngles[1] == 999.0 || receivedAngles[2] == 999.0);
    bool j4_req = (receivedAngles[3] == 999.0);
    bool j6_req = (receivedAngles[5] == 999.0);
    bool homingTriggered = false;

    for (int i = 0; i < 6; i++) {
        if (receivedAngles[i] == 999.0 && JOINTS[i].limitPin != 0 && JOINTS[i].homingSpeed != 0) {
            if (homingState[i] == 0) {
                
                if (i >= 3 && group1_req) {
                    homingState[i] = 20; // 進入狀態 20：大群組排隊
                    homingTriggered = true;
                } 
                else if (i == 4 && j6_req) {
                    homingState[i] = 10; // J5 等 J6
                    homingTriggered = true;
                } else if (i == 5 && j4_req) {
                    homingState[i] = 10; // J6 等 J4
                    homingTriggered = true;
                } 
                else {
                    homingState[i] = 1; 
                    steppers[i]->setRampLen(0); 
                    long homingSpd = abs(JOINTS[i].homingSpeed) * 10;
                    steppers[i]->setSpeedSteps(homingSpd);
                    int dir = (JOINTS[i].homingSpeed > 0) ? 1 : -1;
                    steppers[i]->rotate(dir); 
                    Serial.print(">>> Homing Start: J"); Serial.println(i + 1);
                    homingTriggered = true;
                }
            } 
        }
    }
    if (homingTriggered) Serial.println("OK");

    // ==========================================
    // 4. 一般移動分派 (Jog / PTP / LIN)
    // ==========================================
    if (!isAnyHoming() && !homingTriggered) {
        long deltaSteps[6] = {0};
        float timeNeeded[6] = {0.0};
        float maxTime = 0.0;

        for (int i = 0; i < 6; i++) {
            if (receivedAngles[i] != 999.0) {
                long targetSteps = receivedAngles[i] * getStepsPerDeg(i);
                deltaSteps[i] = abs(targetSteps - steppers[i]->currentPosition());
                
                if (moveMode == 1) { 
                    float currentMaxSpeedSec = (JOINTS[i].maxSpeedSteps10 * param7) / 10.0;
                    if (currentMaxSpeedSec > 0 && deltaSteps[i] > 0) {
                        timeNeeded[i] = deltaSteps[i] / currentMaxSpeedSec;
                        if (timeNeeded[i] > maxTime) maxTime = timeNeeded[i]; 
                    }
                }
            }
        }

        for (int i = 0; i < 6; i++) {
            if (receivedAngles[i] != 999.0) {
                long targetSteps = receivedAngles[i] * getStepsPerDeg(i);
                
                if (moveMode == 1 && maxTime > 0.0 && deltaSteps[i] > 0) {
                    // PTP
                    float syncStepsPerSec = deltaSteps[i] / maxTime;
                    long mobaSpeed = (long)(syncStepsPerSec * 10.0);
                    if (mobaSpeed < 10) mobaSpeed = 10; 
                    steppers[i]->setSpeedSteps(mobaSpeed);
                    
                    int dynamicRamp = deltaSteps[i] * 0.3; 
                    if (dynamicRamp > JOINTS[i].rampSteps) dynamicRamp = JOINTS[i].rampSteps;
                    if (dynamicRamp < 5) {
                        dynamicRamp = deltaSteps[i] / 2; 
                        if (dynamicRamp == 0) dynamicRamp = 1; 
                    }
                    steppers[i]->setRampLen(dynamicRamp);
                    
                } else if (moveMode == 2 && deltaSteps[i] > 0) {
                    // LIN
                    float interval_sec = param7;
                    if (interval_sec < 0.01) interval_sec = 0.01; 
                    
                    float syncStepsPerSec = deltaSteps[i] / interval_sec;
                    long mobaSpeed = (long)(syncStepsPerSec * 10.0);
                    if (mobaSpeed < 1) mobaSpeed = 1; 
                    
                    steppers[i]->setSpeedSteps(mobaSpeed);
                    steppers[i]->setRampLen(5); // 微緩衝
                    
                } else if (moveMode == 0) {
                    // JOG
                    long defaultSpeed = JOINTS[i].maxSpeedSteps10 * param7;
                    if (defaultSpeed < 10) defaultSpeed = 10;
                    
                    steppers[i]->setSpeedSteps(defaultSpeed);
                    steppers[i]->setRampLen(JOINTS[i].rampSteps); 
                }
                
                steppers[i]->writeSteps(targetSteps); 
            }
        }
        normalMoveActive = true;
        Serial.println("OK"); 
    }
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) ndx = numChars - 1;
            } else {
                receivedChars[ndx] = '\0';
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        } else if (rc == startMarker) recvInProgress = true;
    }
}

void loop() {
    recvWithStartEndMarkers();
    if (newData) {
        strcpy(tempChars, receivedChars);
        processCommand();
        newData = false;
    }

    updateHomingLogic();

    bool isMoving = false; 
    for (int i = 0; i < 6; i++) {
        if (steppers[i]->moving()) {
            isMoving = true;
            break; 
        }
    }

    if (normalMoveActive) {
        if (!isMoving) {
            Serial.println("Done");      
            normalMoveActive = false;    
        }
    }

    static unsigned long lastLedToggle = 0;
    const unsigned long LED_BLINK_INTERVAL = 500UL;

    if (isMoving) {
        if (millis() - lastLedToggle >= LED_BLINK_INTERVAL) {
            lastLedToggle = millis();
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        }
    } else {
        digitalWrite(LED_PIN, HIGH);  
    }
}