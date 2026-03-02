/*
 * PAROL6 Controller - Fysetc F6 (MobaTools Version)
 * 特色：平方比例 Ramp 修正、Debounce 濾波、0距離卡死防護
 */

#include <Arduino.h>
#include <MobaTools.h> 

struct JointConfig {
    byte stepPin; byte dirPin; byte enPin; byte limitPin; bool limitActiveState;
    float homingSpeed; float homingPos; long maxSpeedSteps10; int rampSteps;        
};

const JointConfig JOINTS[6] = {
    {54, 55, 38, 0,  LOW,  0,     0, 15000,  300},  // J1
    {60, 61, 56, 63, HIGH, -700,  50, 20000, 400},  // J2
    {43, 48, 58, 15, HIGH,  700, -70, 20000, 400},  // J3
    {26, 28, 24, 0,  LOW,  0,     0, 15000,  200},  // J4
    {36, 34, 30, 0,  HIGH, 0,     0, 15000,  200},  // J5
    {59, 57, 40, 0,  LOW,  0,     0, 20000,  400}   // J6
};

const byte LED_PIN = 13;
const float MICROSTEPS = 8.0;
const float MOTOR_STEPS = 200.0; 
const float GEAR_RATIOS[6] = {6.4, 20.0, 18.1, 4.0, 4.0, 10.0};

MoToStepper M1(200, STEPDIR);
MoToStepper M2(200, STEPDIR);
MoToStepper M3(200, STEPDIR);
MoToStepper M4(200, STEPDIR);
MoToStepper M5(200, STEPDIR);
MoToStepper M6(200, STEPDIR);

MoToStepper* steppers[6] = {&M1, &M2, &M3, &M4, &M5, &M6};

// 歸零狀態機
byte homingState[6] = {0, 0, 0, 0, 0, 0}; 

// 追蹤「一般移動」是否進行中
bool normalMoveActive = false;

const byte numChars = 128;
char receivedChars[numChars];
char tempChars[numChars];
float receivedAngles[6] = {0.0};
boolean newData = false;

float getStepsPerDeg(int axis) {
    return (MOTOR_STEPS * MICROSTEPS * GEAR_RATIOS[axis]) / 360.0;
}

bool isAnyHoming() {
    for(int i = 0; i < 6; i++) {
        if(homingState[i] != 0) return true;
    }
    return false;
}

// 歸零邏輯狀態機
void updateHomingLogic() {
    for (int i = 0; i < 6; i++) {
        
        // 狀態 1：正在尋找開關
        if (homingState[i] == 1 && JOINTS[i].limitPin != 0) {
            // 第一次讀取到觸發電位
            if (digitalRead(JOINTS[i].limitPin) == JOINTS[i].limitActiveState) {
                // 軟體防雜訊濾波 (Debounce)
                delay(3); 
                // 第二次確認
                if (digitalRead(JOINTS[i].limitPin) == JOINTS[i].limitActiveState) {
                    steppers[i]->setRampLen(0);
                    steppers[i]->doSteps(0); 
                    homingState[i] = 2; 
                }
            } 
        }
        // 狀態 2：急停完畢，等待同步退回 Offset
        else if (homingState[i] == 2) {
            if (!steppers[i]->moving()) {
                bool readyToOffset = true;
                
                if (i == 1) { 
                    if (homingState[2] == 1 || (homingState[2] == 2 && steppers[2]->moving())) readyToOffset = false;
                } 
                else if (i == 2) { 
                    if (homingState[1] == 1 || (homingState[1] == 2 && steppers[1]->moving())) readyToOffset = false;
                }

                if (readyToOffset) {
                    steppers[i]->setRampLen(JOINTS[i].rampSteps);
                    steppers[i]->setSpeedSteps(JOINTS[i].maxSpeedSteps10);
                    long offsetSteps = JOINTS[i].homingPos * getStepsPerDeg(i);
                    steppers[i]->doSteps(offsetSteps); 
                    homingState[i] = 3; 
                }
            }
        }
        // 狀態 3：Offset 退回完畢
        else if (homingState[i] == 3) {
            if (!steppers[i]->moving()) {
                steppers[i]->setZero(0); 
                steppers[i]->writeSteps(0); 
                homingState[i] = 0;
                Serial.print(">>> Axis "); Serial.print(i + 1);
                Serial.println(" Homing Done (At True Zero) <<<");
            }
        }
    }

    // 所有軸歸零完畢時，送出專屬完成訊號
    static bool wasHoming = false;
    bool stillHoming = isAnyHoming();
    if (wasHoming && !stillHoming) {
        Serial.println("HomingDone");  
    }
    wasHoming = stillHoming;
}

void setup() {
    Serial.begin(250000);
    pinMode(LED_PIN, OUTPUT);

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
    Serial.println("<F6 Unified (MobaTools Auto-Sync) Ready>");
}

void processCommand() {
    // 1. E-STOP 檢查
    if (strncmp(tempChars, "STOP", 4) == 0) {
        for(int i = 0; i < 6; i++) {
            homingState[i] = 0;  
            steppers[i]->doSteps(0);
        }
        normalMoveActive = false;
        Serial.println("!!! E-STOP TRIGGERED !!!");
        return;
    }

    // 🌟 [新增] 嚴格封包驗證機制
    float tempParsed[6] = {0.0};
    float speedFactor = 1.0; 
    int parseCount = 0; // 記錄成功解析了幾個數字

    char * strtokIndx = strtok(tempChars, ",");
    if(strtokIndx != NULL) {
        tempParsed[0] = atof(strtokIndx);
        parseCount++;
    }
    
    for(int i = 1; i < 6; i++) {
        strtokIndx = strtok(NULL, ",");
        if(strtokIndx != NULL) {
            tempParsed[i] = atof(strtokIndx);
            parseCount++;
        }
    }

    // 嘗試解析第 7 個參數 (速度比例)
    strtokIndx = strtok(NULL, ",");
    if(strtokIndx != NULL) {
        float parsedVal = atof(strtokIndx);
        if (parsedVal > 0.0 && parsedVal <= 1.0) speedFactor = parsedVal;
        parseCount++; // 這個參數可有可無，所以 parseCount 可能是 6 或 7
    }

    // ⛔ 核心防護：如果解析不到 6 個關節數值，代表封包破裂！
    if (parseCount < 6) {
        Serial.print("[HW Error] Dropped corrupted packet: ");
        Serial.println(tempChars);
        return; // 直接中斷，保護硬體不暴衝到 0.0！
    }

    // ✅ 確認封包完整安全後，才正式賦值給全域變數
    for(int i = 0; i < 6; i++) {
        receivedAngles[i] = tempParsed[i];
    }

    bool homingTriggered = false;
    
    // 歸零觸發
    for (int i = 0; i < 6; i++) {
        if (receivedAngles[i] == 999.0) {
            if (JOINTS[i].limitPin != 0) {
                if (homingState[i] == 0) {
                    homingState[i] = 1; 
                    steppers[i]->setRampLen(0); 
                    long homingSpd = abs(JOINTS[i].homingSpeed) * 10;
                    steppers[i]->setSpeedSteps(homingSpd);
                    int dir = (JOINTS[i].homingSpeed > 0) ? 1 : -1;
                    steppers[i]->rotate(dir); 
                    Serial.print(">>> Homing Start: J"); Serial.println(i + 1);
                }
                homingTriggered = true; 
            } 
        }
    }

    // 一般移動 (自動尋找瓶頸並同步到達)
    if (!isAnyHoming() && !homingTriggered) {
        
        long deltaSteps[6] = {0};
        float timeNeeded[6] = {0.0};
        float maxTime = 0.0;

        for (int i = 0; i < 6; i++) {
            if (receivedAngles[i] != 999.0) {
                long targetSteps = receivedAngles[i] * getStepsPerDeg(i);
                deltaSteps[i] = abs(targetSteps - steppers[i]->currentPosition());
                float currentMaxSpeedSec = (JOINTS[i].maxSpeedSteps10 * speedFactor) / 10.0;
                if (currentMaxSpeedSec > 0 && deltaSteps[i] > 0) {
                    timeNeeded[i] = deltaSteps[i] / currentMaxSpeedSec;
                    if (timeNeeded[i] > maxTime) maxTime = timeNeeded[i]; 
                }
            }
        }

        for (int i = 0; i < 6; i++) {
            if (receivedAngles[i] != 999.0) {
                long targetSteps = receivedAngles[i] * getStepsPerDeg(i);
                if (maxTime > 0.0 && deltaSteps[i] > 0) {
                    float syncStepsPerSec = deltaSteps[i] / maxTime;
                    long mobaSpeed = (long)(syncStepsPerSec * 10.0);
                    if (mobaSpeed < 1) mobaSpeed = 1; 
                    steppers[i]->setSpeedSteps(mobaSpeed);
                    
                    float speedRatio = (float)mobaSpeed / (float)JOINTS[i].maxSpeedSteps10;
                    // 🌟 採用您完美的 v^2 運動學公式
                    int dynamicRamp = (int)(JOINTS[i].rampSteps * speedRatio * speedRatio);
                    if (dynamicRamp < 5) dynamicRamp = 0;
                    steppers[i]->setRampLen(dynamicRamp);
                } else {
                    steppers[i]->setSpeedSteps(JOINTS[i].maxSpeedSteps10);
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

    for(int i = 0; i < 6; i++) digitalWrite(JOINTS[i].enPin, LOW);

    bool isMoving = false; 
    for (int i = 0; i < 6; i++) {
        if (steppers[i]->moving()) {
            isMoving = true;
            break; 
        }
    }

    if (normalMoveActive) {
        // 🌟 改良：不論剛走完，還是 0 距離沒動，只要 isMoving 是 false 就回報 Done
        if (!isMoving) {
            Serial.println("Done");      
            normalMoveActive = false;    
        }
    }

    // LED 狀態指示
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