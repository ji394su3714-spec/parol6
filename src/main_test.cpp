/*
 * PAROL6 Controller - Fysetc F6 (MobaTools Ultimate Version)
 * 特色：
 * 1. 狀態機絕對防禦護盾：徹底解決 Python 30Hz 座標覆蓋導致的失控暴衝
 * 2. 專業級二次精準偵測：快速撞擊 -> 退後 -> 慢速撞擊 -> 瞬間煞停
 * 3. 250000 鮑率 / 強制 Enable 防掉落
 */

#include <Arduino.h>
#include <MobaTools.h> 

struct JointConfig {
    byte stepPin; byte dirPin; byte enPin; byte limitPin; bool limitActiveState;
    float homingSpeed;    
    float homingPos;      
    long maxSpeedSteps10; 
    int rampSteps;        
};

const JointConfig JOINTS[6] = {
    {54, 55, 38, 0,  LOW,  0,      0,   3200,  85},  // J1
    {60, 61, 56, 63, HIGH, -300, -50,  10000, 833}, // J2 (NC 接法)
    {43, 48, 58, 15, HIGH,  300,  70,   9050, 682}, // J3 
    {26, 28, 24, 0,  LOW,  0,      0,   3000,  75},  // J4
    {36, 34, 30, 0,  HIGH, 0,      0,   3000,  75},  // J5
    {59, 57, 40, 0,  LOW,  0,      0,   5000, 208}   // J6
};

const byte LED_PIN = 13;
const float MICROSTEPS = 8.0;
const float MOTOR_STEPS = 200.0; 
const float GEAR_RATIOS[6] = {6.4, 20.0, 18.1, 4.0, 4.0, 10.0};

MoToStepper motor1(200, STEPDIR);
MoToStepper motor2(200, STEPDIR);
MoToStepper motor3(200, STEPDIR);
MoToStepper motor4(200, STEPDIR);
MoToStepper motor5(200, STEPDIR);
MoToStepper motor6(200, STEPDIR);

MoToStepper* steppers[6] = {&motor1, &motor2, &motor3, &motor4, &motor5, &motor6};

// 【二次偵測狀態機】
// 0: 正常/閒置, 1: 快速尋找, 2: 等待煞車並回彈, 3: 等待回彈完成並慢速尋找, 4: 慢速觸發等待停穩
byte homingState[6] = {0, 0, 0, 0, 0, 0}; 

const byte numChars = 128;
char receivedChars[numChars];
char tempChars[numChars];
float receivedAngles[6] = {0.0};
boolean newData = false;

float getStepsPerDeg(int axis) {
    return (MOTOR_STEPS * MICROSTEPS * GEAR_RATIOS[axis]) / 360.0;
}

// ==========================================
// 歸零核心邏輯 (二次精準偵測)
// ==========================================
void updateHomingLogic() {
    for (int i = 0; i < 6; i++) {
        if (homingState[i] == 0) continue; // 沒在歸零的軸直接跳過，省 CPU

        bool isHit = (JOINTS[i].limitPin != 0) && 
                     (digitalRead(JOINTS[i].limitPin) == JOINTS[i].limitActiveState);

        switch (homingState[i]) {
            case 1: // 階段 1：快速尋找開關
                if (isHit) {
                    steppers[i]->setRampLen(0);
                    steppers[i]->stop(); // 瞬間煞車
                    homingState[i] = 2;
                }
                break;

            case 2: // 階段 2：停穩後，反向回彈 600 步
                if (!steppers[i]->moving()) {
                    long fastSpd = abs(JOINTS[i].homingSpeed) * 10;
                    steppers[i]->setSpeedSteps(fastSpd / 2); // 用一半速度退
                    int dir = (JOINTS[i].homingSpeed > 0) ? 1 : -1;
                    steppers[i]->doSteps(-dir * 600); // 反方向退 600 步
                    homingState[i] = 3;
                }
                break;

            case 3: // 階段 3：回彈完畢，用極慢速(1/4)再次尋找開關
                if (!steppers[i]->moving()) {
                    long fastSpd = abs(JOINTS[i].homingSpeed) * 10;
                    steppers[i]->setSpeedSteps(fastSpd / 4); // 慢速
                    int dir = (JOINTS[i].homingSpeed > 0) ? 1 : -1;
                    steppers[i]->rotate(dir); // 再次啟動
                    homingState[i] = 4;
                }
                break;

            case 4: // 階段 4：二次撞擊開關
                if (isHit) {
                    steppers[i]->stop(); // Ramp 已是 0，保證瞬間鎖死
                    homingState[i] = 5;
                }
                break;

            case 5: // 階段 5：徹底停穩，設定實體座標 (-50.0 等)
                if (!steppers[i]->moving()) {
                    long pos = JOINTS[i].homingPos * getStepsPerDeg(i);
                    steppers[i]->setZero(pos);
                    steppers[i]->writeSteps(pos); // 強制鎖定當前位置
                    
                    // 恢復正常移動參數
                    steppers[i]->setRampLen(JOINTS[i].rampSteps);
                    steppers[i]->setSpeedSteps(JOINTS[i].maxSpeedSteps10);

                    homingState[i] = 0; // 解除歸零防護罩
                    Serial.print(">>> Axis "); Serial.print(i+1); Serial.println(" Homing Done <<<");
                }
                break;
        }
    }
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
    Serial.println("<F6 Unified (MobaTools Ultimate Homing) Ready>");
}

void processCommand() {
    if (strncmp(tempChars, "STOP", 4) == 0) {
        for(int i=0; i<6; i++) {
            homingState[i] = 0; 
            steppers[i]->setRampLen(0);
            steppers[i]->stop();     
        }
        Serial.println("!!! E-STOP !!!");
        return;
    }

    char * strtokIndx = strtok(tempChars, ",");
    if(strtokIndx != NULL) receivedAngles[0] = atof(strtokIndx);
    for(int i=1; i<6; i++) {
        strtokIndx = strtok(NULL, ",");
        if(strtokIndx != NULL) receivedAngles[i] = atof(strtokIndx);
    }

    // ==========================================
    // 通訊過濾器：決定馬達該聽誰的
    // ==========================================
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    for (int i = 0; i < 6; i++) {
        // 觸發歸零 (只在閒置狀態 0 時允許啟動)
        if (receivedAngles[i] == 999.0 && homingState[i] == 0) {
            if (JOINTS[i].limitPin != 0) {
                homingState[i] = 1; // 啟動防護罩
                steppers[i]->setRampLen(0); 
                long homingSpd = abs(JOINTS[i].homingSpeed) * 10;
                steppers[i]->setSpeedSteps(homingSpd);
                int dir = (JOINTS[i].homingSpeed > 0) ? 1 : -1;
                steppers[i]->rotate(dir); 
                Serial.print(">>> Homing Start: J"); Serial.println(i+1);
            }
        }
        
        // 【核心防護罩】：只有該軸不在歸零狀態 (==0)，且不是 999.0 時，才允許更新一般座標。
        // 這完美擋掉了 Python 傳來的提早更新的 -50.0！
        else if (homingState[i] == 0 && receivedAngles[i] != 999.0) {
            steppers[i]->setRampLen(JOINTS[i].rampSteps);
            long targetSteps = receivedAngles[i] * getStepsPerDeg(i);
            steppers[i]->writeSteps(targetSteps); 
        }
    }
    Serial.println("OK"); 
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

    for(int i=0; i<6; i++) digitalWrite(JOINTS[i].enPin, LOW); // 強制 Enable

    static bool wasMoving = false; 
    bool isMoving = false; 

    for (int i = 0; i < 6; i++) {
        if (steppers[i]->moving()) {
            isMoving = true;
            break; 
        }
    }

    if (wasMoving && !isMoving) {
        Serial.println("Done"); 
    }

    wasMoving = isMoving;
}