#include "Comms.h"
#include "Globals.h"
#include "Config.h"

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
    
    // 3. 歸零觸發 
    bool group1_req = (receivedAngles[0] == 999.0 || receivedAngles[1] == 999.0 || receivedAngles[2] == 999.0);
    bool j4_req = (receivedAngles[3] == 999.0);
    bool j6_req = (receivedAngles[5] == 999.0);
    bool homingTriggered = false;

    for (int i = 0; i < 6; i++) {
        if (receivedAngles[i] == 999.0 && JOINTS[i].limitPin != 0 && JOINTS[i].homingSpeed != 0) {
            if (homingState[i] == 0) {
                if (i >= 3 && group1_req) {
                    homingState[i] = 20; 
                    homingTriggered = true;
                } else if (i == 4 && j6_req) {
                    homingState[i] = 10; 
                    homingTriggered = true;
                } else if (i == 5 && j4_req) {
                    homingState[i] = 10; 
                    homingTriggered = true;
                } else {
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

    // 4. 一般移動分派 (Jog / PTP / LIN 統一引擎)
    if (!isAnyHoming() && !homingTriggered) {
        long deltaSteps[6] = {0};
        float timeNeeded[6] = {0.0};
        float maxTime = 0.0;

        // 【預算階段】
        for (int i = 0; i < 6; i++) {
            if (receivedAngles[i] != 999.0) {
                long targetSteps = receivedAngles[i] * getStepsPerDeg(i);
                deltaSteps[i] = abs(targetSteps - steppers[i]->currentPosition());
                
                // Mode 0 (Jog) 和 Mode 1 (PTP) 依靠最高速來預算時間
                if (moveMode == 1 || moveMode == 0) { 
                    float currentMaxSpeedSec = (JOINTS[i].maxSpeedSteps10 * param7) / 10.0;
                    if (currentMaxSpeedSec > 0 && deltaSteps[i] > 0) {
                        timeNeeded[i] = deltaSteps[i] / currentMaxSpeedSec;
                        if (timeNeeded[i] > maxTime) maxTime = timeNeeded[i]; 
                    }
                }
            }
        }

        // Mode 2 (LIN) 直接把 Python 送來的切片時間 (param7) 當作 maxTime
        if (moveMode == 2) {
            maxTime = param7;
            if (maxTime < 0.01) maxTime = 0.01; // 防呆底線
        }

        // 【正式發車階段】
        for (int i = 0; i < 6; i++) {
            if (receivedAngles[i] != 999.0) {
                long targetSteps = receivedAngles[i] * getStepsPerDeg(i);
                
                if ((moveMode == 0 || moveMode == 1 || moveMode == 2) && maxTime > 0.0 && deltaSteps[i] > 0) {
                    
                    // 1. 速度同步與彈性追跡 (Elastic Tracking)
                    // PTP 需要精準停靠給 1.0；Jog 和 LIN 給予 0.95 讓馬達永遠處於「微幅追趕狀態」，吃掉 USB 延遲！
                    float speedTolerance = (moveMode == 1) ? 1.0 : 0.95; 
                    float syncStepsPerSec = (deltaSteps[i] / maxTime) * speedTolerance;
                    
                    long mobaSpeed = (long)(syncStepsPerSec * 10.0);
                    
                    // 關鍵防護網：絕對速度天花板！防止失步骨折！
                    if (mobaSpeed > JOINTS[i].maxSpeedSteps10) {
                        mobaSpeed = JOINTS[i].maxSpeedSteps10; 
                        // 可選：如果你想在終端機看到哪一軸超速了，可以把下面這行取消註解
                        // Serial.print("Warning: J"); Serial.print(i+1); Serial.println(" Speed Capped!");
                    }
                    
                    if (mobaSpeed < 1) mobaSpeed = 1; 
                    steppers[i]->setSpeedSteps(mobaSpeed);
                    
                    // 2. 關鍵分流：Ramp 策略必須各自獨立！
                    if (moveMode == 1 || moveMode == 0) {
                        // PTP 與 JOG：距離長，由 Arduino 負責產生避震 Ramp
                        float accelTimeSec = 0.3; 
                        long syncRamp = syncStepsPerSec * (accelTimeSec / 2.0);
                        if (syncRamp < 5) syncRamp = 5; 
                        steppers[i]->setRampLen(syncRamp);
                    } else {
                        // LIN：距離極短，Python 已經算好 S 曲線了
                        // 因為有 0.95 的滯後係數，馬達不會停，Ramp 設為 0 也能完美平滑變速
                        steppers[i]->setRampLen(0); 
                    }
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
                if (ndx >= NUM_CHARS) ndx = NUM_CHARS - 1; // 修正使用常數
            } else {
                receivedChars[ndx] = '\0';
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        } else if (rc == startMarker) recvInProgress = true;
    }
}