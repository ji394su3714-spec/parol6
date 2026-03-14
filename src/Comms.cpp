#include "Comms.h"
#include "Globals.h"
#include "Config.h"

void processCommand() {
    if (strncmp(tempChars, "STOP", 4) == 0) {
        for(int i = 0; i < 6; i++) {
            homingState[i] = 0;  
            steppers[i]->doSteps(0);
        }

        // 補上這段！緊急把水桶裡的水全部倒掉！
        bufHead = 0;
        bufTail = 0;
        bufCount = 0;
        isBufPlaying = false;
        pendingOK = false;
        
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

    // 4. 一般移動分派 (Jog / PTP / LIN 大一統引擎)
    if (!isAnyHoming() && !homingTriggered) {

        // 核心分流：如果是 LIN 模式 (Mode 2)，直接裝入環形緩衝區！
        if (moveMode == 2) {
            if (bufCount < BUF_SIZE) {
                for(int i = 0; i < 6; i++) {
                    if (receivedAngles[i] != 999.0) {
                        ringBuf[bufHead].targetSteps[i] = receivedAngles[i] * getStepsPerDeg(i);
                    } else {
                        // 防呆：沒收到角度就維持當前目標
                        ringBuf[bufHead].targetSteps[i] = steppers[i]->currentPosition(); 
                    }
                }
                ringBuf[bufHead].interval_ms = (unsigned long)(param7 * 1000.0);

                bufHead = (bufHead + 1) % BUF_SIZE;
                bufCount++;

                // 水桶沒滿就馬上回 OK；滿了就扣留 OK (啟動背壓防護)
                if (bufCount < BUF_SIZE) {
                    Serial.println("OK");
                } else {
                    pendingOK = true;
                }
            }
            normalMoveActive = true;
            return; // 裝桶完畢，直接下班離開函式！
        }

        // 以下為 Jog (Mode 0) 與 PTP (Mode 1) 的即時發車邏輯
        long deltaSteps[6] = {0};
        float timeNeeded[6] = {0.0};
        float maxTime = 0.0;

        // 【第一階段：收集距離與基礎時間預算】
        for (int i = 0; i < 6; i++) {
            if (receivedAngles[i] != 999.0) {
                long targetSteps = receivedAngles[i] * getStepsPerDeg(i);
                deltaSteps[i] = abs(targetSteps - steppers[i]->currentPosition());
                
                float currentMaxSpeedSec = (JOINTS[i].maxSpeedSteps10 * param7) / 10.0;
                if (currentMaxSpeedSec > 0 && deltaSteps[i] > 0) {
                    timeNeeded[i] = deltaSteps[i] / currentMaxSpeedSec;
                    if (timeNeeded[i] > maxTime) maxTime = timeNeeded[i]; 
                }
            }
        }

        // 【第二階段：防失步安全網 (全體等比例降速)】
        // 檢查 Jog/PTP 的移動會不會超速，若會，則強迫拉長全隊的時間！
        for (int i = 0; i < 6; i++) {
            if (receivedAngles[i] != 999.0 && deltaSteps[i] > 0) {
                float minSafeTime = deltaSteps[i] / (JOINTS[i].maxSpeedSteps10 / 10.0);
                if (minSafeTime > maxTime) {
                    maxTime = minSafeTime; 
                }
            }
        }

        // 【第三階段：正式發車 (僅處理 Jog 與 PTP)】
        for (int i = 0; i < 6; i++) {
            if (receivedAngles[i] != 999.0) {
                long targetSteps = receivedAngles[i] * getStepsPerDeg(i);
                
                if (maxTime > 0.0 && deltaSteps[i] > 0) {
                    
                    // PTP 需要精準停靠給 1.0；Jog 連續移動給 0.95 滯後避震
                    float speedTolerance = (moveMode == 1) ? 1.0 : 0.95; 
                    float syncStepsPerSec = (deltaSteps[i] / maxTime) * speedTolerance;
                    
                    long mobaSpeed = (long)(syncStepsPerSec * 10.0);
                    if (mobaSpeed < 1) mobaSpeed = 1; 
                    steppers[i]->setSpeedSteps(mobaSpeed);
                    
                    // PTP 與 Jog 皆由 Arduino 負責產生避震 Ramp
                    float accelTimeSec = 0.3; 
                    long syncRamp = syncStepsPerSec * (accelTimeSec / 2.0);
                    if (syncRamp < 5) syncRamp = 5; 
                    steppers[i]->setRampLen(syncRamp);
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