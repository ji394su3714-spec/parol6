#include "Comms.h"
#include "Globals.h"
#include "Config.h"

void processCommand() {
    // 1. 緊急停止 (E-STOP)
    if (strncmp(tempChars, "STOP", 4) == 0) {
        for(int i = 0; i < 6; i++) {
            homingState[i] = 0;  
            steppers[i]->doSteps(0);
        }

        // 緊急把水桶裡的水全部倒掉！
        bufHead = 0;
        bufTail = 0;
        bufCount = 0;
        isBufPlaying = false;
        pendingOK = false;
        
        normalMoveActive = false;
        Serial.println("!!! E-STOP TRIGGERED !!!");
        return;
    }

    // 2. 字串解析
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

    // 4. 一般移動分派 (大一統引擎)
    if (!isAnyHoming() && !homingTriggered) {

        // ==========================================
        // 模式 2：大一統串流模式 (Streaming) - 塞進水桶
        // 負責：PTP, LIN, CIRC (Python 已計算好完美加減速)
        // ==========================================
        if (moveMode == 2) {
            if (bufCount < BUF_SIZE) {
                for(int i = 0; i < 6; i++) {
                    // 🌟 補上這行：強制關閉硬體加減速，徹底交給 Python 控制！
                    steppers[i]->setRampLen(0);
                    
                    if (receivedAngles[i] != 999.0) {
                        ringBuf[bufHead].targetSteps[i] = receivedAngles[i] * getStepsPerDeg(i);
                    } else {
                        // 防呆：沒收到角度就維持當前目標
                        ringBuf[bufHead].targetSteps[i] = steppers[i]->currentPosition(); 
                    }
                }
                ringBuf[bufHead].interval_us = (unsigned long)(param7 * 1000000.0);

                bufHead = (bufHead + 1) % BUF_SIZE;
                bufCount++;

                // 水桶沒滿就馬上回 OK；滿了就扣留 OK (啟動背壓防護)
                if (bufCount < BUF_SIZE) {
                    Serial.println("OK");
                } else {
                    pendingOK = true;
                }
            } else {
                Serial.println("BufferFull");
            }
            normalMoveActive = true;
            return; // 裝桶完畢，直接下班離開函式！
        }
        
        // ==========================================
        // 模式 0：手動/點動模式 (Jogging)
        // 負責：UI 滑桿、Jog 按鈕 (交給硬體 MobaTools 防暴衝)
        // ==========================================
        else if (moveMode == 0) {
            for (int i = 0; i < 6; i++) {
                if (receivedAngles[i] != 999.0) {
                    long targetSteps = receivedAngles[i] * getStepsPerDeg(i);
                    
                    // 根據 UI 傳來的百分比計算速度
                    float currentMaxSpeedSec = (JOINTS[i].jointControlSpd10 * param7) / 10.0;
                    long mobaSpeed = (long)(currentMaxSpeedSec * 10.0);
                    if (mobaSpeed < 1) mobaSpeed = 1; 

                    steppers[i]->setSpeedSteps(mobaSpeed);
                    
                    // 啟用硬體加減速 (維持手動操作的避震手感)
                    steppers[i]->setRampLen(JOINTS[i].rampSteps);
                    steppers[i]->writeSteps(targetSteps); 
                }
            }
            normalMoveActive = true;
            Serial.println("OK"); 
        }
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
                if (ndx >= NUM_CHARS) ndx = NUM_CHARS - 1; 
            } else {
                receivedChars[ndx] = '\0';
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        } else if (rc == startMarker) recvInProgress = true;
    }
}