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

    // 2. 升級版字串解析
    long tempParsedSteps[6] = {0};
    float param7 = 1.0;
    int moveMode = 0;
    int parseCount = 0; 
    
    char * strtokIndx = strtok(tempChars, ",");
    while (strtokIndx != NULL && parseCount < 8) {
        if (parseCount < 6) {
            tempParsedSteps[parseCount] = atol(strtokIndx); // 極速轉為長整數
        } else if (parseCount == 6) {
            param7 = atof(strtokIndx); // 只有第7參數 (速度比例/時間) 允許用 float 讀取
        } else if (parseCount == 7) {
            moveMode = atoi(strtokIndx);
        }
        parseCount++;
        strtokIndx = strtok(NULL, ",");
    }

    if (parseCount < 6) return; // 錯誤封包防護

    for(int i = 0; i < 6; i++) {
        receivedSteps[i] = tempParsedSteps[i];
    }

    // 3. 歸零觸發
    bool group1_req = (receivedSteps[0] == 999999 || receivedSteps[1] == 999999 || receivedSteps[2] == 999999);
    bool j4_req = (receivedSteps[3] == 999999);
    bool j6_req = (receivedSteps[5] == 999999);
    bool homingTriggered = false;

    for (int i = 0; i < 6; i++) {
        if (receivedSteps[i] == 999999 && JOINTS[i].limitPin != 0 && JOINTS[i].homingSpeed != 0) {
            if (homingState[i] == 0) {
                if (i >= 3 && group1_req) { homingState[i] = 20; homingTriggered = true; } 
                else if (i == 4 && j6_req) { homingState[i] = 10; homingTriggered = true; } 
                else if (i == 5 && j4_req) { homingState[i] = 10; homingTriggered = true; } 
                else {
                    homingState[i] = 1; 
                    steppers[i]->setRampLen(100); 
                    long homingSpd = abs(JOINTS[i].homingSpeed) * 10;
                    steppers[i]->setSpeedSteps(homingSpd);
                    steppers[i]->rotate((JOINTS[i].homingSpeed > 0) ? 1 : -1); 
                    homingTriggered = true;
                }
            } 
        }
    }
    if (homingTriggered) Serial.println("OK");

    // 4. 一般移動分派
    if (!isAnyHoming() && !homingTriggered) {

        // 模式 1：串流模式 (Streaming)
        if (moveMode == 1) {
            if (bufCount < BUF_SIZE) {
                for(int i = 0; i < 6; i++) {
                    steppers[i]->setRampLen(0);
                    if (receivedSteps[i] != 999999) {
                        // 核心修改：零計算，直接把步數塞進水桶！
                        ringBuf[bufHead].targetSteps[i] = receivedSteps[i]; 
                    } else {
                        ringBuf[bufHead].targetSteps[i] = steppers[i]->currentPosition(); 
                    }
                }
                ringBuf[bufHead].interval_us = (unsigned long)param7; 

                bufHead = (bufHead + 1) % BUF_SIZE;
                bufCount++;
            } else {
                Serial.println("BufferFull");
            }
            normalMoveActive = true;
            return;
        }
        
        // 模式 0：手動/點動模式 (PTP 點對點引擎)
        else if (moveMode == 0) {
            float speedFactor = (param7 <= 0.0) ? 1.0 : param7;
            long deltaSteps[6] = {0};
            float maxTime = 0.0;

            // 1. 尋找「速度瓶頸」：誰需要花最多時間跑到終點？
            for (int i = 0; i < 6; i++) {
                if (receivedSteps[i] != 999999) {
                    deltaSteps[i] = abs(receivedSteps[i] - steppers[i]->currentPosition());
                    float v_max = (JOINTS[i].jointControlSpd10 * speedFactor) / 10.0;
                    if (v_max > 0) {
                        float t_needed = deltaSteps[i] / v_max;
                        if (t_needed > maxTime) maxTime = t_needed;
                    }
                }
            }

            // 2. 核心修復：尋找「加速度瓶頸」：誰需要花最多時間起步？
            float maxRampTime = 0.0;
            if (maxTime > 0) {
                for (int i = 0; i < 6; i++) {
                    if (deltaSteps[i] > 0) {
                        float v_target = deltaSteps[i] / maxTime; // 該軸的目標巡航速度
                        float absoluteMaxSpeedSec = JOINTS[i].jointControlSpd10 / 10.0;
                        
                        // 根據 a = V^2 / 2S 算出這顆馬達的物理極限加速度
                        float a_max = (absoluteMaxSpeedSec * absoluteMaxSpeedSec) / (2.0 * JOINTS[i].rampSteps);
                        if (a_max > 0) {
                            float t_ramp_needed = v_target / a_max; // 達到目標速度所需的時間
                            if (t_ramp_needed > maxRampTime) maxRampTime = t_ramp_needed;
                        }
                    }
                }
            }

            // 3. 應用完美的「雙重同步」
            for (int i = 0; i < 6; i++) {
                if (receivedSteps[i] != 999999) {
                    if (deltaSteps[i] > 0 && maxTime > 0) {
                        // A. 同步巡航速度
                        float syncSpeedSec = deltaSteps[i] / maxTime;
                        long mobaSpeed = (long)(syncSpeedSec * 10.0);
                        if (mobaSpeed < 1) mobaSpeed = 1;

                        // B. 同步加速斜坡 (強迫所有軸使用最慢的那顆馬達的起步時間 maxRampTime)
                        // 根據 S = 0.5 * V * T，算出這顆馬達該設定多長的斜坡步數
                        long syncRamp = (long)(0.5 * syncSpeedSec * maxRampTime);
                        if (syncRamp < 0) syncRamp = 0;

                        steppers[i]->setSpeedSteps(mobaSpeed);
                        steppers[i]->setRampLen(syncRamp);
                    }
                    steppers[i]->writeSteps(receivedSteps[i]); 
                }
            }
            normalMoveActive = true;
            Serial.println("OK"); 
        }

        // ==========================================
        // 模式 2：連續寸動模式 (Continuous Jogging) 專用
        // ==========================================
        else if (moveMode == 2) {
            int axis = receivedSteps[0]; // 接收指定的關節 (0~5)
            int dir = receivedSteps[1];  // 接收方向 (1正轉, -1反轉, 0煞車停止)
            float speedFactor = param7;  // 接收速度檔位 (0.25 ~ 1.0)

            if (axis >= 0 && axis < 6) {
                if (dir == 0) {
                    // 收到煞車指令：交給 MobaTools 依照設定的斜坡完美煞停
                    steppers[axis]->rotate(0); 
                } else {
                    // 收到無限轉動指令：算出絕對極速並套用檔位比例
                    float absoluteMaxSpeedSec = JOINTS[axis].jointControlSpd10 / 10.0;
                    long jogSpeed = (long)(absoluteMaxSpeedSec * speedFactor * 10.0);
                    if (jogSpeed < 10) jogSpeed = 10;
                    
                    steppers[axis]->setSpeedSteps(jogSpeed);
                    steppers[axis]->setRampLen(JOINTS[axis].rampSteps); 
                    
                    // 啟動無限轉動 (1 或 -1)
                    steppers[axis]->rotate(dir);
                }
            }
            normalMoveActive = true;
            Serial.println("OK");
        }
    }
}

void recvWithStartEndMarkers() {
    static bool recvInProgress = false;
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