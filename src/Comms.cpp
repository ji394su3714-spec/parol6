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

    // 攔截 Python 傳來的動態設定指令
    if (strncmp(tempChars, "SET_CFG", 7) == 0) {
        char * strtokIndx = strtok(tempChars, ","); // SET_CFG
        
        strtokIndx = strtok(NULL, ",");
        if (strtokIndx != NULL) global_hw_max_steps = atol(strtokIndx);
        
        strtokIndx = strtok(NULL, ",");
        if (strtokIndx != NULL) global_T_acc = atof(strtokIndx);
        
        Serial.print("Config OK: MaxSteps="); Serial.print(global_hw_max_steps);
        Serial.print(", T_acc="); Serial.println(global_T_acc);
        return;
    }

    // 2. 升級版字串解析 (純整數極速解析)
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

    // 3. 歸零觸發 (將 999.0 改為 999999)
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
                    steppers[i]->setRampLen(0); 
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

        // 模式 1：串流模式 (Streaming) - 塞進水桶
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
                ringBuf[bufHead].interval_us = (unsigned long)param7; // Python已經換算成微秒了

                bufHead = (bufHead + 1) % BUF_SIZE;
                bufCount++;
                if (bufCount < BUF_SIZE) Serial.println("OK");
                else pendingOK = true;
            } else {
                Serial.println("BufferFull");
            }
            normalMoveActive = true;
            return;
        }

        // 模式 2：原生 PTP
        else if (moveMode == 2) {
            float speedFactor = (param7 <= 0.0) ? 1.0 : param7;
            float maxTime = 0.0;
            long deltaSteps[6] = {0};

            for (int i = 0; i < 6; i++) {
                if (receivedSteps[i] != 999999) {
                    // 零計算，直接讀取步數
                    deltaSteps[i] = abs(receivedSteps[i] - steppers[i]->currentPosition());
                    float v_max = (global_hw_max_steps / 10.0) * speedFactor;
                    float t_needed = deltaSteps[i] / v_max;
                    if (t_needed > maxTime) maxTime = t_needed;
                }
            }

            for (int i = 0; i < 6; i++) {
                if (receivedSteps[i] != 999999 && deltaSteps[i] > 0) {
                    float syncSpeed = deltaSteps[i] / maxTime;
                    long mobaSpeed = (long)(syncSpeed * 10.0 + 0.5);
                    if (mobaSpeed < 10) mobaSpeed = 10;

                    long dynamicRamp = (long)((syncSpeed * global_T_acc) / 2.0);
                    if (dynamicRamp > deltaSteps[i] / 2) dynamicRamp = deltaSteps[i] / 2;

                    steppers[i]->setSpeedSteps(mobaSpeed);
                    steppers[i]->setRampLen(dynamicRamp); 
                    
                    // 零計算，直接發送目標步數
                    steppers[i]->writeSteps(receivedSteps[i]);
                }
            }
            normalMoveActive = true; 
            Serial.println("OK");    
            return;
        }
        
        // 模式 0：手動/點動模式 (終極平滑多軸同步版)
        else if (moveMode == 0) {
            float speedFactor = (param7 <= 0.0) ? 1.0 : param7;
            long deltaSteps[6] = {0};
            float maxTime = 0.0;

            // 1. 預覽移動量，找出需要最長時間的「瓶頸軸」
            for (int i = 0; i < 6; i++) {
                if (receivedSteps[i] != 999999) {
                    // 計算距離目前位置還有多遠
                    deltaSteps[i] = abs(receivedSteps[i] - steppers[i]->currentPosition());
                    float v_max = (JOINTS[i].jointControlSpd10 * speedFactor) / 10.0;
                    if (v_max > 0) {
                        float t_needed = deltaSteps[i] / v_max;
                        if (t_needed > maxTime) maxTime = t_needed;
                    }
                }
            }

            // 2. 應用同步速度與「等比物理加速度」
            for (int i = 0; i < 6; i++) {
                if (receivedSteps[i] != 999999) {
                    if (deltaSteps[i] > 0 && maxTime > 0) {
                        // A. 同步最高速度
                        float syncSpeedSec = deltaSteps[i] / maxTime;
                        long mobaSpeed = (long)(syncSpeedSec * 10.0);
                        if (mobaSpeed < 1) mobaSpeed = 1;

                        // B. 同步斜坡 (🌟 核心魔法：保持物理加速度恆定！)
                        // 根據 s = v^2 / 2a，速度的縮放比例，斜坡步數必須是「平方倍」縮放
                        float originalMaxSpeedSec = (JOINTS[i].jointControlSpd10 * speedFactor) / 10.0;
                        float ratio = syncSpeedSec / originalMaxSpeedSec;
                        
                        long syncRamp = (long)(JOINTS[i].rampSteps * ratio * ratio);
                        
                        // 拔除舊版錯誤的 deltaSteps/2 限制，放心交給 MobaTools 處理短距離三角形！
                        if (syncRamp < 0) syncRamp = 0;

                        steppers[i]->setSpeedSteps(mobaSpeed);
                        steppers[i]->setRampLen(syncRamp);
                    }
                    
                    // 3. 發送目標步數
                    // MobaTools 底層緩衝非常優異，只要加速度斜坡正確，
                    // 即使你連發封包不斷推遲目標點，它也能無縫接軌、不抖不震！
                    steppers[i]->writeSteps(receivedSteps[i]); 
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