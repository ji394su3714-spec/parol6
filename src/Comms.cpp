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

    // 4. 一般移動分派
    if (!isAnyHoming() && !homingTriggered) {

        // 模式 1：串流模式 (Streaming) - 塞進水桶
        // 負責：PTP, LIN, CIRC (Python 已計算好完美加減速)
        if (moveMode == 1) {
            if (bufCount < BUF_SIZE) {
                for(int i = 0; i < 6; i++) {
                    // 關閉硬體加減速，徹底交給 Python 控制！
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
            return;
        }

        // 模式 2：原生 PTP (純時間加減速，零切片直驅)
        else if (moveMode == 2) {
            float speedFactor = param7; // Python 傳來的速度比例
            if (speedFactor <= 0.0) speedFactor = 1.0;

            // 加減速時間(秒)
            //float T_acc = 0.15; //移到path_manager

            float maxTime = 0.0;
            long deltaSteps[6] = {0};

            // 1. 尋找瓶頸：算出哪一根軸要花最久的時間
            for (int i = 0; i < 6; i++) {
                if (receivedAngles[i] != 999.0) {
                    long targetSteps = receivedAngles[i] * getStepsPerDeg(i);
                    deltaSteps[i] = abs(targetSteps - steppers[i]->currentPosition());
                    
                    // 該軸的物理極速 * 速度比例
                    float v_max = (global_hw_max_steps / 10.0) * speedFactor;
                    float t_needed = deltaSteps[i] / v_max;
                    if (t_needed > maxTime) {
                        maxTime = t_needed;
                    }
                }
            }

            // 2. 同步發車與「動態時間斜率」計算
            for (int i = 0; i < 6; i++) {
                if (receivedAngles[i] != 999.0 && deltaSteps[i] > 0) {
                    // 算出該軸配合瓶頸的同步巡航速度 (步/秒)
                    float syncSpeed = deltaSteps[i] / maxTime;
                    long mobaSpeed = (long)(syncSpeed * 10.0 + 0.5);
                    if (mobaSpeed < 10) mobaSpeed = 10;

                    // 用「時間」反推「步數」，徹底消滅煩人的 ramp 陣列！
                    long dynamicRamp = (long)((syncSpeed * global_T_acc) / 2.0);
                    
                    // 防呆：如果移動距離太短，加速段最多只能佔總距離的一半 (變成三角形軌跡)
                    if (dynamicRamp > deltaSteps[i] / 2) {
                        dynamicRamp = deltaSteps[i] / 2;
                    }

                    // 覆蓋原本寫死的 ramp，套用我們動態算出來的時間步數！
                    steppers[i]->setSpeedSteps(mobaSpeed);
                    steppers[i]->setRampLen(dynamicRamp); 
                    
                    long targetSteps = receivedAngles[i] * getStepsPerDeg(i);
                    steppers[i]->writeSteps(targetSteps);
                }
            }

            normalMoveActive = true; 
            Serial.println("OK");    
            return;
        }
        
        // 模式 0：手動/點動模式 (Jogging)
        // 負責：UI 滑桿、Jog 按鈕 (交給硬體 MobaTools 防暴衝)
        else if (moveMode == 0) {
            for (int i = 0; i < 6; i++) {
                if (receivedAngles[i] != 999.0) {
                    long targetSteps = receivedAngles[i] * getStepsPerDeg(i);
                    
                    // 根據 UI 傳來的百分比計算速度
                    float currentMaxSpeedSec = (JOINTS[i].jointControlSpd10 * param7) / 10.0;
                    long mobaSpeed = (long)(currentMaxSpeedSec * 10.0);
                    if (mobaSpeed < 1) mobaSpeed = 1; 

                    steppers[i]->setSpeedSteps(mobaSpeed);
                    
                    // 啟用硬體加減速 (維持手動操作的避震手感)---->可能需要修復同步問題
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