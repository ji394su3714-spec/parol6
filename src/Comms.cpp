#include "Comms.h"
#include "Globals.h"
#include "Config.h"
#include "MotionEngine.h" 

void processCommand() {
    if (strncmp(tempChars, "STOP", 4) == 0) {
        for(int i = 0; i < 6; i++) homingState[i] = 0;  
        emergencyStopEngine(); 
        normalMoveActive = false;
        Serial.println("!!! E-STOP TRIGGERED !!!");
        return;
    }

    long tempParsedSteps[6] = {0};
    float param7 = 1.0;
    int moveMode = 0;
    int parseCount = 0; 
    
    char * strtokIndx = strtok(tempChars, ",");
    while (strtokIndx != NULL && parseCount < 8) {
        if (parseCount < 6) tempParsedSteps[parseCount] = atol(strtokIndx); 
        else if (parseCount == 6) param7 = atof(strtokIndx); 
        else if (parseCount == 7) moveMode = atoi(strtokIndx);
        parseCount++;
        strtokIndx = strtok(NULL, ",");
    }

    if (parseCount < 6) return; 
    for(int i = 0; i < 6; i++) receivedSteps[i] = tempParsedSteps[i];

    // 3. 歸零觸發 (適配新引擎的 jogAxis 觸發)
    bool group1_req = (receivedSteps[0] == 999999 || receivedSteps[1] == 999999 || receivedSteps[2] == 999999);
    bool j4_req = (receivedSteps[3] == 999999);
    bool j6_req = (receivedSteps[5] == 999999);
    bool homingTriggered = false;

    for (int i = 0; i < 6; i++) {
        if (receivedSteps[i] == 999999 && JOINT_PINS[i].limitPin != 0 && HOMING_CFG[i].homingSpeed != 0) {
            if (homingState[i] == 0) {
                if (i >= 3 && group1_req) { homingState[i] = 20; homingTriggered = true; } 
                else if (i == 4 && j6_req) { homingState[i] = 10; homingTriggered = true; } 
                else if (i == 5 && j4_req) { homingState[i] = 10; homingTriggered = true; } 
                else {
                    homingState[i] = 1; 
                    long homingSpdSec = abs(HOMING_CFG[i].homingSpeed);
                    
                    jogAxis(i, (HOMING_CFG[i].homingSpeed > 0) ? 1 : -1, (float)homingSpdSec, getAxisAccel(i), true); 
                    homingTriggered = true;
                }
            } 
        }
    }
    if (homingTriggered) Serial.println("OK");

    if (!isAnyHoming() && !homingTriggered) {
        // ==========================================
        // 模式 1：串流模式 (Streaming / CAM 插補)
        // ==========================================
        if (moveMode == 1) {
            long target[6];
            for(int i = 0; i < 6; i++) {
                if (receivedSteps[i] != 999999) target[i] = receivedSteps[i]; 
                else target[i] = getAxisPosition(i); 
            }
            while (!pushMotionPoint(target[0], target[1], target[2], target[3], target[4], target[5], (uint32_t)param7)) {
                delay(1); 
            }
            normalMoveActive = true;
            Serial.println("OK"); 
            return;
        }
        
        // ==========================================
        // 模式 0：手動 / 點動模式 (PTP 點對點)
        // ==========================================
        else if (moveMode == 0) {
            float speedFactor = (param7 <= 0.0) ? 1.0 : param7;
            long target[6];
            float maxTime = 0.0;

            for (int i = 0; i < 6; i++) {
                if (receivedSteps[i] != 999999) {
                    target[i] = receivedSteps[i];
                    long delta = abs(target[i] - getAxisPosition(i));
                    float v_max = (SPEED_CFG[i].jointControlSpd10 * speedFactor) / 10.0f; 
                    if (v_max > 0) {
                        float t_needed = delta / v_max;
                        if (t_needed > maxTime) maxTime = t_needed;
                    }
                } else {
                    target[i] = getAxisPosition(i);
                }
            }
            uint32_t interval_us = (uint32_t)(maxTime * 1000000.0f);
            if (interval_us < 10000) interval_us = 10000;

            pushMotionPoint(target[0], target[1], target[2], target[3], target[4], target[5], interval_us);
            normalMoveActive = true;
            Serial.println("OK"); 
        }

        // ==========================================
        // 模式 2：連續寸動模式 (Continuous Jogging)
        // ==========================================
        else if (moveMode == 2) {
            int axis = receivedSteps[0]; 
            int dir = receivedSteps[1];  
            float speedFactor = param7;  

            if (axis >= 0 && axis < 6) {
                // 取得該軸專屬的平滑加速度
                float accel = getAxisAccel(axis);

                if (dir == 0) {
                    jogAxis(axis, 0, 0.0f, accel, false); 
                } else {
                    float absoluteMaxSpeedSec = SPEED_CFG[axis].jointControlSpd10 / 10.0f;
                    float jogSpeedSec = absoluteMaxSpeedSec * speedFactor;
                    if (jogSpeedSec < 10.0f) jogSpeedSec = 10.0f;
                    
                    jogAxis(axis, dir, jogSpeedSec, accel, false);
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