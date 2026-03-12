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

    // 4. 一般移動分派
    if (!isAnyHoming() && !homingTriggered) {
        long deltaSteps[6] = {0};
        float timeNeeded[6] = {0.0};
        float maxTime = 0.0;

        for (int i = 0; i < 6; i++) {
            if (receivedAngles[i] != 999.0) {
                long targetSteps = receivedAngles[i] * getStepsPerDeg(i);
                deltaSteps[i] = abs(targetSteps - steppers[i]->currentPosition());
                
                if (moveMode == 1 || moveMode == 0) { 
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
                
                if ((moveMode == 1 || moveMode == 0) && maxTime > 0.0 && deltaSteps[i] > 0) {
                    float syncStepsPerSec = deltaSteps[i] / maxTime;
                    long mobaSpeed = (long)(syncStepsPerSec * 10.0);
                    if (mobaSpeed < 10) mobaSpeed = 10; 
                    steppers[i]->setSpeedSteps(mobaSpeed);
                    
                    float accelTimeSec = (moveMode == 0) ? 0.3 : 0.2; 
                    long syncRamp = syncStepsPerSec * (accelTimeSec / 2.0);
                    if (syncRamp < 5) syncRamp = 5; 
                    steppers[i]->setRampLen(syncRamp);
                    
                } else if (moveMode == 2 && deltaSteps[i] > 0) {
                    float interval_sec = param7;
                    if (interval_sec < 0.01) interval_sec = 0.01; 
                    float syncStepsPerSec = deltaSteps[i] / interval_sec;
                    long mobaSpeed = (long)(syncStepsPerSec * 10.0);
                    if (mobaSpeed < 1) mobaSpeed = 1; 
                    steppers[i]->setSpeedSteps(mobaSpeed);
                    steppers[i]->setRampLen(5); 
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