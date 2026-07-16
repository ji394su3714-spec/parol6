#include "Globals.h"
#include "Config.h"
#include "MotionEngine.h"

float getAxisAccel(int axis) {
    if (axis < 0 || axis >= 6) return 50000.0f;
    float max_v = SPEED_CFG[axis].maxSpeedSteps10 / 10.0f; // 該軸的絕對極速
    float ramp = (float)HOMING_CFG[axis].rampSteps;         // 該軸的加速步數
    if (ramp <= 0.0f) return 50000.0f;                  // 防呆
    
    // 物理公式 a = V^2 / 2S
    return (max_v * max_v) / (2.0f * ramp);
}

// 加入 current_vel 判斷
bool isAxisMoving(int axis) {
    if (axis < 0 || axis >= 6) return false;
    noInterrupts();
    bool moving = (axes[axis].target_pos != axes[axis].current_pos) || 
                  (axes[axis].current_vel > 0.0f) || 
                  (axes[axis].target_vel > 0.0f);
    interrupts();
    return moving;
}

void moveToRelative(int axis, long relativeSteps, float speedSec) {
    if (axis < 0 || axis >= 6) return;

    float accel = getAxisAccel(axis);
    
    moveAxisIndependent(axis, relativeSteps, speedSec, accel);
}

void updateHomingLogic() {
    const float J6_PREP_ANGLE = -90.0; 
    static unsigned long j6DelayStartTime = 0; 
    
    for (int i = 0; i < 6; i++) {
        // 狀態 1：【第一段】快速尋找開關
        if (homingState[i] == 1 && JOINT_PINS[i].limitPin != 0) {
            if (digitalRead(JOINT_PINS[i].limitPin) == LIMIT_ACTIVE_STATE[i]) {
                delay(3); 
                if (digitalRead(JOINT_PINS[i].limitPin) == LIMIT_ACTIVE_STATE[i]) {
                    
                    // 換成瞬間硬停！保證歸零的絕對精準度
                    stopAxisInstant(i); 
                    homingState[i] = 2; 
                }
            } 
        }
        else if (homingState[i] == 2) {
            if (!isAxisMoving(i)) {
                bool readyToBounce = true;
                if (i <= 2) {
                    for (int j = 0; j < 3; j++) {
                        if (homingState[j] == 1 || (homingState[j] == 2 && isAxisMoving(j))) { readyToBounce = false; break; }
                    }
                }
                if (readyToBounce) {
                    int bounceDir = (HOMING_CFG[i].homingSpeed > 0) ? -1 : 1;
                    long bounceDist = HOMING_CFG[i].bounceSteps * bounceDir;
                    float speedSec = abs(HOMING_CFG[i].homingSpeed) * 1.3f; 
                    moveToRelative(i, bounceDist, speedSec); 
                    homingState[i] = 12; 
                }
            }
        }
        else if (homingState[i] == 12) {
            if (!isAxisMoving(i)) {
                bool readyToSecondTap = true;
                if (i <= 2) {
                    for (int j = 0; j < 3; j++) {
                        if (homingState[j] == 2 || (homingState[j] == 12 && isAxisMoving(j))) { readyToSecondTap = false; break; }
                    }
                }
                if (readyToSecondTap) {
                    int dir = (HOMING_CFG[i].homingSpeed > 0) ? 1 : -1;
                    float slowSpeedSec = abs(HOMING_CFG[i].homingSpeed) * 0.5f; 
                    jogAxis(i, dir, slowSpeedSec, getAxisAccel(i), true); 
                    homingState[i] = 13; 
                }
            }
        }
        // 狀態 13：【第二段】尋找開關
        else if (homingState[i] == 13) {
            if (digitalRead(JOINT_PINS[i].limitPin) == LIMIT_ACTIVE_STATE[i]) {
                delay(3); 
                if (digitalRead(JOINT_PINS[i].limitPin) == LIMIT_ACTIVE_STATE[i]) {
                    
                    // 換成瞬間硬停！保證歸零的絕對精準度
                    stopAxisInstant(i); 
                    homingState[i] = 14; 
                }
            } 
        }
        else if (homingState[i] == 14) {
            if (!isAxisMoving(i)) {
                bool readyToOffset = true;
                if (i <= 2) {
                    for (int j = 0; j < 3; j++) {
                        if (homingState[j] == 13 || (homingState[j] == 14 && isAxisMoving(j))) { readyToOffset = false; break; }
                    }
                }
                if (i == 5) {
                    readyToOffset = false; 
                    long prepSteps = J6_PREP_ANGLE * getStepsPerDeg(i);
                    float prepSpeedSec = SPEED_CFG[i].maxSpeedSteps10 / 10.0f;
                    moveToRelative(i, prepSteps, prepSpeedSec);
                    homingState[i] = 4; 
                }
                if (i == 4) { readyToOffset = false; homingState[i] = 6; }
                
                if (readyToOffset) {
                    float offsetSpeedSec = abs(HOMING_CFG[i].homingSpeed) * 2.0f; 
                    if (i == 3) offsetSpeedSec = abs(HOMING_CFG[i].homingSpeed) * 1.5f;
                    long offsetSteps = HOMING_CFG[i].homingPos * getStepsPerDeg(i);
                    moveToRelative(i, offsetSteps, offsetSpeedSec); 
                    homingState[i] = 3; 
                }
            }
        }
        else if (homingState[i] == 3) {
            if (!isAxisMoving(i)) {
                setAxisPosition(i, 0); 
                homingState[i] = 0;
                Serial.print(">>> Axis "); Serial.print(i + 1); Serial.println(" Homing Done <<<");

                if (i <= 2) {
                    if (homingState[0] == 0 && homingState[1] == 0 && homingState[2] == 0) {
                        bool j4_waiting = (homingState[3] == 20);
                        bool j6_waiting = (homingState[5] == 20);
                        for (int k = 3; k < 6; k++) {
                            if (homingState[k] == 20) {
                                if (k == 4 && j6_waiting) homingState[k] = 10; 
                                else if (k == 5 && j4_waiting) homingState[k] = 10; 
                                else {
                                    homingState[k] = 1;  
                                    float speedSec = abs(HOMING_CFG[k].homingSpeed);
                                    // 補上加速度參數
                                    jogAxis(k, (HOMING_CFG[k].homingSpeed > 0) ? 1 : -1, speedSec, getAxisAccel(i), true);
                                }
                            }
                        }
                    }
                }
                if (i == 3 && homingState[5] == 10) {
                    homingState[5] = 1;
                    float speedSec = abs(HOMING_CFG[5].homingSpeed);
                    jogAxis(5, (HOMING_CFG[5].homingSpeed > 0) ? 1 : -1, speedSec, getAxisAccel(i), true);
                }
            }
        }
        else if (homingState[i] == 4 && i == 5) {
            if (!isAxisMoving(i)) {
                homingState[i] = 5; 
                if (homingState[4] == 10) {
                    homingState[4] = 1;
                    float speedSec = abs(HOMING_CFG[4].homingSpeed);
                    jogAxis(4, (HOMING_CFG[4].homingSpeed > 0) ? 1 : -1, speedSec, getAxisAccel(i), true);
                }
            }
        }
        else if (homingState[i] == 6 && i == 4) {
            if (homingState[5] == 5) {
                float speedSec = abs(HOMING_CFG[4].homingSpeed) * 1.4f;
                long offsetJ5 = HOMING_CFG[4].homingPos * getStepsPerDeg(4);
                moveToRelative(4, offsetJ5, speedSec);
                homingState[4] = 3; 
                homingState[5] = 15; 
                j6DelayStartTime = millis(); 
            }
        }
        else if (homingState[i] == 15 && i == 5) {
            if (millis() - j6DelayStartTime >= 500) { 
                float speedSec = abs(HOMING_CFG[5].homingSpeed);
                float remainingAngle = HOMING_CFG[5].homingPos - J6_PREP_ANGLE;
                long offsetJ6 = remainingAngle * getStepsPerDeg(5);
                moveToRelative(5, offsetJ6, speedSec);
                homingState[5] = 3; 
            }
        }
    }

    static bool wasHoming = false;
    bool stillHoming = isAnyHoming();
    if (wasHoming && !stillHoming) { Serial.println("HomingDone"); }
    wasHoming = stillHoming;
}