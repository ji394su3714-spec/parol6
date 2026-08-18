#include "Globals.h"
#include "MotionEngine.h"

static unsigned long limitDebounce[6] = {0}; 

bool isAnyHoming() {
    for (int i = 0; i < 6; i++) {
        if (homingState[i] != HOME_IDLE) return true;
    }
    return false;
}

bool isAxisMoving(int axis) {
    if (axis < 0 || axis >= 6) return false;
    LockMotionEngine(); 
    bool moving = (axes[axis].target_pos != axes[axis].current_pos) || 
                  (axes[axis].current_vel > 0.0f) || 
                  (axes[axis].target_vel > 0.0f);
    UnlockMotionEngine(); 
    return moving;
}

void moveToRelative(int axis, long relativeSteps, float speedSec, float rampRatio = 1.0f) {
    if (axis < 0 || axis >= 6) return;
    float accel = getAxisAccel(axis, rampRatio); 
    moveAxisIndependent(axis, relativeSteps, speedSec, accel);
}

bool isGroup1Ready(byte currentState) {
    for (int j = 0; j < 3; j++) {
        if (homingState[j] == HOME_IDLE) continue;
        if (homingState[j] < currentState) return false;
        if (homingState[j] == currentState && isAxisMoving(j)) return false;
    }
    return true;
}

bool startHomingSequence(long targets[6]) {
    bool group1_req = (targets[0] == 999999 || targets[1] == 999999 || targets[2] == 999999);
    bool j4_req = (targets[3] == 999999);
    bool j6_req = (targets[5] == 999999);
    bool homingTriggered = false;

    for (int i = 0; i < 6; i++) {
        if (targets[i] == 999999 && JOINT_PINS[i].limitPin != 0 && HOMING_CFG[i].homingSpeed != 0) {
            
            if (homingState[i] == HOME_IDLE) {
                if (i >= 3 && group1_req) { 
                    homingState[i] = HOME_WAIT_J123; 
                    homingTriggered = true; 
                } 
                else if (i == 4 && j6_req) { 
                    homingState[i] = HOME_WAIT_J6_PREP; 
                    homingTriggered = true; 
                } 
                else if (i == 5 && j4_req) { 
                    homingState[i] = HOME_WAIT_J4; 
                    homingTriggered = true; 
                } 
                else {
                    homingState[i] = HOME_FAST_SEARCH; 
                    float fastSpeed = abs(HOMING_CFG[i].homingSpeed);
                    int searchDir = SGN(HOMING_CFG[i].homingSpeed);
                    
                    // 替換：0.5f -> 0.8f
                    jogAxis(i, searchDir, fastSpeed, getAxisAccel(i, 1.0f), true); 
                    homingTriggered = true;
                }
            } 
        }
    }
    return homingTriggered;
}

void updateHomingLogic() {
    const long J6_PREP_STEPS = -16000; 
    static unsigned long j6DelayStartTime = 0; 
    static unsigned long j4DoneTime = 0;

    for (int i = 0; i < 6; i++) {
        if (homingState[i] == HOME_IDLE) continue;

        int searchDir = SGN(HOMING_CFG[i].homingSpeed); 
        float fastSpeed = abs(HOMING_CFG[i].homingSpeed);
        float slowSpeed = fastSpeed * 0.5f;

        switch (homingState[i]) {
            
            case HOME_WAIT_J123:
                if (homingState[0] == HOME_IDLE && homingState[1] == HOME_IDLE && homingState[2] == HOME_IDLE) {
                    if (i == 3) { 
                        homingState[i] = HOME_FAST_SEARCH; 
                        // 替換：預設 1.0f -> 1.5f
                        jogAxis(i, searchDir, fastSpeed, getAxisAccel(i, 1.5f), true);
                    } else { 
                        homingState[i] = HOME_WAIT_J4; 
                    }
                }
                break;

            case HOME_WAIT_J4:
                if (homingState[3] == HOME_IDLE) {
                    if (j4DoneTime == 0) j4DoneTime = millis();
                    
                    if (millis() - j4DoneTime >= 100) {
                        if (i == 5) {
                            homingState[i] = HOME_FAST_SEARCH;
                            // 替換：預設 1.0f -> 1.5f
                            jogAxis(i, searchDir, fastSpeed, getAxisAccel(i, 1.5f), true);
                        }
                        else if (i == 4) {
                            if (homingState[5] != HOME_IDLE) {
                                homingState[i] = HOME_WAIT_J6_PREP; 
                            } else {
                                homingState[i] = HOME_FAST_SEARCH;  
                                // 替換：預設 1.0f -> 1.5f
                                jogAxis(i, searchDir, fastSpeed, getAxisAccel(i, 1.5f), true);
                            }
                        }
                    }
                }
                break;

            case HOME_FAST_SEARCH:
            case HOME_SLOW_SEARCH:
                if (digitalRead(JOINT_PINS[i].limitPin) == LIMIT_ACTIVE_STATE[i]) { 
                    if (limitDebounce[i] == 0) limitDebounce[i] = millis();
                    else if (millis() - limitDebounce[i] >= 3) {
                        stopAxisInstant(i); 
                        
                        if (homingState[i] == HOME_FAST_SEARCH) {
                            homingState[i] = HOME_BOUNCE;
                        } else {
                            homingState[i] = HOME_START_OFFSET; 
                        }
                        limitDebounce[i] = 0; 
                    }
                } else {
                    limitDebounce[i] = 0; 
                }
                break;

            case HOME_BOUNCE:
                if (!isAxisMoving(i)) {
                    if (i <= 2 && !isGroup1Ready(HOME_BOUNCE)) break; 

                    long bounceDist = HOMING_CFG[i].bounceSteps * -searchDir; 
                    // 替換：0.2f -> 0.3f
                    moveToRelative(i, bounceDist, fastSpeed * 1.3f, 0.3f);
                    homingState[i] = HOME_WAIT_BOUNCE;
                }
                break;

            case HOME_WAIT_BOUNCE:
                if (!isAxisMoving(i)) {
                    if (i <= 2 && !isGroup1Ready(HOME_WAIT_BOUNCE)) break; 
                    
                    // 替換：0.5f -> 0.8f
                    jogAxis(i, searchDir, slowSpeed, getAxisAccel(i, 1.0f), true);
                    homingState[i] = HOME_SLOW_SEARCH;
                }
                break;

            case HOME_START_OFFSET:
                if (!isAxisMoving(i)) {
                    if (i <= 2 && !isGroup1Ready(HOME_START_OFFSET)) break; 

                    if (i == 5) {
                        // 替換：預設 1.0f -> 1.5f
                        moveToRelative(5, J6_PREP_STEPS, SPEED_CFG[5].maxSpeed, 1.5f); 
                        homingState[5] = HOME_J6_WAIT_PREP_DONE; 
                    }
                    else if (i == 4) {
                        homingState[4] = HOME_J5_WAIT_J6;
                    }
                    else {
                        float offsetSpeed = (i == 3) ? fastSpeed * 1.5f : fastSpeed * 2.0f;
                        // 替換：1.5f -> 2.5f
                        moveToRelative(i, HOMING_CFG[i].homingPos, offsetSpeed, 2.5f); 
                        homingState[i] = HOME_DONE;
                    }
                }
                break;

            case HOME_J6_WAIT_PREP_DONE:
                if (!isAxisMoving(i)) {
                    homingState[5] = HOME_J6_WAIT_J5;
                    
                    if (homingState[4] == HOME_WAIT_J6_PREP) {
                        homingState[4] = HOME_FAST_SEARCH;
                        // 替換：預設 1.0f -> 1.5f
                        jogAxis(4, SGN(HOMING_CFG[4].homingSpeed), abs(HOMING_CFG[4].homingSpeed), getAxisAccel(4, 1.5f), true); 
                    }
                }
                break;

            case HOME_J5_WAIT_J6:
                if (!isAxisMoving(i)) {
                    if (homingState[5] == HOME_J6_WAIT_J5 || homingState[5] == HOME_IDLE) {
                        // 替換：1.0f -> 1.5f
                        moveToRelative(4, HOMING_CFG[4].homingPos, fastSpeed * 1.5f, 1.5f); 
                        homingState[4] = HOME_DONE;
                        
                        if (homingState[5] == HOME_J6_WAIT_J5) {
                            homingState[5] = HOME_J6_FINAL_OFFSET;
                            j6DelayStartTime = millis();
                        }
                    }
                }
                break;

            case HOME_J6_FINAL_OFFSET:
                if (millis() - j6DelayStartTime >= 200) {
                    long offsetJ6 = HOMING_CFG[5].homingPos - J6_PREP_STEPS; 
                    // 替換：1.0f -> 1.5f
                    moveToRelative(5, offsetJ6, fastSpeed * 1.5f, 1.5f);
                    homingState[5] = HOME_DONE;
                }
                break;

            case HOME_DONE:
                if (!isAxisMoving(i)) {
                    setAxisPosition(i, 0);
                    homingState[i] = HOME_IDLE;
                    Serial.print(">>> Axis "); Serial.print(i + 1); Serial.println(" Homing Done <<<");
                }
                break;
        }
    }

    static bool wasHoming = false;
    bool stillHoming = isAnyHoming(); 
    if (wasHoming && !stillHoming) { 
        Serial.println("HomingDone"); 
        j4DoneTime = 0; 
    }
    wasHoming = stillHoming;
}