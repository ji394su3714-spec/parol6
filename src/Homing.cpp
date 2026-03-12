#include "Homing.h"
#include "Globals.h"
#include "Config.h"

// 核心：二段式歸零狀態機 (Double-Tap)
void updateHomingLogic() {
    const float J6_PREP_ANGLE = 90.0; 
    static unsigned long j6DelayStartTime = 0; 
    
    for (int i = 0; i < 6; i++) {
        // 狀態 1：【第一段】快速尋找開關
        if (homingState[i] == 1 && JOINTS[i].limitPin != 0) {
            if (digitalRead(JOINTS[i].limitPin) == JOINTS[i].limitActiveState) {
                delay(3); 
                if (digitalRead(JOINTS[i].limitPin) == JOINTS[i].limitActiveState) {
                    steppers[i]->setRampLen(0);
                    steppers[i]->stop(); 
                    homingState[i] = 2; 
                }
            } 
        }
        // 狀態 2：急停完畢，確認隊友後，執行【短回彈】
        else if (homingState[i] == 2) {
            if (!steppers[i]->moving()) {
                bool readyToBounce = true;
                if (i <= 2) {
                    for (int j = 0; j < 3; j++) {
                        if (homingState[j] == 1 || (homingState[j] == 2 && steppers[j]->moving())) {
                            readyToBounce = false;
                            break;
                        }
                    }
                }
                if (readyToBounce) {
                    steppers[i]->setRampLen(20);
                    steppers[i]->setSpeedSteps(abs(JOINTS[i].homingSpeed) * 15);
                    int bounceDir = (JOINTS[i].homingSpeed > 0) ? -1 : 1;
                    long bounceDist = JOINTS[i].bounceSteps * bounceDir;
                    steppers[i]->doSteps(bounceDist); 
                    homingState[i] = 12; 
                }
            }
        }
        // 狀態 12：回彈完畢，確認隊友後，執行【第二段尋找】
        else if (homingState[i] == 12) {
            if (!steppers[i]->moving()) {
                bool readyToSecondTap = true;
                if (i <= 2) {
                    for (int j = 0; j < 3; j++) {
                        if (homingState[j] == 2 || (homingState[j] == 12 && steppers[j]->moving())) {
                            readyToSecondTap = false;
                            break;
                        }
                    }
                }
                if (readyToSecondTap) {
                    steppers[i]->setRampLen(0);
                    steppers[i]->setSpeedSteps(abs(JOINTS[i].homingSpeed) * 5); 
                    int dir = (JOINTS[i].homingSpeed > 0) ? 1 : -1;
                    steppers[i]->rotate(dir); 
                    homingState[i] = 13; 
                }
            }
        }
        // 狀態 13：【第二段】尋找開關
        else if (homingState[i] == 13) {
            if (digitalRead(JOINTS[i].limitPin) == JOINTS[i].limitActiveState) {
                delay(3); 
                if (digitalRead(JOINTS[i].limitPin) == JOINTS[i].limitActiveState) {
                    steppers[i]->setRampLen(0);
                    steppers[i]->stop(); 
                    homingState[i] = 14; 
                    Serial.print(">>> J"); Serial.print(i + 1); Serial.println(" Hit 2! Ready for final offset.");
                }
            } 
        }
        // 狀態 14：第二次急停完畢，決定 Final Offset 或 Prep 退回策略
        else if (homingState[i] == 14) {
            if (!steppers[i]->moving()) {
                bool readyToOffset = true;
                if (i <= 2) {
                    for (int j = 0; j < 3; j++) {
                        if (homingState[j] == 13 || (homingState[j] == 14 && steppers[j]->moving())) {
                            readyToOffset = false;
                            break;
                        }
                    }
                }
                if (i == 5) {
                    readyToOffset = false; 
                    long prepSteps = J6_PREP_ANGLE * getStepsPerDeg(i);
                    steppers[i]->setRampLen(100);
                    steppers[i]->setSpeedSteps(JOINTS[i].maxSpeedSteps10);
                    steppers[i]->doSteps(prepSteps); 
                    homingState[i] = 4; 
                }
                if (i == 4) {
                    readyToOffset = false;
                    homingState[i] = 6; 
                }
                if (readyToOffset) {
                    if (i <= 2) {
                        steppers[i]->setRampLen(100);
                        steppers[i]->setSpeedSteps(abs(JOINTS[i].homingSpeed) * 20);
                    } 
                    else if (i == 3) {
                        steppers[i]->setRampLen(100);
                        steppers[i]->setSpeedSteps(abs(JOINTS[i].homingSpeed) * 15);
                    }
                    long offsetSteps = JOINTS[i].homingPos * getStepsPerDeg(i);
                    steppers[i]->doSteps(offsetSteps); 
                    homingState[i] = 3; 
                }
            }
        }
        // 狀態 3：Offset 退回完畢 (真正設為 0)
        else if (homingState[i] == 3) {
            if (!steppers[i]->moving()) {
                steppers[i]->setZero(0); 
                steppers[i]->writeSteps(0); 
                homingState[i] = 0;
                Serial.print(">>> Axis "); Serial.print(i + 1); Serial.println(" Homing Done (At True Zero) <<<");

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
                                    steppers[k]->setRampLen(0);
                                    steppers[k]->setSpeedSteps(abs(JOINTS[k].homingSpeed) * 10);
                                    steppers[k]->rotate((JOINTS[k].homingSpeed > 0) ? 1 : -1);
                                    Serial.print(">>> Homing Start: J"); Serial.println(k + 1);
                                }
                            }
                        }
                    }
                }
                if (i == 3 && homingState[5] == 10) {
                    homingState[5] = 1;
                    steppers[5]->setRampLen(0);
                    steppers[5]->setSpeedSteps(abs(JOINTS[5].homingSpeed) * 10);
                    steppers[5]->rotate((JOINTS[5].homingSpeed > 0) ? 1 : -1);
                    Serial.println(">>> Homing Start: J6 ");
                }
            }
        }
        // 狀態 4：J6 正在前往 90 度預備姿態
        else if (homingState[i] == 4 && i == 5) {
            if (!steppers[i]->moving()) {
                homingState[i] = 5; 
                if (homingState[4] == 10) {
                    homingState[4] = 1;
                    steppers[4]->setRampLen(0);
                    steppers[4]->setSpeedSteps(abs(JOINTS[4].homingSpeed) * 10);
                    steppers[4]->rotate((JOINTS[4].homingSpeed > 0) ? 1 : -1);
                    Serial.println(">>> Homing Start: J5 ");
                }
            }
        }
        // 狀態 6：J5 撞到開關了，等待 J6 準備好
        else if (homingState[i] == 6 && i == 4) {
            if (homingState[5] == 5) {
                steppers[4]->setRampLen(150);
                steppers[4]->setSpeedSteps(abs(JOINTS[4].homingSpeed) * 10);
                long offsetJ5 = JOINTS[4].homingPos * getStepsPerDeg(4);
                steppers[4]->doSteps(offsetJ5);
                homingState[4] = 3; 
                homingState[5] = 15; 
                j6DelayStartTime = millis(); 
            }
        }
        // 狀態 15：J6 專屬的 0.3 秒非阻塞延遲
        else if (homingState[i] == 15 && i == 5) {
            if (millis() - j6DelayStartTime >= 300) { 
                steppers[5]->setRampLen(200); 
                steppers[5]->setSpeedSteps(abs(JOINTS[5].homingSpeed) * 15);
                float remainingAngle = JOINTS[5].homingPos - J6_PREP_ANGLE;
                long offsetJ6 = remainingAngle * getStepsPerDeg(5);
                steppers[5]->doSteps(offsetJ6);
                homingState[5] = 3; 
            }
        }
    }

    static bool wasHoming = false;
    bool stillHoming = isAnyHoming();
    if (wasHoming && !stillHoming) {
        Serial.println("HomingDone");  
    }
    wasHoming = stillHoming;
}