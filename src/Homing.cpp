#include "Globals.h"
#include "MotionEngine.h"

bool isAnyHoming() {
    for(int i = 0; i < 6; i++) {
        if(homingState[i] != 0) return true;
    }
    return false;
}

float getAxisAccel(int axis, float rampRatio) {
    if (axis < 0 || axis >= 6) return 50000.0f;
    
    // 使用controlSpeed作為基準，算出「恆定且舒適」的加速度
    float ref_v = SPEED_CFG[axis].controlSpeed;
    float ramp = (float)HOMING_CFG[axis].rampSteps * rampRatio;        
    if (ramp <= 0.0f) return 50000.0f;                  
    
    return (ref_v * ref_v) / (2.0f * ramp);
}

bool isAxisMoving(int axis) {
    if (axis < 0 || axis >= 6) return false;
    
    LockMotionEngine(); // 統一使用精密手術刀
    bool moving = (axes[axis].target_pos != axes[axis].current_pos) || 
                  (axes[axis].current_vel > 0.0f) || 
                  (axes[axis].target_vel > 0.0f);
    UnlockMotionEngine(); 
    
    return moving;
}

void moveToRelative(int axis, long relativeSteps, float speedSec, float rampRatio = 1.0f) {
    if (axis < 0 || axis >= 6) return;
    float accel = getAxisAccel(axis, rampRatio); // 自動套用舒適加速度
    moveAxisIndependent(axis, relativeSteps, speedSec, accel);
}

void updateHomingLogic() {
    const long J6_PREP_STEPS = -16000; 
    static unsigned long j6DelayStartTime = 0; 
    
    // ==================================================
    // 【新增】：J4 交接給 J6 的避震喘息計時變數
    // ==================================================
    static unsigned long j4DoneTime = 0;
    static bool waitingToStartJ6 = false;

    for (int i = 0; i < 6; i++) {
        
        // ==================================================
        // 【緩起硬停】狀態 1：第一段快速尋找開關 (碰到馬上停)
        // ==================================================
        if (homingState[i] == 1 && JOINT_PINS[i].limitPin != 0) {
            if (digitalRead(JOINT_PINS[i].limitPin) == LIMIT_ACTIVE_STATE[i]) {
                delay(3); 
                if (digitalRead(JOINT_PINS[i].limitPin) == LIMIT_ACTIVE_STATE[i]) {
                    stopAxisInstant(i); // 瞬間硬停！
                    homingState[i] = 2; 
                }
            } 
        }
        
        // ==================================================
        // 【緩起緩停】狀態 2：離開開關的短距離反彈
        // ==================================================
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
                    
                    // 套用 0.2 倍 Ramp：加速距離變短，起步更俐落
                    moveToRelative(i, bounceDist, speedSec, 0.2f); 
                    homingState[i] = 12;
                }
            }
        }
        
        // ==================================================
        // 【緩起硬停】狀態 12：準備進行第二段慢速尋找
        // ==================================================
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
                    
                    // 套用 0.5 倍 Ramp：極短距加速，迅速進入慢速巡航
                    jogAxis(i, dir, slowSpeedSec, getAxisAccel(i, 0.5f), true); 
                    homingState[i] = 13;
                }
            }
        }
        
        // ==================================================
        // 【緩起硬停】狀態 13：第二段慢速尋找開關 (碰到馬上停)
        // ==================================================
        else if (homingState[i] == 13) {
            if (digitalRead(JOINT_PINS[i].limitPin) == LIMIT_ACTIVE_STATE[i]) {
                delay(3); 
                if (digitalRead(JOINT_PINS[i].limitPin) == LIMIT_ACTIVE_STATE[i]) {
                    stopAxisInstant(i); // 瞬間硬停確保精準度！
                    homingState[i] = 14; 
                }
            } 
        }
        
        // ==================================================
        // 【緩起緩停】狀態 14：走到各自的 Home Offset 偏移值
        // ==================================================
        else if (homingState[i] == 14) {
            if (!isAxisMoving(i)) {
                bool readyToOffset = true;
                // J1~J3 必須等待彼此都完成狀態 13 才能一起動
                if (i <= 2) {
                    for (int j = 0; j < 3; j++) {
                        if (homingState[j] == 13 || (homingState[j] == 14 && isAxisMoving(j))) { readyToOffset = false; break; }
                    }
                }
                // J6 專屬：進入 Offset 前的預備退讓動作
                if (i == 5) {
                    readyToOffset = false; 
                    float speedSec = SPEED_CFG[i].maxSpeed;
                    moveToRelative(i, J6_PREP_STEPS, speedSec);
                    homingState[i] = 4; 
                }
                if (i == 4) { readyToOffset = false; homingState[i] = 6; }

                // 執行 J1~J4 的 Offset 運動
                if (readyToOffset) {
                    float offsetSpeedSec = abs(HOMING_CFG[i].homingSpeed) * 2.0f; 
                    if (i == 3) offsetSpeedSec = abs(HOMING_CFG[i].homingSpeed) * 1.5f;
                    long offsetSteps = HOMING_CFG[i].homingPos; 
                    
                    // J1~J4 在走 Offset 時改成 1.5f 讓動作放緩
                    moveToRelative(i, offsetSteps, offsetSpeedSec, 1.5f); 
                    homingState[i] = 3; 
                }
            }
        }
        
        // ==================================================
        // 【緩起硬停】狀態 3：群組等待與觸發後續軸
        // ==================================================
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
                                    jogAxis(k, (HOMING_CFG[k].homingSpeed > 0) ? 1 : -1, speedSec, getAxisAccel(k), true);
                                }
                            }
                        }
                    }
                }
                
                // 【重點修改區】：J4 歸零完成，交接給 J6
                if (i == 3 && homingState[5] == 10) {
                    j4DoneTime = millis();
                    waitingToStartJ6 = true;
                    // 將 J6 推入過渡等待狀態 11，防止重複啟動與在同個迴圈內讀取極限開關
                    homingState[5] = 11; 
                }
            }
        }
        else if (homingState[i] == 4 && i == 5) {
            if (!isAxisMoving(i)) {
                homingState[i] = 5; 
                if (homingState[4] == 10) {
                    homingState[4] = 1;
                    float speedSec = abs(HOMING_CFG[4].homingSpeed);
                    jogAxis(4, (HOMING_CFG[4].homingSpeed > 0) ? 1 : -1, speedSec, getAxisAccel(4), true);
                }
            }
        }
        
        // ==================================================
        // 【緩起緩停】狀態 6 & 15：J5/J6 補償聯動
        // ==================================================
        else if (homingState[i] == 6 && i == 4) {
            // 【安全優化】：如果 J6 在等待 (State 5) 才連動；若無參與歸零 (State 0)，則直接放行
            if (homingState[5] == 5 || homingState[5] == 0) {
                float speedSec = abs(HOMING_CFG[4].homingSpeed) * 1.5f;
                long offsetJ5 = HOMING_CFG[4].homingPos;
                moveToRelative(4, offsetJ5, speedSec, 1.0f);
                homingState[4] = 3; 
                
                if (homingState[5] == 5) {
                    homingState[5] = 15; 
                    j6DelayStartTime = millis(); 
                }
            }
        }
        else if (homingState[i] == 15 && i == 5) {
            // 等待 J5 先行動 200 毫秒
            if (millis() - j6DelayStartTime >= 200) { 
                float speedSec = abs(HOMING_CFG[5].homingSpeed)* 1.5f;
                long offsetJ6 = HOMING_CFG[5].homingPos - J6_PREP_STEPS;
                // 明確加入 1.0f，確保 J6 最終補償步數時會平滑煞車
                moveToRelative(5, offsetJ6, speedSec, 1.0f);
                homingState[5] = 3; 
            }
        }
    }

    // ==================================================
    // 【延遲啟動處理區】：脫離 `for` 迴圈陷阱，安全啟動 J6
    // ==================================================
    if (waitingToStartJ6 && (millis() - j4DoneTime >= 100)) { 
        waitingToStartJ6 = false;
        homingState[5] = 1; // 100ms 雜訊與震波消散後，正式推回尋歸狀態 1
        float speedSec = abs(HOMING_CFG[5].homingSpeed);
        jogAxis(5, (HOMING_CFG[5].homingSpeed > 0) ? 1 : -1, speedSec, getAxisAccel(5), true);
    }

    static bool wasHoming = false;
    bool stillHoming = isAnyHoming(); 
    if (wasHoming && !stillHoming) { Serial.println("HomingDone"); }
    wasHoming = stillHoming;
}