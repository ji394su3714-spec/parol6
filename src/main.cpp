#include <Arduino.h>
#include <SPI.h>
#include <MobaTools.h>

// 引入拆分出來的模組
#include "TMC_RawSPI.h"
#include "config.h"
#include "Globals.h"
#include "Homing.h"
#include "Comms.h"
#include "EndEffector.h" //  1. 引入夾爪模組

// S6 專屬 6 軸腳位矩陣
const JointConfig JOINTS[6] = {
    // Step, Dir,  En,   Limit, ActiveState, homingSpd, homingPos, bounce, ctrlSpd10, maxSpd10, ramp
    {PE11, PE10, PE9,  PB14,  LOW,   1200,  -28,  1000, 48000, 200000, 1400}, // J1 (X)
    {PD8,  PB12, PD9,  PB13,  HIGH, -2400,   50,  2000, 68000, 200000, 2000}, // J2 (Y)
    {PD14, PD13, PD15, PA0,   HIGH,  3000,  -70,  2400, 68000, 200000, 2000}, // J3 (Z)
    {PD5,  PD6,  PD4,  PA1,   LOW,   3800, -144,  1600, 60000, 200000, 1000}, // J4 (E0)
    {PE6,  PC13, PE5,  PA2,   HIGH,  2800, -125,  1200, 60000, 200000, 1000}, // J5 (E1)
    {PE2,  PE4,  PE3,  PA3,   LOW,   5000,    2,  1600, 80000, 200000, 1400}  // J6 (E2)
};

const MotorCurrentConfig MOTOR_CURRENTS[6] = {
    {1300, 0.5f}, //J1
    {1200, 0.75f}, //J2
    {1200, 0.75f}, //J3
    {1000, 0.5f}, //J4
    {1000, 0.5f}, //J5
    {850,  0.5f}  //J6
};

const float GEAR_RATIOS[6] = {6.4, 20.0, 18.095, 4.0, 4.0, 10.0};

// MobaTools 實例 (使用 32 微步基礎: 200 * 32 = 6400)
MoToStepper stepper_J1(6400, STEPDIR);
MoToStepper stepper_J2(6400, STEPDIR);
MoToStepper stepper_J3(6400, STEPDIR);
MoToStepper stepper_J4(6400, STEPDIR);
MoToStepper stepper_J5(6400, STEPDIR);
MoToStepper stepper_J6(6400, STEPDIR); 

MoToStepper* steppers[6] = {&stepper_J1, &stepper_J2, &stepper_J3, &stepper_J4, &stepper_J5, &stepper_J6};

byte homingState[6] = {0, 0, 0, 0, 0, 0}; 
bool normalMoveActive = false;

char receivedChars[NUM_CHARS];
char tempChars[NUM_CHARS];
long receivedSteps[6] = {0};

BufPoint ringBuf[BUF_SIZE];
int bufHead = 0;
int bufTail = 0;
int bufCount = 0;
bool isBufPlaying = false;
bool pendingOK = false;

// 狀態變數
long lastBufTarget[6] = {0}; 
unsigned long lastPointTimeUs = 0;
unsigned long currentPointIntervalUs = 0;
unsigned long firstPacketWaitUs = 0; 

// 核心播放機
void updateRingBuffer() {
    if (bufCount > 0) {
        unsigned long nowUs = micros(); 
        
        // 1. 起步蓄水池：硬體級的完美冷卻防抖
        if (!isBufPlaying) {
            if (firstPacketWaitUs == 0) firstPacketWaitUs = nowUs;
            
            // 如果 0.5 秒內水桶還沒滿 25 點，就繼續等！
            //這給了 Python 足夠的時間來緩衝和計算，確保發車就是滿滿的流暢！
            if (bufCount < 25 && (nowUs - firstPacketWaitUs < 500000)) {
                return;
            }
            firstPacketWaitUs = 0;
            // 發車瞬間，將「數學起點」對齊當前的「物理位置」
            for(int i = 0; i < 6; i++) {
                lastBufTarget[i] = steppers[i]->currentPosition();
            }
            lastPointTimeUs = nowUs;
            currentPointIntervalUs = 0; 
            isBufPlaying = true;
        }

        // 2. 核心定時器：確保每個點都在正確的時間被送出來
        if (nowUs - lastPointTimeUs >= currentPointIntervalUs) {
            
            BufPoint pt = ringBuf[bufTail];
            bufTail = (bufTail + 1) % BUF_SIZE;
            bufCount--;
            Serial.println("OK");

            unsigned long interval = pt.interval_us;
            if (interval < 5000) interval = 5000;

            // 核心：純粹的開環前饋 (Open-Loop Feed-Forward)
            for (int i = 0; i < 6; i++) {
                long target = pt.targetSteps[i];
                
                // 回歸純淨數學：絕對信任理論值！絕對不看 currentPosition()！
                long idealDelta = abs(target - lastBufTarget[i]); 

                if (idealDelta > 0) {
                    unsigned long mobaSpeed = (idealDelta * 10000UL) / (interval / 1000UL); 
                 
                // 配合 S6 與 32 微步，將天花板直接設為 MobaTools 的極限 300000
                if (mobaSpeed > 300000) mobaSpeed = 300000; 

                if (mobaSpeed < 10) mobaSpeed = 10;

                    steppers[i]->setSpeedSteps(mobaSpeed);
                    steppers[i]->setRampLen(0); 
                } 
                
                steppers[i]->writeSteps(target);
                lastBufTarget[i] = target;
            }

            lastPointTimeUs += currentPointIntervalUs; // 推進時鐘
            
            // 防護：如果 Windows 卡頓太久，防止 Arduino 內部時鐘累積過多債務而暴衝
            if (nowUs > lastPointTimeUs + 50000) { 
                lastPointTimeUs = nowUs; 
            }
            currentPointIntervalUs = interval;
        }
        
    } else {
        // 3. 乾淨俐落的收尾
        if (isBufPlaying) {
            bool moving = false;
            for(int i = 0; i < 6; i++) {
                if(steppers[i]->stepsToDo() > 0) moving = true;
            }

            // 只要六顆馬達都真正在物理上停下來了，就宣佈路徑徹底結束
            if (!moving) {
                isBufPlaying = false;
                firstPacketWaitUs = 0; 
            }
        }
    }
}

bool newData = false;

float getStepsPerDeg(int axis) {
    return (MOTOR_STEPS * MICROSTEPS * GEAR_RATIOS[axis]) / 360.0;
}

bool isAnyHoming() {
    for(int i = 0; i < 6; i++) {
        if(homingState[i] != 0) return true;
    }
    return false;
}

void setup() {
    Serial.begin(250000); 
    delay(1000);
    Serial.println("\n--- S6 Controller Booting ---");

    initEndEffector(); //  2. 啟動夾爪

    // 初始化所有 SPI CS 腳位
    const uint8_t all_cs_pins[6] = {X_CS_PIN, Y_CS_PIN, Z_CS_PIN, E0_CS_PIN, E1_CS_PIN, E2_CS_PIN};
    for(int i=0; i<6; i++) {
        pinMode(all_cs_pins[i], OUTPUT);
        digitalWrite(all_cs_pins[i], HIGH);
    }

    // 初始化時序：將 SPI 設定移入迴圈，確保晶片被喚醒後才寫入暫存器
    for (int i = 0; i < 6; i++) {
        // 1. 綁定運動腳位
        steppers[i]->attach(JOINTS[i].stepPin, JOINTS[i].dirPin);
        
        // 2. 先拉低 EN 腳位，把 TMC2240 從休眠中喚醒
        pinMode(JOINTS[i].enPin, OUTPUT);
        digitalWrite(JOINTS[i].enPin, LOW); 
        
        delay(10);// 開機穩定時間
        
        // 3. 寫入 SPI 暫存器，設定電流和方向反轉
        setupTMC2240_RawSPI(all_cs_pins[i], MOTOR_CURRENTS[i].run_mA, MOTOR_CURRENTS[i].hold_ratio, true);

        // 4. 初始化極限開關
        if (JOINTS[i].limitPin != 0) {
            pinMode(JOINTS[i].limitPin, INPUT_PULLUP);
        }
        steppers[i]->setSpeedSteps(JOINTS[i].maxSpeedSteps10);
        steppers[i]->setRampLen(JOINTS[i].rampSteps); 
    }

    Serial.println("<S6 Controller OS Ready>");
}

void loop() {
    updateRingBuffer(); 

    recvWithStartEndMarkers();

    if (newData) {
        strcpy(tempChars, receivedChars);
        if (!parseEndEffectorCmd(tempChars)) {
            processCommand();
        }
        newData = false;
    }

    updateHomingLogic();

    // ==========================================
    //  判斷手臂是否完全空閒 (準備傳給夾爪)
    // ==========================================
    // 1. 檢查六顆馬達在物理上是否還在轉動
    bool isMoving = false; 
    for (int i = 0; i < 6; i++) {
        if (steppers[i]->stepsToDo() != 0) { 
            isMoving = true;
            break; 
        }
    }

    if (normalMoveActive) {
        if (!isMoving) {
            Serial.println("Done");      
            normalMoveActive = false;    
        }
    }

    // 2. 嚴格定義空閒：水庫沒水 + 播放器關閉 + 馬達實體停轉
    bool isArmIdle = (bufCount == 0 && !isBufPlaying && !isMoving);

    // 3. 把空閒狀態傳給夾爪，讓夾爪決定要不要作動
    updateEndEffector(isArmIdle); 
    
    // 溫度讀取安全機制：確保移動中和播放中絕對不進行 SPI 讀取
    static unsigned long lastTempReport = 0;
    if (millis() - lastTempReport >= 30000) {
        lastTempReport = millis();
        
        if (!isBufPlaying && !isMoving) {
            noInterrupts();
            float tempJ1 = readTMC2240Temp(X_CS_PIN);
            float tempJ2 = readTMC2240Temp(Y_CS_PIN);
            float tempJ3 = readTMC2240Temp(Z_CS_PIN);
            float tempJ4 = readTMC2240Temp(E0_CS_PIN);
            float tempJ5 = readTMC2240Temp(E1_CS_PIN);
            float tempJ6 = readTMC2240Temp(E2_CS_PIN);
            interrupts();
            
            Serial.print("[Thermal] J1:"); Serial.print(tempJ1, 1);
            Serial.print("C | J2:"); Serial.print(tempJ2, 1);
            Serial.print("C | J3:"); Serial.print(tempJ3, 1);
            Serial.print("C | J4:"); Serial.print(tempJ4, 1);
            Serial.print("C | J5:"); Serial.print(tempJ5, 1);
            Serial.print("C | J6:"); Serial.print(tempJ6, 1);
            Serial.println("C");
        }
    }
}