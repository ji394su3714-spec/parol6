#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>
#include <MobaTools.h>

// 引入拆分出來的模組
#include "TMC_RawSPI.h"
#include "Config.h"
#include "Globals.h"
#include "Homing.h"
#include "Comms.h"

// 實體變數定義區
/*
    byte stepPin; byte dirPin; byte enPin; byte limitPin; bool limitActiveState;
    float homingSpeed; float homingPos; long bounceSteps; 
    long jointControlSpd10; long maxSpeedSteps10; 
    int rampSteps;     
*/
const JointConfig JOINTS[6] = {
    {54, 55, 38,  2,  LOW,   300, -28,  250, 12000, 65000, 350}, 
    {60, 61, 56, 12,  HIGH, -600,  50,  500, 17000, 65000, 500}, 
    {43, 48, 58, 14,  HIGH,  750, -70,  600, 17000, 65000, 500}, 
    {26, 28, 24, 49,  LOW,   950, -144, 400, 15000, 65000, 250}, 
    {36, 34, 30, 63,  HIGH,  700, -125, 300, 15000, 65000, 250}, 
    {59, 57, 40, 64,  LOW,  1200,    2, 400, 20000, 65000, 350}  
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

TMC5160Stepper driver_J2(Y_CS_PIN, R_SENSE_5160);       

MoToStepper stepper_J1(1600, STEPDIR);
MoToStepper stepper_J2(1600, STEPDIR);
MoToStepper stepper_J3(1600, STEPDIR);
MoToStepper stepper_J4(1600, STEPDIR);
MoToStepper stepper_J5(1600, STEPDIR);
MoToStepper stepper_J6(1600, STEPDIR); 

MoToStepper* steppers[6] = {&stepper_J1, &stepper_J2, &stepper_J3, &stepper_J4, &stepper_J5, &stepper_J6};

long global_hw_max_steps = 65000;
float global_T_acc = 0.2;
byte homingState[6] = {0, 0, 0, 0, 0, 0}; 
bool normalMoveActive = false;

char receivedChars[NUM_CHARS];
char tempChars[NUM_CHARS];
long receivedSteps[6] = {0};

BufPoint ringBuf[BUF_SIZE];
byte bufHead = 0;
byte bufTail = 0;
byte bufCount = 0;
bool isBufPlaying = false;
bool pendingOK = false;

// 狀態變數
long lastBufTarget[6] = {0}; 
unsigned long lastPointTimeUs = 0;
unsigned long currentPointIntervalUs = 0;
unsigned long firstPacketWaitUs = 0; 

// 核心播放機 (極簡純數學開環版)
void updateRingBuffer() {
    if (bufCount > 0) {
        unsigned long nowUs = micros(); 
        
        // 1. 起步蓄水池：硬體級的完美冷卻防抖
        // 蓄滿 100 個封包 (約 1.5 秒的路徑) 才出發，讓 Python 有絕對的餘裕算數學和印 Log！
        if (!isBufPlaying) {
            if (firstPacketWaitUs == 0) firstPacketWaitUs = nowUs;
            
            // 如果水庫裡的水不到 100 滴，且等待時間還沒超過 2 秒，就繼續憋氣
            if (bufCount < 100 && (nowUs - firstPacketWaitUs < 2000000)) {
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

        // 2. 絕對時鐘推進 (嚴格遵守 Python 給的 0.015s 切片節奏)
        if (nowUs - lastPointTimeUs >= currentPointIntervalUs) {
            
            BufPoint pt = ringBuf[bufTail];
            bufTail = (bufTail + 1) % BUF_SIZE;
            bufCount--;

            unsigned long interval = pt.interval_us;
            if (interval < 5000) interval = 5000; // 防呆底線

            // 核心：純粹的開環前饋 (Open-Loop Feed-Forward)
            for (int i = 0; i < 6; i++) {
                long target = pt.targetSteps[i];
                
                // 回歸純淨數學：絕對信任理論值！絕對不看 currentPosition()！
                long idealDelta = abs(target - lastBufTarget[i]); 

                if (idealDelta > 0) {
                    unsigned long mobaSpeed = (idealDelta * 10000UL) / (interval / 1000UL);                  
                    if (mobaSpeed > 65535) mobaSpeed = 65535;
                    if (mobaSpeed < 10) mobaSpeed = 10; 

                    steppers[i]->setSpeedSteps(mobaSpeed);
                    steppers[i]->setRampLen(0); 
                } 
                
                steppers[i]->writeSteps(target);
                lastBufTarget[i] = target;
            }

            // 推進時鐘
            lastPointTimeUs += currentPointIntervalUs;
            
            // 防護：如果 Windows 卡頓太久，防止 Arduino 內部時鐘累積過多債務而暴衝
            if (nowUs > lastPointTimeUs + 50000) { 
                lastPointTimeUs = nowUs; 
            }
            
            currentPointIntervalUs = interval;
            
            // 處理通訊：水桶有空間了，跟 Python 要下一個封包
            if (pendingOK && bufCount < BUF_SIZE) {
                Serial.println("OK");
                pendingOK = false;
            }
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

boolean newData = false;

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
    Serial.println("\n--- System Booting ---");

    pinMode(53, OUTPUT);        digitalWrite(53, HIGH);
    // 初始化所有 SPI CS 腳位，預設為 HIGH
    pinMode(X_CS_PIN, OUTPUT);  digitalWrite(X_CS_PIN, HIGH);
    pinMode(Y_CS_PIN, OUTPUT);  digitalWrite(Y_CS_PIN, HIGH);
    pinMode(Z_CS_PIN, OUTPUT);  digitalWrite(Z_CS_PIN, HIGH);
    pinMode(E0_CS_PIN, OUTPUT); digitalWrite(E0_CS_PIN, HIGH);
    pinMode(E1_CS_PIN, OUTPUT); digitalWrite(E1_CS_PIN, HIGH);
    pinMode(E2_CS_PIN, OUTPUT); digitalWrite(E2_CS_PIN, HIGH);

    SPI.begin();
    pinMode(LED_PIN, OUTPUT);

    driver_J2.begin();
    driver_J2.toff(5);
    driver_J2.rms_current(MOTOR_CURRENTS[1].run_mA, MOTOR_CURRENTS[1].hold_ratio);
    driver_J2.microsteps(8);
    driver_J2.en_pwm_mode(true);
    driver_J2.pwm_autoscale(true);

    // TMC2240 原生 SPI 模組
    setupTMC2240_RawSPI(X_CS_PIN,  MOTOR_CURRENTS[0].run_mA, MOTOR_CURRENTS[0].hold_ratio);
    setupTMC2240_RawSPI(Z_CS_PIN,  MOTOR_CURRENTS[2].run_mA, MOTOR_CURRENTS[2].hold_ratio);
    setupTMC2240_RawSPI(E0_CS_PIN, MOTOR_CURRENTS[3].run_mA, MOTOR_CURRENTS[3].hold_ratio);
    setupTMC2240_RawSPI(E1_CS_PIN, MOTOR_CURRENTS[4].run_mA, MOTOR_CURRENTS[4].hold_ratio);
    setupTMC2240_RawSPI(E2_CS_PIN, MOTOR_CURRENTS[5].run_mA, MOTOR_CURRENTS[5].hold_ratio);

    for (int i = 0; i < 6; i++) {
        steppers[i]->attach(JOINTS[i].stepPin, JOINTS[i].dirPin);
        pinMode(JOINTS[i].enPin, OUTPUT);
        digitalWrite(JOINTS[i].enPin, LOW); 
        if (JOINTS[i].limitPin != 0) {
            pinMode(JOINTS[i].limitPin, INPUT_PULLUP);
        }
        steppers[i]->setSpeedSteps(JOINTS[i].maxSpeedSteps10);
        steppers[i]->setRampLen(JOINTS[i].rampSteps); 
    }

    Serial.println("<F6 Controller OS Ready>");
}

void loop() {
    updateRingBuffer(); 

    recvWithStartEndMarkers();
    if (newData) {
        strcpy(tempChars, receivedChars);
        processCommand();
        newData = false;
    }

    updateHomingLogic();

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

    static unsigned long lastLedToggle = 0;
    const unsigned long LED_BLINK_INTERVAL = 500UL;

    if (isMoving) {
        if (millis() - lastLedToggle >= LED_BLINK_INTERVAL) {
            lastLedToggle = millis();
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        }
    } else {
        digitalWrite(LED_PIN, HIGH);  
    }

    // 監控所有驅動器的溫度狀態，每 30 秒報告一次（前提是目前沒有在執行路徑）
    static unsigned long lastTempReport = 0;
    if (millis() - lastTempReport >= 30000) {
        lastTempReport = millis();
        
        if (!isBufPlaying && !isMoving) {
            String statusJ2 = readTMC5160ThermalStatus(Y_CS_PIN);
            float tempJ1 = readTMC2240Temp(X_CS_PIN);
            float tempJ3 = readTMC2240Temp(Z_CS_PIN);
            float tempJ4 = readTMC2240Temp(E0_CS_PIN);
            float tempJ5 = readTMC2240Temp(E1_CS_PIN);
            float tempJ6 = readTMC2240Temp(E2_CS_PIN);
            
            Serial.print("[Thermal] J1:"); Serial.print(tempJ1, 1);
            Serial.print("C | J2:"); Serial.print(statusJ2);
            Serial.print("C | J3:"); Serial.print(tempJ3, 1);
            Serial.print("C | J4:"); Serial.print(tempJ4, 1);
            Serial.print("C | J5:"); Serial.print(tempJ5, 1);
            Serial.print("C | J6:"); Serial.print(tempJ6, 1);
            Serial.println("C");
        }
    }
}