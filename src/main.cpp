#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TMCStepper.h>
#include <MobaTools.h>

// 引入拆分出來的模組
#include "TMC_RawSPI.h"
#include "Config.h"
#include "Globals.h"
#include "Homing.h"
#include "Comms.h"

// 實體變數定義區
const JointConfig JOINTS[6] = {
    {54, 55, 38,  2,  LOW,   300, -28,  250, 12000, 50000, 350}, 
    {60, 61, 56, 12,  HIGH, -600,  50,  450, 17000, 50000, 350}, 
    {43, 48, 58, 14,  HIGH,  750, -70,  650, 17000, 50000, 350}, 
    {26, 28, 24, 15,  LOW,   950, -145, 400, 15000, 50000, 350}, 
    {36, 34, 30, 63,  HIGH,  700, -124, 300, 15000, 50000, 350}, 
    {59, 57, 40, 64,  LOW,   900,    2, 400, 20000, 50000, 350}  //<--在gui進階選項裡可調整(maxSpeedSteps10)
};

const MotorCurrentConfig MOTOR_CURRENTS[6] = {
    {1100, 0.25f}, //J1
    {1100, 0.75f}, //J2
    {900,  0.75f}, //J3
    {900,  0.25f}, //J4
    {900,  0.25f}, //J5
    {750,  0.25f}  //J6
};

const float GEAR_RATIOS[6] = {6.4, 20.0, 18.1, 4.0, 4.0, 10.0};

SoftwareSerial serial_J1(71, 72);
SoftwareSerial serial_J3(78, 79);
SoftwareSerial serial_J4(76, 77);

TMC2209Stepper driver_J1(&serial_J1, R_SENSE_2209, 0);
TMC5160Stepper driver_J2(Y_CS_PIN, R_SENSE_5160);       
TMC2209Stepper driver_J3(&serial_J3, R_SENSE_2209, 0);
TMC2209Stepper driver_J4(&serial_J4, R_SENSE_2209, 0);

MoToStepper stepper_J1(1600, STEPDIR);
MoToStepper stepper_J2(1600, STEPDIR);
MoToStepper stepper_J3(1600, STEPDIR);
MoToStepper stepper_J4(1600, STEPDIR);
MoToStepper stepper_J5(1600, STEPDIR);
MoToStepper stepper_J6(1600, STEPDIR); 

MoToStepper* steppers[6] = {&stepper_J1, &stepper_J2, &stepper_J3, &stepper_J4, &stepper_J5, &stepper_J6};

byte homingState[6] = {0, 0, 0, 0, 0, 0}; 
bool normalMoveActive = false;

char receivedChars[NUM_CHARS];
char tempChars[NUM_CHARS];
float receivedAngles[6] = {0.0};

BufPoint ringBuf[BUF_SIZE];
byte bufHead = 0;
byte bufTail = 0;
byte bufCount = 0;
bool isBufPlaying = false;
bool pendingOK = false;

// 修正版計時與補償變數
long lastBufTarget[6] = {0}; 
unsigned long lastPointTimeUs = 0;
unsigned long currentPointIntervalUs = 0;
float speedRemainder[6] = {0.0}; 
unsigned long firstPacketWaitUs = 0; // 新增：蓄水池計時器

// 核心播放機：
void updateRingBuffer() {
    if (bufCount > 0) {
        unsigned long nowUs = micros(); 
        
        if (!isBufPlaying) {
            // 優化 ：工業級蓄水池 (Pre-buffering)
            // 強制等待水桶裝滿至少 4 個切片，或者等待超過 50 毫秒才發車！
            // 這樣能徹底吸收 Windows 的通訊浮動，告別斷流卡頓！
            if (firstPacketWaitUs == 0) firstPacketWaitUs = nowUs;

            if (bufCount < 4 && (nowUs - firstPacketWaitUs < 50000)) {
                return; // 繼續憋氣蓄水，不准發車
            }

            // 水蓄夠了，準備發車！
            firstPacketWaitUs = 0;
            for(int i = 0; i < 6; i++) {
                lastBufTarget[i] = steppers[i]->currentPosition();
                speedRemainder[i] = 0.0; 
            }
            lastPointTimeUs = nowUs;
            currentPointIntervalUs = 0; 
            isBufPlaying = true;
        }

        if (nowUs - lastPointTimeUs >= currentPointIntervalUs) {
            
            BufPoint pt = ringBuf[bufTail];
            bufTail = (bufTail + 1) % BUF_SIZE;
            bufCount--;

            float maxTime = pt.interval_us / 1000000.0;
            if (maxTime < 0.005) maxTime = 0.005;

            long deltaSteps[6] = {0};
            for (int i = 0; i < 6; i++) {
                deltaSteps[i] = abs(pt.targetSteps[i] - lastBufTarget[i]); 
            }

            for (int i = 0; i < 6; i++) {
                if (deltaSteps[i] > 0) {
                    float minSafeTime = deltaSteps[i] / (JOINTS[i].maxSpeedSteps10 / 10.0);
                    if (minSafeTime > maxTime) {
                        maxTime = minSafeTime;
                    }
                }
            }

            for (int i = 0; i < 6; i++) {
                if (deltaSteps[i] > 0) {
                    float syncStepsPerSec = (deltaSteps[i] / maxTime); 
                    
                    float exactSpeed10 = (syncStepsPerSec * 10.0) + speedRemainder[i];
                    long mobaSpeed = (long)exactSpeed10;
                    
                    speedRemainder[i] = exactSpeed10 - (float)mobaSpeed; 

                    if (mobaSpeed < 10) mobaSpeed = 10; 

                    steppers[i]->setSpeedSteps(mobaSpeed);
                    steppers[i]->setRampLen(0); 
                    steppers[i]->writeSteps(pt.targetSteps[i]);
                }
                lastBufTarget[i] = pt.targetSteps[i];
            }

            lastPointTimeUs += currentPointIntervalUs;
            if (nowUs > lastPointTimeUs + 50000) { 
                lastPointTimeUs = nowUs; 
            }
            
            currentPointIntervalUs = (unsigned long)(maxTime * 1000000.0);
            
            if (pendingOK && bufCount < BUF_SIZE) {
                Serial.println("OK");
                pendingOK = false;
            }
        }
    } else {
        if (isBufPlaying) {
            bool moving = false;
            for(int i = 0; i < 6; i++) if(steppers[i]->moving()) moving = true;
            if (!moving) {
                isBufPlaying = false;
                firstPacketWaitUs = 0; // 重置蓄水池
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
    pinMode(Y_CS_PIN, OUTPUT);  digitalWrite(Y_CS_PIN, HIGH);
    pinMode(E1_CS_PIN, OUTPUT); digitalWrite(E1_CS_PIN, HIGH);
    pinMode(E2_CS_PIN, OUTPUT); digitalWrite(E2_CS_PIN, HIGH);

    SPI.begin();
    pinMode(LED_PIN, OUTPUT);

    serial_J1.begin(115200);
    serial_J3.begin(115200);
    serial_J4.begin(115200);

    auto setupTMC2209 = [](TMC2209Stepper &drv, uint16_t mA, float hold_ratio) {
        drv.begin();
        drv.pdn_disable(true);     
        drv.I_scale_analog(false); 
        drv.toff(5);               
        drv.rms_current(mA, hold_ratio); 
        drv.microsteps(8); 
        drv.en_spreadCycle(false); 
        drv.pwm_autoscale(true);   
        drv.TCOOLTHRS(0); 
    };

    setupTMC2209(driver_J1, MOTOR_CURRENTS[0].run_mA, MOTOR_CURRENTS[0].hold_ratio);
    setupTMC2209(driver_J3, MOTOR_CURRENTS[2].run_mA, MOTOR_CURRENTS[2].hold_ratio);
    setupTMC2209(driver_J4, MOTOR_CURRENTS[3].run_mA, MOTOR_CURRENTS[3].hold_ratio);

    driver_J2.begin();
    driver_J2.toff(5);
    driver_J2.rms_current(MOTOR_CURRENTS[1].run_mA, MOTOR_CURRENTS[1].hold_ratio);
    driver_J2.microsteps(8);
    driver_J2.en_pwm_mode(true);
    driver_J2.pwm_autoscale(true);

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

    // 彩蛋解法：0 毫秒延遲 + 絕對防搶拍
    bool isMoving = false; 
    for (int i = 0; i < 6; i++) {
        // 只要還有任何一步沒走完，就代表任務還在進行中！
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

    // 每隔 15 秒鐘，印出溫度狀態
    static unsigned long lastTempReport = 0;
    if (millis() - lastTempReport >= 15000) {
        lastTempReport = millis();
        
        if (!isBufPlaying && !isMoving) {
            String statusJ2 = readTMC5160ThermalStatus(Y_CS_PIN);
            float tempJ5 = readTMC2240Temp(E1_CS_PIN);
            float tempJ6 = readTMC2240Temp(E2_CS_PIN);
            Serial.print("[Thermal] J2(5160): ");
            Serial.print(statusJ2);
            Serial.print("  |  J5(2240): ");
            Serial.print(tempJ5, 1);
            Serial.print(" °C  |  J6(2240): ");
            Serial.print(tempJ6, 1);
            Serial.println(" °C");
        }
    }
}