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
    // step, dir, en, lim, lim_active, h_spd, h_pos, bounce, j_ctrl_spd, max_spd, ramp
    {54, 55, 38,  2,  LOW,   300, -28,  250, 12000, 50000, 300}, // J1 (2209)
    {60, 61, 56, 12,  HIGH, -600,  50,  450, 17000, 50000, 400}, // J2 (5160)
    {43, 48, 58, 14,  HIGH,  750, -70,  650, 17000, 50000, 400}, // J3 (2209)
    {26, 28, 24, 15,  LOW,   900, -145, 400, 15000, 50000, 200}, // J4 (2209)
    {36, 34, 30, 63,  HIGH,  700, -124, 300, 15000, 50000, 200}, // J5 (2209)
    {59, 57, 40, 64,  LOW,   900,    2, 400, 20000, 50000, 200}  // J6 (2240)
};

// 電流設定陣列
const MotorCurrentConfig MOTOR_CURRENTS[6] = {
    // run_mA, hold_ratio
    {1100,  0.4f}, // J1
    {1100,  0.7f}, // J2
    {900,   0.7f}, // J3
    {900,  0.25f}, // J4
    {900,  0.25f}, // J5
    {750,  0.25f}  // J6
};

const float GEAR_RATIOS[6] = {6.4, 20.0, 18.1, 4.0, 4.0, 10.0};

// 宣告 UART 通訊埠 (J1, J3, J4)
SoftwareSerial serial_J1(71, 72);
SoftwareSerial serial_J3(78, 79);
SoftwareSerial serial_J4(76, 77);

// 宣告驅動器物件 (J6 由 Bare-Metal SPI 接管，無需物件)
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

// 實體化環形緩衝區變數
BufPoint ringBuf[BUF_SIZE];
byte bufHead = 0;
byte bufTail = 0;
byte bufCount = 0;
bool isBufPlaying = false;
bool pendingOK = false;

// 實體化新的計時變數
long lastBufTarget[6] = {0}; 
unsigned long lastPointTimeUs = 0;
unsigned long currentPointIntervalUs = 0;

// 核心播放機：絕對座標微超速追跡版 (純理論值，無碎震)
void updateRingBuffer() {
    if (bufCount > 0) {
        unsigned long nowUs = micros(); 
        
        if (!isBufPlaying) {
            for(int i = 0; i < 6; i++) {
                lastBufTarget[i] = steppers[i]->currentPosition();
            }
            lastPointTimeUs = nowUs;
            currentPointIntervalUs = 0; 
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

            // 防爆網
            for (int i = 0; i < 6; i++) {
                if (deltaSteps[i] > 0) {
                    float minSafeTime = deltaSteps[i] / (JOINTS[i].maxSpeedSteps10 / 10.0);
                    if (minSafeTime > maxTime) {
                        maxTime = minSafeTime;
                    }
                }
            }

            // 算速度並發車 
            for (int i = 0; i < 6; i++) {
                if (deltaSteps[i] > 0) {
                    float syncStepsPerSec = (deltaSteps[i] / maxTime) * 1.0; 
                    long mobaSpeed = (long)(syncStepsPerSec * 10.0 + 0.5);                    
                    if (mobaSpeed < 100) mobaSpeed = 100;

                    steppers[i]->setSpeedSteps(mobaSpeed);
                    steppers[i]->setRampLen(0); 
                    steppers[i]->writeSteps(pt.targetSteps[i]);
                }
                lastBufTarget[i] = pt.targetSteps[i];
            }

            lastPointTimeUs = nowUs;
            currentPointIntervalUs = (unsigned long)(maxTime * 1000000.0);
            isBufPlaying = true;

            if (pendingOK && bufCount < BUF_SIZE) {
                Serial.println("OK");
                pendingOK = false;
            }
        }
    } else {
        if (isBufPlaying) {
            bool moving = false;
            for(int i = 0; i < 6; i++) if(steppers[i]->moving()) moving = true;
            if (!moving) isBufPlaying = false;
        }
    }
}

boolean newData = false;

// 輔助函式
float getStepsPerDeg(int axis) {
    return (MOTOR_STEPS * MICROSTEPS * GEAR_RATIOS[axis]) / 360.0;
}

bool isAnyHoming() {
    for(int i = 0; i < 6; i++) {
        if(homingState[i] != 0) return true;
    }
    return false;
}

// 啟動與主迴圈
void setup() {
    Serial.begin(250000); 
    delay(1000);
    Serial.println("\n--- System Booting ---");

// 預防 SPI 當機：把所有的 CS 腳位在 SPI 啟動前強制拉高
    pinMode(53, OUTPUT);        digitalWrite(53, HIGH);
    pinMode(Y_CS_PIN, OUTPUT);  digitalWrite(Y_CS_PIN, HIGH);
    pinMode(E1_CS_PIN, OUTPUT); digitalWrite(E1_CS_PIN, HIGH);
    pinMode(E2_CS_PIN, OUTPUT); digitalWrite(E2_CS_PIN, HIGH);

    SPI.begin();
    pinMode(LED_PIN, OUTPUT);

    serial_J1.begin(115200);
    serial_J3.begin(115200);
    serial_J4.begin(115200);

    // 驅動TMC2209初始化
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

    //Serial.println("[Init] Configuring UART Drivers (J1, J3, J4)...");
    setupTMC2209(driver_J1, MOTOR_CURRENTS[0].run_mA, MOTOR_CURRENTS[0].hold_ratio);
    setupTMC2209(driver_J3, MOTOR_CURRENTS[2].run_mA, MOTOR_CURRENTS[2].hold_ratio);
    setupTMC2209(driver_J4, MOTOR_CURRENTS[3].run_mA, MOTOR_CURRENTS[3].hold_ratio);

    //Serial.println("[Init] Configuring J2 (5160 SPI)...");
    driver_J2.begin();
    driver_J2.toff(5);
    driver_J2.rms_current(MOTOR_CURRENTS[1].run_mA, MOTOR_CURRENTS[1].hold_ratio);
    driver_J2.microsteps(8);
    driver_J2.en_pwm_mode(true);
    driver_J2.pwm_autoscale(true);

    //Serial.println("[Init] Configuring J5 & J6 (2240 Bare-Metal SPI)...");
    setupTMC2240_RawSPI(E1_CS_PIN, MOTOR_CURRENTS[4].run_mA, MOTOR_CURRENTS[4].hold_ratio);
    setupTMC2240_RawSPI(E2_CS_PIN, MOTOR_CURRENTS[5].run_mA, MOTOR_CURRENTS[5].hold_ratio);

    //Serial.println("[Init] Attaching Motors...");
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

    updateRingBuffer(); // 每微秒都在檢查要不要換檔

    recvWithStartEndMarkers();
    if (newData) {
        strcpy(tempChars, receivedChars);
        processCommand();
        newData = false;
    }

    updateHomingLogic();

    bool isMoving = false; 
    for (int i = 0; i < 6; i++) {
        if (steppers[i]->moving()) {
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

    // 每隔 5 秒鐘，印出溫度狀態
    static unsigned long lastTempReport = 0;
    if (millis() - lastTempReport >= 5000) {
        lastTempReport = millis();
        
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