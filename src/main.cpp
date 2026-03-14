#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TMCStepper.h>
#include <MobaTools.h>

// 引入拆分出來的模組
#include "Config.h"
#include "Globals.h"
#include "Homing.h"
#include "Comms.h"

// 實體變數定義區 (全專案只有這裡會佔用記憶體)
const JointConfig JOINTS[6] = {
    // step, dir, en, lim, lim_active, h_spd, h_pos, bounce, max_spd, ramp, run_mA, hold_ratio
    {54, 55, 38,  2,  LOW,   300,  -28,  250, 14000, 800, 950, 0.5f}, 
    {60, 61, 56, 12,  HIGH, -600,   50,  450, 17000, 800,  1000, 0.5f}, 
    {43, 48, 58, 14,  HIGH,  750,  -70,  550, 17000, 600,  900, 0.5f}, 
    {26, 28, 24, 15,  LOW,   1000, -145,  400, 17000, 200,  900, 0.25f}, 
    {36, 34, 30, 63,  HIGH,  750,  -124, 300, 17000, 200,  850, 0.25f}, 
    {59, 57, 40, 64,  LOW,  1000,    2,  400, 22000, 300,  680, 0.25f}  
};

const float GEAR_RATIOS[6] = {6.4, 20.0, 18.1, 4.0, 4.0, 10.0};

SoftwareSerial serial_J1(71, 72);
SoftwareSerial serial_J3(78, 79);
SoftwareSerial serial_J4(76, 77);
SoftwareSerial serial_J5(80, 81);

TMC2209Stepper driver_J1(&serial_J1, R_SENSE_2209, 0);
TMC5160Stepper driver_J2(Y_CS_PIN, R_SENSE_5160);       
TMC2209Stepper driver_J3(&serial_J3, R_SENSE_2209, 0);
TMC2209Stepper driver_J4(&serial_J4, R_SENSE_2209, 0);
TMC2209Stepper driver_J5(&serial_J5, R_SENSE_2209, 0);

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
unsigned long lastPointTime = 0;
unsigned long currentPointInterval = 0;

// 核心播放機：獨立於 USB 通訊之外的純硬體執行器
void updateRingBuffer() {
    if (bufCount > 0) {
        unsigned long now = millis();
        // 提早 2 毫秒換檔 (0.95 滯後係數的終極版)，確保無縫接軌！
        if (!isBufPlaying || (now - lastPointTime >= (currentPointInterval > 2 ? currentPointInterval - 2 : 0))) {
            
            // 1. 從水桶拿出一個點
            BufPoint pt = ringBuf[bufTail];
            bufTail = (bufTail + 1) % BUF_SIZE;
            bufCount--;

            float maxTime = pt.interval_ms / 1000.0;
            if (maxTime < 0.01) maxTime = 0.01;

            long deltaSteps[6] = {0};
            for (int i = 0; i < 6; i++) {
                deltaSteps[i] = abs(pt.targetSteps[i] - steppers[i]->currentPosition());
            }

            // 防爆網：等比例拉長時間 (保護 LIN 軌跡不骨折！)
            // 如果 Python 送來的這個切片太過激進，強迫拉長這段切片的播放時間
            for (int i = 0; i < 6; i++) {
                if (deltaSteps[i] > 0) {
                    float minSafeTime = deltaSteps[i] / (JOINTS[i].maxSpeedSteps10 / 10.0);
                    if (minSafeTime > maxTime) {
                        maxTime = minSafeTime;
                    }
                }
            }

            // 3. 算速度並發車 
            for (int i = 0; i < 6; i++) {
                if (deltaSteps[i] > 0) {
                    float syncStepsPerSec = deltaSteps[i] / maxTime;
                    long mobaSpeed = (long)(syncStepsPerSec * 10.0);
                    if (mobaSpeed < 1) mobaSpeed = 1;

                    steppers[i]->setSpeedSteps(mobaSpeed);
                    steppers[i]->setRampLen(0); // LIN 是連續的，把 Ramp 交給硬體時間接管
                    steppers[i]->writeSteps(pt.targetSteps[i]);
                }
            }

            // 4. 更新計時器
            lastPointTime = now;
            currentPointInterval = (unsigned long)(maxTime * 1000.0);
            isBufPlaying = true;

            // 5. 水桶有空位了！把扣留的 OK 釋放，叫 Python 繼續塞資料
            if (pendingOK && bufCount < BUF_SIZE) {
                Serial.println("OK");
                pendingOK = false;
            }
        }
    } else {
        if (isBufPlaying) {
            bool moving = false;
            for(int i=0; i<6; i++) if(steppers[i]->moving()) moving = true;
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
    SPI.begin();
    pinMode(LED_PIN, OUTPUT);

    serial_J1.begin(115200);
    serial_J3.begin(115200);
    serial_J4.begin(115200);
    serial_J5.begin(115200);

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

    setupTMC2209(driver_J1, JOINTS[0].runCurrent_mA, JOINTS[0].holdCurrentRatio);
    setupTMC2209(driver_J3, JOINTS[2].runCurrent_mA, JOINTS[2].holdCurrentRatio);
    setupTMC2209(driver_J4, JOINTS[3].runCurrent_mA, JOINTS[3].holdCurrentRatio);
    setupTMC2209(driver_J5, JOINTS[4].runCurrent_mA, JOINTS[4].holdCurrentRatio);

    driver_J2.begin();
    driver_J2.toff(5);
    driver_J2.rms_current(JOINTS[1].runCurrent_mA, JOINTS[1].holdCurrentRatio); 
    driver_J2.microsteps(8);
    driver_J2.en_pwm_mode(true);
    driver_J2.pwm_autoscale(true);

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
}