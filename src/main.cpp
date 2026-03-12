#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TMCStepper.h>
#include <MobaTools.h>

// 引入我們拆分出來的模組
#include "Config.h"
#include "Globals.h"
#include "Homing.h"
#include "Comms.h"

// ==========================================
// 實體變數定義區 (全專案只有這裡會佔用記憶體)
// ==========================================

const JointConfig JOINTS[6] = {
    // step, dir, en, lim, lim_active, h_spd, h_pos, bounce, max_spd, ramp, run_mA, hold_ratio
    {54, 55, 38,  2,  LOW,   300,  -30,  250, 14000, 800, 1000, 0.5f}, 
    {60, 61, 56, 12,  HIGH, -600,   54,  450, 17000, 600,  950,  0.6f}, 
    {43, 48, 58, 14,  HIGH,  750,  -70,  550, 17000, 600,  850,  0.5f}, 
    {26, 28, 24, 15,  LOW,   1000, -145,  400, 14000, 300,  850, 0.25f}, 
    {36, 34, 30, 63,  HIGH, 1000,  -125, 400, 14000, 300,  850, 0.25f}, 
    {59, 57, 40, 64,  LOW,  1400,    2,  400, 22000, 300,  680, 0.25f}  
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
boolean newData = false;

// --- 輔助函式實作 ---
float getStepsPerDeg(int axis) {
    return (MOTOR_STEPS * MICROSTEPS * GEAR_RATIOS[axis]) / 360.0;
}

bool isAnyHoming() {
    for(int i = 0; i < 6; i++) {
        if(homingState[i] != 0) return true;
    }
    return false;
}

// ==========================================
// 🚀 啟動與主迴圈
// ==========================================
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