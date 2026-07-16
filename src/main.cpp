#include <Arduino.h>
#include <SPI.h>

// 引入拆分出來的模組
#include "TMC_RawSPI.h"
#include "Config.h"
#include "Globals.h"
#include "Homing.h"
#include "Comms.h"
#include "MotionEngine.h"
#include "EndEffector.h" 

// 1. 腳位定義 (Step, Dir, En, Limit)
const JointPins JOINT_PINS[6] = {
    {PE11, PE10, PE9,  PB14}, // J1
    {PD8,  PB12, PD9,  PB13}, // J2
    {PD14, PD13, PD15, PA0 }, // J3
    {PD5,  PD6,  PD4,  PA1 }, // J4
    {PE6,  PC13, PE5,  PA2 }, // J5
    {PE2,  PE4,  PE3,  PA3 }  // J6
};

// 2. 極限觸發邏輯 (ActiveState)
const bool LIMIT_ACTIVE_STATE[6] = {
    LOW, HIGH, HIGH, LOW, HIGH, LOW
};

// 3. 歸零參數 (homingSpd, homingPos, bounce, ramp)
const HomingConfig HOMING_CFG[6] = {
    { 1200, -118, 1000, 1200}, // J1
    {-2400,   50, 2000, 1400}, // J2
    { 3000,  -70, 2400, 1400}, // J3
    { 3800, -144, 1600, 1000}, // J4
    { 2800, -125, 1200, 1000}, // J5
    { 5000,    2, 1600, 1200}  // J6
};

// 4. 馬達速度 (ctrlSpd10, maxSpd10)
const SpeedConfig SPEED_CFG[6] = {
    {48000, 200000}, // J1
    {68000, 200000}, // J2
    {68000, 200000}, // J3
    {60000, 200000}, // J4
    {60000, 200000}, // J5
    {80000, 200000}  // J6
};

// 5. 馬達電流 (run_mA, hold_ratio)
const MotorCurrentConfig MOTOR_CURRENTS[6] = {
    {1300, 0.5f}, {1200, 0.75f}, {1200, 0.75f}, 
    {1000, 0.5f}, {1000, 0.5f}, {850,  0.5f} 
};

const float GEAR_RATIOS[6] = {6.4, 20.0, 18.095, 4.0, 4.0, 10.0};

const int32_t AXIS_MAX_LIMIT[6] = { 29013,  24889,  22518,  13156,  7822,  37333};
const int32_t AXIS_MIN_LIMIT[6] = {-8533, -17778, -22518, -6756, -7822, -21333};

// ==========================================
// 2. 全域狀態變數
// ==========================================
char receivedChars[NUM_CHARS];
char tempChars[NUM_CHARS];
long receivedSteps[6] = {0};
bool newData = false;
bool normalMoveActive = false;

byte homingState[6] = {0, 0, 0, 0, 0, 0};

// 輔助函式
float getStepsPerDeg(int axis) {
    return (MOTOR_STEPS * MICROSTEPS * GEAR_RATIOS[axis]) / 360.0f;
}

bool isAnyHoming() {
    for(int i = 0; i < 6; i++) {
        if(homingState[i] != 0) return true;
    }
    return false;
}


// ==========================================
// 3. 主程式 Setup & Loop
// ==========================================
void setup() {
    Serial.begin(250000); 
    delay(1000);
    Serial.println("\n--- S6 Controller Booting ---");

    initEndEffector();

    // 啟動 TMC2240 SPI 暫存器配置
    const uint8_t all_cs_pins[6] = {X_CS_PIN, Y_CS_PIN, Z_CS_PIN, E0_CS_PIN, E1_CS_PIN, E2_CS_PIN};
    for(int i = 0; i < 6; i++) {
        pinMode(all_cs_pins[i], OUTPUT);
        digitalWrite(all_cs_pins[i], HIGH);
        
        pinMode(JOINT_PINS[i].enPin, OUTPUT);
        digitalWrite(JOINT_PINS[i].enPin, LOW); // 喚醒驅動器
        delay(10);
        
        // 傳入 true，硬體接管方向反轉
        setupTMC2240_RawSPI(all_cs_pins[i], MOTOR_CURRENTS[i].run_mA, MOTOR_CURRENTS[i].hold_ratio, true);
        
        if (JOINT_PINS[i].limitPin != 0) {
            pinMode(JOINT_PINS[i].limitPin, INPUT_PULLUP);
        }
    }
    
    // 區域陣列化：傳給引擎後即刻釋放記憶體，保持全域乾淨
    const uint8_t engine_step_pins[6] = {JOINT_PINS[0].stepPin, JOINT_PINS[1].stepPin, JOINT_PINS[2].stepPin,
                                         JOINT_PINS[3].stepPin, JOINT_PINS[4].stepPin, JOINT_PINS[5].stepPin};
    const uint8_t engine_dir_pins[6]  = {JOINT_PINS[0].dirPin, JOINT_PINS[1].dirPin, JOINT_PINS[2].dirPin,
                                         JOINT_PINS[3].dirPin, JOINT_PINS[4].dirPin, JOINT_PINS[5].dirPin};

    // 啟動雙核心運動引擎
    Init_MotionEngine(engine_step_pins, engine_dir_pins);

    Serial.println("<S6 Controller OS Ready>");
}

void loop() {
    recvWithStartEndMarkers();

    if (newData) {
        strcpy(tempChars, receivedChars);
        if (!parseEndEffectorCmd(tempChars)) {
            processCommand(); 
        }
        newData = false;
    }

    updateHomingLogic(); 

    // 判斷是否完全空閒 (傳給夾爪與回報 Done)
    bool isArmIdle = !isEngineMoving();
    updateEndEffector(isArmIdle); 
    
    if (normalMoveActive && isArmIdle) {
        Serial.println("Done");      
        normalMoveActive = false;    
    }
    
    // 溫度讀取安全機制
    static unsigned long lastTempReport = 0;
    if (millis() - lastTempReport >= 30000) {
        lastTempReport = millis();
        
        if (isArmIdle) { 
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