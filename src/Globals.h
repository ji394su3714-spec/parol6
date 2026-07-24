#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>

// ==========================================
// 1. 輕量化結構定義
// ==========================================
struct JointPins {
    byte stepPin; byte dirPin; byte enPin; byte limitPin;
};

struct HomingConfig {
    float homingSpeed; long homingPos; long bounceSteps; int rampSteps;
};

struct SpeedConfig {
    float controlSpeed; float maxSpeed;    
};

struct MotorCurrentConfig {
    uint16_t run_mA; float hold_ratio;
};

// ==========================================
// 2. 核心硬體常數
// ==========================================
//constexpr float MICROSTEPS = 32.0f; 
//constexpr float MOTOR_STEPS = 200.0f; 

// ==========================================
// 3. 全域平行陣列宣告
// ==========================================
extern const JointPins JOINT_PINS[6];
extern const bool LIMIT_ACTIVE_STATE[6]; 
extern const HomingConfig HOMING_CFG[6];
extern const SpeedConfig SPEED_CFG[6];
extern const MotorCurrentConfig MOTOR_CURRENTS[6];
extern const int32_t AXIS_MAX_LIMIT[6];
extern const int32_t AXIS_MIN_LIMIT[6];

// ==========================================
// 4. 共用狀態變數
// ==========================================
extern byte homingState[6]; 
extern bool normalMoveActive;

// ==========================================
// 5. UART 通訊解析暫存區
// ==========================================
#define NUM_CHARS 128
extern char receivedChars[];
extern char tempChars[];
extern long receivedSteps[6];
extern bool newData;

// ==========================================
// 6. 共用輔助函式宣告
// ==========================================
bool isAnyHoming(); 
float getAxisAccel(int axis, float rampRatio = 1.0f); 
void updateHomingLogic(); 

#endif