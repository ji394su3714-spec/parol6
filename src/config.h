#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
// 1. 輕量化結構定義
// ==========================================

// 1. 腳位群
struct JointPins {
    byte stepPin; byte dirPin; byte enPin; byte limitPin;
};

// 2. 歸零群
struct HomingConfig {
    float homingSpeed; long homingPos; long bounceSteps; int rampSteps;
};

// 3. 速度群
struct SpeedConfig {
    float controlSpeed; float maxSpeed;    
};

// 4. 電流群
struct MotorCurrentConfig {
    uint16_t run_mA; float hold_ratio;
};

// ==========================================
// 2. 核心硬體常數
// ==========================================
constexpr float MICROSTEPS = 32.0f; 
constexpr float MOTOR_STEPS = 200.0f; 

#define X_CS_PIN  PE7
#define Y_CS_PIN  PE15
#define Z_CS_PIN  PD10
#define E0_CS_PIN PD7
#define E1_CS_PIN PC14
#define E2_CS_PIN PC15

// ==========================================
// 3. 全域平行陣列宣告 (Structure of Arrays)
// ==========================================
extern const JointPins JOINT_PINS[6];
extern const bool LIMIT_ACTIVE_STATE[6]; 
extern const HomingConfig HOMING_CFG[6];
extern const SpeedConfig SPEED_CFG[6];

extern const MotorCurrentConfig MOTOR_CURRENTS[6];
extern const int32_t AXIS_MAX_LIMIT[6];
extern const int32_t AXIS_MIN_LIMIT[6];

#endif