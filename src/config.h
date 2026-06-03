#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// 關節配置與硬體參數結構
struct JointConfig {
    byte stepPin; byte dirPin; byte enPin; byte limitPin; bool limitActiveState;
    float homingSpeed; float homingPos; long bounceSteps; 
    long jointControlSpd10; long maxSpeedSteps10; 
    int rampSteps;      
};

// 馬達電流配置結構
struct MotorCurrentConfig {
    uint16_t run_mA; 
    float hold_ratio;
};

// 🌟 S6 全 6 軸 CS 腳位定義 (對應 X, Y, Z, E0, E1, E2)
#define X_CS_PIN  PE7
#define Y_CS_PIN  PE15
#define Z_CS_PIN  PD10
#define E0_CS_PIN PD7
#define E1_CS_PIN PC14
#define E2_CS_PIN PC15

// 宣告外部常數陣列
extern const JointConfig JOINTS[6];
extern const MotorCurrentConfig MOTOR_CURRENTS[6];
extern const float GEAR_RATIOS[6];

// 全域硬體常數
const float MICROSTEPS = 32.0; // 🌟 配合 TMC2240 改為 32 微步
const float MOTOR_STEPS = 200.0; 

#endif