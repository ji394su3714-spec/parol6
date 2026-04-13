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

#define R_SENSE_5160 0.075f
//#define R_SENSE_2240 0.075f // 為了滿足函式庫數學運算，必須給 0.075f

#define X_CS_PIN  70 // J1
#define Y_CS_PIN  39 // J2
#define Z_CS_PIN  74 // J3
#define E0_CS_PIN 47 // J4
#define E1_CS_PIN 32 // J5
#define E2_CS_PIN 42 // J6

// 宣告外部常數陣列
extern const JointConfig JOINTS[6];
extern const MotorCurrentConfig MOTOR_CURRENTS[6];
extern const float GEAR_RATIOS[6];

// 全域硬體常數
const byte LED_PIN = 13;
const float MICROSTEPS = 8.0;
const float MOTOR_STEPS = 200.0; 

#endif