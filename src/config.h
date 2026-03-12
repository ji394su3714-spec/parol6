#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// 1. 關節配置與硬體參數結構
struct JointConfig {
    byte stepPin; byte dirPin; byte enPin; byte limitPin; bool limitActiveState;
    float homingSpeed; float homingPos; long bounceSteps; 
    long maxSpeedSteps10; int rampSteps;
    uint16_t runCurrent_mA; float holdCurrentRatio;        
};

// 驅動晶片採樣電阻與腳位
#define R_SENSE_2209 0.11f  
#define R_SENSE_5160 0.075f
#define Y_CS_PIN 39

// 宣告外部常數陣列
extern const JointConfig JOINTS[6];
extern const float GEAR_RATIOS[6];

// 全域硬體常數
const byte LED_PIN = 13;
const float MICROSTEPS = 8.0;
const float MOTOR_STEPS = 200.0; 

#endif