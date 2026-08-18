#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>

// ==========================================
// 1. 巨集定義 (Macros)
// ==========================================
#define NUM_CHARS 128
#define SGN(x) (((x) > 0) - ((x) < 0)) // 取得正負號

// ==========================================
// 2. 列舉與結構定義 (Enums & Structs)
// ==========================================
enum HomingPhase : byte {
    HOME_IDLE = 0,
    
    // 基礎尋歸動作
    HOME_FAST_SEARCH = 1,
    HOME_BOUNCE = 2,
    HOME_WAIT_BOUNCE = 3,
    HOME_SLOW_SEARCH = 4,
    HOME_START_OFFSET = 5,  
    HOME_DONE = 6,
    
    // 聯動與群組等待狀態
    HOME_WAIT_J123 = 10,
    HOME_WAIT_J4 = 11,
    HOME_WAIT_J6_PREP = 12, //J5 等待 J6 到達預備位置
    
    // J5/J6 專屬防撞交響曲
    HOME_J6_WAIT_PREP_DONE = 20, 
    HOME_J6_WAIT_J5 = 21,
    HOME_J5_WAIT_J6 = 22,
    HOME_J6_FINAL_OFFSET = 23
};

struct JointPins {
    byte stepPin; byte dirPin; byte enPin; byte limitPin;
};

struct HomingConfig {
    float homingSpeed; long homingPos; long bounceSteps; 
};

struct SpeedConfig {
    float controlSpeed; float maxSpeed; int rampSteps;
};

struct MotorCurrentConfig {
    uint16_t run_mA; float hold_ratio;
};

// ==========================================
// 3. 核心硬體常數與設定 (Constants & Configs)
// ==========================================
extern const JointPins JOINT_PINS [6];
extern const bool LIMIT_ACTIVE_STATE [6]; 
extern const HomingConfig HOMING_CFG [6];
extern const SpeedConfig SPEED_CFG [6];
extern const MotorCurrentConfig MOTOR_CURRENTS [6];
extern const int32_t AXIS_MAX_LIMIT [6];
extern const int32_t AXIS_MIN_LIMIT [6];

// ==========================================
// 4. 共用狀態變數 (Global Variables)
// ==========================================
extern byte homingState [6]; 
extern bool normalMoveActive ;

extern char receivedChars [NUM_CHARS];
extern char tempChars [NUM_CHARS];
extern long receivedSteps [6];
extern bool newData ;

extern volatile bool is_paused ;

// ==========================================
// 5. 共用輔助函式宣告 (Function Prototypes)
// ==========================================
bool isAnyHoming(); 
float getAxisAccel(int axis, float rampRatio = 1.0f); 
void updateHomingLogic(); 
bool startHomingSequence(long targets[6]);

#endif 