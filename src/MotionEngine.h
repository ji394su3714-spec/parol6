#ifndef MOTION_ENGINE_H
#define MOTION_ENGINE_H

#include <Arduino.h>

#define MOTION_BUF_SIZE 200

// 1. 核心資料結構
struct BufPoint {
    int32_t targetSteps[6];
    uint32_t interval_us; 
};

struct AxisState {
    volatile float current_vel;     // 當前速度 (硬體斜坡用)
    volatile float target_vel;      // 目標速度
    volatile float accel_tick;      // 每 Tick 的加速度增量
    volatile bool  auto_decel;      // 是否啟動硬體自動精準煞車
    
    volatile float accumulator;     
    volatile int32_t current_pos;   
    volatile int32_t target_pos;    
    volatile int8_t  dir_state;     

    GPIO_TypeDef* step_port;        
    uint32_t      step_pin_mask;    
    GPIO_TypeDef* dir_port;        
    uint32_t      dir_pin_mask; 
};

extern volatile AxisState axes[6];
extern volatile BufPoint motion_buf[MOTION_BUF_SIZE];
extern volatile int buf_head;
extern volatile int buf_tail;
extern volatile int buf_count;

// 3. API 函式宣告
void Init_MotionEngine(const uint8_t step_pins[6], const uint8_t dir_pins[6]);
bool pushMotionPoint(long t1, long t2, long t3, long t4, long t5, long t6, uint32_t interval_us);
int getMotionBufferCount();
bool isEngineMoving();

// 高階控制 API
void emergencyStopEngine();
int32_t getAxisPosition(int axis);
void setAxisPosition(int axis, int32_t pos);

// 具有梯形加減速的獨立軸運動 API
void stopAxisInstant(int axis); 
void jogAxis(int axis, int dir, float speedStepsPerSec, float accelStepsPerSec2, bool ignoreLimits);
void moveAxisIndependent(int axis, long relativeSteps, float speedStepsPerSec, float accelStepsPerSec2);

#endif