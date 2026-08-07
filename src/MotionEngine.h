#ifndef MOTION_ENGINE_H
#define MOTION_ENGINE_H

#include <Arduino.h>

// ==========================================
// 引擎專屬鎖
// ==========================================
static inline void LockMotionEngine() {
    NVIC_DisableIRQ(TIM7_IRQn);
    NVIC_DisableIRQ(TIM6_DAC_IRQn);
}

static inline void UnlockMotionEngine() {
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    NVIC_EnableIRQ(TIM7_IRQn);
}

// 將蓄水池縮為 0.5 秒緩衝，讓暫停反應靈敏 (配合 100Hz 串流，50 個點剛好是 0.5 秒)
#define MOTION_BUF_SIZE 50

// ==========================================
// 1. 核心資料結構
// ==========================================
struct BufPoint {
    int32_t  targetSteps[6];
    uint32_t interval_us; 
};

struct AxisState {
    volatile float   current_vel;     // 當前速度 (硬體斜坡用 / 每50kHz Tick)
    volatile float   target_vel;      // 目標速度 (Jog/PTP使用)
    volatile float   accel_tick;      // 加速度增量 (Jog/PTP使用)
    volatile bool    auto_decel;      // 自動精準煞車 (Jog/PTP使用)
    
    // --- Hermite Spline (PVT) 專用狀態 ---
    volatile float   p0_float;        // 起點位置 (精確浮點)
    volatile float   p1_float;        // 終點位置 (精確浮點)
    volatile float   v0;              // 起點切線速度 (Steps/Sec)
    volatile float   v1;              // 終點切線速度 (Steps/Sec)
    // -------------------------------------
    
    volatile float   accumulator;     // 浮點數微步累加器
    volatile int32_t current_pos;     // 當前絕對步數
    volatile int32_t target_pos;      // 目標絕對步數
    volatile int8_t  dir_state;       // 方向狀態 (1 或 -1)

    GPIO_TypeDef*    step_port;       // STEP 腳位硬體暫存器
    uint32_t         step_pin_mask;   // STEP 腳位位元遮罩
    GPIO_TypeDef*    dir_port;        // DIR 腳位硬體暫存器
    uint32_t         dir_pin_mask;    // DIR 腳位位元遮罩
};

// ==========================================
// 2. 全域變數宣告
// ==========================================
extern volatile AxisState axes[6];
extern volatile BufPoint  motion_buf[MOTION_BUF_SIZE];
extern volatile int       buf_head;
extern volatile int       buf_tail;
extern volatile int       buf_count;

extern volatile bool      is_engine_running;
extern volatile bool      is_pvt_mode;

extern volatile bool      is_estop_latched;

// ==========================================
// 3. API 函式宣告
// ==========================================
void Init_MotionEngine(const uint8_t step_pins[6], const uint8_t dir_pins[6]);
bool pushMotionPoint(long t1, long t2, long t3, long t4, long t5, long t6, uint32_t interval_us);
int  getMotionBufferCount();
bool isEngineMoving();

// 高階控制 API
void    emergencyStopEngine();
int32_t getAxisPosition(int axis);
void    setAxisPosition(int axis, int32_t pos);

// 具有梯形加減速的獨立軸運動 API
void stopAxisInstant(int axis); 
void jogAxis(int axis, int dir, float speedStepsPerSec, float accelStepsPerSec2, bool ignoreLimits);
void moveAxisIndependent(int axis, long relativeSteps, float speedStepsPerSec, float accelStepsPerSec2);
void updateAbsoluteTargets(long targets[6], float speedFactor);

#endif