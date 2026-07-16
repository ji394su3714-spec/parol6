#include "MotionEngine.h"
#include "Config.h" // 確保有引入 Config

constexpr float STEP_FREQ = 50000.0f;                 // 神經中斷頻率 50kHz
constexpr float ACCEL_DENOM = STEP_FREQ * STEP_FREQ;  // 加速度轉換分母 (2,500,000,000)
constexpr uint32_t TICK_US = 20;                      // 每個 Tick 的微秒數 (1,000,000 / 50,000)
constexpr int32_t JOG_INFINITY = 2000000000;          // 無限寸動時的向量極點

// ----------------------------------------------------------
// DWT 硬體時鐘初始化 (維持不變)
// ----------------------------------------------------------
void Init_DWT() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

__attribute__((always_inline)) static inline void delay_1us_DWT() {
    uint32_t start_cycle = DWT->CYCCNT;
    while ((DWT->CYCCNT - start_cycle) < 180) { __NOP(); }
}

volatile AxisState axes[6];
volatile BufPoint motion_buf[MOTION_BUF_SIZE];
volatile int buf_head = 0;
volatile int buf_tail = 0;
volatile int buf_count = 0;

volatile uint32_t segment_ticks_total = 0;
volatile uint32_t segment_ticks_current = 0;
volatile bool is_engine_running = false;

HardwareTimer *timer_planner = NULL; 
HardwareTimer *timer_step = NULL;    

// ==========================================
// 外部呼叫 API 實作
// ==========================================
bool pushMotionPoint(long t1, long t2, long t3, long t4, long t5, long t6, uint32_t interval_us) {
    if (buf_count >= MOTION_BUF_SIZE) return false; 
    
    motion_buf[buf_head].targetSteps[0] = t1;
    motion_buf[buf_head].targetSteps[1] = t2;
    motion_buf[buf_head].targetSteps[2] = t3;
    motion_buf[buf_head].targetSteps[3] = t4;
    motion_buf[buf_head].targetSteps[4] = t5;
    motion_buf[buf_head].targetSteps[5] = t6;
    motion_buf[buf_head].interval_us = interval_us;
    
    buf_head = (buf_head + 1) % MOTION_BUF_SIZE;
    
    noInterrupts();
    buf_count++;
    if (!is_engine_running) {
        segment_ticks_current = 0;
        segment_ticks_total = 0;
        is_engine_running = true; 
    }
    interrupts();
    return true;
}

int getMotionBufferCount() { return buf_count; }

bool isEngineMoving() {
    bool moving = false;
    for (int i = 0; i < 6; i++) {
        if (axes[i].current_vel > 0.0f || axes[i].target_vel > 0.0f) moving = true;
    }
    return is_engine_running || (buf_count > 0) || moving;
}

// 硬體級瞬間煞車 (無視斜坡，動力瞬間歸零)
void stopAxisInstant(int axis) {
    if (axis < 0 || axis >= 6) return;
    
    noInterrupts();
    axes[axis].target_vel = 0.0f;
    axes[axis].current_vel = 0.0f; 
    axes[axis].target_pos = axes[axis].current_pos; 
    axes[axis].auto_decel = false;
    
    bool any_moving = false;
    for (int i = 0; i < 6; i++) {
        if (axes[i].current_vel > 0.0f) any_moving = true;
    }
    if (!any_moving) {
        is_engine_running = false;
        segment_ticks_current = 0;
        segment_ticks_total = 0;
    }
    interrupts();
}

void emergencyStopEngine() {
    noInterrupts();
    buf_head = 0; buf_tail = 0; buf_count = 0;
    is_engine_running = false;
    segment_ticks_current = 0; segment_ticks_total = 0;   
    for (int i = 0; i < 6; i++) {
        axes[i].target_vel = 0.0f;
        axes[i].current_vel = 0.0f; 
        axes[i].target_pos = axes[i].current_pos; 
    }
    interrupts();
}

int32_t getAxisPosition(int axis) {
    if (axis < 0 || axis >= 6) return 0;
    return axes[axis].current_pos;
}

void setAxisPosition(int axis, int32_t pos) {
    if (axis < 0 || axis >= 6) return;
    noInterrupts();
    axes[axis].current_pos = pos;
    axes[axis].target_pos = pos;
    interrupts();
}

// 具有梯形加減速的獨立軸運動 API
void moveAxisIndependent(int axis, long relativeSteps, float speedStepsPerSec, float accelStepsPerSec2) {
    if (axis < 0 || axis >= 6 || relativeSteps == 0) return;
    
    noInterrupts();
    axes[axis].dir_state = (relativeSteps > 0) ? 1 : -1;
    
    // 寫入純粹的物理電位
    if (axes[axis].dir_state > 0) {
        axes[axis].dir_port->BSRR = axes[axis].dir_pin_mask;         // 拉高 HIGH
    } else {
        axes[axis].dir_port->BSRR = (axes[axis].dir_pin_mask << 16); // 拉低 LOW
    }

    axes[axis].target_pos = axes[axis].current_pos + relativeSteps;
    
    axes[axis].target_vel = speedStepsPerSec / STEP_FREQ;
    axes[axis].accel_tick = accelStepsPerSec2 / ACCEL_DENOM;
    axes[axis].auto_decel = true; 
    
    segment_ticks_current = 0;
    segment_ticks_total = 0xFFFFFFFF; 
    is_engine_running = true;
    interrupts();
}

// 具有平滑起步與煞車的寸動 API
void jogAxis(int axis, int dir, float speedStepsPerSec, float accelStepsPerSec2, bool ignoreLimits) {
    if (axis < 0 || axis >= 6) return;

    noInterrupts();
    if (dir == 0 || speedStepsPerSec <= 0.0f) {
        axes[axis].target_vel = 0.0f; 
        axes[axis].auto_decel = false;
        
        bool any_moving = false;
        for (int i = 0; i < 6; i++) if (axes[i].current_vel > 0.0f) any_moving = true;
        if (!any_moving) {
            is_engine_running = false;
            segment_ticks_current = 0; segment_ticks_total = 0;
        }
    } else {
        if (!ignoreLimits) {
            if ((dir > 0 && axes[axis].current_pos >= AXIS_MAX_LIMIT[axis]) ||
                (dir < 0 && axes[axis].current_pos <= AXIS_MIN_LIMIT[axis])) {
                axes[axis].target_vel = 0.0f; 
                interrupts(); return;
            }
        }
        
        axes[axis].dir_state = (dir > 0) ? 1 : -1;
        
        // 寫入純粹的物理電位
        if (axes[axis].dir_state > 0) {
            axes[axis].dir_port->BSRR = axes[axis].dir_pin_mask;         // 拉高 HIGH
        } else {
            axes[axis].dir_port->BSRR = (axes[axis].dir_pin_mask << 16); // 拉低 LOW
        }
        
        axes[axis].target_vel = speedStepsPerSec / STEP_FREQ;
    axes[axis].accel_tick = accelStepsPerSec2 / ACCEL_DENOM;
        
        if (ignoreLimits) {
            axes[axis].target_pos = (dir > 0) ? JOG_INFINITY : -JOG_INFINITY;
            axes[axis].auto_decel = false;
        } else {
            axes[axis].target_pos = (dir > 0) ? AXIS_MAX_LIMIT[axis] : AXIS_MIN_LIMIT[axis];
            axes[axis].auto_decel = true; 
        }
        
        segment_ticks_current = 0;
        segment_ticks_total = 0xFFFFFFFF; 
        is_engine_running = true;
    }
    interrupts();
}

// ==========================================
// 3. 神經中斷：50kHz 極速脈衝發射器 (包含硬體梯形斜坡引擎)
// ==========================================
void ISR_StepGenerator() {
    if (!is_engine_running) return;

    if (segment_ticks_current < segment_ticks_total) {
        segment_ticks_current++;
        bool need_pulse = false;

        for (int i = 0; i < 6; i++) {
            
            // 1. 速度平滑爬升/下降 (Trapezoidal Ramp)
            if (axes[i].current_vel != axes[i].target_vel) {
                if (axes[i].current_vel < axes[i].target_vel) {
                    axes[i].current_vel += axes[i].accel_tick;
                    if (axes[i].current_vel > axes[i].target_vel) axes[i].current_vel = axes[i].target_vel;
                } else {
                    axes[i].current_vel -= axes[i].accel_tick;
                    if (axes[i].current_vel < axes[i].target_vel) axes[i].current_vel = axes[i].target_vel;
                }
            }

            // 2. 自動精準減速計算 (僅 PTP 啟動)
            if (axes[i].auto_decel && axes[i].current_vel > 0.0f) {
                int32_t dist = abs(axes[i].target_pos - axes[i].current_pos);
                if (axes[i].accel_tick > 0.0f) {
                    float decel_dist = (axes[i].current_vel * axes[i].current_vel) / (2.0f * axes[i].accel_tick);
                    if ((float)dist <= decel_dist) {
                        axes[i].target_vel = 0.0f; 
                    }
                }
                
                if (dist == 0) {
                    axes[i].target_vel = 0.0f;
                    axes[i].current_vel = 0.0f;
                    axes[i].auto_decel = false;
                }
            }

            // 3. 脈衝累加器
            if (axes[i].current_vel > 0.0f) {
                axes[i].accumulator += axes[i].current_vel;
                
                if (axes[i].accumulator >= 1.0f) {
                    axes[i].accumulator -= 1.0f;
                    axes[i].current_pos += axes[i].dir_state;
                    axes[i].step_port->BSRR = axes[i].step_pin_mask; // HIGH
                    need_pulse = true;
                }
            }
        }

        // 4. 零誤差硬體延遲與復位
        if (need_pulse) {
            delay_1us_DWT();
            for (int i = 0; i < 6; i++) {
                axes[i].step_port->BSRR = (axes[i].step_pin_mask << 16); // LOW
            }
        }
    }
}

// ==========================================
// 4. 大腦中斷：500Hz 軌跡段落載入器 (TIM6)
// ==========================================
void ISR_MotionPlanner() {
    if (!is_engine_running) return;

    if (segment_ticks_current >= segment_ticks_total) {
        if (buf_count > 0) {
            volatile BufPoint* pt = &motion_buf[buf_tail];
            buf_tail = (buf_tail + 1) % MOTION_BUF_SIZE;
            buf_count--;

            if (pt->interval_us < 1000) pt->interval_us = 1000; 
            uint32_t ticks = pt->interval_us / TICK_US;

            int64_t deltas[6];
            int64_t max_delta = 0;
            
            for (int i = 0; i < 6; i++) {
                deltas[i] = (int64_t)pt->targetSteps[i] - (int64_t)axes[i].current_pos;
                int64_t abs_delta = (deltas[i] < 0) ? -deltas[i] : deltas[i];
                if (abs_delta > max_delta) max_delta = abs_delta;
            }

            if (max_delta > (int64_t)ticks) ticks = (uint32_t)max_delta; 

            segment_ticks_total = ticks;
            segment_ticks_current = 0;

            float temp_vel[6] = {0};
            int8_t temp_dir[6] = {1, 1, 1, 1, 1, 1};

            for (int i = 0; i < 6; i++) {
                if (deltas[i] < 0) {
                    temp_dir[i] = -1;
                    temp_vel[i] = (float)(-deltas[i]) / segment_ticks_total;
                } else if (deltas[i] > 0) {
                    temp_dir[i] = 1;
                    temp_vel[i] = (float)(deltas[i]) / segment_ticks_total;
                }
            }

            NVIC_DisableIRQ(TIM7_IRQn); 

            for (int i = 0; i < 6; i++) {
                axes[i].target_pos = pt->targetSteps[i];
                axes[i].dir_state = temp_dir[i];
                
                axes[i].target_vel = temp_vel[i];
                axes[i].current_vel = temp_vel[i]; 
                axes[i].auto_decel = false;
                
                // 寫入純粹的物理電位
                if (temp_vel[i] > 0.0f) {
                    if (temp_dir[i] > 0) {
                        axes[i].dir_port->BSRR = axes[i].dir_pin_mask;         // 拉高 HIGH
                    } else {
                        axes[i].dir_port->BSRR = (axes[i].dir_pin_mask << 16); // 拉低 LOW
                    }
                }
            }

            NVIC_EnableIRQ(TIM7_IRQn); 

        } else {
            is_engine_running = false;
            for (int i = 0; i < 6; i++) {
                axes[i].target_vel = 0.0f;
                axes[i].current_vel = 0.0f;
            }
        }
    }
}

// 引擎初始化
void Init_MotionEngine(const uint8_t step_pins[6], const uint8_t dir_pins[6]) {
    Init_DWT(); 
    for (int i = 0; i < 6; i++) {
        axes[i].current_vel = 0.0f;
        axes[i].target_vel = 0.0f;
        axes[i].accel_tick = 1.0f;
        axes[i].auto_decel = false;
        axes[i].accumulator = 0.0f;
        axes[i].current_pos = 0;
        axes[i].target_pos = 0;
        axes[i].dir_state = 1;

        pinMode(step_pins[i], OUTPUT);
        pinMode(dir_pins[i], OUTPUT);
        
        axes[i].step_port = digitalPinToPort(step_pins[i]);
        axes[i].step_pin_mask = digitalPinToBitMask(step_pins[i]);
        
        axes[i].dir_port = digitalPinToPort(dir_pins[i]);
        axes[i].dir_pin_mask = digitalPinToBitMask(dir_pins[i]);
    }
    timer_planner = new HardwareTimer(TIM6);
    timer_step = new HardwareTimer(TIM7);
    timer_planner->setOverflow(5000, HERTZ_FORMAT); 
    timer_planner->attachInterrupt(ISR_MotionPlanner);
    timer_step->setOverflow(50000, HERTZ_FORMAT); 
    timer_step->attachInterrupt(ISR_StepGenerator);
    NVIC_SetPriority(TIM7_IRQn, 1); 
    NVIC_SetPriority(TIM6_DAC_IRQn, 2); 
    timer_planner->resume();
    timer_step->resume();
    Serial.println("[System] PVT Engine Online!");
}