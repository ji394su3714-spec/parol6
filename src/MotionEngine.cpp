#include "MotionEngine.h"
#include "Globals.h"
constexpr float STEP_FREQ = 100000.0f;                 // 升級為 100kHz
constexpr float ACCEL_DENOM = STEP_FREQ * STEP_FREQ;  
constexpr uint32_t TICK_US = 10;                       // 100kHz 對應的週期是 10us

// 將 100kHz 降頻 100 倍，讓梯形加減速依然在 1kHz 的頻率下運算
constexpr int MATH_PRESCALER = 100;

constexpr int32_t JOG_INFINITY = 2000000000;          

// TIM6 (Hermite 插補器) 升級為 1000Hz
constexpr float TIM6_FREQ = 1000.0f;
constexpr float T_TIM6 = 1.0f / TIM6_FREQ;            // 0.001s

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

volatile bool is_pvt_mode = false;
volatile uint32_t pvt_ticks_total = 0;
volatile uint32_t pvt_ticks_current = 0;

// 全域急停鎖存旗標
volatile bool is_estop_latched = false; 

HardwareTimer *timer_planner = NULL; 
HardwareTimer *timer_step = NULL;    

// ==========================================
// 外部呼叫 API 實作
// ==========================================
bool pushMotionPoint(long t1, long t2, long t3, long t4, long t5, long t6, uint32_t interval_us) {
    // 安全防禦 A：如果系統處於急停鎖存狀態，直接拒絕任何點位排隊！
    if (is_estop_latched) return false;

    if (buf_count >= MOTION_BUF_SIZE) return false; 
    
    if (interval_us < 1000) interval_us = 1000; 
    
    // 安全防禦 B：PVT 軟限位檢查！
    long temp_targets[6] = {t1, t2, t3, t4, t5, t6};
    for (int i = 0; i < 6; i++) {
        if (JOINT_PINS[i].limitPin != 0) {
            if (temp_targets[i] > AXIS_MAX_LIMIT[i] || temp_targets[i] < AXIS_MIN_LIMIT[i]) {
                // 超限直接攔截，不印 Serial，直接拒收
                return false; 
            }
        }
    }
    
    motion_buf[buf_head].targetSteps[0] = t1;
    motion_buf[buf_head].targetSteps[1] = t2;
    motion_buf[buf_head].targetSteps[2] = t3;
    motion_buf[buf_head].targetSteps[3] = t4;
    motion_buf[buf_head].targetSteps[4] = t5;
    motion_buf[buf_head].targetSteps[5] = t6;
    motion_buf[buf_head].interval_us = interval_us;
    
    buf_head = (buf_head + 1) % MOTION_BUF_SIZE;
    
    LockMotionEngine();
    buf_count++;
    
    if (!is_engine_running) {
        if (buf_count >= 15) {
            pvt_ticks_current = 0;
            pvt_ticks_total = 0;
            is_pvt_mode = true;       
            is_engine_running = true; 
        }
    }
    UnlockMotionEngine();
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

// PVT 同步毀滅機制與單軸硬停
void stopAxisInstant(int axis) {
    if (axis < 0 || axis >= 6) return;
    
    LockMotionEngine(); 
    
    if (is_pvt_mode) {
        buf_head = 0; buf_tail = 0; buf_count = 0;
        is_engine_running = false;
        is_pvt_mode = false;
        for (int i = 0; i < 6; i++) {
            axes[i].target_vel = 0.0f;
            axes[i].current_vel = 0.0f; 
            axes[i].target_pos = axes[i].current_pos; 
            axes[i].v0 = 0.0f; axes[i].v1 = 0.0f;
        }
        UnlockMotionEngine();
        return;
    }

    // 獨立軸模式煞車
    axes[axis].target_vel = 0.0f;
    axes[axis].current_vel = 0.0f; 
    axes[axis].target_pos = axes[axis].current_pos; 
    axes[axis].auto_decel = false;
    
    bool any_moving = false;
    for (int i = 0; i < 6; i++) {
        // 【修復點】：必須同時檢查 target_vel！只要有軸「想動」，引擎總開關就不准關！
        if (axes[i].current_vel > 0.0f || axes[i].target_vel > 0.0f) any_moving = true;
    }
    
    if (!any_moving) {
        is_engine_running = false;
        segment_ticks_current = 0; 
        segment_ticks_total = 0;
    }
    UnlockMotionEngine(); 
}

void emergencyStopEngine() {
    LockMotionEngine(); 
    buf_head = 0; buf_tail = 0; buf_count = 0;
    is_engine_running = false;
    is_pvt_mode = false;
    segment_ticks_current = 0; segment_ticks_total = 0;   
    pvt_ticks_current = 0; pvt_ticks_total = 0;
    for (int i = 0; i < 6; i++) {
        axes[i].target_vel = 0.0f;
        axes[i].current_vel = 0.0f; 
        axes[i].target_pos = axes[i].current_pos; 
        axes[i].v0 = 0.0f; axes[i].v1 = 0.0f;
    }
    UnlockMotionEngine(); 
}

int32_t getAxisPosition(int axis) {
    if (axis < 0 || axis >= 6) return 0;
    return axes[axis].current_pos;
}

float getAxisAccel(int axis, float rampRatio) { 
    if (axis < 0 || axis >= 6) return 50000.0f;
    
    float ref_v = SPEED_CFG[axis].controlSpeed; 
    float ramp = (float)SPEED_CFG[axis].rampSteps * rampRatio; 
    
    if (ramp <= 0.0f) return 50000.0f;                  
    return (ref_v * ref_v) / (2.0f * ramp);
}

void setAxisPosition(int axis, int32_t pos) {
    if (axis < 0 || axis >= 6) return;
    LockMotionEngine(); 
    axes[axis].current_pos = pos;
    axes[axis].target_pos = pos;
    UnlockMotionEngine(); 
}

void moveAxisIndependent(int axis, long relativeSteps, float speedStepsPerSec, float accelStepsPerSec2) {
    if (axis < 0 || axis >= 6 || relativeSteps == 0) return;
    
    LockMotionEngine(); 
    is_pvt_mode = false; 
    
    axes[axis].dir_state = (relativeSteps > 0) ? 1 : -1;
    if (axes[axis].dir_state > 0) axes[axis].dir_port->BSRR = axes[axis].dir_pin_mask;
    else axes[axis].dir_port->BSRR = (axes[axis].dir_pin_mask << 16);

    axes[axis].target_pos = axes[axis].current_pos + relativeSteps;
    axes[axis].target_vel = speedStepsPerSec / STEP_FREQ;
    axes[axis].accel_tick = accelStepsPerSec2 / ACCEL_DENOM;
    axes[axis].auto_decel = true; 
    
    segment_ticks_current = 0;
    segment_ticks_total = 0xFFFFFFFF; 
    is_engine_running = true;
    UnlockMotionEngine(); 
}

// ==========================================
// 全新：多軸絕對座標獨立更新 (Independent Absolute Tracking)
// ==========================================
void updateAbsoluteTargets(long targets[6], float speedFactor) {
    LockMotionEngine(); 
    is_pvt_mode = false; 
    bool needs_engine_start = false;

    for (int i = 0; i < 6; i++) {
        // 真正的互不干涉：如果是 999999，完全不要碰這個軸！
        // 讓它保留原本的 target_pos 與 current_vel 繼續跑它自己的路
        if (targets[i] == 999999) {
            continue; 
        }

        long currentPos = axes[i].current_pos;
        long targetPos = targets[i];
        
        // 直接更新為最新的絕對終點
        axes[i].target_pos = targetPos; 
        
        long delta = targetPos - currentPos;

        if (delta != 0) {
            // 決定方向
            axes[i].dir_state = (delta > 0) ? 1 : -1;
            if (axes[i].dir_state > 0) axes[i].dir_port->BSRR = axes[i].dir_pin_mask;
            else axes[i].dir_port->BSRR = (axes[i].dir_pin_mask << 16);

            // 套用該軸自己的性能極限，不跟別人同步
            float v_max = SPEED_CFG[i].controlSpeed * speedFactor;
            if (v_max < 10.0f) v_max = 10.0f;
            
            axes[i].target_vel = v_max / STEP_FREQ;
            axes[i].accel_tick = getAxisAccel(i) / ACCEL_DENOM;
            axes[i].auto_decel = true; 
            
            needs_engine_start = true;
        }
    }

    if (needs_engine_start) {
        // 重置全域計時保護，並確保引擎啟動
        segment_ticks_current = 0;
        segment_ticks_total = 0xFFFFFFFF; 
        is_engine_running = true; 
    }
    UnlockMotionEngine(); 
}

void jogAxis(int axis, int dir, float speedStepsPerSec, float accelStepsPerSec2, bool ignoreLimits) {
    if (axis < 0 || axis >= 6) return;

    LockMotionEngine(); 
    is_pvt_mode = false; 

    if (dir == 0 || speedStepsPerSec <= 0.0f) {
        axes[axis].target_vel = 0.0f; 
        axes[axis].auto_decel = false;
        
        bool any_moving = false;
        for (int i = 0; i < 6; i++) {
            // 【修復點】：同上，防範誤殺剛起步的軸
            if (axes[i].current_vel > 0.0f || axes[i].target_vel > 0.0f) any_moving = true;
        }
        if (!any_moving) {
            is_engine_running = false;
            segment_ticks_current = 0; 
            segment_ticks_total = 0;
        }
    } else {
        if (!ignoreLimits) {
            if ((dir > 0 && axes[axis].current_pos >= AXIS_MAX_LIMIT[axis]) ||
                (dir < 0 && axes[axis].current_pos <= AXIS_MIN_LIMIT[axis])) {
                axes[axis].target_vel = 0.0f; 
                UnlockMotionEngine(); 
                return;
            }
        }
        
        axes[axis].dir_state = (dir > 0) ? 1 : -1;
        if (axes[axis].dir_state > 0) axes[axis].dir_port->BSRR = axes[axis].dir_pin_mask;
        else axes[axis].dir_port->BSRR = (axes[axis].dir_pin_mask << 16);
        
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
    UnlockMotionEngine(); 
}

// ==========================================
// 3. 小腦中斷：100kHz 極速脈衝發射器 (TIM7)
// ==========================================
void ISR_StepGenerator() {
    // ==========================================
    // 優化 1：零時脈耗損脈衝 (Zero-Delay Pulse)
    // 進入中斷的第一件事，先把上次中斷拉高的 Step 腳位全部拉低。
    // 這會自然產生一個近乎 10us 的完美高電位脈衝，
    // 徹底消滅原本卡死 CPU 的 delay_1us_DWT() 迴圈等待！
    // ==========================================
    for (int i = 0; i < 6; i++) {
        axes[i].step_port->BSRR = (axes[i].step_pin_mask << 16); 
    }

    if (!is_engine_running) return;

    // 宣告靜態計數器，用來分頻
    static int math_tick = 0;

    if (!is_pvt_mode) {
        
        // ==========================================
        // 優化 2：時間切片降頻 (Decimation)
        // ==========================================
        math_tick++;
        if (math_tick >= MATH_PRESCALER) {
            math_tick = 0; // 重置計數器
            bool is_active = false;
            
            if (segment_ticks_current < segment_ticks_total) {
                segment_ticks_current += MATH_PRESCALER; // 一次補齊流失的 tick 數
                
                for (int i = 0; i < 6; i++) {
                    // 因為降頻了，單次迴圈要加上的加速度必須按比例放大，物理曲線才會不變
                    float effective_accel = axes[i].accel_tick * MATH_PRESCALER;

                    if (axes[i].current_vel != axes[i].target_vel) {
                        if (axes[i].current_vel < axes[i].target_vel) {
                            axes[i].current_vel += effective_accel;
                            if (axes[i].current_vel > axes[i].target_vel) axes[i].current_vel = axes[i].target_vel;
                        } else {
                            axes[i].current_vel -= effective_accel;
                            if (axes[i].current_vel < axes[i].target_vel) axes[i].current_vel = axes[i].target_vel;
                        }
                    }

                    if (axes[i].auto_decel) {
                        int32_t dist = abs(axes[i].target_pos - axes[i].current_pos);
                        
                        // 判斷目前應該前進的方向
                        int32_t dir_to_target = (axes[i].target_pos >= axes[i].current_pos) ? 1 : -1;
                        
                        // 【修復點】：除了精準等於 0，如果發現「行進方向與目標方向反了」，代表已經過衝 (Overshoot)！
                        if (dist == 0 || (axes[i].dir_state != dir_to_target && dist > 0)) {
                            axes[i].current_pos = axes[i].target_pos; // 強制拉回精確座標，不再錯位
                            axes[i].target_vel = 0.0f; 
                            axes[i].current_vel = 0.0f; 
                            axes[i].auto_decel = false;
                        } else {
                            if (axes[i].current_vel > 0.0f && axes[i].accel_tick > 0.0f) {
                                float decel_dist = (axes[i].current_vel * axes[i].current_vel) / (2.0f * axes[i].accel_tick);
                                if ((float)dist <= decel_dist) axes[i].target_vel = 0.0f; 
                            }
                            if (axes[i].target_vel == 0.0f) {
                                float min_vel = 10.0f / STEP_FREQ; 
                                if (axes[i].current_vel < min_vel) axes[i].current_vel = min_vel;
                            }
                        }
                    }

                    if (axes[i].current_vel > 0.0f || axes[i].target_vel > 0.0f) {
                        is_active = true;
                    }
                }
            }
            
            if (!is_active) {
                is_engine_running = false;
                return; 
            }
        } 
    }

    // ==========================================
    // 核心 DDA 步進產生器
    // ==========================================
    for (int i = 0; i < 6; i++) {
        // 這個區塊只有非常輕量的 1 次浮點加法與 1 次比較
        if (axes[i].current_vel > 0.0f) {
            axes[i].accumulator += axes[i].current_vel;
            if (axes[i].accumulator >= 1.0f) {
                axes[i].accumulator -= 1.0f;
                axes[i].current_pos += axes[i].dir_state;
                axes[i].step_port->BSRR = axes[i].step_pin_mask; // 拉高 Step 腳位
            }
        }
    }
}

// ==========================================
// 4. 大腦中斷：1000Hz 軌跡段落載入器 (TIM6)
// ==========================================
void ISR_MotionPlanner() {
    if (!is_engine_running || !is_pvt_mode) return;

    if (pvt_ticks_current >= pvt_ticks_total) {
        if (buf_count > 0) {
            volatile BufPoint* pt1 = &motion_buf[buf_tail];
            float T1_sec = pt1->interval_us * 1e-6f;

            for (int i = 0; i < 6; i++) {
                axes[i].p0_float = (float)axes[i].current_pos;  
                axes[i].p1_float = (float)pt1->targetSteps[i];
                axes[i].v0 = axes[i].v1; 

                if (buf_count > 1) {
                    int next_idx = (buf_tail + 1) % MOTION_BUF_SIZE;
                    volatile BufPoint* pt2 = &motion_buf[next_idx];
                    float T2_sec = pt2->interval_us * 1e-6f;
                    axes[i].v1 = ((float)pt2->targetSteps[i] - axes[i].p0_float) / (T1_sec + T2_sec);
                } else {
                    axes[i].v1 = 0.0f; 
                }
            }

            pvt_ticks_total = pt1->interval_us / 1000; // TIM6_FREQ 為 1000Hz
            if (pvt_ticks_total < 1) pvt_ticks_total = 1;
            pvt_ticks_current = 0;

            buf_tail = (buf_tail + 1) % MOTION_BUF_SIZE;
            buf_count--;
        } else {
            is_engine_running = false;
            is_pvt_mode = false;
            for (int i = 0; i < 6; i++) {
                axes[i].current_vel = 0.0f; axes[i].v0 = 0.0f; axes[i].v1 = 0.0f;
            }
            return;
        }
    }

    pvt_ticks_current++;
    float t = (float)pvt_ticks_current / (float)pvt_ticks_total;
    float t2 = t * t;
    float t3 = t2 * t;

    float h01 = -2.0f * t3 + 3.0f * t2;
    float h10 = t3 - 2.0f * t2 + t;
    float h11 = t3 - t2;
    float T_sec = (float)pvt_ticks_total * T_TIM6;

    // 核心優化 1：對 Hermite 基準函數進行一階微分，求取速度導函數
    float dh01_dt = -6.0f * t2 + 6.0f * t;
    float dh10_dt = 3.0f * t2 - 4.0f * t + 1.0f;
    float dh11_dt = 3.0f * t2 - 2.0f * t;

    // 最佳化：在「鎖外」預先算好速度需求，避免佔用 TIM7 時間
    float next_v_req[6];
    for (int i = 0; i < 6; i++) {
        float D = axes[i].p1_float - axes[i].p0_float;
        
        // 1. 理論理想位置 (用於計算誤差)
        float delta_p = h01 * D + h10 * axes[i].v0 * T_sec + h11 * axes[i].v1 * T_sec;
        float target_pos_exact = axes[i].p0_float + delta_p;
        
        // 核心優化 2：理論前饋速度 (Feedforward Velocity)
        // 直接使用微積分求出的絕對平滑速度，完全不受「整數步」的雜訊干擾！
        float v_feedforward = (dh01_dt * D + dh10_dt * axes[i].v0 * T_sec + dh11_dt * axes[i].v1 * T_sec) / T_sec;

        // 核心優化 3：柔性位置回授 (Soft P-Controller)
        // 修正物理累積誤差，將原先殘暴的 Kp=1000 降至溫和的 Kp=15
        float step_diff = target_pos_exact - (float)axes[i].current_pos;
        float v_feedback = step_diff * 15.0f; 

        // 最終要求的平滑速度
        next_v_req[i] = v_feedforward + v_feedback; 
    }

    // TIM7 局部防護鎖，解決 ISR 競態條件
    NVIC_DisableIRQ(TIM7_IRQn);
    
    for (int i = 0; i < 6; i++) {
        float v_req = next_v_req[i];
        
        if (v_req > 0.0f) {
            axes[i].dir_state = 1;
            axes[i].dir_port->BSRR = axes[i].dir_pin_mask;
            axes[i].current_vel = v_req / STEP_FREQ; 
        } else if (v_req < 0.0f) {
            axes[i].dir_state = -1;
            axes[i].dir_port->BSRR = (axes[i].dir_pin_mask << 16);
            axes[i].current_vel = -v_req / STEP_FREQ;
        } else {
            axes[i].current_vel = 0.0f;
        }

        // 速度飽和限制，絕對防止悄悄漏步
        if (axes[i].current_vel > 1.0f) {
            axes[i].current_vel = 1.0f; 
        }
    }
    
    NVIC_EnableIRQ(TIM7_IRQn);
}

void Init_MotionEngine(const uint8_t step_pins[6], const uint8_t dir_pins[6]) {
    Init_DWT(); 
    for (int i = 0; i < 6; i++) {
        axes[i].current_vel = 0.0f; axes[i].target_vel = 0.0f;
        axes[i].accel_tick = 1.0f; axes[i].auto_decel = false;
        axes[i].accumulator = 0.0f; axes[i].current_pos = 0;
        axes[i].target_pos = 0; axes[i].dir_state = 1;
        axes[i].v0 = 0.0f; axes[i].v1 = 0.0f;

        pinMode(step_pins[i], OUTPUT); pinMode(dir_pins[i], OUTPUT);
        axes[i].step_port = digitalPinToPort(step_pins[i]);
        axes[i].step_pin_mask = digitalPinToBitMask(step_pins[i]);
        axes[i].dir_port = digitalPinToPort(dir_pins[i]);
        axes[i].dir_pin_mask = digitalPinToBitMask(dir_pins[i]);
    }
    timer_planner = new HardwareTimer(TIM6);
    timer_step = new HardwareTimer(TIM7);
    
    // TIM6 1000Hz 初始化
    timer_planner->setOverflow(1000, HERTZ_FORMAT); 
    timer_planner->attachInterrupt(ISR_MotionPlanner);
    
    // 直接綁定 STEP_FREQ 常數，100kHz！
    timer_step->setOverflow((uint32_t)STEP_FREQ, HERTZ_FORMAT); 
    
    timer_step->attachInterrupt(ISR_StepGenerator);
    NVIC_SetPriority(TIM7_IRQn, 1); 
    NVIC_SetPriority(TIM6_DAC_IRQn, 2); 
    timer_planner->resume();
    timer_step->resume();
}