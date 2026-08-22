#include "MotionEngine.h"
#include "Globals.h"

constexpr float STEP_FREQ = 100000.0f;                 // 100kHz 步進脈衝頻率
constexpr float ACCEL_DENOM = STEP_FREQ * STEP_FREQ;  
constexpr uint32_t TICK_US = 10;                       // 100kHz 對應的週期是 10us

// 將 100kHz 降頻 100 倍，讓梯形加減速依然在 1kHz 的頻率下運算
constexpr int MATH_PRESCALER = 100;

constexpr int32_t JOG_INFINITY = 2000000000;          

// TIM6 (Hermite 插補器) 升級為 1000Hz
constexpr float TIM6_FREQ = 1000.0f;
constexpr float T_TIM6 = 1.0f / TIM6_FREQ;

volatile AxisState axes[6];
volatile BufPoint motion_buf[MOTION_BUF_SIZE];
volatile int buf_head = 0;
volatile int buf_tail = 0;
volatile int buf_count = 0;

volatile bool is_engine_running = false;

volatile bool is_pvt_mode = false;
volatile float pvt_time_current = 0.0f;               
volatile uint32_t pvt_ticks_total = 0;

// 全域時間流速變數，預設為正常流速 1.0
volatile float current_time_scale = 1.0f; 

// 全域急停鎖存旗標
volatile bool is_estop_latched = false; 

HardwareTimer *timer_planner = NULL; 
HardwareTimer *timer_step = NULL;    

// ==========================================
// PVT 軌跡點位推播 (Mode 1 專用)
// ==========================================
bool pushMotionPoint(long t1, long t2, long t3, long t4, long t5, long t6, uint32_t interval_us) {
    if (is_estop_latched) return false;
    if (buf_count >= MOTION_BUF_SIZE) return false; 
    if (interval_us < 1000) interval_us = 1000; 
    
    long temp_targets[6] = {t1, t2, t3, t4, t5, t6};
    for (int i = 0; i < 6; i++) {
        if (JOINT_PINS[i].limitPin != 0) {
            if (temp_targets[i] > AXIS_MAX_LIMIT[i] || temp_targets[i] < AXIS_MIN_LIMIT[i]) {
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
            pvt_time_current = 0.0f;
            pvt_ticks_total = 0;
            is_pvt_mode = true;       
            is_engine_running = true; 
        }
    }
    UnlockMotionEngine();
    return true;
}

// ==========================================
// 取得當前緩衝區點位數量
// ==========================================
int getMotionBufferCount() { return buf_count; }

// ==========================================
// 檢查引擎是否正在運動 (包含排隊中或馬達運行中)
// ==========================================
bool isEngineMoving() {
    bool moving = false;
    for (int i = 0; i < 6; i++) {
        if (axes[i].current_vel > 0.0f || axes[i].target_vel > 0.0f) moving = true;
    }
    return is_engine_running || (buf_count > 0) || moving;
}

// ==========================================
// 單軸/全軸瞬間硬煞車
// 用於極限觸發或特定異常狀態下的強制停止
// ==========================================
void stopAxisInstant(int axis) {
    if (axis < 0 || axis >= 6) return;
    
    LockMotionEngine(); 
    current_time_scale = 1.0f;
    
    if (is_pvt_mode) {
        buf_head = 0; buf_tail = 0; buf_count = 0;
        is_engine_running = false;
        is_pvt_mode = false;
        for (int i = 0; i < 6; i++) {
            axes[i].target_vel = 0.0f;
            axes[i].current_vel = 0.0f; 
            axes[i].accumulator = 0.0f;
            axes[i].target_pos = axes[i].current_pos; 
            axes[i].v0 = 0.0f; axes[i].v1 = 0.0f;
        }
        UnlockMotionEngine();
        return;
    }

    axes[axis].target_vel = 0.0f;
    axes[axis].current_vel = 0.0f; 
    axes[axis].accumulator = 0.0f;
    axes[axis].target_pos = axes[axis].current_pos; 
    axes[axis].auto_decel = false;
    
    bool any_moving = false;
    for (int i = 0; i < 6; i++) {
        if (axes[i].current_vel > 0.0f || axes[i].target_vel > 0.0f) any_moving = true;
    }
    
    if (!any_moving) {
        is_engine_running = false;
    }
    UnlockMotionEngine(); 
}

// ==========================================
// 全域系統急停 (E-STOP)
// 瞬間摧毀所有軌跡緩衝，強制重置引擎狀態
// ==========================================
void emergencyStopEngine() {
    LockMotionEngine(); 
    current_time_scale = 1.0f;
    
    buf_head = 0; buf_tail = 0; buf_count = 0;
    is_engine_running = false;
    is_pvt_mode = false;
    pvt_time_current = 0.0f; 
    pvt_ticks_total = 0;
    for (int i = 0; i < 6; i++) {
        axes[i].target_vel = 0.0f;
        axes[i].current_vel = 0.0f; 
        axes[i].target_pos = axes[i].current_pos; 
        axes[i].v0 = 0.0f; axes[i].v1 = 0.0f;
    }
    UnlockMotionEngine(); 
}

// ==========================================
// 取得指定軸的當前真實絕對步數
// ==========================================
int32_t getAxisPosition(int axis) {
    if (axis < 0 || axis >= 6) return 0;
    return axes[axis].current_pos;
}

// ==========================================
// 取得指定軸的動態加速度 (Steps/Sec^2)
// ==========================================
float getAxisAccel(int axis, float rampRatio) { 
    if (axis < 0 || axis >= 6) return 50000.0f;
    
    float ref_v = SPEED_CFG[axis].controlSpeed; 
    float ramp = (float)SPEED_CFG[axis].rampSteps * rampRatio; 
    
    if (ramp <= 0.0f) return 50000.0f;                  
    return (ref_v * ref_v) / (2.0f * ramp);
}

// ==========================================
// 強制覆寫指定軸的當前絕對座標 (常用於 Homing)
// ==========================================
void setAxisPosition(int axis, int32_t pos) {
    if (axis < 0 || axis >= 6) return;
    LockMotionEngine(); 
    axes[axis].current_pos = pos;
    axes[axis].target_pos = pos;
    UnlockMotionEngine(); 
}

// ==========================================
// 獨立軸相對定距移動 (具備梯形加減速)
// ==========================================
void moveAxisIndependent(int axis, long relativeSteps, float speedStepsPerSec, float accelStepsPerSec2) {
    if (axis < 0 || axis >= 6 || relativeSteps == 0) return;
    
    LockMotionEngine(); 
    is_pvt_mode = false; 

    // 【修復核心】：強制清除微步累加器與殘留速度
    axes[axis].current_vel = 0.0f;
    axes[axis].accumulator = 0.0f;
    
    axes[axis].dir_state = (relativeSteps > 0) ? 1 : -1;
    if (axes[axis].dir_state > 0) axes[axis].dir_port->BSRR = axes[axis].dir_pin_mask;
    else axes[axis].dir_port->BSRR = (axes[axis].dir_pin_mask << 16);

    axes[axis].target_pos = axes[axis].current_pos + relativeSteps;
    axes[axis].target_vel = speedStepsPerSec / STEP_FREQ;
    axes[axis].accel_tick = accelStepsPerSec2 / ACCEL_DENOM;
    axes[axis].auto_decel = true; 
    
    is_engine_running = true;
    UnlockMotionEngine(); 
}

// ==========================================
// Mode 0：獨立絕對座標追蹤 (UI 滑桿專用)
// 讓指定軸以梯形加減速前往絕對座標
// ==========================================
void updateAbsoluteTargets(long targets[6], float speedFactor) {
    LockMotionEngine(); 
    is_pvt_mode = false; 
    current_time_scale = 1.0f;

    buf_head = 0; buf_tail = 0; buf_count = 0;
    bool needs_engine_start = false;

    for (int i = 0; i < 6; i++) {
        if (targets[i] == 999999) continue; 

        long currentPos = axes[i].current_pos;
        long targetPos = targets[i];
        
        axes[i].target_pos = targetPos; 
        long delta = targetPos - currentPos;

        if (delta != 0) {

            // 【建議補上】：讓 UI 絕對座標移動也具備平滑起步能力！
            axes[i].current_vel = 0.0f;
            axes[i].accumulator = 0.0f;

            axes[i].dir_state = (delta > 0) ? 1 : -1;
            if (axes[i].dir_state > 0) axes[i].dir_port->BSRR = axes[i].dir_pin_mask;
            else axes[i].dir_port->BSRR = (axes[i].dir_pin_mask << 16);

            float v_max = SPEED_CFG[i].controlSpeed * speedFactor;
            if (v_max < 10.0f) v_max = 10.0f;
            
            axes[i].target_vel = v_max / STEP_FREQ;
            axes[i].accel_tick = getAxisAccel(i) / ACCEL_DENOM;
            axes[i].auto_decel = true; 
            
            needs_engine_start = true;
        }
    }

    if (needs_engine_start) {
        is_engine_running = true; 
    }
    UnlockMotionEngine(); 
}

// ==========================================
// Mode 2：連續寸動 (Joint Jogging)
// 無限距移動直到收到停止指令
// ==========================================
void jogAxis(int axis, int dir, float speedStepsPerSec, float accelStepsPerSec2, bool ignoreLimits) {
    if (axis < 0 || axis >= 6) return;

    LockMotionEngine(); 
    is_pvt_mode = false; 
    current_time_scale = 1.0f;

    buf_head = 0; buf_tail = 0; buf_count = 0;

    if (dir == 0 || speedStepsPerSec <= 0.0f) {
        axes[axis].target_vel = 0.0f; 
        axes[axis].auto_decel = false;
        
        bool any_moving = false;
        for (int i = 0; i < 6; i++) {
            if (axes[i].current_vel > 0.0f || axes[i].target_vel > 0.0f) any_moving = true;
        }
        if (!any_moving) {
            is_engine_running = false;
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

        // 【修復核心】：強制清除微步累加器與殘留速度，確保平滑起步不抖動！
        axes[axis].current_vel = 0.0f;
        axes[axis].accumulator = 0.0f;
        
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
        
        is_engine_running = true;
    }
    UnlockMotionEngine(); 
}

// ==========================================
// 小腦中斷 (TIM7)：100kHz 極速脈衝發射器 & DDA 運算
// 負責精確產生 STEP 脈衝，並處理獨立運動的梯形加減速
// ==========================================
void ISR_StepGenerator() {
    for (int i = 0; i < 6; i++) {
        axes[i].step_port->BSRR = (axes[i].step_pin_mask << 16); 
    }

    if (!is_engine_running) return;

    static int math_tick = 0;

    if (!is_pvt_mode) {
        math_tick++;
        if (math_tick >= MATH_PRESCALER) {
            math_tick = 0; 
            bool is_active = false;
            
            for (int i = 0; i < 6; i++) {
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
                    int32_t dir_to_target = (axes[i].target_pos >= axes[i].current_pos) ? 1 : -1;
                    
                    if (dist == 0 || (axes[i].dir_state != dir_to_target && dist > 0)) {
                        axes[i].current_pos = axes[i].target_pos; 
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
            
            if (!is_active) {
                is_engine_running = false;
                return; 
            }
        } 
    }

    for (int i = 0; i < 6; i++) {
        if (axes[i].current_vel > 0.0f) {
            axes[i].accumulator += axes[i].current_vel;
            if (axes[i].accumulator >= 1.0f) {
                axes[i].accumulator -= 1.0f;
                axes[i].current_pos += axes[i].dir_state;
                axes[i].step_port->BSRR = axes[i].step_pin_mask; 
            }
        }
    }
}

// ==========================================
// 大腦中斷 (TIM6)：1000Hz 軌跡段落載入器 & PVT 插補器
// 負責 Hermite 曲線微積分、時間膨脹暫停控制與速度前饋計算
// ==========================================
void ISR_MotionPlanner() {
    if (!is_engine_running || !is_pvt_mode) return;

    if (is_paused) {
        if (current_time_scale > 0.0f) current_time_scale -= 0.005f; 
        if (current_time_scale < 0.0f) current_time_scale = 0.0f;
    } else {
        if (current_time_scale < 1.0f) current_time_scale += 0.005f; 
        if (current_time_scale > 1.0f) current_time_scale = 1.0f;
    }

    if (pvt_time_current >= (float)pvt_ticks_total) {
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

            pvt_ticks_total = pt1->interval_us / 1000; 
            if (pvt_ticks_total < 1) pvt_ticks_total = 1;
            
            pvt_time_current = 0.0f; 
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

    pvt_time_current += current_time_scale; 

    if (current_time_scale == 0.0f) {
        for (int i = 0; i < 6; i++) axes[i].current_vel = 0.0f;
        return;
    }

    float t = pvt_time_current / (float)pvt_ticks_total;
    float t2 = t * t;
    float t3 = t2 * t;

    float h01 = -2.0f * t3 + 3.0f * t2;
    float h10 = t3 - 2.0f * t2 + t;
    float h11 = t3 - t2;
    float T_sec = (float)pvt_ticks_total * T_TIM6;

    float dh01_dt = -6.0f * t2 + 6.0f * t;
    float dh10_dt = 3.0f * t2 - 4.0f * t + 1.0f;
    float dh11_dt = 3.0f * t2 - 2.0f * t;

    float next_v_req[6];
    for (int i = 0; i < 6; i++) {
        float D = axes[i].p1_float - axes[i].p0_float;
        
        float delta_p = h01 * D + h10 * axes[i].v0 * T_sec + h11 * axes[i].v1 * T_sec;
        float target_pos_exact = axes[i].p0_float + delta_p;
        
        float v_feedforward = (dh01_dt * D + dh10_dt * axes[i].v0 * T_sec + dh11_dt * axes[i].v1 * T_sec) / T_sec;
        float step_diff = target_pos_exact - (float)axes[i].current_pos;
        float v_feedback = step_diff * 15.0f; 

        next_v_req[i] = (v_feedforward + v_feedback) * current_time_scale; 
    }

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

        if (axes[i].current_vel > 1.0f) {
            axes[i].current_vel = 1.0f; 
        }
    }
    
    NVIC_EnableIRQ(TIM7_IRQn);
}

// ==========================================
// 運動引擎初始化
// 綁定硬體腳位並啟動 TIM6 與 TIM7 雙核定時器
// ==========================================
void Init_MotionEngine(const uint8_t step_pins[6], const uint8_t dir_pins[6]) {
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
    
    timer_planner->setOverflow(1000, HERTZ_FORMAT); 
    timer_planner->attachInterrupt(ISR_MotionPlanner);
    
    timer_step->setOverflow((uint32_t)STEP_FREQ, HERTZ_FORMAT); 
    
    timer_step->attachInterrupt(ISR_StepGenerator);
    NVIC_SetPriority(TIM7_IRQn, 1); 
    NVIC_SetPriority(TIM6_DAC_IRQn, 2); 
    timer_planner->resume();
    timer_step->resume();
}