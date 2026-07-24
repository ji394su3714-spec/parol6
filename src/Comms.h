#ifndef COMMS_H
#define COMMS_H

// ==========================================
// 通訊接收與執行引擎
// ==========================================
void receiveBinaryLoop();

// ==========================================
// 時間膨脹與暫停控制
// ==========================================
extern volatile bool is_paused;
extern volatile float pause_multiplier;

#endif