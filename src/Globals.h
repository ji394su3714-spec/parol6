#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <MobaTools.h>

// 宣告共用的馬達與狀態變數
extern MoToStepper* steppers[6];
extern byte homingState[6]; 
extern bool normalMoveActive;

// 通訊緩衝區設定
#define NUM_CHARS 128
extern char receivedChars[NUM_CHARS];
extern char tempChars[NUM_CHARS];
extern float receivedAngles[6];
extern boolean newData;

// 宣告共用的輔助函式
float getStepsPerDeg(int axis);
bool isAnyHoming();

// 環形緩衝區 (Ring Buffer) 變數
#define BUF_SIZE 100

struct BufPoint {
    long targetSteps[6];
    unsigned long interval_us;
};

extern BufPoint ringBuf[BUF_SIZE];
extern byte bufHead;
extern byte bufTail;
extern byte bufCount;
extern bool isBufPlaying;
extern bool pendingOK;

extern long lastBufTarget[6]; 
extern unsigned long lastPointTimeUs;
extern unsigned long currentPointIntervalUs;

extern long global_hw_max_steps;
extern float global_T_acc;

void updateRingBuffer();
#endif