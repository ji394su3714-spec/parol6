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

#endif