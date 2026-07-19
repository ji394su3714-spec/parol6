#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>

// ==========================================
// 1. 共用狀態變數
// ==========================================
extern byte homingState[6]; 
extern bool normalMoveActive;

// ==========================================
// 2. UART 通訊解析暫存區
// ==========================================
#define NUM_CHARS 128
extern char receivedChars[NUM_CHARS];
extern char tempChars[NUM_CHARS];
extern long receivedSteps[6];
extern bool newData;

// ==========================================
// 3. 共用輔助函式宣告
// ==========================================
float getAxisAccel(int axis, float rampRatio = 1.0f);
bool isAnyHoming();

#endif