#include "Globals.h"

// ==========================================
// 1. 共用狀態變數實體
// ==========================================
byte homingState[6] = {0, 0, 0, 0, 0, 0}; 
bool normalMoveActive = false;

// ==========================================
// 2. UART 通訊解析暫存區實體
// ==========================================
char receivedChars[NUM_CHARS];
char tempChars[NUM_CHARS];
long receivedSteps[6] = {0};
bool newData = false;

// ==========================================
// 3. 通訊與暫停控制實體
// ==========================================
volatile bool is_paused = false;