#ifndef TMC_RAWSPI_H
#define TMC_RAWSPI_H

#include <Arduino.h>
#include <SPI.h>

// 初始化 TMC2240 (包含 3A 量程解鎖與電流設定)
// invert_dir 預設為 false。若需反轉方向，傳入 true 即可。
void setupTMC2240_RawSPI(uint8_t cs_pin, uint16_t run_mA, float hold_ratio, bool invert_dir = false);

// 讀取 TMC2240 核心溫度
float readTMC2240Temp(uint8_t cs_pin);

// 讀取 TMC5160 溫度狀態
String readTMC5160ThermalStatus(uint8_t cs_pin);

#endif