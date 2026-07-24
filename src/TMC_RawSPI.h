#ifndef TMC_RAWSPI_H
#define TMC_RAWSPI_H

#include <Arduino.h>
#include <SPI.h>

// 初始化 TMC2240
// invert_dir 預設為 false。若需反轉方向，傳入 true 即可。
void setupTMC2240_RawSPI(uint8_t cs_pin, uint16_t run_mA, float hold_ratio, bool invert_dir = false);
float readTMC2240Temp(uint8_t cs_pin);


#define X_CS_PIN  PE7
#define Y_CS_PIN  PE15
#define Z_CS_PIN  PD10
#define E0_CS_PIN PD7
#define E1_CS_PIN PC14
#define E2_CS_PIN PC15

// 系統級溫度管理
void reportSystemTemperatures();
void checkThermalAlarms(bool isArmIdle);

#endif