#include "TMC_RawSPI.h"

// 內部專用的輔助寫入函數 (前面加 static 代表不對外公開)
static void writeReg(uint8_t cs, uint8_t addr, uint32_t data) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(cs, LOW);
    SPI.transfer(addr | 0x80); 
    SPI.transfer((data >> 24) & 0xFF);
    SPI.transfer((data >> 16) & 0xFF);
    SPI.transfer((data >> 8) & 0xFF);
    SPI.transfer(data & 0xFF);
    digitalWrite(cs, HIGH);
    SPI.endTransaction();
}

void setupTMC2240_RawSPI(uint8_t cs_pin, uint16_t run_mA, float hold_ratio, bool invert_dir) {
    // 1. 開啟 StealthChop (安靜模式) + 根據參數決定是否反轉方向
    uint32_t gconf_val = invert_dir ? 0x0000000A : 0x00000002;
    writeReg(cs_pin, 0x00, gconf_val); 
    
    // 2. 解鎖 3A 量程
    writeReg(cs_pin, 0x0A, 0x00000002); 
    
    // 3. 電流換算 (基於 2121mA RMS 極限)
    uint8_t irun = (run_mA * 31) / 2121; 
    if (irun > 31) irun = 31;
    if (irun < 1) irun = 1;
    
    uint8_t ihold = (uint8_t)(irun * hold_ratio); 
    uint8_t iholddelay = 6; 
    
    uint32_t ihold_irun_val = ((uint32_t)iholddelay << 16) | ((uint32_t)irun << 8) | (uint32_t)ihold;
    writeReg(cs_pin, 0x10, ihold_irun_val);
    
    // 4. TOFF=5, MRES=5 (8微步)
    writeReg(cs_pin, 0x6C, 0x15410155);
}

float readTMC2240Temp(uint8_t cs_pin) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(cs_pin, LOW);
    SPI.transfer(0x51 & 0x7F); 
    SPI.transfer(0); SPI.transfer(0); SPI.transfer(0); SPI.transfer(0);
    digitalWrite(cs_pin, HIGH);
    SPI.endTransaction();

    delayMicroseconds(10); 

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(cs_pin, LOW);
    SPI.transfer(0x51 & 0x7F); 
    uint32_t val = 0;
    val |= (uint32_t)SPI.transfer(0) << 24;
    val |= (uint32_t)SPI.transfer(0) << 16;
    val |= (uint32_t)SPI.transfer(0) << 8;
    val |= (uint32_t)SPI.transfer(0);
    digitalWrite(cs_pin, HIGH);
    SPI.endTransaction();

    uint16_t adc_temp_raw = val & 0xFFFF;
    return (adc_temp_raw - 2038.0) / 7.7;
}

String readTMC5160ThermalStatus(uint8_t cs_pin) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(cs_pin, LOW);
    SPI.transfer(0x6F & 0x7F); 
    SPI.transfer(0); SPI.transfer(0); SPI.transfer(0); SPI.transfer(0);
    digitalWrite(cs_pin, HIGH);
    SPI.endTransaction();

    delayMicroseconds(10); 

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(cs_pin, LOW);
    SPI.transfer(0x6F & 0x7F); 
    uint32_t val = 0;
    val |= (uint32_t)SPI.transfer(0) << 24;
    val |= (uint32_t)SPI.transfer(0) << 16;
    val |= (uint32_t)SPI.transfer(0) << 8;
    val |= (uint32_t)SPI.transfer(0);
    digitalWrite(cs_pin, HIGH);
    SPI.endTransaction();

    bool otpw = (val >> 25) & 0x01;
    bool ot   = (val >> 24) & 0x01;

    if (ot)   return "ERR (>150C)";
    if (otpw) return "WARN (>120C)";
    return "OK (<120C)";
}