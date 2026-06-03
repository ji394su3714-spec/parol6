#include "TMC_RawSPI.h"

// ==========================================
// 定義 S6 專屬的馬達 SPI 腳位
// ==========================================
#define TMC_MOSI PE14
#define TMC_MISO PE13
#define TMC_SCK  PE12

// 確保軟體 SPI 腳位只被初始化一次
static bool spi_initialized = false;
static void initSoftSPI() {
    if (!spi_initialized) {
        pinMode(TMC_MOSI, OUTPUT);
        pinMode(TMC_MISO, INPUT);
        pinMode(TMC_SCK, OUTPUT);
        digitalWrite(TMC_SCK, HIGH); // SPI Mode 3: Clock is HIGH when idle
        spi_initialized = true;
    }
}

// 軟體 SPI 傳輸引擎 (Bit-banging Mode 3)
static uint8_t softSpiTransfer(uint8_t data) {
    uint8_t rx_data = 0;
    for (int i = 7; i >= 0; i--) {
        digitalWrite(TMC_SCK, LOW);
        if (data & (1 << i)) {
            digitalWrite(TMC_MOSI, HIGH);
        } else {
            digitalWrite(TMC_MOSI, LOW);
        }
        
        delayMicroseconds(1); 
        
        digitalWrite(TMC_SCK, HIGH);
        if (digitalRead(TMC_MISO)) {
            rx_data |= (1 << i);
        }
        delayMicroseconds(1);
    }
    return rx_data;
}

// 內部專用的輔助寫入函數
static void writeReg(uint8_t cs, uint8_t addr, uint32_t data) {
    initSoftSPI();
    
    // CS 腳位的輸出宣告，確保每次寫入前都設定為 OUTPUT 模式
    pinMode(cs, OUTPUT);
    digitalWrite(cs, LOW);
    
    softSpiTransfer(addr | 0x80); 
    softSpiTransfer((data >> 24) & 0xFF);
    softSpiTransfer((data >> 16) & 0xFF);
    softSpiTransfer((data >> 8) & 0xFF);
    softSpiTransfer(data & 0xFF);
    digitalWrite(cs, HIGH);
}

void setupTMC2240_RawSPI(uint8_t cs_pin, uint16_t run_mA, float hold_ratio, bool invert_dir) {
    // 1. GCONF 核心設定 
    // Bit 1 (0x02): 靜音模式
    // Bit 2 (0x04): 微步濾波
    uint32_t gconf_val = 0x02 | 0x04;  
    
    // 🌟 修正：TMC2240 的 Shaft 反轉位元是 Bit 4 (0x10)！
    if (invert_dir) {
        gconf_val |= 0x10; 
    }
    
    writeReg(cs_pin, 0x00, gconf_val); 
    
    // 2. 解鎖 3A 量程 
    writeReg(cs_pin, 0x0A, 0x00000002);
    
    // 3. 智慧電流設定 (IHOLD_IRUN)
    uint8_t irun = (run_mA * 31) / 2121; 
    if (irun > 31) irun = 31;
    if (irun < 1) irun = 1;
    
    uint8_t ihold = (uint8_t)(irun * hold_ratio); 
    uint8_t iholddelay = 1; 
    
    uint32_t ihold_irun_val = ((uint32_t)iholddelay << 16) | ((uint32_t)irun << 8) | (uint32_t)ihold;
    writeReg(cs_pin, 0x10, ihold_irun_val);
    
    // 4. CHOPCONF 設定 (改為 32 微步硬體插值)
    // 0x15410155 (8微步)
    // 0x14410155 (16微步)
    // 0x13410155 (32微步)
    // ==========================================
    writeReg(cs_pin, 0x6C, 0x13410155);

    // 5. PWMCONF 設定 (StealthChop 智動調諧)
    writeReg(cs_pin, 0x70, 0xC40C001E);

    // 6. TPOWERDOWN (待機降流延遲)
    writeReg(cs_pin, 0x11, 0x0000002E);
}

float readTMC2240Temp(uint8_t cs_pin) {
    initSoftSPI();
    
    // 同樣補上 CS 腳位的輸出宣告
    pinMode(cs_pin, OUTPUT);
    
    digitalWrite(cs_pin, LOW);
    softSpiTransfer(0x51 & 0x7F); 
    softSpiTransfer(0); softSpiTransfer(0); softSpiTransfer(0); softSpiTransfer(0);
    digitalWrite(cs_pin, HIGH);

    delayMicroseconds(10); 

    digitalWrite(cs_pin, LOW);
    softSpiTransfer(0x51 & 0x7F); 
    uint32_t val = 0;
    val |= (uint32_t)softSpiTransfer(0) << 24;
    val |= (uint32_t)softSpiTransfer(0) << 16;
    val |= (uint32_t)softSpiTransfer(0) << 8;
    val |= (uint32_t)softSpiTransfer(0);
    digitalWrite(cs_pin, HIGH);

    uint16_t adc_temp_raw = val & 0xFFFF;
    return (adc_temp_raw - 2038.0) / 7.7;
}