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
    // ==========================================
    // 1. GCONF 核心設定 (最關鍵的絲滑優化！)
    // ==========================================
    // Bit 1 (0x02) = en_pwm_mode (開啟 StealthChop 靜音模式)
    // Bit 2 (0x04) = multistep_filt (開啟硬體脈衝濾波，專治 Arduino 軟體發波的微小抖動！)
    // Bit 3 (0x08) = shaft (方向反轉)
    uint32_t gconf_val = 0x02 | 0x04;  // 預設 0x06 (靜音 + 濾波)
    if (invert_dir) gconf_val |= 0x08; // 若反轉則變成 0x0E
    writeReg(cs_pin, 0x00, gconf_val); 
    
    // ==========================================
    // 2. 解鎖 3A 量程 (啟用內部高精度取樣電阻)
    // ==========================================
    writeReg(cs_pin, 0x0A, 0x00000002); 
    
    // ==========================================
    // 3. 智慧電流設定 (IHOLD_IRUN)
    // ==========================================
    uint8_t irun = (run_mA * 31) / 2121; 
    if (irun > 31) irun = 31;
    if (irun < 1) irun = 1;
    
    uint8_t ihold = (uint8_t)(irun * hold_ratio); 
    
    // 將煞車延遲從 1 微微調高到 2 (大約 0.24 秒)。
    // 這樣可以保留俐落的停機，但消除瞬間斷電的「扣」一聲突兀感。
    uint8_t iholddelay = 1; // 0: 0.06s, 1: 0.24s, 2: 0.5s, 3: 0.9s
    
    uint32_t ihold_irun_val = ((uint32_t)iholddelay << 16) | ((uint32_t)irun << 8) | (uint32_t)ihold;
    writeReg(cs_pin, 0x10, ihold_irun_val);
    
    // ==========================================
    // 4. CHOPCONF 設定 (256 微步硬體插值)
    // ==========================================
    // 0x15410155 解碼：
    // Bit 28 = 1 (開啟 INTPOL，將我們的 8 微步硬體補幀到 256 微步的平滑度)
    // Bit 27..24 = 5 (MRES 設定為 8 微步)
    // Bit 3..0 = 5 (TOFF 啟動時間)
    writeReg(cs_pin, 0x6C, 0x15410155);

    // ==========================================
    // 5. PWMCONF 設定 (StealthChop 智動調諧)
    // ==========================================
    // 0xC40C001E 解碼：
    // 強制開啟 pwm_autoscale 與 pwm_autograd (Bits 18, 19)。
    // 讓 TMC2240 自動偵測馬達線圈的電感量，把電流波形燙到最平！
    writeReg(cs_pin, 0x70, 0xC40C001E);

    // ==========================================
    // 6. 新增：TPOWERDOWN (待機降流延遲)
    // ==========================================
    // 設定馬達在「最後一個脈衝」結束後，要發呆多久才允許降到 ihold。
    // 算法：延遲時間 = TPOWERDOWN * 0.0218 秒
    // 我們設定 0x2E (十進位 46)，46 * 0.0218 = 約 1.0 秒。
    // 這保證了手臂在 15ms 切片執行期間，絕對不可能偷偷降電流！
    writeReg(cs_pin, 0x11, 0x0000002E);
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