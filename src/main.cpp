#include <Arduino.h>
#include <MobaTools.h>

struct AxisConfig {byte step, dir, limit; int speed, ramp; long bounce; int homeDir; const char* name;
};

// --- 固定參數區 ---
const AxisConfig cfgJ4 = {3, 6, A1, 200, 100, 2650, -1, "J4"};
const AxisConfig cfgJ5 = {12, 13, A0, 200, 100, 0, -1, "J5"};
const AxisConfig cfgJ6 = {4, 7, A3, 400, 200, -4000, 1, "J6"};

const byte J6_ENABLE = 8;
MoToStepper M4(1600, STEPDIR), M6(1600, STEPDIR), M5(1600, STEPDIR);

// --- 具備二次偵測功能的歸零函式 ---
void executeHoming(MoToStepper &stepper, const AxisConfig &cfg) {
    Serial.print(F(">> Homing ")); Serial.println(cfg.name);
    
    // --- 第一階段：快速搜尋 ---
    stepper.setSpeed(cfg.speed); 
    stepper.rotate(cfg.homeDir);
    while (digitalRead(cfg.limit) == LOW); 
    stepper.stop();
    while (stepper.moving());

    // --- 第二階段：小幅回彈 ---
    // 往搜尋方向的反向移動 200 步
    stepper.move(cfg.homeDir * -200); 
    while (stepper.moving());

    // --- 第三階段：二次偵測 ---
    stepper.setSpeed(100); // 提升精準度
    stepper.rotate(cfg.homeDir);
    while (digitalRead(cfg.limit) == LOW); 
    stepper.stop();
    while (stepper.moving());

    // --- 第四階段：正式歸位 ---
    if (cfg.bounce != 0) {
        stepper.setSpeed(cfg.speed); // 恢復正常速度
        stepper.move(cfg.bounce);
        while (stepper.moving());
    }
    
    stepper.setZero();
    Serial.print(cfg.name); Serial.println(F(" OK."));
}

void setup() {
    Serial.begin(9600);
    pinMode(J6_ENABLE, OUTPUT);
    digitalWrite(J6_ENABLE, LOW);

    // 初始化 Pin 腳
    pinMode(cfgJ4.limit, INPUT_PULLUP);
    M4.attach(cfgJ4.step, cfgJ4.dir); M4.setRampLen(cfgJ4.ramp);
    pinMode(cfgJ6.limit, INPUT_PULLUP);
    M6.attach(cfgJ6.step, cfgJ6.dir); M6.setRampLen(cfgJ6.ramp);
    pinMode(cfgJ5.limit, INPUT_PULLUP);
    M5.attach(cfgJ5.step, cfgJ5.dir); M5.setRampLen(cfgJ5.ramp);

    Serial.println(F("Ready. Input 'H' to start Homing..."));
    
    while (true) {
        if (Serial.available() > 0) {
            char cmd = Serial.read();
            if (cmd == 'H' || cmd == 'h') break;
        }
    }

    // 依序執行二次偵測歸零
    executeHoming(M4, cfgJ4);
    executeHoming(M6, cfgJ6);
    executeHoming(M5, cfgJ5);

    // 同步位移
    M6.setSpeed(cfgJ6.speed); M5.setSpeed(cfgJ5.speed);
    M6.move(4000); M5.move(2200); 
    while (M6.moving() || M5.moving());
    M6.setZero(); M5.setZero();

    Serial.println(F("--- Online ---"));
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        Serial.print(F("> Cmd: ")); Serial.println(input); // 讓你知道有收到字
        input.toUpperCase();

        int spaceIdx = input.indexOf(' ');
        if (spaceIdx == -1) return;

        String axis = input.substring(0, spaceIdx);
        long steps = input.substring(spaceIdx + 1).toInt();

        if (axis == "J4") { M4.move(steps); }
        else if (axis == "J6") { M6.move(steps); }
        else if (axis == "J5") { M5.move(steps); }
    }
}