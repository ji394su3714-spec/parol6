#ifndef END_EFFECTOR_H
#define END_EFFECTOR_H

#include <Arduino.h>
#include <Servo.h> // 👑 替換為 Arduino 內建標準伺服馬達庫

void initEndEffector();
bool parseEndEffectorCmd(char* cmdStr);
void updateEndEffector(bool isArmIdle);

#endif