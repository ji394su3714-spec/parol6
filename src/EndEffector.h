#ifndef END_EFFECTOR_H
#define END_EFFECTOR_H

#include <Arduino.h>
#include <MobaTools.h> // 用 MobaTools 統一管理

void initEndEffector();
bool parseEndEffectorCmd(char* cmdStr);
void updateEndEffector(bool isArmIdle);

#endif