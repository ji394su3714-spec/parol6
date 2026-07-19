#ifndef END_EFFECTOR_H
#define END_EFFECTOR_H

#include <Arduino.h>
#include <Servo.h>

void initEndEffector();
bool parseEndEffectorCmd(char* cmdStr);
void updateEndEffector(bool isArmIdle);

#endif