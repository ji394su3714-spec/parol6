#ifndef END_EFFECTOR_H
#define END_EFFECTOR_H

#include <Arduino.h>
// #include <Servo.h>  <-- 確保這行被註解或刪除！

void initEndEffector();
void setGripperTarget(int percent);
void updateEndEffector(bool isArmIdle);

#endif