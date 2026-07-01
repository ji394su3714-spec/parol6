#include "EndEffector.h"

#define SERVO_PIN PB10 
MoToServo gripperServo; 

const int GRIPPER_OPEN = 1020;   
const int GRIPPER_CLOSE = 1920; 

int gripper_current_us = GRIPPER_OPEN; 
int gripper_target_us = GRIPPER_OPEN;  
unsigned long servo_last_time = 0; 
int servo_step_size = 50;  
int servo_step_delay = 20; 

bool ee_cmd_waiting = false;
int pending_target_us = GRIPPER_OPEN;

bool waiting_for_servo_arrival = false; // 記錄夾爪是否正在作動中

void initEndEffector() {
    gripperServo.attach(SERVO_PIN); 
    gripperServo.write(GRIPPER_OPEN); 
}

bool parseEndEffectorCmd(char* cmdStr) {
    if (strncmp(cmdStr, "EE,", 3) == 0) {
        char* strtokIndx = strtok(cmdStr, ","); 
        strtokIndx = strtok(NULL, ",");         
        
        if (strtokIndx != NULL) {
            if (strcmp(strtokIndx, "SERVO") == 0) {
                strtokIndx = strtok(NULL, ",");
                if (strtokIndx != NULL) {
                    int val = atoi(strtokIndx);
                    pending_target_us = map(val, 0, 100, GRIPPER_OPEN, GRIPPER_CLOSE);
                    pending_target_us = constrain(pending_target_us, GRIPPER_OPEN, GRIPPER_CLOSE);
                }
            } 
            else if (strcmp(strtokIndx, "DIGITAL") == 0) {
                strtokIndx = strtok(NULL, ",");
                if (strtokIndx != NULL) {
                    int val = atoi(strtokIndx);
                    pending_target_us = (val == 1) ? GRIPPER_CLOSE : GRIPPER_OPEN;
                }
            }
        }
        
        ee_cmd_waiting = true;
        return true; 
    }
    return false; 
}

void updateEndEffector(bool isArmIdle) {
    // 1. 等待手臂停穩後，才把任務交給夾爪
    if (ee_cmd_waiting && isArmIdle) {
        gripper_target_us = pending_target_us;
        ee_cmd_waiting = false;
        waiting_for_servo_arrival = true; // 夾爪開始移動，上鎖！
    }

    // 2. 夾爪平滑移動邏輯
    if (gripper_current_us != gripper_target_us) {
        if (millis() - servo_last_time >= servo_step_delay) {
            int distance = gripper_target_us - gripper_current_us;

            if (abs(distance) > servo_step_size) {
                if (distance > 0) gripper_current_us += servo_step_size;
                else gripper_current_us -= servo_step_size;
            } else {
                gripper_current_us = gripper_target_us;
            }
            
            gripperServo.write(gripper_current_us);
            servo_last_time = millis();
        }
    } else {
        // 關鍵修改：夾爪到位後，印出專屬暗號，絕不用 "OK"
        if (waiting_for_servo_arrival) {
            Serial.println("<EE_DONE>"); 
            waiting_for_servo_arrival = false;
        }
    }
}