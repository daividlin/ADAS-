#ifndef _MAIN_LOOP_H
#define _MAIN_LOOP_H

#include "sysinit.h"
#include "math.h"
#include "robot_action.h"
#include "string.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "gps.h"
#include "..\HARDWARE\joystick\joystick.h"
#include "Comm2Controller.h"

void mainTask(void *pvPara);
void checkBoardTask(void *pvPara);
void checkHuaweiCmdTask(void *pvPara);
void moveCtrlALGTask(void *pvPara);
void analysisGPSTask(void *pvPara);
void debugUsartSendTask(void *pvPara);

#endif // !_MAIN_LOOP_H

