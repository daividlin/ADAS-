#ifndef _TASK_CONFIG_H
#define _TASK_CONFIG_H

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#define eventMoveCrtlFlag	(1<<0)				//ÊÂ¼þÎ»
#define eventAnalysisGpsFlag	(1<<1)
#define eventDebugUsartSendFlag	(1<<2)
#define eventReadHuaWeiFlag	(1<<3)

#define _CHECK_BOARD_STK 256
#define _CHECK_BOARD_PRO 30
TaskHandle_t checkBoardTaskHandle;

#define _CHCECK_HUAWEI_CMD_STK 256
#define _CKECK_HUAWEI_PRO 5
TaskHandle_t checkHuaweiCmdHandle;

#define _MOVE_CTRL_ALG_STK 256
#define _MOVE_CTRL_ALG_PRO 4
TaskHandle_t moveCrtlALGHandle;

#define _ANALYSIS_GPS_STK 256
#define _ANALYSIS_GPS_pro 3
TaskHandle_t analysisGPSTHandle;

#define _DEBUG_USART_SEND_STK 256
#define _DEBUG_USART_SEND_pro 2
TaskHandle_t debugUsartSendHandle;


#endif // !_TASK_CONFIG_H

