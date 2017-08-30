#include "mainloop.h"
#include "taskconfigs.h"

extern TaskHandle_t mainTaskHandle;

//************************************
// 函数:    mainTask
// 返回值:   void
// 描述:初始化外设，建立其他任务，删除本任务
// 参数列表: void * pvPara
//作者：LIN.HRG
//************************************
void mainTask(void *pvPara)
{
	peripheralsUARTInit();
	joyStickIOConfig();
	LED_Init();	
	GPIO_Config();
	CAN1_Mode_Init(1, 6, 7, 6, 0);	//CAN初始化,波特率500Kbps 
	vTaskDelay(2000);
	MPU_Init();
	vTaskDelay(20);
	mpu_dmp_init();
	initGPSData();
	initGyroData();
	motorInit();
	taskENTER_CRITICAL();
	xTaskCreate((TaskFunction_t)checkHuaweiCmdTask,
		(const char*) "CheckHuaweiCmdTask",
		(u16)_CHCECK_HUAWEI_CMD_STK,
		(void *)NULL,
		(UBaseType_t)_CKECK_HUAWEI_PRO,
		(TaskHandle_t)&checkHuaweiCmdHandle);
	xTaskCreate((TaskFunction_t)moveCtrlALGTask,
		(const char*) "MoveCrtlAlgTask",
		(u16)_MOVE_CTRL_ALG_STK,
		(void *)NULL,
		(UBaseType_t)_MOVE_CTRL_ALG_PRO,
		(TaskHandle_t)&moveCrtlALGHandle);
	xTaskCreate((TaskFunction_t)analysisGPSTask,
		(const char*) "AnalysisGPSTask",
		(u16)_ANALYSIS_GPS_STK,
		(void *)NULL,
		(UBaseType_t)_ANALYSIS_GPS_pro,
		(TaskHandle_t)&analysisGPSTHandle);
	xTaskCreate((TaskFunction_t)debugUsartSendTask,
		(const char*) "DebugUsartSendTask",
		(u16)_DEBUG_USART_SEND_STK,
		(void *)NULL,
		(UBaseType_t)_DEBUG_USART_SEND_pro,
		(TaskHandle_t)&debugUsartSendHandle);
	xTaskCreate((TaskFunction_t)checkBoardTask,
		(const char*) "CheckBoardTask",
		(u16)_CHECK_BOARD_STK,
		(void *)NULL,
		(UBaseType_t)_CHECK_BOARD_PRO,
		(TaskHandle_t)&checkBoardTaskHandle);
	vTaskDelete(mainTaskHandle);
	taskEXIT_CRITICAL();

}

void checkBoardTask(void *pvPara)
{
	static portTickType xLastWakeTime;
	const portTickType xFrequency = 200;
	peripheralsTimersInit();
	xLastWakeTime = xTaskGetTickCount();
	while (1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		LED0 = !LED0;
	}
}

void checkHuaweiCmdTask(void *pvPara)
{
	static portTickType xLastWakeTime;
	const portTickType xFrequency = 5;
	xLastWakeTime = xTaskGetTickCount();
	while (1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		rxHuaweiCmd();
	}
}


void moveCtrlALGTask(void *pvPara)
{
	static portTickType xLastWakeTime;
	const portTickType xFrequency = 10;
	xLastWakeTime = xTaskGetTickCount();
	while (1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		is3288CmdLost(&lineplan.heart_beat_rx_rk3288, &lineplan.flag_new_cmdframe);
		if (lineplan.flag_new_cmdframe_old == 1 && lineplan.flag_new_cmdframe == 0)
		{
			cmd.cmdstop = 1;
		}
		lineplan.flag_new_cmdframe_old = lineplan.flag_new_cmdframe;
		global_px = -robot_motion.x + GPS_STANDARD_X;//小车坐标系->大地坐标系
		global_py = robot_motion.y + GPS_STANDARD_Y;
		OnButtonFansuan(global_px, global_py, &HUAWEI_status.latitude, &HUAWEI_status.longitude);//大地坐标系到椭球坐标系
		memset(uart2rk3288.tdata, 0, sizeof(uart2rk3288.tdata));
		sprintf(uart2rk3288.tdata, "$MOVETO,%0.12f,%0.10f,%0.10f,%0.4f,%0.4f*", \
			HUAWEI_cmd.time, HUAWEI_status.longitude, HUAWEI_status.latitude, robot_motion.v, robot_motion.heading);
		uart2rk3288.length = strlen(uart2rk3288.tdata);
		uart2rk3288.sendno = 0;
#ifdef ROBOT1				
		USART1->CR1 |= (1 << 7);
#endif			
#ifdef ROBOT2		
		USART6->CR1 |= (1 << 7);
#endif				
#if MPU6050
		rxOMGMPU6050();
#else
		rd_omg_gyro();
#endif			
		parsePosition();
		checkRK3288Msg();//recive cmd
		parseCMD();
		excuteRK3288CMD();
		speed2MotorCalc(control.target_v, control.target_omg);
	}
}

void analysisGPSTask(void *pvPara)
{
	static portTickType xLastWakeTime;
	const portTickType xFrequency = 100;
	xLastWakeTime = xTaskGetTickCount();
	while (1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		analysisGPS();
	}
}

void debugUsartSendTask(void *pvPara)
{
	static portTickType xLastWakeTime;
	const portTickType xFrequency = 200;//周期执行的系节拍数
	xLastWakeTime = xTaskGetTickCount();
	while (1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		memset(uart2cmdboard.tdata, 0, sizeof(uart2cmdboard.tdata));
		sprintf(uart2cmdboard.tdata, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.3f,%.3f,%d,%.3f\r\n", lineplan.x, lineplan.y, robot_motion.x, robot_motion.y, \
			lineplan.v, robot_motion.v, fittingR, control.offsetx, lineplan.heading, robot_motion.heading * 180 / PI, lineplan.type, control.target_omg, control.decel_stage, GPS_Information.Qual, control.target_v);
		uart2cmdboard.length = strlen(uart2cmdboard.tdata);
		uart2cmdboard.sendno = 0;
		USART3->CR1 |= (1 << 7);
	}
}







