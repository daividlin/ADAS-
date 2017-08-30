#include "mainloop.h"
#include "FreeRTOS.h"
#include "task.h"

#define _MAIN_TASK_STK 256
#define _MAIN_TASK_PRO 2
TaskHandle_t mainTaskHandle;


//************************************
// ����:    main
// ����ֵ:   int
// ����: ��ʼ��ϵͳʱ�ӣ�����������
// �����б�: void
//���ߣ�LIN.HRG
//************************************
int main(void)
{
    stm32SysInit();
	xTaskCreate((TaskFunction_t) mainTask, 
		(const char*) "MainTask", 
		(u16)_MAIN_TASK_STK, 
		(void *)NULL, 
		(UBaseType_t)_MAIN_TASK_PRO, 
		(TaskHandle_t)&mainTaskHandle);
	vTaskStartScheduler();
}

