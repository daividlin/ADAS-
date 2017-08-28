#include "mainloop.h"
#include "FreeRTOS.h"
#include "task.h"

#define _MAIN_TASK_STK 256
#define _MAIN_TASK_PRO 2
TaskHandle_t mainTaskHandle;

//************************************
// Method:    main
// FullName:  main
// Access:    public 
// Returns:   int
// Qualifier:初始化外设，对操作系统进行初始化，创建主任务，主任务中
			//规划其他任务
// Parameter: void
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

