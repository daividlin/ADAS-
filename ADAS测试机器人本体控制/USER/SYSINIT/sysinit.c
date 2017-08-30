#include "sysinit.h"
#include "FreeRTOS.h"
#include "timers.h"
//************************************
// ����:    stm32SysInit
// ����ֵ:   void
// ����:����ʱ�ӷ�Ƶ������������ʱ����ʱ��
// �����б�: void
//���ߣ�LIN.HRG
//************************************
void stm32SysInit(void)
{
	Stm32_Clock_Init(336, 8, 2, 7);//����ʱ��,168Mhz 
	delay_init(168);			//��ʱ��ʼ��  
}

//************************************
// ����:    peripheralsTimersInit
// ����ֵ:   void
// ����:��ʼ��Ӳ����ʱ��
// �����б�: void
//���ߣ�LIN.HRG
//************************************
void peripheralsTimersInit(void)
{
//	TIM3_Int_Init(100 - 1, 8400 - 1);//10ms
	TIM4_Int_Init(10 - 1, 84 - 1);//10ms
//	TIM5_Int_Init(10 - 1, 8400 - 1);//10ms
}

//************************************
// ����:    peripheralsUARTInit
// ����ֵ:   void
// ����:��ʼ��USART����
//���ߣ�LIN.HRG
//************************************
void peripheralsUARTInit()
{
	uart1_init(84, 115200);	
	uart2_init(42, 115200);
	uart3_init(42, 115200);
	uart4_init(42, 115200);
	uart6_init(84, 115200);
}

//************************************
// ����:    motorInit
// ����ֵ:   void
// ����:
// �����б�: void
//���ߣ�LIN.HRG
//************************************
void motorInit(void)
{
	EnableMotorDrive(MOTORDRIVE_ID_LEFT);
	vTaskDelay(20);
	EnableMotorDrive(MOTORDRIVE_ID_RIGHT);
	vTaskDelay(20);
	set_param_motec(MOTORDRIVE_ID_LEFT, MOTORDRIVE_PARAM_NUN_LIFEGUARD_TIME, MOTORDRIVE_PARAM_VALUE_LIFEGUARD_TIME);
	vTaskDelay(20);
	set_param_motec(MOTORDRIVE_ID_RIGHT, MOTORDRIVE_PARAM_NUN_LIFEGUARD_TIME, MOTORDRIVE_PARAM_VALUE_LIFEGUARD_TIME);
	vTaskDelay(20);
	set_param_motec(MOTORDRIVE_ID_LEFT, MOTORDRIVE_PARAM_NUN_LIFEGUARD_MODE, MOTORDRIVE_PARAM_VALUE_LIFEGUARD_MODE);
	vTaskDelay(1);
	set_param_motec(MOTORDRIVE_ID_RIGHT, MOTORDRIVE_PARAM_NUN_LIFEGUARD_MODE, MOTORDRIVE_PARAM_VALUE_LIFEGUARD_MODE);
	vTaskDelay(1);
}

