#include "joystick.h"
JOY_STICK_BUF_TYPE handleData;
volatile u32 JoyCount = 0;
volatile u8 JoyCountFlag = 0;

void TIM4_IRQHandler(void)//1ms
{
	if (TIM4->SR & 0X0001) //溢出中断
	{
		if (JoyCountFlag)
		{
			JoyCount++;
		}
		TIM4->SR &= ~(1 << 0); //清除中断标志位
	}

}


void joyStickIOConfig(void)
{
	RCC->AHB1ENR |= 1 << 2;//使能PC
	GPIO_Set(GPIOC, PIN4, GPIO_MODE_IN, 0, 0, GPIO_PUPD_PD);
	Ex_NVIC_Config(GPIO_C, 4, 3);
	MY_NVIC_Init(3, 0, EXTI4_IRQn, 4);

}


void EXTI4_IRQHandler(void)
{
	if (PCin(4))
	{
		JoyCountFlag = 1;
	}
	else
	{
		JoyCountFlag = 0;
		handleData = js_ppm();
		//			  a[i] = JoyCount;
		//				i++;
		//			if(i == 7)
		//			{i = 0;}
	}

	EXTI->PR = 1 << 4;  //清除LINE0上的中断标志位  
}


JOY_STICK_BUF_TYPE js_ppm(void)
{

	u32 tickdd = 0;
	static u32 a[8] = { 0 };
	static u8 count = 0;
	u8 i = 0;
	static JOY_STICK_BUF_TYPE returnBuf;
	if (0 != JoyCount && (JoyCountFlag == 0))
	{
		tickdd = JoyCount;
		JoyCount = 0;
		//        tickdd/= 860;
		//        tickdd -= 215;
		if (tickdd > 700)
		{
			count = 0;
		}
		else
		{
			a[count++] = tickdd;
		}
	}
	if ((count == 0) || (tickdd > 300000))//最大脉冲是27万。
	{
		count = 0;
		for (i = 0; i < 8; i++)
		{
			returnBuf.val[i] = a[i];
		}
	}
	return returnBuf;

}
