#include "led.h" 

//初始化PF9和PF10为输出口.并使能这两个口的时钟		    
//LED IO初始化
void outPutGPIOConfig(void)
{    	
	brakeConfig();
	RCC->AHB1ENR|=1<<5;//使能PORTF时钟 
	GPIO_Set(GPIOF,PIN9|PIN10,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PF9,PF10设置
	LED0=1;//LED0关闭
	LED1=1;//LED1关闭
}

//************************************
// Method:    brakeConfig
// FullName:  brakeConfig
// Access:    public 
// Returns:   void
// Qualifier:抱闸控制输出，目前使用1路控制2个抱闸
// Parameter: void
//************************************
void brakeConfig(void)
{
	RCC->AHB1ENR|=1<<1;//使能PORTB时钟 
	RCC->AHB1ENR|=1<<2;//使能PORTB时钟 
		
	GPIO_Set(GPIOB,PIN12|PIN13,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PB12,13,15设置
	ERelay1 = 0;
}



