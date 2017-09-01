#include "led.h" 

//��ʼ��PF9��PF10Ϊ�����.��ʹ���������ڵ�ʱ��		    
//LED IO��ʼ��
void outPutGPIOConfig(void)
{    	
	brakeConfig();
	RCC->AHB1ENR|=1<<5;//ʹ��PORTFʱ�� 
	GPIO_Set(GPIOF,PIN9|PIN10,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PF9,PF10����
	LED0=1;//LED0�ر�
	LED1=1;//LED1�ر�
}

//************************************
// Method:    brakeConfig
// FullName:  brakeConfig
// Access:    public 
// Returns:   void
// Qualifier:��բ���������Ŀǰʹ��1·����2����բ
// Parameter: void
//************************************
void brakeConfig(void)
{
	RCC->AHB1ENR|=1<<1;//ʹ��PORTBʱ�� 
	RCC->AHB1ENR|=1<<2;//ʹ��PORTBʱ�� 
		
	GPIO_Set(GPIOB,PIN12|PIN13,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PB12,13,15����
	ERelay1 = 0;
}



