#include "timer.h"
#include "led.h"
void TIM3_Int_Init(u16 arr, u16 psc)
{
	RCC->APB1ENR |= 1 << 1;	//TIM3ʱ��ʹ��    
	TIM3->ARR = arr;  	//�趨�������Զ���װֵ 
	TIM3->PSC = psc;  	//Ԥ��Ƶ��	  
	TIM3->DIER |= 1 << 0;   //��������ж�	  
	TIM3->CR1 |= 0x01;    //ʹ�ܶ�ʱ��3
	MY_NVIC_Init(2, 0, TIM3_IRQn, 4);	//��ռ1�������ȼ�3����2									 
}

void TIM4_Int_Init(u16 arr, u16 psc)
{
	RCC->APB1ENR |= 1 << 2;	//TIM3ʱ��ʹ��    
	TIM4->ARR = arr;  	//�趨�������Զ���װֵ 
	TIM4->PSC = psc;  	//Ԥ��Ƶ��	  
	TIM4->DIER |= 1 << 0;   //��������ж�	  
	TIM4->CR1 |= 0x01;    //ʹ�ܶ�ʱ��3
	MY_NVIC_Init(2, 0, TIM4_IRQn, 4);	//��ռ1�������ȼ�3����2									 
}

void TIM5_Int_Init(u16 arr, u16 psc)
{
	RCC->APB1ENR |= 1 << 3;	//TIM3ʱ��ʹ��    
	TIM5->ARR = arr;  	//�趨�������Զ���װֵ 
	TIM5->PSC = psc;  	//Ԥ��Ƶ��	  
	TIM5->DIER |= 1 << 0;   //��������ж�	  
	TIM5->CR1 |= 0x01;    //ʹ�ܶ�ʱ��3
	MY_NVIC_Init(2, 0, TIM5_IRQn, 4);	//��ռ2�������ȼ�4����2									 
}











