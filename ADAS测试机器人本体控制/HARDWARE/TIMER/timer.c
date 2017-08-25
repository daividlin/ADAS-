#include "timer.h"
#include "led.h"
void TIM3_Int_Init(u16 arr, u16 psc)
{
	RCC->APB1ENR |= 1 << 1;	//TIM3时钟使能    
	TIM3->ARR = arr;  	//设定计数器自动重装值 
	TIM3->PSC = psc;  	//预分频器	  
	TIM3->DIER |= 1 << 0;   //允许更新中断	  
	TIM3->CR1 |= 0x01;    //使能定时器3
	MY_NVIC_Init(2, 0, TIM3_IRQn, 4);	//抢占1，子优先级3，组2									 
}

void TIM4_Int_Init(u16 arr, u16 psc)
{
	RCC->APB1ENR |= 1 << 2;	//TIM3时钟使能    
	TIM4->ARR = arr;  	//设定计数器自动重装值 
	TIM4->PSC = psc;  	//预分频器	  
	TIM4->DIER |= 1 << 0;   //允许更新中断	  
	TIM4->CR1 |= 0x01;    //使能定时器3
	MY_NVIC_Init(2, 0, TIM4_IRQn, 4);	//抢占1，子优先级3，组2									 
}

void TIM5_Int_Init(u16 arr, u16 psc)
{
	RCC->APB1ENR |= 1 << 3;	//TIM3时钟使能    
	TIM5->ARR = arr;  	//设定计数器自动重装值 
	TIM5->PSC = psc;  	//预分频器	  
	TIM5->DIER |= 1 << 0;   //允许更新中断	  
	TIM5->CR1 |= 0x01;    //使能定时器3
	MY_NVIC_Init(2, 0, TIM5_IRQn, 4);	//抢占2，子优先级4，组2									 
}











