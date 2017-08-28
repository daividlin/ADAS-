#ifndef _USART_H
#define _USART_H

#include "sys.h"
#include "usart.h"
#include "delay.h" 
#include "string.h"
#include "..\USER\robot_action.h"
#include "math.h"
#include "ctype.h"
#include "..\HARDWARE\GPS\gps.h"


void uart1_init(u32 pclk2, u32 bound)
{
	float temp;
	u16 mantissa;
	u16 fraction;
	temp = (float)(pclk2 * 1000000) / (bound * 16);//得到USARTDIV@OVER8=0
	mantissa = temp;				 //得到整数部分
	fraction = (temp - mantissa) * 16; //得到小数部分@OVER8=0 
	mantissa <<= 4;
	mantissa += fraction;
	RCC->AHB1ENR |= 1 << 0;   	//使能PORTA口时钟  
	RCC->APB2ENR |= 1 << 4;  	//使能串口1时钟 
	GPIO_Set(GPIOA, PIN9 | PIN10, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_50M, GPIO_PUPD_PU);//PA9,PA10,复用功能,上拉输出
	GPIO_AF_Set(GPIOA, 9, 7);	//PA9,AF7
	GPIO_AF_Set(GPIOA, 10, 7);//PA10,AF7  	   
	//波特率设置
	USART1->BRR = mantissa; 	//波特率设置	 
	USART1->CR1 &= ~(1 << 15); 	//设置OVER8=0 
	USART1->CR1 |= 1 << 3;  	//串口发送使能 
//#if EN_USART1_RX		  	//如果使能了接收
	//使能接收中断 
	USART1->CR1 |= 1 << 2;  	//串口接收使能
	USART1->CR1 |= 1 << 5;    	//接收缓冲区非空中断使能	    	
	MY_NVIC_Init(1, 0, USART1_IRQn, 4);//组2，最低优先级 
//#endif
	USART1->CR1 |= 1 << 13;  	//串口使能
}

void uart2_init(u32 pclk2, u32 bound)
{
	float temp;
	u16 mantissa;
	u16 fraction;
	temp = (float)(pclk2 * 1000000) / (bound * 16);//得到USARTDIV
	mantissa = temp;				 //得到整数部分
	fraction = (temp - mantissa) * 16; //得到小数部分	 
	mantissa <<= 4;
	mantissa += fraction;

	RCC->AHB1ENR |= 1 << 0;   		//使能PORTA口时钟  
//RCC->AHB1ENR|=1<<6;   		//使能PORTG口时钟  
//	GPIO_Set(GPIOG,PIN8,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); 		//PG8推挽输出
	GPIO_Set(GPIOA, PIN2 | PIN3, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_50M, GPIO_PUPD_PU);	//PA2,PA3,复用功能,上拉 
	GPIO_AF_Set(GPIOA, 2, 7);		//PA2,AF7
	GPIO_AF_Set(GPIOA, 3, 7);		//PA3,AF7  	   

	RCC->APB1ENR |= 1 << 17;  		//使能串口2时钟  
	RCC->APB1RSTR |= 1 << 17;   	//复位串口2
	RCC->APB1RSTR &= ~(1 << 17);	//停止复位	   	   
	//波特率设置
	USART2->BRR = mantissa; 		// 波特率设置	 
	USART2->CR1 |= 0X200C;  		//1位停止,无校验位.
	USART2->CR1 |= 1 << 3;  	//串口发送使能 
	//使能接收中断 

	USART2->CR1 |= 1 << 2;  		//串口接收使能
	USART2->CR1 |= 1 << 5;    		//接收缓冲区非空中断使能	    	
	MY_NVIC_Init(1, 0, USART2_IRQn, 4);//组2，最低优先级

	PGout(8) = 0;	// 485EN:0 recv mode, 1 send mode

}

void uart3_init(u32 pclk2, u32 bound)
{
	float temp;
	u16 mantissa;
	u16 fraction;
	temp = (float)(pclk2 * 1000000) / (bound * 16);//得到USARTDIV@OVER8=0
	mantissa = temp;				 //得到整数部分
	fraction = (temp - mantissa) * 16; //得到小数部分@OVER8=0 
	mantissa <<= 4;
	mantissa += fraction;
	RCC->AHB1ENR |= 1 << 1;   	//使能PORTB口时钟  
	RCC->APB1ENR |= 1 << 18;  	//使能串口1时钟 
	GPIO_Set(GPIOB, PIN10 | PIN11, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_50M, GPIO_PUPD_PU);//PB10,PB11,复用功能,上拉输出
	GPIO_AF_Set(GPIOB, 10, 7);	//PB10,AF7
	GPIO_AF_Set(GPIOB, 11, 7);//PB11,AF7  	   
	//波特率设置
	USART3->BRR = mantissa; 	//波特率设置	 
	USART3->CR1 &= ~(1 << 15); 	//设置OVER8=0 
	USART3->CR1 |= 1 << 3;  	//串口发送使能 
	USART3->CR1 |= 1 << 2;  	//串口接收使能
	USART3->CR1 |= 1 << 5;    	//接收缓冲区非空中断使能	    	
	MY_NVIC_Init(1, 0, USART3_IRQn, 4);//组2，最低优先级 

	USART3->CR1 |= 1 << 13;  	//串口使能
}
//////////////////////////////
void uart4_init(u32 pclk2, u32 bound)
{
	float temp;
	u16 mantissa;
	u16 fraction;
	temp = (float)(pclk2 * 1000000) / (bound * 16);//得到USARTDIV@OVER8=0
	mantissa = temp;				 //得到整数部分
	fraction = (temp - mantissa) * 16; //得到小数部分@OVER8=0 
	mantissa <<= 4;
	mantissa += fraction;
	RCC->AHB1ENR |= 1 << 0;   	//使能PORTA口时钟  
	RCC->APB1ENR |= 1 << 19;  	//使能串口1时钟 
	GPIO_Set(GPIOA, PIN0 | PIN1, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_50M, GPIO_PUPD_PU);//PA9,PA10,复用功能,上拉输出
	GPIO_AF_Set(GPIOA, 0, 7);	//PA9,AF7
	GPIO_AF_Set(GPIOA, 1, 7);//PA10,AF7  	   
	//波特率设置
	UART4->BRR = mantissa; 	//波特率设置	 
	UART4->CR1 &= ~(1 << 15); 	//设置OVER8=0 
	UART4->CR1 |= 1 << 3;  	//串口发送使能 
//#if EN_USART1_RX		  	//如果使能了接收
	//使能接收中断 
	UART4->CR1 |= 1 << 2;  	//串口接收使能
	UART4->CR1 |= 1 << 5;    	//接收缓冲区非空中断使能	    	
	MY_NVIC_Init(1, 0, UART4_IRQn, 4);//组2，最低优先级 
//#endif
	UART4->CR1 |= 1 << 13;  	//串口使能
}

void uart6_init(u32 pclk2, u32 bound)
{
	float temp;
	u16 mantissa;
	u16 fraction;
	temp = (float)(pclk2 * 1000000) / (bound * 16);//得到USARTDIV@OVER8=0
	mantissa = temp;				 //得到整数部分
	fraction = (temp - mantissa) * 16; //得到小数部分@OVER8=0 
	mantissa <<= 4;
	mantissa += fraction;
	RCC->AHB1ENR |= 1 << 2;   	//使能PORTC口时钟  
	RCC->APB2ENR |= 1 << 5;  	//使能串口6时钟 
	GPIO_Set(GPIOC, PIN6 | PIN7, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_50M, GPIO_PUPD_PU);//PC6,PC7,复用功能,上拉输出
	GPIO_AF_Set(GPIOC, 6, 8);	//PA9,AF7
	GPIO_AF_Set(GPIOC, 7, 8);//PA10,AF7  	   
	//波特率设置
	USART6->BRR = mantissa; 	//波特率设置	 
	USART6->CR1 &= ~(1 << 15); 	//设置OVER8=0 
	USART6->CR1 |= 1 << 3;  	//串口发送使能 
//#if EN_USART1_RX		  	//如果使能了接收
	//使能接收中断 
	USART6->CR1 |= 1 << 2;  	//串口接收使能
	USART6->CR1 |= 1 << 5;    	//接收缓冲区非空中断使能	    	
	MY_NVIC_Init(1, 0, USART6_IRQn, 4);//组2，最低优先级 
//#endif
	USART6->CR1 |= 1 << 13;  	//串口使能
}



#endif // !_USART_H

