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
	temp = (float)(pclk2 * 1000000) / (bound * 16);//�õ�USARTDIV@OVER8=0
	mantissa = temp;				 //�õ���������
	fraction = (temp - mantissa) * 16; //�õ�С������@OVER8=0 
	mantissa <<= 4;
	mantissa += fraction;
	RCC->AHB1ENR |= 1 << 0;   	//ʹ��PORTA��ʱ��  
	RCC->APB2ENR |= 1 << 4;  	//ʹ�ܴ���1ʱ�� 
	GPIO_Set(GPIOA, PIN9 | PIN10, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_50M, GPIO_PUPD_PU);//PA9,PA10,���ù���,�������
	GPIO_AF_Set(GPIOA, 9, 7);	//PA9,AF7
	GPIO_AF_Set(GPIOA, 10, 7);//PA10,AF7  	   
	//����������
	USART1->BRR = mantissa; 	//����������	 
	USART1->CR1 &= ~(1 << 15); 	//����OVER8=0 
	USART1->CR1 |= 1 << 3;  	//���ڷ���ʹ�� 
//#if EN_USART1_RX		  	//���ʹ���˽���
	//ʹ�ܽ����ж� 
	USART1->CR1 |= 1 << 2;  	//���ڽ���ʹ��
	USART1->CR1 |= 1 << 5;    	//���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(1, 0, USART1_IRQn, 4);//��2��������ȼ� 
//#endif
	USART1->CR1 |= 1 << 13;  	//����ʹ��
}

void uart2_init(u32 pclk2, u32 bound)
{
	float temp;
	u16 mantissa;
	u16 fraction;
	temp = (float)(pclk2 * 1000000) / (bound * 16);//�õ�USARTDIV
	mantissa = temp;				 //�õ���������
	fraction = (temp - mantissa) * 16; //�õ�С������	 
	mantissa <<= 4;
	mantissa += fraction;

	RCC->AHB1ENR |= 1 << 0;   		//ʹ��PORTA��ʱ��  
//RCC->AHB1ENR|=1<<6;   		//ʹ��PORTG��ʱ��  
//	GPIO_Set(GPIOG,PIN8,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); 		//PG8�������
	GPIO_Set(GPIOA, PIN2 | PIN3, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_50M, GPIO_PUPD_PU);	//PA2,PA3,���ù���,���� 
	GPIO_AF_Set(GPIOA, 2, 7);		//PA2,AF7
	GPIO_AF_Set(GPIOA, 3, 7);		//PA3,AF7  	   

	RCC->APB1ENR |= 1 << 17;  		//ʹ�ܴ���2ʱ��  
	RCC->APB1RSTR |= 1 << 17;   	//��λ����2
	RCC->APB1RSTR &= ~(1 << 17);	//ֹͣ��λ	   	   
	//����������
	USART2->BRR = mantissa; 		// ����������	 
	USART2->CR1 |= 0X200C;  		//1λֹͣ,��У��λ.
	USART2->CR1 |= 1 << 3;  	//���ڷ���ʹ�� 
	//ʹ�ܽ����ж� 

	USART2->CR1 |= 1 << 2;  		//���ڽ���ʹ��
	USART2->CR1 |= 1 << 5;    		//���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(1, 0, USART2_IRQn, 4);//��2��������ȼ�

	PGout(8) = 0;	// 485EN:0 recv mode, 1 send mode

}

void uart3_init(u32 pclk2, u32 bound)
{
	float temp;
	u16 mantissa;
	u16 fraction;
	temp = (float)(pclk2 * 1000000) / (bound * 16);//�õ�USARTDIV@OVER8=0
	mantissa = temp;				 //�õ���������
	fraction = (temp - mantissa) * 16; //�õ�С������@OVER8=0 
	mantissa <<= 4;
	mantissa += fraction;
	RCC->AHB1ENR |= 1 << 1;   	//ʹ��PORTB��ʱ��  
	RCC->APB1ENR |= 1 << 18;  	//ʹ�ܴ���1ʱ�� 
	GPIO_Set(GPIOB, PIN10 | PIN11, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_50M, GPIO_PUPD_PU);//PB10,PB11,���ù���,�������
	GPIO_AF_Set(GPIOB, 10, 7);	//PB10,AF7
	GPIO_AF_Set(GPIOB, 11, 7);//PB11,AF7  	   
	//����������
	USART3->BRR = mantissa; 	//����������	 
	USART3->CR1 &= ~(1 << 15); 	//����OVER8=0 
	USART3->CR1 |= 1 << 3;  	//���ڷ���ʹ�� 
	USART3->CR1 |= 1 << 2;  	//���ڽ���ʹ��
	USART3->CR1 |= 1 << 5;    	//���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(1, 0, USART3_IRQn, 4);//��2��������ȼ� 

	USART3->CR1 |= 1 << 13;  	//����ʹ��
}
//////////////////////////////
void uart4_init(u32 pclk2, u32 bound)
{
	float temp;
	u16 mantissa;
	u16 fraction;
	temp = (float)(pclk2 * 1000000) / (bound * 16);//�õ�USARTDIV@OVER8=0
	mantissa = temp;				 //�õ���������
	fraction = (temp - mantissa) * 16; //�õ�С������@OVER8=0 
	mantissa <<= 4;
	mantissa += fraction;
	RCC->AHB1ENR |= 1 << 0;   	//ʹ��PORTA��ʱ��  
	RCC->APB1ENR |= 1 << 19;  	//ʹ�ܴ���1ʱ�� 
	GPIO_Set(GPIOA, PIN0 | PIN1, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_50M, GPIO_PUPD_PU);//PA9,PA10,���ù���,�������
	GPIO_AF_Set(GPIOA, 0, 7);	//PA9,AF7
	GPIO_AF_Set(GPIOA, 1, 7);//PA10,AF7  	   
	//����������
	UART4->BRR = mantissa; 	//����������	 
	UART4->CR1 &= ~(1 << 15); 	//����OVER8=0 
	UART4->CR1 |= 1 << 3;  	//���ڷ���ʹ�� 
//#if EN_USART1_RX		  	//���ʹ���˽���
	//ʹ�ܽ����ж� 
	UART4->CR1 |= 1 << 2;  	//���ڽ���ʹ��
	UART4->CR1 |= 1 << 5;    	//���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(1, 0, UART4_IRQn, 4);//��2��������ȼ� 
//#endif
	UART4->CR1 |= 1 << 13;  	//����ʹ��
}

void uart6_init(u32 pclk2, u32 bound)
{
	float temp;
	u16 mantissa;
	u16 fraction;
	temp = (float)(pclk2 * 1000000) / (bound * 16);//�õ�USARTDIV@OVER8=0
	mantissa = temp;				 //�õ���������
	fraction = (temp - mantissa) * 16; //�õ�С������@OVER8=0 
	mantissa <<= 4;
	mantissa += fraction;
	RCC->AHB1ENR |= 1 << 2;   	//ʹ��PORTC��ʱ��  
	RCC->APB2ENR |= 1 << 5;  	//ʹ�ܴ���6ʱ�� 
	GPIO_Set(GPIOC, PIN6 | PIN7, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_50M, GPIO_PUPD_PU);//PC6,PC7,���ù���,�������
	GPIO_AF_Set(GPIOC, 6, 8);	//PA9,AF7
	GPIO_AF_Set(GPIOC, 7, 8);//PA10,AF7  	   
	//����������
	USART6->BRR = mantissa; 	//����������	 
	USART6->CR1 &= ~(1 << 15); 	//����OVER8=0 
	USART6->CR1 |= 1 << 3;  	//���ڷ���ʹ�� 
//#if EN_USART1_RX		  	//���ʹ���˽���
	//ʹ�ܽ����ж� 
	USART6->CR1 |= 1 << 2;  	//���ڽ���ʹ��
	USART6->CR1 |= 1 << 5;    	//���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(1, 0, USART6_IRQn, 4);//��2��������ȼ� 
//#endif
	USART6->CR1 |= 1 << 13;  	//����ʹ��
}



#endif // !_USART_H

