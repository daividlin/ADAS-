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

ROBOTUART PC2STUsart;
//#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA = 0;       //接收状态标记	  


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

unsigned char head;
unsigned int ccntin;
UARTINT uart2rk3288;
UARTINT uart2cmdboard;
unsigned char usart1_rdata[20], usart1_rlen = 0, usart1_ok = 1;
unsigned int rxcmd_cnt;
//static unsigned int cnt_10mstimer = 0;

//extern unsigned int cnt_5mstimer;
extern unsigned int cnt_huawei_cmd_timer_5ms;
extern unsigned int cnt_huawei_cmd_timer_5ms_dataarrive;
/////////////连接rk3288///////////////////////////
#ifdef ROBOT1
void USART1_IRQHandler(void) //??2??????
{
	unsigned char Recv;
	////////////upload/////////////////////
	if (USART1->SR&(1 << 7))
	{
		if (uart2rk3288.sendno >= uart2rk3288.length)
		{
			USART1->CR1 &= (~(1 << 7));
			uart2rk3288.length = 0;
			uart2rk3288.sendno = 0;
		}
		else
		{
			USART1->DR = uart2rk3288.tdata[uart2rk3288.sendno];
			uart2rk3288.sendno++;
		}
	}
	///////////////recive huawei cmd//////////////////////////////////////		
	if (USART1->SR&(1 << 5))//接收到数据
	{
		Recv = USART1->DR;
		if (HUAWEI_Cmd_buf.dataarrive == 0)
		{
			if (Recv == '$')
			{
				HUAWEI_Cmd_buf.rx_pc = 0;
				head = '$';
			}
			else
			{
				//							if ( HUAWEI_Cmd_buf.rx_pc < sizeof( HUAWEI_Cmd_buf.data ) - 1 )
				{
					HUAWEI_Cmd_buf.rx_pc++;
				}
			}
			if (head == '$')
			{
				HUAWEI_Cmd_buf.data[HUAWEI_Cmd_buf.rx_pc] = Recv;
				if (Recv == '*')    //??????0x0D?????GPS??
				{
					HUAWEI_Cmd_buf.dataarrive = 1;
					head = 0;

					rxcmd_cnt++;
					if (rxcmd_cnt == 1)
					{
						cnt_10mstimer = 1;
						cnt_huawei_cmd_timer_5ms = 1;
						cnt_huawei_cmd_timer_5ms_dataarrive = 1;
						//cnt_5mstimer = 1;
					}
				}
			}
		}
	}
}
#endif

#ifdef ROBOT2
void UART4_IRQHandler(void) //??2??????
{
	unsigned char Recv;

	////////////upload/////////////////////
	if (UART4->SR&(1 << 7))
	{
		if (uart2rk3288.sendno >= uart2rk3288.length)
		{
			UART4->CR1 &= (~(1 << 7));
			uart2rk3288.length = 0;
			uart2rk3288.sendno = 0;
		}
		else
		{
			UART4->DR = uart2rk3288.tdata[uart2rk3288.sendno];
			uart2rk3288.sendno++;
		}
	}

	///////////////recive huawei cmd//////////////////////////////////////		
	if (UART4->SR&(1 << 5))//接收到数据
	{
		Recv = UART4->DR;
		if (HUAWEI_Cmd_buf.dataarrive == 0)
		{
			if (Recv == '$')
			{
				HUAWEI_Cmd_buf.rx_pc = 0;
				head = '$';
			}
			else
			{
				//							if ( HUAWEI_Cmd_buf.rx_pc < sizeof( HUAWEI_Cmd_buf.data ) - 1 )
				{
					HUAWEI_Cmd_buf.rx_pc++;
				}
			}
			if (head == '$')
			{
				HUAWEI_Cmd_buf.data[HUAWEI_Cmd_buf.rx_pc] = Recv;
				if (Recv == '*')    //??????0x0D?????GPS??
				{
					HUAWEI_Cmd_buf.dataarrive = 1;
					head = 0;

					rxcmd_cnt++;
					if (rxcmd_cnt == 1)
					{
						cnt_huawei_cmd_timer_5ms = 1;
						cnt_huawei_cmd_timer_5ms_dataarrive = 1;
						//cnt_5mstimer = 1;
					}
				}
			}
		}
	}
}

void USART6_IRQHandler(void) //??2??????
{
	unsigned char Recv;

	////////////upload/////////////////////
	if (USART6->SR&(1 << 7))
	{
		if (uart2rk3288.sendno >= uart2rk3288.length)
		{
			USART6->CR1 &= (~(1 << 7));
			uart2rk3288.length = 0;
			uart2rk3288.sendno = 0;
		}
		else
		{
			USART6->DR = uart2rk3288.tdata[uart2rk3288.sendno];
			uart2rk3288.sendno++;
		}
	}

	///////////////recive huawei cmd//////////////////////////////////////		
	if (USART6->SR&(1 << 5))//接收到数据
	{
		Recv = USART6->DR;
		if (HUAWEI_Cmd_buf.dataarrive == 0)
		{
			if (Recv == '$')
			{
				HUAWEI_Cmd_buf.rx_pc = 0;
				head = '$';
			}
			else
			{
				//							if ( HUAWEI_Cmd_buf.rx_pc < sizeof( HUAWEI_Cmd_buf.data ) - 1 )
				{
					HUAWEI_Cmd_buf.rx_pc++;
				}
			}
			if (head == '$')
			{
				HUAWEI_Cmd_buf.data[HUAWEI_Cmd_buf.rx_pc] = Recv;
				if (Recv == '*')    //??????0x0D?????GPS??
				{
					HUAWEI_Cmd_buf.dataarrive = 1;
					head = 0;
					rxcmd_cnt++;
					if (rxcmd_cnt == 1)
					{
//						cnt_10mstimer = 1;
						cnt_huawei_cmd_timer_5ms = 1;
						cnt_huawei_cmd_timer_5ms_dataarrive = 1;
						//cnt_5mstimer = 1;
					}
				}
			}
		}
	}
}
#endif

unsigned char head_gps;

void USART2_IRQHandler(void) //??2??????
{
	unsigned char Recv;
	if (USART2->SR&(1 << 5))//接收到数据
	{
		Recv = USART2->DR;
		if (GPS_BL_buf.dataarrive == 0 || GPS_Heading_buf.dataarrive == 0)
		{
			if (Recv == '$')
			{
				GPS_Uart_buf.rx_pc = 0;
				head_gps = '$';
			}
			else
			{
				if (GPS_Uart_buf.rx_pc < sizeof(GPS_Uart_buf.data) - 1)
				{
					GPS_Uart_buf.rx_pc++;
				}
			}
			if (head_gps == '$')
			{
				GPS_Uart_buf.data[GPS_Uart_buf.rx_pc] = Recv;
				if (Recv == '\r')    //??????0x0D?????GPS??
				{
					GPS_Uart_buf.dataarrive = 1;
					head_gps = 0;
					if (strstr(GPS_Uart_buf.data, "GPGGA"))
					{
						memcpy(&GPS_BL_buf, &GPS_Uart_buf, sizeof(GPS_REAL_BUFTYPE));

					}
					if (strstr(GPS_Uart_buf.data, "GPHDT"))
					{
						memcpy(&GPS_Heading_buf, &GPS_Uart_buf, sizeof(GPS_REAL_BUFTYPE));
					}
				}
			}
		}
	}
}
///////连接控制台///////////////////////////////

void USART3_IRQHandler(void)
{
	u8 res;
	if (USART3->SR&(1 << 7))
	{
		if (uart2cmdboard.sendno >= uart2cmdboard.length)
		{
			USART3->CR1 &= (~(1 << 7));
			uart2cmdboard.length = 0;
			uart2cmdboard.sendno = 0;
		}
		else
		{
			USART3->DR = uart2cmdboard.tdata[uart2cmdboard.sendno];
			uart2cmdboard.sendno++;
		}
	}
	if (USART3->SR&(1 << 5))//接收到数据
	{
		res = USART3->DR;

		if (PC2STUsart.olddata == 0xaa && res == 0x55)
		{
			usart1_rdata[0] = 0xaa;
			usart1_rdata[1] = res;
			usart1_rlen = 2;
			usart1_ok = 0;

		}
		else if (usart1_rlen >= 18)   //recv end
		{
			usart1_rdata[0] = usart1_rdata[1] = 0;
			usart1_ok = 1;
		}
		else
		{
			if (usart1_ok == 0)
			{
				usart1_rdata[usart1_rlen] = res;
				PC2STUsart.rdata[usart1_rlen] = usart1_rdata[usart1_rlen];
				usart1_rlen++;
				if (usart1_rlen >= 18)
				{

					if (usart1_rdata[17] == 0xbb)
					{
						PC2STUsart.dataarrive = 1;
					}
				}
			}
		}
		PC2STUsart.olddata = res;
	}
}



#endif // !_USART_H

