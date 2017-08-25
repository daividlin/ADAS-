#ifndef __USART_H
#define __USART_H 
#include "sys.h"
#include "stdio.h"
 
//////////////////////////////////////////////////////////////////////////////////	   
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//串口1初始化 
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/5/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//********************************************************************************
//修改说明
//无
////////////////////////////////////////////////////////////////////////////////// 	


#define USART_REC_LEN  			200  	//定义最大接收字节数 200
//#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收


typedef struct uart_struct
{
	int dataarrive;
	char rdata[20];
	unsigned char olddata;
}ROBOTUART;


typedef struct uart_int_struct
{
	char tdata[100];
	int length;
	int sendno;
}UARTINT;

extern UARTINT uart2rk3288;
extern UARTINT uart2cmdboard;
extern ROBOTUART pc2stm_uart;
extern ROBOTUART dcm2stm_uart;
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	

void uart1_init(u32 pclk2,u32 bound);
void uart2_init(u32 pclk2,u32 bound);
void uart3_init(u32 pclk2,u32 bound);
void uart4_init(u32 pclk2,u32 bound);
void uart6_init(u32 pclk2,u32 bound);

void uart2_sendstr(char* str);
void uart3_sendstr(char* str);
void uart3_send(u8 ch);
void uart1_send(u8 ch);
void uart4_send(u8 ch);
uint8_t robot_send_cmd(uint8_t *pBuf,uint32_t len);
uint8_t robot2uwb_cmd(uint8_t *pBuf,uint32_t len);

extern ROBOTUART pc2stm_uart;
void SendLogData(void);
#endif	   
















