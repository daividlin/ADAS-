#ifndef __USART_H
#define __USART_H 
#include "sys.h"
#include "stdio.h"
 

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
extern ROBOTUART PC2STUsart;
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

extern ROBOTUART PC2STUsart;
void SendLogData(void);
#endif	   
















