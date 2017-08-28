#ifndef __USART_H
#define __USART_H 
#include "sys.h"
#include "stdio.h"
 

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
//#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收




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

void SendLogData(void);
#endif	   
















