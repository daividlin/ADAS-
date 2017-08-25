#ifndef __USART_H
#define __USART_H 
#include "sys.h"
#include "stdio.h"
 
//////////////////////////////////////////////////////////////////////////////////	   
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//����1��ʼ�� 
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/5/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved
//********************************************************************************
//�޸�˵��
//��
////////////////////////////////////////////////////////////////////////////////// 	


#define USART_REC_LEN  			200  	//�����������ֽ��� 200
//#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����


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
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	

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
















