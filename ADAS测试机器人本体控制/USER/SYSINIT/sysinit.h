#ifndef _SYS_INIT_H
#define _SYS_INIT_H
#include "delay.h"
#include "timer.h"
#include "usart.h"
#include "led.h"
#include "key.h"  
#include "my_can.h"
void stm32SysInit(void);

void peripheralsTimersInit(void);
void peripheralsUARTInit(void);
void motorInit(void);
#endif // !_SYS_INIT_H
