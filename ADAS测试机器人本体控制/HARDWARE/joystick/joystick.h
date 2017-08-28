#ifndef _JOY_STICK_H
#define _JOY_STICK_H

#include "stm32f4xx.h" 
#include "sys.h"
#include "delay.h"
//#include "usart.h"

void joyStickIOConfig(void);

typedef struct _JOY_STICK_BUF_TYPE
{
    u32 val[6];
}JOY_STICK_BUF_TYPE;


JOY_STICK_BUF_TYPE js_ppm(void);
extern JOY_STICK_BUF_TYPE handleData;
#endif
