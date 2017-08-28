#ifndef __LED_H
#define __LED_H	 
#include "sys.h" 

//LED端口定义
#define LED0 PFout(9)	// DS0
#define LED1 PFout(10)	// DS1	 
#define ERelay1 PBout(12)
#define ERelay2 PBout(13)
void LED_Init(void);//初始化	
void brakeConfig(void);

#endif

















