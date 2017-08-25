#ifndef __TEST_H
#define __TEST_H
#include "test.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//定时器 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/4
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	  
#define MAIN_MOVE							0x01
#define	SUB_SETSPD						0x01
#define SUB_LEFT							0X02
#define	SUB_RIGHT							0X03
#define	SUB_FORWARD						    0X04
#define	SUB_BACK							0X05
#define	SUB_STOP							0X06


/**************************** ???  ***********************/
//---------------------------------------------------
//base agv
#define LEFT 0
#define RIGHT 1

#define PI 3.1415926535898
//---------------------------------------------------
//Timer
#define TIMER0PERIOD 0.1 //
#define TIMER1PERIOD 0.01 //
#define TIMEOUT_PERIOD_CANTX 1 

//---------------------------------------------------

#define WHEEL_DIA 0.151
#define WHEEL_REDUCE_RATIO 35.0
#define WHEEL_DIS 0.44

//---------------------------------------------------

enum FAULTSTATUS
{
      FALSE,TRUE
};

enum STARTSTATUS
{
     Y,N
};


//---------------------------------------------------------------
//Timer Period
//


struct TIMEOUTCOUNT_Type {
    
	unsigned int GyroRx;
	unsigned int timer;
	
	
};


struct GYRO_Type {
	unsigned char Flag_AllowZerobias;
	unsigned char Flag_ZerobiasComplete;
	double RawOmg_Degs;
	double ZeroBiasOmg_Degs;
	double OmgHis_Degs[1000];
	unsigned char ZeroBiasIndex; 
    double ActualOmg_Degs;
	
};	
struct Cmd_Type
{
    unsigned char dataarrive;
    unsigned char MainType;
    unsigned char SubType;
    
};
struct Robot_Type
{
    unsigned char EnableStatus;
    double V;
    double Omg;
    double WheelOmg;

};

//---------------------------------------------------------------
//Define User's structure variable
//



struct USER_REGS{
       //USER

	   struct TIMEOUTCOUNT_Type TimeoutCount; //
	   struct GYRO_Type Gyro;                 //
	          // LEFT,RIGHT
       struct Robot_Type Robot;

       struct Cmd_Type Cmd;
};



extern struct USER_REGS UserRegs;

#endif



