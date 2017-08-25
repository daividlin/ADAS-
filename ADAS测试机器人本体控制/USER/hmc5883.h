#ifndef __HMC5883L_H
#define __HMC5883L_H
#include"sys.h"
#include"myiic.h"
#define SlaveAddress 0x3c
#define COMPASS_ZEROBIAS 10.6f/180.0f*PI
#define CALIBRATE_CX -257.6236
#define CALIBRATE_CY -112.2877
typedef struct compass_struct
{	
    double x,y,z,angle,angle_bias,xmax,ymax,xmin,ymin,xc,yc,turn_dis,var,buff[10],heading,diffomg[5],omg,
			     mean_diffomg,diff_1st,diff_2nd;
	  char flag_calibrate;
}ROBOTCOMPASS;

extern ROBOTCOMPASS compass;
void Init_HMC5883(void);
void Write_HMC5883(u8 add, u8 da);
u8 Read_HMC5883(u8 REG_Address);
void Multiple_read_HMC5883(u8*BUF);
void Calibrate(double dis);
#endif
