#include "hmc5883.h"
#include "basicfunc.h"
#include "math.h"

ROBOTCOMPASS compass;

#define CalThreshold 0
void Calibrate(double dis)
{   
		if(compass.x > compass.xmax) compass.xmax = compass.x;
		if(compass.x < compass.xmin) compass.xmin = compass.x;
	  if(compass.y > compass.ymax) compass.ymax = compass.y;
		if(compass.y < compass.ymin) compass.ymin = compass.y;
		if(dis>2*PI)
		{
		    compass.flag_calibrate = 1;
			  if(fabs(compass.xmax - compass.xmin) > CalThreshold)
        {
            compass.xc = (compass.xmax + compass.xmin)/2;
        }
        if(fabs(compass.ymax - compass.ymin) > CalThreshold)
        {
            compass.yc= (compass.ymax + compass.ymin)/2;
        }
		}		
}


int rd_angle_hmc5883l(void)
{
    int re=0,i;
	  u8 BUF[6];
	  int x,y,z;
//	  double xc = -147.7499, yc = -161.3090;//椭圆假设求解椭圆中心
//	  double xc = -161.5789, yc = -169.2465;//椭圆假设求解椭圆中心

//	  double xc = -245.38,yc = -95.9224;	
	  Multiple_read_HMC5883(BUF);
		x=BUF[0] << 8 | BUF[1]; //Combine MSB and LSB of X Data output register
	  z=BUF[2] << 8 | BUF[3]; //Combine MSB and LSB of Z Data output register
	  y=BUF[4] << 8 | BUF[5]; //Combine MSB and LSB of Y Data output register
	  if(x>0x7fff)x-=0xffff;	  
    if(y>0x7fff)y-=0xffff;	 
		compass.x = x;
	  compass.y = y;
		//if(compass.flag_calibrate == 1)
		{			
			  compass.angle = (360 - (atan2((double)y-CALIBRATE_CY,(double)x-CALIBRATE_CX) * (180 / PI) + 180))/180*PI; // angle in degrees	  
	      compass.angle = compass.angle + COMPASS_ZEROBIAS;
		    compass.angle = angle2HalfRadian(compass.angle);				
		}
		for(i=1;i<10;i++)
		{
		    compass.buff[i-1] = compass.buff[i];
		}
		compass.buff[10-1] = compass.angle;
	  compass.var = get_var(compass.buff,10);	
		return re;

}	

void Init_HMC5883(void)
{
	IIC_Init();
	Write_HMC5883(0x02, 0x00);		//连续测量模式
}

void Write_HMC5883(u8 add, u8 da)
{
    IIC_Start();                  //起始信号
    IIC_Send_Byte(SlaveAddress);   //发送设备地址+写信号
	IIC_Wait_Ack();

    IIC_Send_Byte(add);    //内部寄存器地址，请参考中文pdf 
	IIC_Wait_Ack();

    IIC_Send_Byte(da);       //内部寄存器数据，请参考中文pdf
	IIC_Wait_Ack();

    IIC_Stop();                   //发送停止信号
}

u8 Read_HMC5883(u8 REG_Address)
{   
	u8 REG_data;
    IIC_Start();                          //起始信号
    IIC_Send_Byte(SlaveAddress);           //发送设备地址+写信号
	IIC_Wait_Ack();

    IIC_Send_Byte(REG_Address);                   //发送存储单元地址，从0开始	
	IIC_Wait_Ack();

    IIC_Start();                          //起始信号
    IIC_Send_Byte(SlaveAddress+1);         //发送设备地址+读信号
	IIC_Wait_Ack();

    REG_data=IIC_Read_Byte(0);              //读出寄存器数据
	IIC_Stop();                           //停止信号
    return REG_data; 
}

//******************************************************
//
//连续读出HMC5883内部角度数据，地址范围0x3~0x5
//
//******************************************************
void Multiple_read_HMC5883(u8*BUF)
{   u8 i;
    IIC_Start();                          //起始信号
    IIC_Send_Byte(SlaveAddress);           //发送设备地址+写信号
	IIC_Wait_Ack();
    IIC_Send_Byte(0x03);                   //发送存储单元地址，从0x3开始	
	IIC_Wait_Ack();
    IIC_Start();                          //起始信号
    IIC_Send_Byte(SlaveAddress+1);         //发送设备地址+读信号
	IIC_Wait_Ack();
	 for (i=0; i<6; i++)                      //连续读取6个地址数据，存储中BUF
    {
        
        if (i == 5)
        {
           BUF[i] = IIC_Read_Byte(0);          //最后一个数据需要回NOACK
        }
        else
        {
          BUF[i] = IIC_Read_Byte(1);          //返回ACK
       }
   }
    IIC_Stop();                          //停止信号
}






