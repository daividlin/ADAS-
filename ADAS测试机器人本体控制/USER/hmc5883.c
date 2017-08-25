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
//	  double xc = -147.7499, yc = -161.3090;//��Բ���������Բ����
//	  double xc = -161.5789, yc = -169.2465;//��Բ���������Բ����

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
	Write_HMC5883(0x02, 0x00);		//��������ģʽ
}

void Write_HMC5883(u8 add, u8 da)
{
    IIC_Start();                  //��ʼ�ź�
    IIC_Send_Byte(SlaveAddress);   //�����豸��ַ+д�ź�
	IIC_Wait_Ack();

    IIC_Send_Byte(add);    //�ڲ��Ĵ�����ַ����ο�����pdf 
	IIC_Wait_Ack();

    IIC_Send_Byte(da);       //�ڲ��Ĵ������ݣ���ο�����pdf
	IIC_Wait_Ack();

    IIC_Stop();                   //����ֹͣ�ź�
}

u8 Read_HMC5883(u8 REG_Address)
{   
	u8 REG_data;
    IIC_Start();                          //��ʼ�ź�
    IIC_Send_Byte(SlaveAddress);           //�����豸��ַ+д�ź�
	IIC_Wait_Ack();

    IIC_Send_Byte(REG_Address);                   //���ʹ洢��Ԫ��ַ����0��ʼ	
	IIC_Wait_Ack();

    IIC_Start();                          //��ʼ�ź�
    IIC_Send_Byte(SlaveAddress+1);         //�����豸��ַ+���ź�
	IIC_Wait_Ack();

    REG_data=IIC_Read_Byte(0);              //�����Ĵ�������
	IIC_Stop();                           //ֹͣ�ź�
    return REG_data; 
}

//******************************************************
//
//��������HMC5883�ڲ��Ƕ����ݣ���ַ��Χ0x3~0x5
//
//******************************************************
void Multiple_read_HMC5883(u8*BUF)
{   u8 i;
    IIC_Start();                          //��ʼ�ź�
    IIC_Send_Byte(SlaveAddress);           //�����豸��ַ+д�ź�
	IIC_Wait_Ack();
    IIC_Send_Byte(0x03);                   //���ʹ洢��Ԫ��ַ����0x3��ʼ	
	IIC_Wait_Ack();
    IIC_Start();                          //��ʼ�ź�
    IIC_Send_Byte(SlaveAddress+1);         //�����豸��ַ+���ź�
	IIC_Wait_Ack();
	 for (i=0; i<6; i++)                      //������ȡ6����ַ���ݣ��洢��BUF
    {
        
        if (i == 5)
        {
           BUF[i] = IIC_Read_Byte(0);          //���һ��������Ҫ��NOACK
        }
        else
        {
          BUF[i] = IIC_Read_Byte(1);          //����ACK
       }
   }
    IIC_Stop();                          //ֹͣ�ź�
}






