#include "mpu6050.h"
#include "sys.h"
#include "delay.h"
//#include "usart.h"   

typedef enum _MPU_REG_ENUM_TYPE
{
	MPU_SELF_TESTX_REG = 0X0D,	//�Լ�Ĵ���X
	MPU_SELF_TESTY_REG = 0X0E,	//�Լ�Ĵ���Y
	MPU_SELF_TESTZ_REG = 0X0F,	//�Լ�Ĵ���Z
	MPU_SELF_TESTA_REG = 0X10,	//�Լ�Ĵ���A
	MPU_SAMPLE_RATE_REG = 0X19,	//����Ƶ�ʷ�Ƶ��
	MPU_CFG_REG = 0X1A,	//���üĴ���
	MPU_GYRO_CFG_REG = 0X1B,	//���������üĴ���
	MPU_ACCEL_CFG_REG = 0X1C,	//���ٶȼ����üĴ���
	MPU_MOTION_DET_REG = 0X1F,	//�˶���ֵⷧ���üĴ���
	MPU_FIFO_EN_REG = 0X23,	//FIFOʹ�ܼĴ���
	MPU_I2CMST_CTRL_REG = 0X24,	//IIC�������ƼĴ���
	MPU_I2CSLV0_ADDR_REG = 0X25,	//IIC�ӻ�0������ַ�Ĵ���
	MPU_I2CSLV0_REG = 0X26,	//IIC�ӻ�0���ݵ�ַ�Ĵ���
	MPU_I2CSLV0_CTRL_REG = 0X27,	//IIC�ӻ�0���ƼĴ���
	MPU_I2CSLV1_ADDR_REG = 0X28,	//IIC�ӻ�1������ַ�Ĵ���
	MPU_I2CSLV1_REG = 0X29,	//IIC�ӻ�1���ݵ�ַ�Ĵ���
	MPU_I2CSLV1_CTRL_REG = 0X2A,	//IIC�ӻ�1���ƼĴ���
	MPU_I2CSLV2_ADDR_REG = 0X2B,	//IIC�ӻ�2������ַ�Ĵ���
	MPU_I2CSLV2_REG = 0X2C,	//IIC�ӻ�2���ݵ�ַ�Ĵ���
	MPU_I2CSLV2_CTRL_REG = 0X2D,	//IIC�ӻ�2���ƼĴ���
	MPU_I2CSLV3_ADDR_REG = 0X2E,	//IIC�ӻ�3������ַ�Ĵ���
	MPU_I2CSLV3_REG = 0X2F,	//IIC�ӻ�3���ݵ�ַ�Ĵ���
	MPU_I2CSLV3_CTRL_REG = 0X30,	//IIC�ӻ�3���ƼĴ���
	MPU_I2CSLV4_ADDR_REG = 0X31,	//IIC�ӻ�4������ַ�Ĵ���
	MPU_I2CSLV4_REG = 0X32,	//IIC�ӻ�4���ݵ�ַ�Ĵ���
	MPU_I2CSLV4_DO_REG = 0X33,	//IIC�ӻ�4д���ݼĴ���
	MPU_I2CSLV4_CTRL_REG = 0X34,	//IIC�ӻ�4���ƼĴ���
	MPU_I2CSLV4_DI_REG = 0X35,	//IIC�ӻ�4�����ݼĴ���
	MPU_I2CMST_STA_REG = 0X36,	//IIC����״̬�Ĵ���
	MPU_INTBP_CFG_REG = 0X37,	//�ж�/��·���üĴ���
	MPU_INT_EN_REG = 0X38,	//�ж�ʹ�ܼĴ���
	MPU_INT_STA_REG = 0X3A,	//�ж�״̬�Ĵ���
	MPU_ACCEL_XOUTH_REG = 0X3B,	//���ٶ�ֵ,X���8λ�Ĵ���
	MPU_ACCEL_XOUTL_REG = 0X3C,	//���ٶ�ֵ,X���8λ�Ĵ���
	MPU_ACCEL_YOUTH_REG = 0X3D,	//���ٶ�ֵ,Y���8λ�Ĵ���
	MPU_ACCEL_YOUTL_REG = 0X3E,	//���ٶ�ֵ,Y���8λ�Ĵ���
	MPU_ACCEL_ZOUTH_REG = 0X3F,	//���ٶ�ֵ,Z���8λ�Ĵ���
	MPU_ACCEL_ZOUTL_REG = 0X40,	//���ٶ�ֵ,Z���8λ�Ĵ���
	MPU_TEMP_OUTH_REG = 0X41,	//�¶�ֵ�߰�λ�Ĵ���
	MPU_TEMP_OUTL_REG = 0X42,	//�¶�ֵ��8λ�Ĵ���
	MPU_GYRO_XOUTH_REG = 0X43,	//������ֵ,X���8λ�Ĵ���
	MPU_GYRO_XOUTL_REG = 0X44,	//������ֵ,X���8λ�Ĵ���
	MPU_GYRO_YOUTH_REG = 0X45,	//������ֵ,Y���8λ�Ĵ���
	MPU_GYRO_YOUTL_REG = 0X46,	//������ֵ,Y���8λ�Ĵ���
	MPU_GYRO_ZOUTH_REG = 0X47,	//������ֵ,Z���8λ�Ĵ���
	MPU_GYRO_ZOUTL_REG = 0X48,	//������ֵ,Z���8λ�Ĵ���
	MPU_I2CSLV0_DO_REG = 0X63,	//IIC�ӻ�0���ݼĴ���
	MPU_I2CSLV1_DO_REG = 0X64,	//IIC�ӻ�1���ݼĴ���
	MPU_I2CSLV2_DO_REG = 0X65,	//IIC�ӻ�2���ݼĴ���
	MPU_I2CSLV3_DO_REG = 0X66,	//IIC�ӻ�3���ݼĴ���
	MPU_I2CMST_DELAY_REG = 0X67,	//IIC������ʱ����Ĵ���
	MPU_SIGPATH_RST_REG = 0X68,	//�ź�ͨ����λ�Ĵ���
	MPU_MDETECT_CTRL_REG = 0X69,	//�˶������ƼĴ���
	MPU_USER_CTRL_REG = 0X6A,	//�û����ƼĴ���
	MPU_PWR_MGMT1_REG = 0X6B,	//��Դ����Ĵ���1
	MPU_PWR_MGMT2_REG = 0X6C,	//��Դ����Ĵ���2 
	MPU_FIFO_CNTH_REG = 0X72,	//FIFO�����Ĵ����߰�λ
	MPU_FIFO_CNTL_REG = 0X73,	//FIFO�����Ĵ����Ͱ�λ
	MPU_FIFO_RW_REG = 0X74,	//FIFO��д�Ĵ���
	MPU_DEVICE_ID_REG = 0X75	//����ID�Ĵ���
}MPU_REG_ENUM_TYPE;

//���AD0��(9��)�ӵ�,IIC��ַΪ0X68(���������λ).
//�����V3.3,��IIC��ַΪ0X69(���������λ).
#define MPU_ADDR				0X68


////��Ϊ�������GND,����תΪ��д��ַ��,Ϊ0XD1��0XD0(�����GND,��Ϊ0XD3��0XD2)  
//#define MPU_READ    0XD1
//#define MPU_WRITE   0XD0


//��ʼ��MPU6050
//����ֵ:0,�ɹ�
//    ����,�������
u8 mpuInit(void)
{
	u8 res;
	IIC_Init();//��ʼ��IIC����
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80);	//��λMPU6050
	delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00);	//����MPU6050 
	MPU_Set_Gyro_Fsr(3);					//�����Ǵ�����,��2000dps
	MPU_Set_Accel_Fsr(0);					//���ٶȴ�����,��2g
	MPU_Set_Rate(50);						//���ò�����50Hz
	MPU_Write_Byte(MPU_INT_EN_REG, 0X00);	//�ر������ж�
	MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00);	//I2C��ģʽ�ر�
	MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);	//�ر�FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80);	//INT���ŵ͵�ƽ��Ч
	res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if (res == MPU_ADDR)//����ID��ȷ
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01);	//����CLKSEL,PLL X��Ϊ�ο�
		MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00);	//���ٶ��������Ƕ�����
		MPU_Set_Rate(50);						//���ò�����Ϊ50Hz
	}
	else return 1;
	return 0;
}
//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3);//���������������̷�Χ  
}
//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3);//���ü��ٶȴ����������̷�Χ  
}
//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data = 0;
	if (lpf >= 188)data = 1;
	else if (lpf >= 98)data = 2;
	else if (lpf >= 42)data = 3;
	else if (lpf >= 20)data = 4;
	else if (lpf >= 10)data = 5;
	else data = 6;
	return MPU_Write_Byte(MPU_CFG_REG, data);//�������ֵ�ͨ�˲���  
}
//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if (rate > 1000)rate = 1000;
	if (rate < 4)rate = 4;
	data = 1000 / rate - 1;
	data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data);	//�������ֵ�ͨ�˲���
	return MPU_Set_LPF(rate / 2);	//�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temperature(void)
{
	u8 buf[2];
	short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
	raw = ((u16)buf[0] << 8) | buf[1];
	temp = 36.53 + ((double)raw) / 340;
	return temp * 100;;
}
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
	u8 buf[6], res;
	res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		*gx = ((u16)buf[0] << 8) | buf[1];
		*gy = ((u16)buf[2] << 8) | buf[3];
		*gz = ((u16)buf[4] << 8) | buf[5];
	}
	return res;;
}
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
	u8 buf[6], res;
	res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		*ax = ((u16)buf[0] << 8) | buf[1];
		*ay = ((u16)buf[2] << 8) | buf[3];
		*az = ((u16)buf[4] << 8) | buf[5];
	}
	return res;;
}
//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
	u8 i;
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 0);//����������ַ+д����	
	if (IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);	//д�Ĵ�����ַ
	IIC_Wait_Ack();		//�ȴ�Ӧ��
	for (i = 0; i < len; i++)
	{
		IIC_Send_Byte(buf[i]);	//��������
		if (IIC_Wait_Ack())		//�ȴ�ACK
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_Stop();
	return 0;
}
//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 MPU_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 0);//����������ַ+д����	
	if (IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);	//д�Ĵ�����ַ
	IIC_Wait_Ack();		//�ȴ�Ӧ��
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 1);//����������ַ+������	
	IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	while (len)
	{
		if (len == 1)*buf = IIC_Read_Byte(0);//������,����nACK 
		else *buf = IIC_Read_Byte(1);		//������,����ACK  
		len--;
		buf++;
	}
	IIC_Stop();	//����һ��ֹͣ���� 
	return 0;
}
//IICдһ���ֽ� 
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Byte(u8 reg, u8 data)
{
	IIC_Start();
	IIC_Send_Byte((MPU_ADDR << 1) | 0);//����������ַ+д����	
	if (IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);	//д�Ĵ�����ַ
	IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	IIC_Send_Byte(data);//��������
	if (IIC_Wait_Ack())	//�ȴ�ACK
	{
		IIC_Stop();
		return 1;
	}
	IIC_Stop();
	return 0;
}
//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
	IIC_Start();
	IIC_Send_Byte((MPU_ADDR << 1) | 0);//����������ַ+д����	
	IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	IIC_Send_Byte(reg);	//д�Ĵ�����ַ
	IIC_Wait_Ack();		//�ȴ�Ӧ��
	IIC_Start();
	IIC_Send_Byte((MPU_ADDR << 1) | 1);//����������ַ+������	
	IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	res = IIC_Read_Byte(0);//��ȡ����,����nACK 
	IIC_Stop();			//����һ��ֹͣ���� 
	return res;
}


