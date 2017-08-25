/****************************************************************************
* Copyright (C), 2013
***************************************************************************/
#include "canopen.h"

//#include "can.h"
#include "stdlib.h"
#include "stdio.h"
#include "led.h"
#include "sys.h"
#include "delay.h"

BaseHalSt gHalData;

/****************************************************************************
* 名    称：void Drivers_Init(void)
* 功    能：initiate the drivers
* 入口参数：null
* 出口参数：null
* 说    明：
* 调用方法：null 
****************************************************************************/

int Drivers_Init(void)
{
	int times = 5;
	int num = 0;
	int ready = 0;
	
	while(times > 0x0)
	{
		LED0=1;//LED0 off
		delay_ms(100);
		LED0=0;//LED0 on
		delay_ms(100);
		times--;
	}
	times = 10;
	while(times >0 && ready != 2)	
	{
	  ready = 0;
		for(num = 0;num < 2;num++)
		{
		  switch(gHalData.WheelHal[num].PowerFlag)
			{
			    case 0:
				  SendSDO(num,IDRXPDO1,0x00000201+num);	
					delay_ms(20);
					SendSDO(num,RXPDO1TYPE,1);	
					delay_ms(20);
					SendSDO(num,NUMRXPDO1,2);	
					delay_ms(20);
					SendSDO(num,RXPDO1OB1,0x60400010);	
					delay_ms(20);
					SendSDO(num,RXPDO2OB2,0x60ff0020);	//0x60ff0020
					delay_ms(20);
					SendSDO(num,IDTXPDO1,0x40000181+num);	
					delay_ms(20);
					SendSDO(num,TXPDO1TYPE,1);	
					delay_ms(20);
					SendSDO(num,NUMTXPDO1,2);	
					delay_ms(20);
					SendSDO(num,TXPDO1OB1,0x60410010);	
					delay_ms(20);
//					SendSDO(num,TXPDO1OB2,0x606c0020);
					SendSDO(num,TXPDO1OB2,0x60640020);	
					delay_ms(20);
					SendSDO(num,HEARTBEATTIME,10000);	
					delay_ms(20);

					SendSDO(num,CMDTYPE,0x03);	//0x03
					delay_ms(20);
					ReadSDO(num,ACTUALTYPE);
					delay_ms(20);
					
					if(0x03 == gHalData.WheelHal[num].ObDict[ACTUALTYPE].Value[0])
					{
						gHalData.WheelHal[num].PowerFlag = 1;
					}
					break;
				case 1:
					SendSDO(num,CONTROLWORD,6);	
					delay_ms(20);
					ReadSDO(num,STATUSWORD);
					delay_ms(20);
					
					if(0x21 == (gHalData.WheelHal[num].ObDict[STATUSWORD].Value[0] & 0x00ff))
					{
					    gHalData.WheelHal[num].PowerFlag = 2;
					}
					break;
				case 2:
					SendSDO(num,CONTROLWORD,15);	
					delay_ms(20);
					ReadSDO(num,STATUSWORD);
					delay_ms(20);

					if(0x37 == (gHalData.WheelHal[num].ObDict[STATUSWORD].Value[0] & 0x00ff))
					{
					    gHalData.WheelHal[num].PowerFlag = 3;
					}
					break;
				case 3:
				    ready++;
					break;
				default:
				    break;
			}
		}
		times--;
	}
	//printf("times and ready are : %d, %d \r\n", times, ready);
	if(2 == ready)
	{
	  PreOPtoOP();
		SendSYNC();
		delay_ms(60);
	}
	else
	{
		return 0;
	}
	return 1;
}
void ObjectConfig(ObjectDictSt* object,unsigned short index,unsigned char subindex,unsigned char size)
{
	object->Index = index;
	object->SubIndex = subindex;
	object->Size = size;
}

int CanOpen_Init(void)
{
  int num = 0;
	
	gHalData.WheelHal[num].PowerFlag = 0;
	gHalData.WheelHal[0].FlagForward = 1;
	gHalData.WheelHal[1].FlagForward = 1;
	
  for(num = 0;num < 2;num++)
	{
	  ObjectConfig(&(gHalData.WheelHal[num].ObDict[CONTROLWORD]),0x6040,0,16);	   //控制字
		ObjectConfig(&(gHalData.WheelHal[num].ObDict[STATUSWORD]),0x6041,0,16);	   //状态字
		ObjectConfig(&(gHalData.WheelHal[num].ObDict[CMDTYPE]),0x6060,0,8);	       //命令运行模式
		ObjectConfig(&(gHalData.WheelHal[num].ObDict[ACTUALTYPE]),0x6061,0,8);	       //当前运行模式
		ObjectConfig(&(gHalData.WheelHal[num].ObDict[TARGETVEL]),0x60ff,0,32);	       //给定命令速度//0x60ff
		ObjectConfig(&(gHalData.WheelHal[num].ObDict[ACTUALTARGETVEL]),0x606b,0,32);  //当前命令速度
		ObjectConfig(&(gHalData.WheelHal[num].ObDict[ACTUALVEL]),0x606c,0,32);	       //当前实际速度
		ObjectConfig(&(gHalData.WheelHal[num].ObDict[TARGETPOS]),0x607a,0,32);	       //给定命令位置
		ObjectConfig(&(gHalData.WheelHal[num].ObDict[ACTUALTARGETPOS]),0x6062,0,32);  //当前命令位置
		ObjectConfig(&(gHalData.WheelHal[num].ObDict[ACTUALPOS]),0x6064,0,32);	       //当前实际位置

		ObjectConfig(&(gHalData.WheelHal[num].ObDict[MAXVEL]),0x607f,0,32);	   //最大速度
		ObjectConfig(&(gHalData.WheelHal[num].ObDict[IDRXPDO1]),0x1400,1,32);	   //接收RXPDO1的COB-ID  0x00000202
		ObjectConfig(&(gHalData.WheelHal[num].ObDict[RXPDO1TYPE]),0x1400,2,8);	   //接收RXPDO1的类型 0x01
		ObjectConfig(&(gHalData.WheelHal[num].ObDict[NUMRXPDO1]),0x1600,0,8);	   //接收RXPDO1的映射的对象数量	0x02
		ObjectConfig(&(gHalData.WheelHal[num].ObDict[RXPDO1OB1]),0x1600,1,32);	   //接收RXPDO1的第一个映射对象	0x60400010
		ObjectConfig(&(gHalData.WheelHal[num].ObDict[RXPDO2OB2]),0x1600,2,32);	   //接收RXPDO1的第二个映射对象	0x60ff0020
		ObjectConfig(&(gHalData.WheelHal[num].ObDict[IDTXPDO1]),0x1800,1,32);	   //发送TXPDO1的COB-ID	0x40000182
		ObjectConfig(&(gHalData.WheelHal[num].ObDict[TXPDO1TYPE]),0x1800,2,8);	   //发送TXPDO1的类型 0x01
		ObjectConfig(&(gHalData.WheelHal[num].ObDict[NUMTXPDO1]),0x1A00,0,8);	   //发送TXPDO1的映射的对象数量	0x02
		ObjectConfig(&(gHalData.WheelHal[num].ObDict[TXPDO1OB1]),0x1A00,1,32);	   //发送TXPDO1的第一个映射对象	0x60410010
		ObjectConfig(&(gHalData.WheelHal[num].ObDict[TXPDO1OB2]),0x1A00,2,32);	   //发送TXPDO1的第二个映射对象	0x606C0020
		ObjectConfig(&(gHalData.WheelHal[num].ObDict[HEARTBEATTIME]),0x1017,0,16);	   //最大速度
	}
	return Drivers_Init();
}

void SendSDO(int drivernum,int objectnum,int value)
{
	u8 Data[8];

  switch(gHalData.WheelHal[drivernum].ObDict[objectnum].Size)
	{
	  case 8:
		  gHalData.WheelHal[drivernum].ObDict[objectnum].Value[0]=value&0x00ff;
			Data[0] = 0x2f;
			break;
		case 16:
		  gHalData.WheelHal[drivernum].ObDict[objectnum].Value[0]=value&0x000000ff;
			gHalData.WheelHal[drivernum].ObDict[objectnum].Value[1]=(value&0x0000ff00)>>8;
			Data[0] = 0x2b;
			break;
		case 24:
		    gHalData.WheelHal[drivernum].ObDict[objectnum].Value[0]=value&0x000000ff;
			gHalData.WheelHal[drivernum].ObDict[objectnum].Value[1]=(value&0x0000ff00)>>8;
			gHalData.WheelHal[drivernum].ObDict[objectnum].Value[2]=(value&0x00ff0000)>>16;
			Data[0] = 0x27;
			break;
		case 32:
		   gHalData.WheelHal[drivernum].ObDict[objectnum].Value[0]=value&0x000000ff;
			gHalData.WheelHal[drivernum].ObDict[objectnum].Value[1]=(value&0x0000ff00)>>8;
			gHalData.WheelHal[drivernum].ObDict[objectnum].Value[2]=(value&0x00ff0000)>>16;
			gHalData.WheelHal[drivernum].ObDict[objectnum].Value[3]=(value&0xff000000)>>24;
			Data[0] = 0x23;
			break;
		default: break;
	}

	Data[1] = gHalData.WheelHal[drivernum].ObDict[objectnum].Index&0x00ff;
	Data[2] = (gHalData.WheelHal[drivernum].ObDict[objectnum].Index&0xff00)>>8;
	Data[3] = gHalData.WheelHal[drivernum].ObDict[objectnum].SubIndex;
	Data[4] = gHalData.WheelHal[drivernum].ObDict[objectnum].Value[0];
	Data[5] = gHalData.WheelHal[drivernum].ObDict[objectnum].Value[1];
	Data[6] = gHalData.WheelHal[drivernum].ObDict[objectnum].Value[2];
	Data[7] = gHalData.WheelHal[drivernum].ObDict[objectnum].Value[3];
	
//	CAN1_Send_Msg_id(0x0600|(drivernum + 1), Data, 8);
}

void ReadSDO(int drivernum,int objectnum)
{
	u8 Data[8];
	
	Data[0] = 0x40;
	Data[1] = gHalData.WheelHal[drivernum].ObDict[objectnum].Index&0x00ff;
	Data[2] = (gHalData.WheelHal[drivernum].ObDict[objectnum].Index&0xff00)>>8;
	Data[3] = gHalData.WheelHal[drivernum].ObDict[objectnum].SubIndex;
	
	CAN1_Send_Msg_id(0x0600|(drivernum + 1), Data, 8);	
}


void PreOPtoOP(void)
{
	u8 Data[2];
	Data[0] =  0x01;
	Data[1] =  0x00;

	CAN1_Send_Msg_id(0x00, Data, 2);
}

void OPtoPreOP(void)
{
	u8 Data[2];
	Data[0] =  0x80;
	Data[1] =  0x00;
	CAN1_Send_Msg_id(0x00, Data, 2);
}

void SendSYNC(void)
{
	u8 Data[2];
	Data[0] =  0x80;
	Data[1] =  0x00;
	CAN1_Send_Msg_id(0x80, Data, 2);
}

void SendPDO(int drivernum)
{
	u8 Data[6];
	
	Data[0] = 0x0f;
	Data[1] = 0x00;
	Data[2] = gHalData.WheelHal[drivernum].CmdMotorvel&0x000000ff;
	Data[3] = (gHalData.WheelHal[drivernum].CmdMotorvel&0x0000ff00)>>8;
	Data[4] = (gHalData.WheelHal[drivernum].CmdMotorvel&0x00ff0000)>>16;
	Data[5] = (gHalData.WheelHal[drivernum].CmdMotorvel&0xff000000)>>24;
	
	CAN1_Send_Msg_id(0x0200|(drivernum + 1), Data, 6);
}

void set_speed(int num, int speed)
{
		gHalData.WheelHal[num].CmdMotorvel = speed;
		SendPDO(num);
		SendSYNC();
}

















