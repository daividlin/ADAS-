/****************************************************************************
* Copyright (C), 2013
***************************************************************************/
#ifndef __CANOPEN_H
#define __CANOPEN_H

enum _ENUM_OBJECT_TYPE
{
	CONTROLWORD = 0,
	STATUSWORD,
	CMDTYPE,
	ACTUALTYPE,
	TARGETVEL,
	ACTUALTARGETVEL,
	ACTUALVEL,
	TARGETPOS,
	ACTUALTARGETPOS,
	ACTUALPOS,
	MAXVEL,
	IDRXPDO1,
	RXPDO1TYPE,
	NUMRXPDO1,
	RXPDO1OB1,
	RXPDO2OB2,
	IDTXPDO1,
	TXPDO1TYPE,
	NUMTXPDO1,
	TXPDO1OB1,
	TXPDO1OB2,
	HEARTBEATTIME
};

typedef struct ObjectDictStruct
{
	unsigned short Index;
	unsigned char SubIndex;
	unsigned char Size;
	unsigned char Value[4];
}ObjectDictSt;

//四个轮子的相关参数
typedef struct _Wheel_Struct_Type
{
	double CmdVel;					//用以保存轮子速度，单位m/s
	double FbVel;
	double WheelVel;       //用以保存轮子速度，单位m/s
	double CmdWheelVel;       //轮子指令速度，单位m/s
	int CmdMotorvel;				//用以保存电机转速，单位r/min
	int FlagForward;			    //用以表示电机正方向；
	double FbPos;
	double LastFbpos;
	unsigned char PowerFlag;
	ObjectDictSt ObDict[24]; 		//用于存储对象字典
}Wheel_Struct_Type;

//用以保存底层数据
typedef struct BaseHalStruct
{
	unsigned int timecount;
	unsigned int Station; 			//用以表示ID卡读卡器读取的ID卡编码
	unsigned char forwardIO[8];		//用以表示前方8个IO状态
	unsigned char backIO[8];		//用以表示后方8个IO状态
	unsigned char leftIO[8];		//用以表示左方8个IO状态
	unsigned char rightIO[8];		//用以表示右方8个IO状态
	unsigned char Pause;			//用以保存从上层读取到的暂停标志位
	unsigned char Start;			//用以保存从上层读取到的启动标志位
	unsigned char ForwardOb;
	unsigned char LeftOb;
	unsigned char InPosition;		//用以保存是否到达目标点的到位标志
	unsigned char FbMotor;			//用以保存与驱动器的485通信是否有返回
	unsigned char Fbobstacle[8];	//用以保存读取到的8个超声波探头测距值
	Wheel_Struct_Type WheelHal[2];			//用以保存2个轮子的相关数据
}BaseHalSt;

extern BaseHalSt gHalData;

int CanOpen_Init(void);
void SendSDO(int drivernum, int objectnum, int value);
void ReadSDO(int drivernum, int objectnum);
void PreOPtoOP(void);
void SendSYNC(void);
void SendPDO(int drivernum);

void set_speed(int num, int speed);

#endif











