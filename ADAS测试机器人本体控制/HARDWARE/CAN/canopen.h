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

//�ĸ����ӵ���ز���
typedef struct _Wheel_Struct_Type
{
	double CmdVel;					//���Ա��������ٶȣ���λm/s
	double FbVel;
	double WheelVel;       //���Ա��������ٶȣ���λm/s
	double CmdWheelVel;       //����ָ���ٶȣ���λm/s
	int CmdMotorvel;				//���Ա�����ת�٣���λr/min
	int FlagForward;			    //���Ա�ʾ���������
	double FbPos;
	double LastFbpos;
	unsigned char PowerFlag;
	ObjectDictSt ObDict[24]; 		//���ڴ洢�����ֵ�
}Wheel_Struct_Type;

//���Ա���ײ�����
typedef struct BaseHalStruct
{
	unsigned int timecount;
	unsigned int Station; 			//���Ա�ʾID����������ȡ��ID������
	unsigned char forwardIO[8];		//���Ա�ʾǰ��8��IO״̬
	unsigned char backIO[8];		//���Ա�ʾ��8��IO״̬
	unsigned char leftIO[8];		//���Ա�ʾ��8��IO״̬
	unsigned char rightIO[8];		//���Ա�ʾ�ҷ�8��IO״̬
	unsigned char Pause;			//���Ա�����ϲ��ȡ������ͣ��־λ
	unsigned char Start;			//���Ա�����ϲ��ȡ����������־λ
	unsigned char ForwardOb;
	unsigned char LeftOb;
	unsigned char InPosition;		//���Ա����Ƿ񵽴�Ŀ���ĵ�λ��־
	unsigned char FbMotor;			//���Ա�������������485ͨ���Ƿ��з���
	unsigned char Fbobstacle[8];	//���Ա����ȡ����8��������̽ͷ���ֵ
	Wheel_Struct_Type WheelHal[2];			//���Ա���2�����ӵ��������
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











