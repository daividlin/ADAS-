#ifndef __MY_CAN_H
#define __MY_CAN_H 		

#include "sys.h"	
#include "typeinclude.h"
//电机控制相关参数
#define SPD_SET_LEN           8

#define MOTORDRIVE_ID_LEFT         0x01
#define MOTORDRIVE_ID_RIGHT        0x02

#define MOTORDRIVE_CMD_SET_VELOCITY           0x6f
#define MOTORDRIVE_CMD_GET_ACT_VELOCITY           0x3f
#define MOTORDRIVE_CMD_ENABLE         0x15
#define MOTORDRIVE_CMD_DISABLE  0x16
#define MOTORDRIVE_CMD_CLEAR_ERROR  0x17

#define MOTORDRIVE_CMD_BRAKE_OPERATION  0x1e
#define MOTORDRIVE_CMD_SET_PARAM 0x96

#define MOTORDRIVE_CMD_BRAKEOPERATION_ENABLE 1  
#define MOTORDRIVE_CMD_BRAKEOPERATION_DISABLE 0

#define MOTORDRIVE_PARAM_NUN_LIFEGUARD_MODE 276
#define MOTORDRIVE_PARAM_VALUE_LIFEGUARD_MODE 9999
#define MOTORDRIVE_PARAM_NUN_LIFEGUARD_TIME 274
#define MOTORDRIVE_PARAM_VALUE_LIFEGUARD_TIME 100

//CAN1接收RX0中断使能
#define CAN1_RX0_INT_ENABLE			1		 			//0,不使能;1,使能.								    
									 							 				    
u8 can1ModeConfig(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化
u8 CAN1_Tx_Msg(u32 id,u8 ide,u8 rtr,u8 len,u8 *dat);	//发送数据
u8 CAN1_Tx_Staus(u8 mbox);  							//返回发送状态
unsigned char CAN1_Receive_Msg(unsigned char *buf);							//接收数据
unsigned char CAN1_Send_Msg(unsigned char * msg,unsigned char len,unsigned int id);						//发送数据
void CAN1_Rx_Msg(unsigned char fifox,unsigned int *id,unsigned char *ide,unsigned char *rtr,unsigned char *len,unsigned char *dat);//接收数据
unsigned char MOTECCanSend(unsigned char len,unsigned char addr,unsigned short cmdfunc,unsigned char *data);
unsigned char MOTECSetMotorSpd(unsigned char addr,int spd);//spd:rpm
unsigned char getMotorSpeed_MOTEC(unsigned char addr);//spd:rpm
unsigned char wheelMotorSpdSet(int speedLeft,int speedRight);
unsigned char getWheelMotorSpeed2Buf(void);
unsigned char EnableMotorDrive(unsigned char addr);
unsigned char set_param_motec(unsigned char addr,unsigned short param_num,unsigned short param_value);//spd:rpm
unsigned char send_cmd_motordrive(unsigned char addr,unsigned char cmd_id,unsigned short sdata1,unsigned short sdata2);
void getMotorVelo(u8 adr);
void syntronMotorVeloSet(u8 adr, int spd);
#endif














