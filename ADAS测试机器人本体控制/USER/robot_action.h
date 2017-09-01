#ifndef __ROBOT_ACTION_H
#define __ROBOT_ACTION_H
#include "typeinclude.h"
#include "basicfunc.h"
#include "canopen.h"
#include "my_can.h"
#include "math.h"
#include "led.h" 
#include "delay.h"  
#include "string.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "gps.h"
#include "..\HARDWARE\joystick\joystick.h"
#include "FreeRTOS.h"
#include "timers.h"

#define ROBOT2
#define SPICLK PBout(13) // SPI_CLK IO
#define SPITEA PBout(12) // SPI_CS IO
#define SPISOMI PBin(14)// SPI_SOMI IO
#define SPISIMO PBout(15)// SPI_SIMO IO
#define TIME_IO PCout(0)
#define GYRO_DIR -1.0
#define MPU6050 1
//------------------------------------------------
//cmd
#define SUB_MOVE 0xf0
#define SUB_TURN 0xf1
#define INITIAL_POS 0xf2
#define E_STOP 0xf3
#define MOVE_S 0xf4
#define TURN_LEFT 0xf5
#define ENABLE 0Xf6
#define DISABLE 0Xf7
#define BRAKE_ENABLE 0Xf8
#define BRAKE_DISABLE 0Xf9
#define TURN_RIGHT 0xfa
#define SUB_TRACKING 0xfb
#define MOVE_BACK 0xfc
//---------------------------------------------------
//base agv
#define LEFT 0
#define RIGHT 1
//---------------------------------------------------
//Timer

#define TIMER_PERIOD 0.01f //
#define MOVE_OFFSET_P 5 //3//2.5
#define MOVE_OFFSET_I 0.0001//0.015
#define MOVE_OFFSET_D 0
#define MOVE_HEADING_P 1//0.6//1.0
#define MOVE_HEADING_I 0.0001//0.0001
#define MOVE_HEADING_D 0
#define MOVE__P 0.8
#define MOVE__I 0.0002
#define MOVE__D 0
//---------------------------------------------------
#ifdef ROBOT1
#define WHEEL_DIA 0.125
#define _WHEEL_REDUCE_RATIO 5.39
#define WHEEL_DIS 0.637
#endif
#ifdef ROBOT2
#define WHEEL_DIA 0.1
#define _WHEEL_REDUCE_RATIO 2.4
#define WHEEL_DIS 0.62
#endif
#ifdef ROBOT3
#define WHEEL_DIA 0.125
#define _WHEEL_REDUCE_RATIO 5.39
#define WHEEL_DIS 0.637
#endif
#define TASK_READY 2
#define TASK_DOING 1
#define TASK_FINISH 0
#define STOP 0
#define MOTION_DOING 1
#define MOTION_FINISH 0
#define LINE_TYPE_STRAIGHT 1
#define LINE_TYPE_CURVE 2
#define ERROR_INDEX_MOTOR 0
#define ERROR_INDEX_OFFSET 1
#define ERROR_LEVEL 1

typedef struct _GYRO_VAL_STRUCT_TYPE
{
	short x;
	short y;
	short z;

}GYRO_VAL_STRUCT_TYPE;
//---------------------------------------------------
//extern ROADPOS road_pos[50];

extern MOTION_STRUCT_TYPE robot_motion;
extern CTRL_STRUCT_TYPE control;
extern CMD_STRUCT_TYPE cmd;
extern TASK_STRUCT_TYPE task[2];
extern TASK_STRUCT_TYPE tasktype;
extern GYRO_STRUCT_TYPE gyro;
extern float fittingR;
extern float adjustr;

extern USART_SEND_STRUCT_TYPE uart2rk3288;
extern USART_SEND_STRUCT_TYPE uart2cmdboard;
extern ROBOTUART PC2STUsart;
extern ROBOTUART dcm2stm_uart;

/*
�������ܣ�ʹ��PID�����ٶ�
���������err---ƫ��ֵ��speed_dst---�ٶ�Ŀ��ٷֱ�0~20
����ֵ������ƫ����
*/
//int speed_pid(float err, float speed_dst);

///*
//�������ܣ�ֱ�ӵ���Ŀ���
//���������ָ��Ŀ�������x,y����
//����ֵ��1---�ɹ���0---ʧ��
//*/
//int goto_pos(float x, float y);

///*
//�������ܣ�ѧϰ����
//���������mode: -1��ʼ��ȡ����㣬0~999---��ȡ�������ɲ�����"1:/point_pos_num.dat"�ļ���
//����ֵ��1---�ɹ���0---ʧ��
//��ע��ʹ��ȫ�ֱ���road_pos[100]����ѧϰ��������㣬pos_num����ѧϰ������������
//*/
//int get_road(int num);

///*
//�������ܣ���"1:/point_pos_num.dat"�ļ��ж�ȡroad_pos[100]��pos_num
//����ֵ��1---�ɹ���0---ʧ��
//��ע��ʹ��ȫ�ֱ���road_pos[100];�����ȡ���������
//*/
//int read_road(int num);

///*
//�������ܣ���ȡ�������
//����ֵ���������
//*/
//int get_nearest_point();
extern void parsePosition(void);
void bufPushToLeft(u32 *ptr, u32 val);
static void robotStopCtrl(void);
extern void speed2MotorCalc(double tv, double tomg);
static void getcmdomg(double targetheading, double nowheading);
extern void excuteRK3288CMD(void);
extern  void parseCMD(void);
static unsigned long sendCmdBySpi(unsigned long cmd, int bitnum);
extern void rd_omg_gyro(void);
extern int initGyroData(void);
extern void rxOMGMPU6050(void);
#endif

