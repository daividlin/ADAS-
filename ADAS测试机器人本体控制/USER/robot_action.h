#ifndef __ROBOT_ACTION_H
#define __ROBOT_ACTION_H
#include "typeinclude.h"
//#include "basicfunc.h"
//////////////////////////////////////////////
//-----------------------------------------------
//define spi io
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
//---------------------------------------------------
//extern ROADPOS road_pos[50];

extern UPLOG_UNION_TYPE uplog;
extern int pos_num;
extern MOTION_STRUCT_TYPE robot_motion;
extern CTRL_STRUCT_TYPE control;
extern CMD_STRUCT_TYPE cmd;
extern TASK_STRUCT_TYPE task[2];
extern TASK_STRUCT_TYPE tasktype;
extern GYRO_STRUCT_TYPE gyro;
extern float fittingR;
extern float adjustr;

extern CURVE_PLAN_STRUCT_TYPE curve_plan;
extern ERR_STRUCT_TYPE error;;
/*
函数功能：使用PID控制速度
输入参数：err---偏差值，speed_dst---速度目标百分比0~20
返回值：调节偏移量
*/
//int speed_pid(float err, float speed_dst);

///*
//函数功能：直接到达目标点
//输入参数：指定目标坐标点x,y坐标
//返回值：1---成功，0---失败
//*/
//int goto_pos(float x, float y);

///*
//函数功能：学习函数
//输入参数：mode: -1开始获取坐标点，0~999---获取坐标点完成并存入"1:/point_pos_num.dat"文件中
//返回值：1---成功，0---失败
//备注：使用全局变量road_pos[100]保存学习到的坐标点，pos_num保存学习到的坐标点个数
//*/
//int get_road(int num);

///*
//函数功能：从"1:/point_pos_num.dat"文件中读取road_pos[100]和pos_num
//返回值：1---成功，0---失败
//备注：使用全局变量road_pos[100];保存读取到的坐标点
//*/
//int read_road(int num);

///*
//函数功能：获取最近点编号
//返回值：最近点编号
//*/
//int get_nearest_point();
extern void parsePosition(void);
static void robotStopCtrl(void);
extern void speed2MotorCalc(double tv, double tomg);
static void getcmdomg(double targetheading, double nowheading);
extern void excuteRK3288CMD(void);
extern  void parseCMD(void);
static unsigned long SendCMD_SPI(unsigned long cmd, int bitnum);
//static int InitialSPIGYRO(void);
extern void rd_omg_gyro(void);
//static void SPI_Init(void);
extern int initial_data(void);
extern void rxOMGMPU6050(void);
//int rd_angle_hmc5883l(void);
//int curve_planning(CURVEPLAN *ioplan);
//int curve_planning1(CURVEPLAN *ioplan);
extern unsigned int rxcmd_cnt;
int error_process(ERR_STRUCT_TYPE error);
#endif

