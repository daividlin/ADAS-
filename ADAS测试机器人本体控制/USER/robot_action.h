#ifndef __ROBOT_ACTION_H
#define __ROBOT_ACTION_H

#include "basicfunc.h"
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
typedef struct curve_plan_sturct {
	double x, y, heading, v;
	int type;
	int heart_beat_rx_rk3288;
	int flag_new_cmdframe;
	int flag_new_cmdframe_old;
}CURVEPLAN;

//---------------------------------------------------
typedef struct error_sturct {
	unsigned char table[10];
	unsigned char level;
}ROBOTERRORTYPE;
//---------------------------------------------------


typedef struct initial_struct
{
	double circle_ang, circle_x, circle_y, circle_r_sum, circle_center_x, circle_center_y, \
		uwbtag2space_ang, uwbtag2robot_ang_sum;
	short circle_xy_cnt, circle_ang_cnt, stage;

}ROBOTINITIAL;


typedef struct motion_struct
{
	int i, flag_timer_5ms, flag_timer_10ms, flag_timer_100ms, timer_count_100ms, flag_timer_200ms, timer_count_200ms, flag_timer_500ms, timer_count_500ms, flag_timer_1s, timer_count_1s,
		type, status;
	double x, y, heading, v, omg, omg_motor, omg_gyro, omg_heading;
}ROBOTMOTIONTYPE;

typedef struct _GYRO_STRUCT_TYPE {
	double omg_deg_raw, omg_his_add, heading;//,omg_his[1000];
	double omg_deg_zerobias, omg_deg_correct, aacx, aacy, aacz;
	int zero_bias_flag, flag_static, index;
}GYRO_STRUCT_TYPE;

typedef struct control_struct
{
	double offsetx, offsety, pre_offsety, offset_heading, pre_offset_heading, \
		target_x, target_y, target_heading, target_v, pre_target_v, target_omg, \
		max_v, max_acc, min_v, max_omg, min_omg, max_angacc, \
		offsetx_int, ki_offset_value, ki_heading_value, \
		cmd_delta_heading, cmd_heading, \
		decel_dis, decel_ang, direction, decel_stage, \
		tol_offsetx, tol_offsety, tol_ang, compesate_ang, targetomg_offsetheading, targetomg_offsety;
	ROBOTINITIAL initial;
	int handle_switch;
}ROBOTCONTROLTYPE;

typedef struct cmd_struct
{
	int dataarrive, type, cmdstop;
	double target_x, target_y, target_heading, test_target_x[3], test_target_y[3];
}ROBOTCMDTYPE;

typedef struct task_struct
{
	int type, status, cur_num; double target_x, target_y, target_heading, target_max_omg, target_max_v;

}ROBOTTASKTYPE;

typedef struct pos_struct
{
	float sa, sx, sy, sz;
}ROADPOS;

typedef union uplog_union
{
	float f;
	unsigned char c[4];
}UPLOG;

typedef struct log_struct
{
	char header0;
	char header1;
	char len;
	char id;
	char type;
	char gyroSt;    //陀螺状态
	short gpsSt;     //GPS状态
	int posX;     //位置x
	int posY;
	int posAngle;
	int compassAng;
	int magAng;
	int gpsX;
	int gpsY;
	char gpsQual;
	char statsNum;
	char crc;

}LOG_STRUCT;


typedef struct ctrlcmd_struct
{
	char header0;
	char header1;
	char len;
	char id;
	int type;
	int targetX;     //位置x
	int targetY;
	int targetAngle;
	int crc;

}CTRLCMD_STRUCT_TYPE;
//extern ROADPOS road_pos[50];

extern UPLOG uplog;
extern int pos_num;
extern ROBOTMOTIONTYPE robot_motion;
extern ROBOTCONTROLTYPE control;
extern ROBOTCMDTYPE cmd;
extern ROBOTTASKTYPE task[2];
extern ROBOTTASKTYPE tasktype;
extern GYRO_STRUCT_TYPE gyro;

extern CURVEPLAN curve_plan;
extern ROBOTERRORTYPE error;;
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




void parsePosition(void);

void robotStopCtrl(void);
void speed2MotorCalc(double tv, double tomg);

void getcmdomg(double targetheading, double nowheading);

void excuteRK3288CMD(void);

void parseCMD(void);
unsigned long SendCMD_SPI(unsigned long cmd, int bitnum);
int InitialSPIGYRO(void);
void rd_omg_gyro(void);
void SPI_Init(void);
void move_test(void);
int action_initial_ang(void);
void initial_angle(void);
void tx_uwb(void);
int parase_uwb(void);
int initial_data(void);
void rxOMGMPU6050(void);

int rd_angle_hmc5883l(void);
int curve_planning(CURVEPLAN *ioplan);
int curve_planning1(CURVEPLAN *ioplan);
extern unsigned int rxcmd_cnt;

int error_process(ROBOTERRORTYPE error);
#endif

