#ifndef _TYPE_INCLUDE_H
#define _TYPE_INCLUDE_H
#include "stm32f4xx.h"
#define PI 3.1415926535898

typedef struct _ROBOT_PID_STRUCT_TYPE
{
	double p;
	double	i;
	double	d;
	double	value;
	double	ivalue;
}ROBOT_PID_STRUCT_TYPE;

typedef struct _HUAWEI_CMD_STRUCT_TYPE
{
	double time;
	double longitude;
	double latitude;
	double speed;
	double heading;
}HUAWEI_CMD_STRUCT_TYPE;

typedef struct _KALMAN_STRUCT_TYPE
{
	double x, z, r, p, q, k;
}KALMAN_STRUCT_TYPE;

typedef struct _GPS_STRUCT_TYPE
{
	double N, a, f, e, B, L, x, y, z, lat, lon, prex, prey, spd, prespd, h, compute_heading_dis, heading, preheading, prepreheading, headingspd, xy[6][2];
	char flag_dataarrive, flag_confidence, flag_timeout, qual, flag_headingconfidence, flag_heading_dataarrive;
	int timeout_cnt;
	KALMAN_STRUCT_TYPE kalman_x, kalman_y, kalman_dx, kalman_dy, kalman_heading, kalman_dheading;
	ROBOT_PID_STRUCT_TYPE piddx, piddy, piddheading;
}GPS_STRUCT_TYPE;

typedef struct _GPS_INFORMATION_STRUCT_TYPE
{
	unsigned char Real_Locate;            //???????
	unsigned char Located;                //?????
	unsigned char Locate_Mode;         //????,2D?3D
	char UTC_Time[7];                       //??
	char UTC_Date[7];                      //??
	char Latitude[12];                       //??
	char NS_Indicator;                       //N=???,S=???
	char Longitude[13];                     //??
	char EW_Indicator;                      //E=??,W=??
	unsigned char Qual;                     //zhiliangzhishifu
	double Lat;
	double Lon;
	double Lat1;
	double Lon1;
	double Speed;                            //????
	double Course;                            //????
	double PDOP;                             //????
	double HDOP;                             //????
	double VDOP;                             //????
	double MSL_Altitude;                   //MSL????
	double RMS;                             //伪距标准差
	double LatStd;                          //纬度标准差
	double LonStd;                           //经度标准差
	double Heading;
	unsigned char Use_EPH_Sum;       //卫星数
	unsigned char User_EPH[12];        //??????????
	unsigned short EPH_State[12][4]; //?????12?????????????????
}GPS_INFORMATION_STRUCT_TYPE; //GPS??

typedef struct _GPS_REALBUF_STRUCT_TYPE
{
	char dataarrive;
	char data[256];                      //??GPS?????
	volatile unsigned short rx_pc;    //????
}GPS_REALBUF_STRUCT_TYPE; //GPS???????

typedef struct _CURVE_PLAN_STRUCT_TYPE {
	double x, y, heading, v;
	int type;
	int heart_beat_rx_rk3288;
	int flag_new_cmdframe;
	int flag_new_cmdframe_old;
}CURVE_PLAN_STRUCT_TYPE;

//---------------------------------------------------
typedef struct _ERR_STRUCT_TYPE {
	unsigned char table[10];
	unsigned char level;
}ERR_STRUCT_TYPE;
//---------------------------------------------------


typedef struct _INIT_STRUCT_TYPE
{
	double circle_ang, circle_x, circle_y, circle_r_sum, circle_center_x, circle_center_y, \
		uwbtag2space_ang, uwbtag2robot_ang_sum;
	short circle_xy_cnt, circle_ang_cnt, stage;

}INIT_STRUCT_TYPE;


typedef struct _MOTION_STRUCT_TYPE
{
	int i, flag_timer_5ms, flag_timer_10ms, flag_timer_100ms, timer_count_100ms, flag_timer_200ms, timer_count_200ms, flag_timer_500ms, timer_count_500ms, flag_timer_1s, timer_count_1s,
		type, status;
	double x, y, heading, v, omg, omg_motor, omg_gyro, omg_heading;
}MOTION_STRUCT_TYPE;

typedef struct _GYRO_STRUCT_TYPE {
	double omg_deg_raw, omg_his_add, heading;//,omg_his[1000];
	double omg_deg_zerobias, omg_deg_correct, aacx, aacy, aacz;
	int zero_bias_flag, flag_static, index;
}GYRO_STRUCT_TYPE;

typedef struct _CTRL_STRUCT_TYPE
{
	double offsetx, offsety, pre_offsety, offset_heading, pre_offset_heading, \
		target_x, target_y, target_heading, target_v, pre_target_v, target_omg, \
		max_v, max_acc, min_v, max_omg, min_omg, max_angacc, \
		offsetx_int, ki_offset_value, ki_heading_value, \
		cmd_delta_heading, cmd_heading, \
		decel_dis, decel_ang, direction, decel_stage, \
		tol_offsetx, tol_offsety, tol_ang, compesate_ang, targetomg_offsetheading, targetomg_offsety;
	INIT_STRUCT_TYPE initial;
	int handle_switch;
}CTRL_STRUCT_TYPE;

typedef struct _CMD_STRUCT_TYPE
{
	int dataarrive, type, cmdstop;
	double target_x, target_y, target_heading, test_target_x[3], test_target_y[3];
}CMD_STRUCT_TYPE;

typedef struct _TASK_STRUCT_TYPE
{
	int type, status, cur_num; double target_x, target_y, target_heading, target_max_omg, target_max_v;

}TASK_STRUCT_TYPE;

typedef struct _POS_STRUCT_TYPE
{
	float sa, sx, sy, sz;
}POS_STRUCT_TYPE;

typedef union _UPLOG_UNION_TYPE
{
	float f;
	unsigned char c[4];
}UPLOG_UNION_TYPE;



typedef struct _CTRL_CMD_STRUCT_TYPE
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

}CTRL_CMD_STRUCT_TYPE;

typedef struct _WEIGTED_MEAN_STRUCT_TYPE
{
	float dat[4];
	float fDat;
}WEIGTED_MEAN_STRUCT_TYPE;

typedef enum _BOOL
{
	FALSE = 0,
	TRUE = 1
}BOOL;
typedef enum _VALIDITY_ENUM_TYPE
{
	INVALID = 0,
	VALID = 1

}VALIDITY_ENUM_TYPE;



#endif // !_TYPE_INCLUDE_H
