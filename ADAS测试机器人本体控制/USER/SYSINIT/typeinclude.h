#ifndef _TYPE_INCLUDE_H
#define _TYPE_INCLUDE_H
#include "stm32f4xx.h"
#define PI 3.1415926535898

typedef struct PID_struct
{
	double p, i, d, value, ivalue;
}ROBOTPID;

typedef struct huawei_cmd_struct
{
	double time;
	double longitude;
	double latitude;
	double speed;
	double heading;
}HUAWEI_CMDTYPE;

typedef struct kalman_struct
{
	double x, z, r, p, q, k;
}KALMAN;

typedef struct gps_struct
{
	double N, a, f, e, B, L, x, y, z, lat, lon, prex, prey, spd, prespd, h, compute_heading_dis, heading, preheading, prepreheading, headingspd, xy[6][2];
	char flag_dataarrive, flag_confidence, flag_timeout, qual, flag_headingconfidence, flag_heading_dataarrive;
	int timeout_cnt;
	KALMAN kalman_x, kalman_y, kalman_dx, kalman_dy, kalman_heading, kalman_dheading;
	ROBOTPID piddx, piddy, piddheading;
}ROBOTGPSTYPE;

typedef struct _GPS_Information
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
}GPS_INFORMATION; //GPS??

typedef struct _GPS_Real_buf
{
	char dataarrive;
	char data[256];                      //??GPS?????
	volatile unsigned short rx_pc;    //????
}GPS_REAL_BUFTYPE; //GPS???????

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



typedef struct weigtedmean_struct
{
	float dat[4];
	float fDat;
}WEIGTEDMEAN;

typedef enum _BOOL
{
	FALSE = 0,
	TRUE = 1
}BOOL;



#endif // !_TYPE_INCLUDE_H
