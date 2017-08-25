#ifndef __GPS_H
#define __GPS_H 

#include "delay.h"  
#include "string.h"
#include "math.h"
#include "..\USER\basicfunc.h"

#define Valid 1
#define Invalid 0
#define GPS_STANDARD_X 3459265//3459316.7-54
#define GPS_STANDARD_Y 369367//367073.6 +2290 + 2345
#define GPS_STANDARD_HEADING 178.43
#define SCANER_STANDARD_HEADING 1.569986592358889f
#define GPS_CORRECT_L 0.157
#define XX 0
#define YY 1

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

extern ROBOTGPSTYPE gps;
extern GPS_INFORMATION GPS_Information;
extern GPS_REAL_BUFTYPE GPS_Real_buf;
extern GPS_REAL_BUFTYPE HUAWEI_Cmd_buf;
extern GPS_REAL_BUFTYPE GPS_Heading_buf;
extern GPS_REAL_BUFTYPE GPS_BL_buf;
extern GPS_REAL_BUFTYPE GPS_Uart_buf;

extern HUAWEI_CMDTYPE HUAWEI_cmd;
extern HUAWEI_CMDTYPE HUAWEI_status;;
int parase_gps(void);
int gps_correct(void);
int gps_compute_heading(void);

int Gauss_projection(double *x, double *y, double *z, double B, double L);

void Real_GPS_Command_Process(void);
void Real_HUAWEI_Command_Process(void);
void analysisGPS(void);
char* Real_Process_DH(char* buffer, unsigned char num);

void Creat_DH_Index(char* buffer);
void Creat_CMD_Index(char* buffer);
unsigned char Calc_GPS_Sum(const char* Buffer);

char rx_gps(void);

char rxHuaweiCmd(void);

extern void OnButtonFansuan(double x, double y, double *B, double *L);














#endif

