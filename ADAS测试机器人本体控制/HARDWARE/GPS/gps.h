#ifndef __GPS_H
#define __GPS_H 

#include "delay.h"  
#include "string.h"
#include "math.h"
#include "typeinclude.h"


#define Valid 1
#define Invalid 0
#define GPS_STANDARD_X 3459265//3459316.7-54
#define GPS_STANDARD_Y 369367//367073.6 +2290 + 2345
#define GPS_STANDARD_HEADING 178.43
#define SCANER_STANDARD_HEADING 1.569986592358889f
#define GPS_CORRECT_L 0.157
#define XX 0
#define YY 1


extern ROBOTGPSTYPE gps;
extern GPS_INFORMATION GPS_Information;
extern GPS_REAL_BUFTYPE GPS_Real_buf;
extern GPS_REAL_BUFTYPE HUAWEI_Cmd_buf;
extern GPS_REAL_BUFTYPE GPS_Heading_buf;
extern GPS_REAL_BUFTYPE GPS_BL_buf;
extern GPS_REAL_BUFTYPE GPS_Uart_buf;

extern HUAWEI_CMDTYPE HUAWEI_cmd;
extern HUAWEI_CMDTYPE HUAWEI_status;;
extern double global_px, global_py;
extern CURVEPLAN lineplan;

static int parase_gps(void);
static int gps_correct(void);
//static int gps_compute_heading(void);

static int Gauss_projection(double *x, double *y, double *z, double B, double L);

static void Real_GPS_Command_Process(void);
extern void analysisGPS(void);
static char* Real_Process_DH(char* buffer, unsigned char num);

static void Creat_DH_Index(char* buffer);
static void Creat_CMD_Index(char* buffer);
static unsigned char Calc_GPS_Sum(const char* Buffer);
extern char rxHuaweiCmd(void);
static void Real_HUAWEI_Command_Process(void);

extern void initGPSData(void);
static char rx_gps(void);


extern void OnButtonFansuan(double x, double y, double *B, double *L);


#endif

