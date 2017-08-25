#ifndef __UWB_H
#define __UWB_H 

#include "delay.h"  
#include "string.h"
#include "math.h"
#include "..\USER\basicfunc.h"



typedef struct uwbfdb_struct
{
    unsigned short seq;
	  short leftspd;
	  short rightspd;
	  short forwardspd;

}UWBFDB;




typedef struct uwb_struct
{	
	int xy_buff_num,heading_buff_num,flag_stable,getheading_cnt,getxy_cnt,dis_cnt,deltaxy_buff_num,flag_data_arrive,flag_data_normal,flag_solve_var_x,flag_solve_var_y;
	double origin_x,origin_y,pre_origin_x,pre_origin_y,raw_x,raw_y,now_x,now_y,now_x1,now_y1,now_heading,x_buff_100ms[10],y_buff_100ms[10],x_buff_100mm[10],y_buff_100mm[10],heading_buff[10],
		creat_buff_dis,creat_buff_dis_tol,delta_x,delta_y,delta_heading,delta_x_buff[10],delta_y_buff[10],x_v,y_v,
	  offset_r,offset_alfa,delta_now_x,delta_now_y,delta_now_heading,average,heading_max,heading_min,heading_sum,now_heading_var,heading_var_tol,
	 raw_x_var,raw_y_var,pre_raw_x_var,pre_raw_y_var,raw_xy_var_tol,updata_dis,updata_dis_tol,stable_dis,rms_x,rms_y,stable_x_sum,stable_y_sum;
   unsigned short seq,pre_seq,frame_loss_cnt,stable_cnt,led_cnt,offline_cnt,unchanged_cnt,buff_cnt;
	 KALMAN kalman_x,kalman_y,kalman_dx,kalman_dy;
}ROBOTUWB;


extern ROBOTUWB uwb;
extern UWBFDB uwbfdb;








#endif



