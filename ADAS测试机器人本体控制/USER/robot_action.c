#include "basicfunc.h"
#include "robot_action.h"
#include "canopen.h"
#include "my_can.h"
#include "usart.h"
#include "math.h"
#include "led.h" 
#include "delay.h"  
#include "string.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "..\HARDWARE\GPS\gps.h"
#include "..\HARDWARE\joystick\joystick.h"
#include "FreeRTOS.h"
#include "timers.h"

float movespd = 0;
GYRO_STRUCT_TYPE gyro;
extern CURVE_PLAN_STRUCT_TYPE lineplan;
float fittingR;
extern float signOMG;
float tomg;
float adjustr;

MOTION_STRUCT_TYPE robot_motion;
CTRL_STRUCT_TYPE control;
CMD_STRUCT_TYPE cmd;
TASK_STRUCT_TYPE task[2];
TASK_STRUCT_TYPE tasktype;
ERR_STRUCT_TYPE error;


extern int initial_data(void)
{
	int re = 0;
	gyro.flag_static = 1;
	gyro.zero_bias_flag = 0;
	return re;
}

int error_process(ERR_STRUCT_TYPE error)
{
	if (error.level == 1)
	{
		send_cmd_motordrive(MOTORDRIVE_ID_LEFT, MOTORDRIVE_CMD_DISABLE, 0, 0);
		vTaskDelay(5);
		send_cmd_motordrive(MOTORDRIVE_ID_LEFT, MOTORDRIVE_CMD_DISABLE, 0, 0);
	}
	return 0;
}

extern void rxOMGMPU6050(void)
{
	short gyrox, gyroy, gyroz;	//陀螺仪原始数据
	{
		MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);	//得到陀螺仪数据
		gyro.omg_deg_raw = (double)(gyroz)*(-0.061)*1.0416;
		if (robot_motion.type == STOP)
		{
			gyro.flag_static = TRUE;
		}
		else
		{
			gyro.flag_static = FALSE;
		}
		if (gyro.flag_static == TRUE && gyro.zero_bias_flag == FALSE)
		{
			gyro.omg_his_add = gyro.omg_his_add + gyro.omg_deg_raw;
			gyro.index++;
			if (gyro.index >= 500)
			{
				gyro.zero_bias_flag = TRUE;
				gyro.index = 0;
				gyro.omg_deg_zerobias = gyro.omg_his_add / 500.0;
				gyro.omg_his_add = 0;
			}

		}
		//当前值减零偏值
		if (gyro.zero_bias_flag == TRUE)
		{
			gyro.omg_deg_correct = gyro.omg_deg_raw - gyro.omg_deg_zerobias;
			gyro.index = 0;
		}
		else
		{
			gyro.omg_deg_correct = gyro.omg_deg_raw;
		}
	}
}

extern void rd_omg_gyro(void)
{
	unsigned long re;
	unsigned short rawvalue, st;
	short kk;
	re = SendCMD_SPI(0x20000000, 32);
	st = (re << 4) >> 30;
	if (st == 1)
	{
		rawvalue = (re << 6) >> 16;
		kk = (short)rawvalue;
		gyro.omg_deg_raw = (double)kk*0.0125;
		//计算零偏值，静止状态取1000次陀螺仪的平均值
		if (robot_motion.type == STOP)
		{
			gyro.flag_static = 1;
		}
		else
		{
			gyro.flag_static = 0;
		}
		if (gyro.flag_static == 1 && gyro.zero_bias_flag == 0)
		{
			gyro.omg_his_add = gyro.omg_his_add + gyro.omg_deg_raw;
			gyro.index++;
			if (gyro.index >= 1000)
			{
				gyro.zero_bias_flag = 1;
				gyro.index = 0;
				gyro.omg_deg_zerobias = gyro.omg_his_add / 1000.0;
				gyro.omg_his_add = 0;
			}
		}
		//当前值减零偏值
		if (gyro.zero_bias_flag == 1)
		{
			gyro.omg_deg_correct = gyro.omg_deg_raw - gyro.omg_deg_zerobias;
			gyro.index = 0;
		}
		else
		{
			gyro.omg_deg_correct = gyro.omg_deg_raw;
		}
	}
}

//static void SPI_Init(void)
//{
//	RCC->AHB1ENR |= 1 << 1;//使能PORTB时钟 
//	//test
//	RCC->AHB1ENR |= 1 << 2;//使能PORTB时钟 
//	GPIO_Set(GPIOC, PIN0, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_100M, GPIO_PUPD_PU); //PB12,13,15设置
//	////
//	GPIO_Set(GPIOB, PIN12 | PIN13 | PIN15, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_100M, GPIO_PUPD_PU); //PB12,13,15设置
//	GPIO_Set(GPIOB, PIN14, GPIO_MODE_IN, GPIO_OTYPE_PP, GPIO_SPEED_100M, GPIO_PUPD_PU); //PB14设置
//
//}


void DSP28x_usDelay(int ti)
{
	delay_us(ti);
}

unsigned long SendCMD_SPI(unsigned long cmd, int bitnum)
{
	unsigned long re = 0;
	unsigned char i = bitnum;

	SPITEA = 0;
	re = 0x00000000;
	while (i--)
	{
		re <<= 1;
		DSP28x_usDelay(3);
		SPICLK = 0;
		DSP28x_usDelay(3);
		if (cmd & 0x80000000)
		{
			SPISIMO = 1;
		}
		else
		{
			SPISIMO = 0;
		}
		cmd <<= 1;
		DSP28x_usDelay(3);
		SPICLK = 1;
		DSP28x_usDelay(3);
		if (SPISOMI)
		{
			re |= 0x00000001;
		}
	}
	SPICLK = 0;
	DSP28x_usDelay(3);
	SPITEA = 1;
	DSP28x_usDelay(3);
	return re;
}

//static int InitialSPIGYRO(void)
//{
//	DSP28x_usDelay(400000);
//	SendCMD_SPI(0x20000003, 32);
//	DSP28x_usDelay(100000);
//	SendCMD_SPI(0x20000000, 32);
//	DSP28x_usDelay(100000);
//	SendCMD_SPI(0x20000000, 32);
//	DSP28x_usDelay(100000);
//	SendCMD_SPI(0x20000000, 32);
//	DSP28x_usDelay(100000);
//	SendCMD_SPI(0x20000000, 32);
//	DSP28x_usDelay(100000);
//	SendCMD_SPI(0x20000000, 32);
//	DSP28x_usDelay(100000);
//	SendCMD_SPI(0x20000000, 32);
//	DSP28x_usDelay(100000);
//	SendCMD_SPI(0x20000000, 32);
//	DSP28x_usDelay(100000);
//	SendCMD_SPI(0x20000000, 32);
//	DSP28x_usDelay(100000);
//	SendCMD_SPI(0x20000000, 32);
//	DSP28x_usDelay(100000);
//	SendCMD_SPI(0x20000000, 32);
//	DSP28x_usDelay(100000);
//	SendCMD_SPI(0x20000000, 32);
//	DSP28x_usDelay(100000);
//	return 1;
//}




void parase_cmd_turning(void)
{
	task[0].type = SUB_TURN;
	task[0].status = TASK_READY;
	//	  cmd.target_heading = atan2(cmd.target_y - robot_motion.y, cmd.target_x - robot_motion.x);
	task[0].target_x = cmd.target_x;
	task[0].target_y = cmd.target_y;
	task[0].target_heading = robot_motion.heading + getAngleDiff(cmd.target_heading, robot_motion.heading);
	task[0].target_max_v = 0.2;
	task[0].target_max_omg = 0.2;
}
int action_turning(void)
{
	double targetheading;
	double heading;
	heading = robot_motion.heading;
	targetheading = control.target_heading;
	control.offset_heading = angle2HalfRadian(targetheading - heading);
	control.min_omg = 0.007;
	control.max_angacc = 0.2;
	control.tol_ang = PI / 180.0;
	control.compesate_ang = 2 * PI / 180.0;
	control.decel_ang = control.target_omg * control.target_omg / 2.0 / control.max_angacc + control.compesate_ang;
	if (control.decel_ang >= fabs(control.offset_heading) && control.decel_stage == 0)
	{
		control.decel_stage = 1;
	}

	if (control.decel_stage == 0)
	{
		control.target_omg = fabs(control.target_omg) + TIMER_PERIOD * control.max_angacc;
		control.target_omg = getSign(control.offset_heading)*control.target_omg;
		if (fabs(control.target_omg) > control.max_omg)
		{
			control.target_omg = getSign(control.offset_heading)*control.max_omg;
		}
	}
	else if (control.decel_stage == 1)
	{
		control.target_omg = control.target_omg - getSign(control.target_omg)*TIMER_PERIOD*control.max_angacc;
	}
	else if (control.decel_stage == 2)
	{
		control.target_omg = getSign(control.offset_heading)*control.min_omg*2.5;

	}

	if (control.decel_stage == 1 && fabs(control.target_omg) <= control.min_omg)
	{
		if (fabs(control.offset_heading) < control.tol_ang)
		{
			control.target_omg = 0;
			return TASK_FINISH;
		}
		control.decel_stage = 2;
	}

	if (fabs(control.offset_heading) > control.tol_ang)
	{
		return TASK_DOING;
	}
	else
	{
		control.target_omg = 0;
		return TASK_FINISH;
	}
}

void parse_cmd_moving(void)
{
	double target_heading;
	target_heading = atan2(cmd.target_y - robot_motion.y, cmd.target_x - robot_motion.x);
	task[0].type = SUB_TURN;
	task[0].status = TASK_READY;
	task[0].target_x = cmd.target_x;
	task[0].target_y = cmd.target_y;
	task[0].target_heading = robot_motion.heading + getAngleDiff(target_heading, robot_motion.heading);
	task[0].target_max_v = 0.2;
	task[0].target_max_v = cmd.target_heading * 180 / PI;
	task[0].target_max_omg = 0.2;
	task[1].type = SUB_MOVE;
	task[1].status = TASK_READY;
	task[1].target_x = cmd.target_x;
	task[1].target_y = cmd.target_y;
	task[1].target_heading = robot_motion.heading + getAngleDiff(target_heading, robot_motion.heading);
	task[1].target_max_v = 0.2;
	task[1].target_max_v = cmd.target_heading * 180 / PI;
	task[1].target_max_omg = 0.5;
}

void parse_cmd_moving1(void)
{
	cmd.target_heading = atan2(lineplan.y - robot_motion.y, lineplan.x - robot_motion.x);
	task[0].type = SUB_TRACKING;
	task[0].status = TASK_READY;
	task[0].target_x = lineplan.x;
	task[0].target_y = lineplan.y;
	task[0].target_heading = robot_motion.heading + getAngleDiff(cmd.target_heading, robot_motion.heading);
	task[0].target_max_v = 0.3;
	task[0].target_max_omg = 0.5;
}

int action_moving1(void)
{
	double targetheading;//nowheading;
	double omg, r_robot;
	double pvalue_heading = 0, ivalue_heading = 0;
	static double x0, y0, x1, y1, x2, y2, xc, yc;
	static float int_r_offset = 0;
	double offset_p, offset_i, offset_r, adjust_r;//
	double pvalue_offset = 0, ivalue_offset = 0;
	double offsety, offset_heading;
	static double ki_offset_value = 0, ki_heading_value = 0;
	double targetomg_offsetheading, targetomg_offsety;
	static int lineplan_type_old;
	static double ki_offsetx_value;
	double headAtanLastTime = 0.0, headAtanCurTime = 0.0, deltas, k0, k1;
	double r;
	unsigned char newdata;
	offset_p = 0.5;//0.5
	offset_i = 0.001;
	/////注意int_r_offset清空
	if (lineplan.flag_new_cmdframe_old == 0 && lineplan.flag_new_cmdframe == 1)
	{
		ki_offset_value = 0;
		ki_heading_value = 0;
		ki_offsetx_value = 0;
		int_r_offset = 0;
	}
	if (fabs(x2 - lineplan.x) > 0.000001 || fabs(y2 - lineplan.y) > 0.000001)
	{
		x0 = x1;
		y0 = y1;
		x1 = x2;
		y1 = y2;
		x2 = lineplan.x;
		y2 = lineplan.y;
		newdata = 1;
	}
	else
	{
		newdata = 0;
	}
	if (fabs(x0) > 0.00001 && fabs(x1) > 0.00001 && newdata == 1)
	{
		headAtanLastTime = atan2(y1 - y0, x1 - x0);
		headAtanCurTime = atan2(y2 - y1, x2 - x1);
		deltas = sqrt((y2 - y1)*(y2 - y1) + (x2 - x1)*(x2 - x1));
		if (fabs(headAtanCurTime*1000.0 - headAtanLastTime*1000.0) > 0.0001)
		{
			r = deltas / (headAtanCurTime - headAtanLastTime);
		}
		else
		{
			lineplan.type = LINE_TYPE_STRAIGHT;
		}
		//qiu yuanxin
		/////////////////
		if (fabs(tan(headAtanLastTime)) > 1)
		{
			k0 = -1 / tan(headAtanLastTime);
			k1 = -1 / tan(headAtanCurTime);
			if (fabs(k0 - k1) > 0.00001)
			{
				xc = (y1 - y0 + k0*x0 - k1*x1) / (k0 - k1);
				yc = y0 + k0*(xc - x0);
			}
		}
		else
		{
			k0 = -tan(headAtanLastTime);
			k1 = -tan(headAtanCurTime);
			if (fabs(k0 - k1) > 0.00001)
			{
				yc = (x1 - x0 + k0*y0 - k1*y1) / (k0 - k1);
				xc = x0 + k0*(yc - y0);
			}
		}
		//judge 
		if (fabs(r) > 40 || fabs(r) < 0.5)
		{
			lineplan.type = LINE_TYPE_STRAIGHT;
		}
		else
		{
			lineplan.type = LINE_TYPE_CURVE;
		}
	}
	else
	{
		lineplan.type = LINE_TYPE_STRAIGHT;
	}

	fittingR = r;
	targetheading = lineplan.heading;//atan2(lineplan.y-robot_motion.y,lineplan.x-robot_motion.x);
						/////////////////////////////////		
		//清空积分量
	if (lineplan_type_old != lineplan.type)
	{
		if (lineplan.type == LINE_TYPE_CURVE)
		{
			int_r_offset = 0;
		}
		if (lineplan.type == LINE_TYPE_STRAIGHT)
		{
			ki_offset_value = 0;
			ki_heading_value = 0;
		}
	}
	lineplan_type_old = lineplan.type;


	//////////////////////////////////////////////////
	switch (lineplan.type)
	{
		/*规划轨迹半径小于1km，则按半径调整角速率*/
	case LINE_TYPE_CURVE:
		//求取小车当前偏离圆弧距离offset_r
//					r_robot = sqrt((robot_motion.x - xc)*(robot_motion.x - xc)+(robot_motion.y - yc)*(robot_motion.y - yc));		
//					offset_r = r - r_robot;					
//					if(fabs(offset_r)<0.0001){int_r_offset = 0;}
//					else{int_r_offset += offset_r;}		
//					adjust_r = offset_p*offset_r + offset_i*int_r_offset;
//					
//					adjustr = r_robot;
//					r += adjust_r;		
					//需要监控adjust_r,tagetheading ,nowheading
//					if(r > 0){omg = sign(getanglediff(targetheading,nowheading))*fabs(lineplan.v/r/2.0);}
//					else{omg = 0;}
//				
		break;
		/*规划轨迹半径大于1km,按offsety和目标当前角度差调节*/
	case LINE_TYPE_STRAIGHT:
		//					//轨迹偏移量调整
		//			    offsety = lineplan.x*sin(-targetheading) + lineplan.y*cos(-targetheading) 
		//										- robot_motion.y*cos(-targetheading) - robot_motion.x*sin(-targetheading);
		//					
		//			    de_offsety = offsety;
		//			    ki_offset_value+=offsety;  
		//					pvalue_offset = sign(robot_motion.v)*MOVE_OFFSET_P * offsety;
		//					ivalue_offset = sign(robot_motion.v)*MOVE_OFFSET_I * ki_offset_value;
		//					targetomg_offsety = pvalue_offset + ivalue_offset;
		//					//角度差调整
		//					offset_heading = getanglediff(lineplan.heading, robot_motion.heading);
		//					ki_heading_value+=offset_heading;    
		//					pvalue_heading = MOVE_HEADING_P * offset_heading;
		//					ivalue_heading = MOVE_HEADING_I * ki_heading_value;    
		//					targetomg_offsetheading = pvalue_heading + ivalue_heading;		
		//					omg = targetomg_offsetheading + targetomg_offsety;
		break;
	default:
		omg = 0;

		break;
	}

	offsety = lineplan.x*sin(-targetheading) + lineplan.y*cos(-targetheading)
		- robot_motion.y*cos(-targetheading) - robot_motion.x*sin(-targetheading);

	if (fabs(offsety) > 0.4)
	{
		error.table[ERROR_INDEX_OFFSET] = 1;
		error.level = 1;
	}
	//	de_offsety = offsety;
	ki_offset_value += offsety;
	pvalue_offset = getSign(robot_motion.v)*MOVE_OFFSET_P * offsety;
	ivalue_offset = getSign(robot_motion.v)*MOVE_OFFSET_I * ki_offset_value;
	targetomg_offsety = pvalue_offset + ivalue_offset;
	//角度差调整
	offset_heading = getAngleDiff(lineplan.heading, robot_motion.heading);
	ki_heading_value += offset_heading;
	pvalue_heading = MOVE_HEADING_P * offset_heading;
	ivalue_heading = MOVE_HEADING_I * ki_heading_value;
	targetomg_offsetheading = pvalue_heading + ivalue_heading;
	omg = targetomg_offsetheading + targetomg_offsety;


	control.offsetx = lineplan.x*cos(-targetheading) - lineplan.y*sin(-targetheading) + robot_motion.y*sin(-targetheading) - robot_motion.x*cos(-targetheading);
	ki_offsetx_value += control.offsetx;

	control.target_v = lineplan.v + 0.05*control.offsetx + 0.001*ki_offsetx_value;

	if (lineplan.type == LINE_TYPE_CURVE)
	{
		r_robot = getSign((headAtanCurTime - headAtanLastTime))*sqrt((robot_motion.x - xc)*(robot_motion.x - xc) + (robot_motion.y - yc)*(robot_motion.y - yc));
		offset_r = r - r_robot;
		if (fabs(offset_r) < 0.0001) { int_r_offset = 0; }
		else { int_r_offset += offset_r; }
		adjust_r = offset_p*offset_r + offset_i*int_r_offset;
		adjustr = r_robot;
		r += adjust_r;
		if (fabs(r) > 0.5)
		{
			omg = omg + control.target_v / r;
		}
		else { omg = 0; }
	}
	control.target_omg = omg;
	if (lineplan.v == 0)
	{
		control.target_v = 0;
		control.target_omg = 0;
		lineplan_type_old = 0;
		lineplan.type = 0;
		return TASK_FINISH;
	}
	else
	{
		return TASK_DOING;
	}
}

int action_moving(void)
{
	double targetx, targety, targetheading, nowx, nowy;
	targetx = control.target_x;
	targety = control.target_y;
	targetheading = control.target_heading;
	nowx = robot_motion.x;
	nowy = robot_motion.y;
	control.offsetx = targetx*cos(-targetheading) - targety*sin(-targetheading) + nowy*sin(-targetheading) - nowx*cos(-targetheading);
	control.offsety = targetx*sin(-targetheading) + targety*cos(-targetheading) - nowy*cos(-targetheading) - nowx*sin(-targetheading);
	control.min_v = 0.007;
	control.max_acc = 0.5;
	control.tol_offsetx = 0.05;
	control.decel_dis = control.target_v * control.target_v / 2.0 / control.max_acc + 0.12;
	if (control.decel_dis >= fabs(control.offsetx) && control.decel_stage == 0)
	{
		control.decel_stage = 1;
	}
	if (control.decel_stage == 0)
	{
		control.target_v = fabs(control.target_v) + TIMER_PERIOD*control.max_acc;
		control.target_v = getSign(control.offsetx)*control.target_v;
		if (fabs(control.target_v) > control.max_v)
		{
			control.target_v = getSign(control.offsetx)*control.max_v;
		}
	}
	else if (control.decel_stage == 1)
	{
		control.target_v = control.target_v - getSign(control.target_v)*TIMER_PERIOD*control.max_acc;
	}
	else if (control.decel_stage == 2)
	{
		control.target_v = getSign(control.offsetx)*control.min_v*2.5;
		control.offsety = 0;
	}
	if (control.decel_stage == 1 && fabs(control.target_v) <= control.min_v)
	{
		if (fabs(control.offsetx) < control.tol_offsetx)
			return TASK_FINISH;
		control.decel_stage = 2;
	}

	control.target_omg = 0;
	//gps信号不可用时，不重新计算目标航向角。
	getcmdomg(control.target_heading, robot_motion.heading);

	if (fabs(control.offsetx) > control.tol_offsetx)
	{
		return TASK_DOING;
	}
	else
	{
		control.target_v = 0;
		control.target_omg = 0;
		return TASK_FINISH;
	}
}

void getcmdomg(double targetheading, double nowheading)
{
	double pvalue_offset = 0, ivalue_offset = 0, dvalue_offset = 0;
	double pvalue_heading = 0, ivalue_heading = 0, dvalue_heading = 0;
	double d_heading = 0, d_offset = 0;
	control.ki_offset_value += control.offsety;
	d_offset = control.offsety - control.pre_offsety;
	pvalue_offset = getSign(robot_motion.v)*MOVE_OFFSET_P * control.offsety;
	ivalue_offset = getSign(robot_motion.v)*MOVE_OFFSET_I * control.ki_offset_value;
	dvalue_offset = getSign(robot_motion.v)*MOVE_OFFSET_D * d_offset;
	control.targetomg_offsety = pvalue_offset + ivalue_offset + dvalue_offset;
	//gps信号不可用时，不对位置偏差作矫正。
	if (gps.flag_confidence == 0)
	{
		control.targetomg_offsety = 0;
	}
	control.cmd_heading = control.target_heading;
	control.offset_heading = getAngleDiff(control.cmd_heading, robot_motion.heading);
	control.ki_heading_value += control.offset_heading;
	d_heading = control.offset_heading - control.pre_offset_heading;
	pvalue_heading = MOVE_HEADING_P * control.offset_heading;
	ivalue_heading = MOVE_HEADING_I * control.ki_heading_value;
	dvalue_heading = MOVE_HEADING_D * d_heading;
	control.targetomg_offsetheading = pvalue_heading + ivalue_heading + dvalue_heading;
	control.target_omg = control.targetomg_offsetheading + control.targetomg_offsety;
	control.pre_offsety = control.offsety;
	control.pre_offset_heading = control.offset_heading;
}

void parase_cmd_initial_pos(void)
{
	gps.kalman_heading.x = robot_motion.heading;
	robot_motion.x = gps.x;
	robot_motion.y = gps.y;
}

extern void parseCMD(void)
{
	if (cmd.dataarrive == 1)
	{
		cmd.dataarrive = 0;
		cmd.cmdstop = 0;
		switch (cmd.type)
		{
		case SUB_MOVE:
			parse_cmd_moving();
			break;
		case SUB_TRACKING:
			parse_cmd_moving1();
			break;
		case ENABLE:
			vTaskDelay(2);
			EnableMotorDrive(MOTORDRIVE_ID_LEFT);
			vTaskDelay(2);
			EnableMotorDrive(MOTORDRIVE_ID_RIGHT);
			vTaskDelay(2);
			break;
		case DISABLE:
			vTaskDelay(2);
			send_cmd_motordrive(MOTORDRIVE_ID_LEFT, MOTORDRIVE_CMD_DISABLE, 0, 0);
			vTaskDelay(2);
			send_cmd_motordrive(MOTORDRIVE_ID_RIGHT, MOTORDRIVE_CMD_DISABLE, 0, 0);
			vTaskDelay(2);
			break;
		case BRAKE_ENABLE:
			send_cmd_motordrive(MOTORDRIVE_ID_LEFT, MOTORDRIVE_CMD_BRAKEOPERATION, MOTORDRIVE_CMD_BRAKEOPERATION_ENABLE, 0);
			vTaskDelay(2);
			ERelay1 = 1;
			send_cmd_motordrive(MOTORDRIVE_ID_RIGHT, MOTORDRIVE_CMD_BRAKEOPERATION, MOTORDRIVE_CMD_BRAKEOPERATION_ENABLE, 0);
			vTaskDelay(2);
			break;
		case BRAKE_DISABLE:
			send_cmd_motordrive(MOTORDRIVE_ID_LEFT, MOTORDRIVE_CMD_BRAKEOPERATION, MOTORDRIVE_CMD_BRAKEOPERATION_DISABLE, 0);
			vTaskDelay(2);
			ERelay1 = 0;
			send_cmd_motordrive(MOTORDRIVE_ID_RIGHT, MOTORDRIVE_CMD_BRAKEOPERATION, MOTORDRIVE_CMD_BRAKEOPERATION_DISABLE, 0);
			vTaskDelay(2);
			break;
		case SUB_TURN:
			parase_cmd_turning();
			break;
		case E_STOP:
			cmd.cmdstop = 1;
			control.target_omg = 0.0;//rad/s	
			robot_motion.type = STOP;
			break;
		case INITIAL_POS:
			parase_cmd_initial_pos();
			break;
		case MOVE_S:
			robot_motion.type = MOVE_S;
			movespd = cmd.target_heading *180.0f / PI;
			break;
		case MOVE_BACK:
			robot_motion.type = MOVE_BACK;
			movespd = cmd.target_heading *180.0f / PI;
			break;
		case TURN_LEFT:
			robot_motion.type = TURN_LEFT;
			control.target_omg = cmd.target_heading *180.0f / PI;//rad/s
			break;
		case TURN_RIGHT:
			robot_motion.type = TURN_RIGHT;
			control.target_omg = -cmd.target_heading *180.0f / PI;//rad/s
			break;
		default:
			break;
		}
	}
}

extern void excuteRK3288CMD(void)
{
	int i;
	int get_job = 0;
	for (i = 0; i < 2; i++)
	{
		if (task[i].status == TASK_DOING)
		{
			get_job = 1;
			break;
		}
		if (task[i].status == TASK_READY)
		{
			tasktype.cur_num = i;
			memset(&control, 0, sizeof(struct _CTRL_STRUCT_TYPE));
			control.target_x = task[i].target_x;
			control.target_y = task[i].target_y;
			control.target_heading = task[i].target_heading;
			control.max_v = task[i].target_max_v;
			control.max_omg = task[i].target_max_omg;
			control.decel_stage = 0;
			get_job = 1;
			break;
		}
	}
	if (get_job == 1 && cmd.cmdstop == 0)
	{
		robot_motion.status = MOTION_DOING;
	}
	else
	{
		robot_motion.status = MOTION_FINISH;
		task[tasktype.cur_num].type = STOP;
	}
	switch (task[tasktype.cur_num].type)
	{
	case SUB_MOVE:
		robot_motion.type = SUB_MOVE;
		task[tasktype.cur_num].status = action_moving();
		break;
	case SUB_TRACKING:
		robot_motion.type = SUB_TRACKING;
		task[tasktype.cur_num].status = action_moving1();
		break;
	case SUB_TURN:
		robot_motion.type = SUB_TURN;
		task[tasktype.cur_num].status = action_turning();
		break;
	default:
		break;
	}

}

extern void parsePosition(void)
{
	double heading;
	static int zero_bias_last_flag;
	getWheelMotorSpeed2Buf();
#ifdef ROBOT1
	gHalData.WheelHal[0].WheelVel = ((double)gHalData.WheelHal[0].FbVel) / _WHEEL_REDUCE_RATIO*PI / 60.0*WHEEL_DIA;
	gHalData.WheelHal[1].WheelVel = -((double)gHalData.WheelHal[1].FbVel) / _WHEEL_REDUCE_RATIO*PI / 60.0*WHEEL_DIA;
	robot_motion.v = (gHalData.WheelHal[0].WheelVel + gHalData.WheelHal[1].WheelVel)*0.5;
	robot_motion.omg_motor = (gHalData.WheelHal[1].WheelVel - gHalData.WheelHal[0].WheelVel) / WHEEL_DIS;
#endif
#ifdef ROBOT2
	gHalData.WheelHal[0].WheelVel = -((double)gHalData.WheelHal[0].FbVel) / _WHEEL_REDUCE_RATIO*PI / 60.0*WHEEL_DIA;
	gHalData.WheelHal[1].WheelVel = ((double)gHalData.WheelHal[1].FbVel) / _WHEEL_REDUCE_RATIO*PI / 60.0*WHEEL_DIA;
	robot_motion.v = (gHalData.WheelHal[0].WheelVel + gHalData.WheelHal[1].WheelVel)*0.5;
	robot_motion.omg_motor = (gHalData.WheelHal[0].WheelVel - gHalData.WheelHal[1].WheelVel) / WHEEL_DIS;
#endif
	robot_motion.omg_gyro = GYRO_DIR*gyro.omg_deg_correct*PI / 180.0;
	if (zero_bias_last_flag == 0 && gyro.zero_bias_flag == 1)
	{
		robot_motion.heading = gps.heading;
		robot_motion.omg_heading = gps.heading;
		robot_motion.x = (float)gps.x;
		robot_motion.y = (float)gps.y;
		lineplan.x = robot_motion.x;
		lineplan.y = robot_motion.y;
		lineplan.heading = robot_motion.heading;
	}
	zero_bias_last_flag = gyro.zero_bias_flag;
	////////////////////////////////////////check motion status/////////////////////////////////////////////////////////////
	if (gHalData.WheelHal[0].ObDict[ACTUALVEL].Value[0] == 0 && gHalData.WheelHal[1].ObDict[ACTUALVEL].Value[0] == 0\
		&& gHalData.WheelHal[0].CmdMotorvel == 0 && gHalData.WheelHal[1].CmdMotorvel == 0\
		&& robot_motion.type != STOP && robot_motion.status != MOTION_DOING)
	{
		robot_motion.type = STOP;
	}
	////////////////////////////////////choose omg/////////////////////////////////////////////////////
	if (gyro.zero_bias_flag == 1)
	{
		robot_motion.omg = robot_motion.omg_gyro;
	}
	else
	{
		robot_motion.omg = robot_motion.omg_motor;
	}
	if ((robot_motion.type == SUB_MOVE || robot_motion.type == SUB_TRACKING) && (fabs(robot_motion.v) <= 0.007*2.5))
	{
		robot_motion.omg = robot_motion.omg_motor;
	}
	robot_motion.x = robot_motion.x + robot_motion.v * TIMER_PERIOD * cos(robot_motion.heading);
	robot_motion.y = robot_motion.y + robot_motion.v * TIMER_PERIOD * sin(robot_motion.heading);
	heading = robot_motion.heading + robot_motion.omg *TIMER_PERIOD;

	robot_motion.heading = robot_motion.heading + getAngleDiff(heading, robot_motion.heading);
	heading = robot_motion.omg_heading + robot_motion.omg_motor *TIMER_PERIOD;
	robot_motion.omg_heading = robot_motion.omg_heading + getAngleDiff(heading, robot_motion.omg_heading);
	robot_motion.omg_heading = angle2HalfRadian(robot_motion.omg_heading);
}

int joyStickCTRL(JOY_STICK_BUF_TYPE handleDataTemp)
{
	static u32 val_temp1[5] = { 0 }, val_temp2[5] = { 0 }, val_temp3[5] = { 0 };
	int re = 0;
	double tg_v = 0, tg_omg = 0, max_v = 5, max_omg = 0, sign_v, sign_omg;
	static double handle_target_v = 0, handle_target_omg = 0;
	tg_v = handle_target_v;
	tg_omg = handle_target_omg;
	val_temp1[0] = val_temp1[1];
	val_temp1[1] = val_temp1[2];
	val_temp1[2] = val_temp1[3];
	val_temp1[3] = val_temp1[4];
	val_temp1[4] = handleDataTemp.val[1];
	handleDataTemp.val[1] = GetMedianNum1(&val_temp1[0], 5);
	val_temp3[0] = val_temp3[1];
	val_temp3[1] = val_temp3[2];
	val_temp3[2] = val_temp3[3];
	val_temp3[3] = val_temp3[4];
	val_temp3[4] = handleDataTemp.val[3];
	handleDataTemp.val[3] = GetMedianNum1(&val_temp3[0], 5);
	val_temp2[0] = val_temp2[1];
	val_temp2[1] = val_temp2[2];
	val_temp2[2] = val_temp2[3];
	val_temp2[3] = val_temp2[4];
	val_temp2[4] = handleDataTemp.val[2];
	handleDataTemp.val[2] = GetMedianNum1(&val_temp2[0], 5);
	sign_v = signH(((float)handleDataTemp.val[1]) - 110.0f);
	sign_omg = -signH(((float)handleDataTemp.val[3]) - 110.0f);
	max_omg = (((float)handleDataTemp.val[2]) - 60.0f)*0.005f;
	if (max_omg > 0.05)
	{
		if (sign_v == 0)
		{
			tg_v = tg_v;
		}
		else if (sign_v < 0)
		{
			if (tg_v > 0.1)
			{
				tg_v -= 0.5f*TIMER_PERIOD;
			}
			else if (tg_v < -0.1)
			{
				tg_v += 0.5f*TIMER_PERIOD;
			}
			else
			{
				tg_v = 0.0;//m/s					  
			}
		}
		else
		{
			if (fabs(tg_v) < max_v)
			{
				tg_v += 0.5f*TIMER_PERIOD;
			}
			else
			{
				tg_v = max_v;//m/s					  
			}
		}

		if (sign_omg == 0)
		{
			if (tg_omg > 0.1)
			{
				tg_omg -= 0.2f*TIMER_PERIOD;
			}
			else if (tg_omg < -0.1)
			{
				tg_omg += 0.2f*TIMER_PERIOD;
			}
			else
			{
				tg_omg = 0.0;//m/s					  
			}
		}
		else
		{
			tg_omg = max_omg * sign_omg;
		}
		if (fabs(tg_v) > 6.0f)
		{
			tg_v = getSign(tg_v)*5.0f;
		}
		if (fabs(tg_omg) > 0.5f)
		{
			tg_omg = getSign(tg_omg)*0.5f;
		}
	}
	else
	{
		if (tg_v > 0.1)
		{
			tg_v -= 0.5f*TIMER_PERIOD;
		}
		else if (tg_v < -0.1)
		{
			tg_v += 0.5f*TIMER_PERIOD;
		}
		else
		{
			tg_v = 0.0;//m/s					  
		}
		tg_omg = 0;
	}
	handle_target_v = tg_v;
	handle_target_omg = tg_omg;
	control.target_v = (double)tg_v;
	control.target_omg = (double)tg_omg;
	return re;
}
void robotStopCtrl(void)
{
	if (control.target_v > 0.1)
	{
		control.target_v -= 0.5f*TIMER_PERIOD;
	}
	else if (control.target_v < -0.1)
	{
		control.target_v += 0.5f*TIMER_PERIOD;
	}
	else
	{
		control.target_v = 0.0;//m/s					  
	}
}

extern void speed2MotorCalc(double tv, double tomg)
{
	if (gyro.zero_bias_flag == 0)
	{
		tv = 0;
		tomg = 0;
	}
	control.target_v = tv;//m/s
	control.target_omg = tomg;//rad/s
	if (robot_motion.type == MOVE_S)
	{
		if (control.target_v < movespd)
		{
			control.target_v += 0.5f*TIMER_PERIOD;
		}
		else
		{
			control.target_v = movespd;//m/s					  
		}
		control.target_omg = 0.0;//rad/s
	}
	if (robot_motion.type == MOVE_BACK)
	{
		if (control.target_v > -movespd)
		{
			control.target_v -= 0.5f*TIMER_PERIOD;
		}
		else
		{
			control.target_v = -movespd;//m/s					  
		}
		control.target_omg = 0.0;//rad/s
	}
	joyStickCTRL(handleData);
	if (cmd.cmdstop == 1)
	{
		robotStopCtrl();
	}		////////////////////////////////debug/////////////////////
		/////////////////////////////////////////////////
#ifdef ROBOT1
	gHalData.WheelHal[0].CmdWheelVel = (2 * control.target_v - control.target_omg*WHEEL_DIS) / 2.0;//m/s
	gHalData.WheelHal[1].CmdWheelVel = (2 * control.target_v + control.target_omg*WHEEL_DIS) / 2.0;

	gHalData.WheelHal[0].CmdMotorvel = gHalData.WheelHal[0].CmdWheelVel / WHEEL_DIA / PI*60.0*_WHEEL_REDUCE_RATIO;
	gHalData.WheelHal[1].CmdMotorvel = -gHalData.WheelHal[1].CmdWheelVel / WHEEL_DIA / PI*60.0*_WHEEL_REDUCE_RATIO;
#endif

#ifdef ROBOT2
	gHalData.WheelHal[0].CmdWheelVel = (2 * control.target_v - control.target_omg*WHEEL_DIS) / 2.0;//m/s
	gHalData.WheelHal[1].CmdWheelVel = (2 * control.target_v + control.target_omg*WHEEL_DIS) / 2.0;

	gHalData.WheelHal[0].CmdMotorvel = -gHalData.WheelHal[0].CmdWheelVel / WHEEL_DIA / PI*60.0*_WHEEL_REDUCE_RATIO;
	gHalData.WheelHal[1].CmdMotorvel = gHalData.WheelHal[1].CmdWheelVel / WHEEL_DIA / PI*60.0*_WHEEL_REDUCE_RATIO;
#endif		
	wheelMotorSpdSet(gHalData.WheelHal[0].CmdMotorvel, gHalData.WheelHal[1].CmdMotorvel);
}






