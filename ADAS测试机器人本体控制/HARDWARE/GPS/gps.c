#include "..\USER\robot_action.h"
#include "delay.h"  
#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "gps.h"
#include "ctype.h"
#include "..\USER\basicfunc.h"
GPS_INFORMATION_STRUCT_TYPE GPS_Information;
GPS_REALBUF_STRUCT_TYPE GPS_Real_buf;
GPS_REALBUF_STRUCT_TYPE GPS_Uart_buf;
GPS_STRUCT_TYPE gps;
GPS_REALBUF_STRUCT_TYPE GPS_Heading_buf;
GPS_REALBUF_STRUCT_TYPE GPS_BL_buf;
double L0;
extern CURVE_PLAN_STRUCT_TYPE lineplantest;
double global_px, global_py;
double gps_corr_x, gps_corr_y;
CURVE_PLAN_STRUCT_TYPE lineplan;

char rx_gps(void)
{
	if (GPS_BL_buf.dataarrive == 1)
	{
		memcpy(&GPS_Real_buf, &GPS_BL_buf, sizeof(GPS_REALBUF_STRUCT_TYPE));
	}
	if (Calc_GPS_Sum(GPS_Real_buf.data) == VALID)
	{
		GPS_BL_buf.dataarrive = 0;
		Creat_DH_Index(GPS_Real_buf.data);
		Real_GPS_Command_Process();
	}

	if (GPS_Heading_buf.dataarrive == 1)
	{
		memcpy(&GPS_Real_buf, &GPS_Heading_buf, sizeof(GPS_REALBUF_STRUCT_TYPE));
	}
	if (Calc_GPS_Sum(GPS_Real_buf.data) == VALID)
	{
		GPS_Heading_buf.dataarrive = 0;
		Creat_DH_Index(GPS_Real_buf.data);
		Real_GPS_Command_Process();
	}
	return 1;
}

/********************************************************************************************************
**������Ϣ :int parase_gps(void)
**�������� :����gps������Ϣ
		//gps��ֵ����������
			  //1��gps�뵱ǰС������λ��ƫ���С
			  //2��gps����Ƕ��������Ƕ�ƫ���С������gps�Ƕ��ȶ�
			  //3��gps����λ�ñ仯�ȶ����ұ仯�ٶȲ�Ϊ0
				//4��gps����
				//5��gps�汱��������course
				//6����γ�ȱ�׼��
				//7��gpsˮƽ��������dop
				//8��gps����ָʾqual
**������� :
**������� :
*********************************************************************************************************/
int parase_gps(void)
{
	int re = 0;
	gps.compute_heading_dis += robot_motion.v*TIMER_PERIOD * 10;
	if (gps.flag_headingconfidence == 1)
	{
		gps.heading = angle2HalfRadian((GPS_STANDARD_HEADING - GPS_Information.Heading)*PI / 180);
	}
	if (gps.flag_dataarrive == 1)
	{
		gps.flag_dataarrive = 0;//������ݵ����־λ
		gps.flag_timeout = 0;//�����ʱ��־λ
		gps.timeout_cnt = 0;//��ճ�ʱ������
		gps.prex = gps.x;//����x
		gps.prey = gps.y;//����y
		gps.prespd = gps.spd;//����gps��������	  		
		Gauss_projection(&gps.x, &gps.y, &gps.z, GPS_Information.Lat, GPS_Information.Lon);//��˹ͶӰ-����任
		gps.x = -(gps.x - GPS_STANDARD_X);//gps�������ϵ����궨
		gps.y = gps.y - GPS_STANDARD_Y;
		////////////�Ƕ���Ҫ��֤///////////////////////////////
#ifdef ROBOT2		  
//			  gps.x = gps.x - GPS_CORRECT_L*cos(robot_motion.heading);		//����������ϵ�궨		
//			  gps.y = gps.y - GPS_CORRECT_L*sin(robot_motion.heading);
		gps.x = gps.x + GPS_CORRECT_L*cos(robot_motion.heading);		//����������ϵ�궨		
		gps.y = gps.y + GPS_CORRECT_L*sin(robot_motion.heading);
#endif			  

		gps.spd = sqrt((gps.prex - gps.x)*(gps.prex - gps.x) + (gps.prey - gps.y)*(gps.prey - gps.y)); //����gps����
	}
	else
	{
		gps.timeout_cnt++;
		if (gps.timeout_cnt > 2)
		{
			gps.flag_timeout = 1;
		}
	}
	//(fabs(gps.x - robot_motion.x) < 0.5)&&(fabs(gps.y - robot_motion.y)<0.5)
//&&(GPS_Information.LatStd < 0.05)&&(GPS_Information.LonStd < 0.05)		
	if ((gps.spd < 0.5) && (gps.prespd < 0.5) && (GPS_Information.Qual == 4) && (gps.flag_timeout == 0))
	{
		gps.flag_confidence = 1;
		LED0 = ~LED0;
	}
	else
	{
		gps.flag_confidence = 0;
	}
	return re;

}

/********************************************************************************************************
**������Ϣ :int gps_correct(void)
**�������� :�Ե�ǰС��λ�˲�����
**������� :
**������� :
*********************************************************************************************************/
int gps_correct(void)
{
	int re = 0;
	double fuzzyboundary_heading[3] = { 1.5 / 180.0*PI,3.0 / 180.0*PI,5.0 / 180.0*PI };
	double fuzzypid_heading[4] = { 0.1,0.2,0.3,0.4 };
	//		double fuzzyboundary_xy[3]={0.03,0.06,0.09};
	//		double fuzzypid_xy[4]={0.1,0.2,0.5,0.8};
	double ph = 0;
	if (robot_motion.type == SUB_MOVE || robot_motion.type == MOVE_S || robot_motion.type == SUB_TRACKING
		|| robot_motion.type == TURN_LEFT || robot_motion.type == TURN_RIGHT)
	{
		//kalman filter
		  //����x��y��heading��gps��ֵ
		  //�Բ�ֵkalman�˲�����
		if (gps.flag_confidence == 1)
		{
			gps.kalman_dx.z = gps.x - robot_motion.x;
			gps.kalman_dy.z = gps.y - robot_motion.y;
			kalman(&gps.kalman_dx);
			kalman(&gps.kalman_dy);
			gps.piddx.ivalue += gps.kalman_dx.x;
			gps.piddx.value = gps.piddx.p * gps.kalman_dx.x + gps.piddx.i * gps.piddx.ivalue;
			gps.piddy.ivalue += gps.kalman_dy.x;
			gps.piddy.value = gps.piddy.p * gps.kalman_dy.x + gps.piddy.i * gps.piddy.ivalue;
			robot_motion.x = robot_motion.x + gps.piddx.value;
			robot_motion.y = robot_motion.y + gps.piddy.value;
		}
		else
		{
			gps.kalman_dx.x = 0;
			gps.kalman_dy.x = 0;
		}
		//��ֱ��ʱ��gps�Ƕ�����
		if (gps.flag_headingconfidence == 1)
		{
			gps.kalman_dheading.z = getAngleDiff(gps.heading, robot_motion.heading);
			kalman(&gps.kalman_dheading);
			fuzzy_pid(&ph, fuzzypid_heading, fuzzyboundary_heading, gps.kalman_dheading.x, 3);
			gps.piddheading.p = 0.1;
			robot_motion.heading = robot_motion.heading + gps.kalman_dheading.x * gps.piddheading.p;
		}
		else
		{
			gps.kalman_dheading.x = 0;
		}
	}
	else
	{
		gps.kalman_dx.x = 0;
		gps.kalman_dy.x = 0;
		gps.kalman_dheading.x = 0;
		if (gps.flag_confidence == 1)
		{
			robot_motion.x = gps.x;
			robot_motion.y = gps.y;
			robot_motion.heading = gps.heading;
		}
	}
	return re;
}


/********************************************************************************************************
**������Ϣ :int Gauss_projection(double *x,double *y,double *z,double B,double L)
**�������� :����γ����Ϣͨ����˹ͶӰת��Ϊwgs84������ϵ��
**������� :��γ����Ϣ
**������� :���xy����
*********************************************************************************************************/
int Gauss_projection(double *x, double *y, double *z, double B, double L)
{
	double a0, a2, a4, a6, a8;
	double square_e1, c;
	double p0 = 57.2957795130823208767981;
	double t, k, V, N, p, dL, X;
	double angle_to_radian = PI / 180.0;
	double tempx, tempy;
	int m_index = 2;
	switch (m_index)
	{
	case 0:                //??
		a0 = 111134.8610828;
		a2 = -16036.48022;
		a4 = 16.82805;
		a6 = -2.197E-02;
		a8 = 3E-05;
		c = 6399698.901782711;
		square_e1 = 6.738525414683E-03;
		break;
	case 1:            //1975
		a0 = 111133.0046793;
		a2 = -16038.52818;
		a4 = 16.83263;
		a6 = -2.198E-02;
		a8 = 3E-05;
		c = 6399596.6519880105;
		square_e1 = 6.739501819473E-03;
		break;
	case 2:       //wgs
		a0 = 111132.9525494;
		a2 = -16038.50840;
		a4 = 16.83260;
		a6 = -2.198E-02;
		a8 = 3E-05;
		c = 6399593.6258;
		square_e1 = 6.73949674227E-03;
		break;
	default:
		break;
	}
	L0 = 3 * ((int)((L - 1.3) / 3) + 1);
	t = tan(B*angle_to_radian);
	k = square_e1*cos(B*angle_to_radian)*cos(B*angle_to_radian);
	V = sqrt(1 + k);
	N = c / V;
	dL = L - L0;
	p = cos(B*angle_to_radian)*dL / p0;
	X = a0*B + a2*sin(2 * B*angle_to_radian) + a4*sin(4 * B*angle_to_radian) + a6*sin(6 * B*angle_to_radian) + a8*sin(8 * B*angle_to_radian);
	tempx = X + N*t*(1 + ((5 - t*t + (9 + 4 * k)*k) + ((61 + (t*t - 58)*t*t + (9 - 11 * t*t) * 30 * k) + (1385 + (-3111 + (543 - t*t)*t*t)*t*t)*p*p / 56)*p*p / 30)*p*p / 12)*p*p / 2;
	tempy = 500000 + N*(1 + ((1 - t*t + k) + ((5 + t*t*(t*t - 18 - 58 * k) + 14 * k) + (61 + (-479 + (179 - t*t)*t*t)*t*t)*p*p / 42)*p*p / 20)*p*p / 6)*p;
	*x = tempx;
	*y = tempy;
	return 0;
}

void calcMotion2Location(double x, double y, double *B, double *L)
{
	double q0, q2, q4, q6, q8;
	double square_e1, c;
	int daicha;
	double m_x, m_y;
	int m_index;
	int m_daiindex;
	double p0 = 57.2957795130823208767981;
	double radian_to_angle = 180.0 / PI;
	double tf, Nf, vf, Y, q, Bf, k, B0, dL;
	double max_x, max_y, min_x, min_y;
	max_x = 10002137.5;
	min_x = -10002137.5;
	m_index = 2;
	m_daiindex = 0;
	daicha = 3 * m_daiindex + 3;
	switch (m_index)
	{
	case 0:                //??
		c = 6399698.901782711;
		square_e1 = 6.738525414683E-03;
		q0 = 157046064.12328E-15;
		q2 = 25258869461.858E-12;
		q4 = -14919317.6572E-12;
		q6 = 120717.4265E-12;
		q8 = -1075.1509E-12;
		break;
	case 1:            //1975
		c = 6399596.6519880105;
		square_e1 = 6.739501819473E-03;
		q0 = 157048687.47416E-15;
		q2 = 2526252791.9786E-12;
		q4 = -14923644.4356E-12;
		q6 = 120769.9608E-12;
		q8 = -1075.7700E-12;
		break;
	case 2:       //wgs
		c = 6399593.6258;
		square_e1 = 6.73949674227E-03;
		q0 = 157048761.142065E-15;
		q2 = 2526250855.8867E-12;
		q4 = -14923621.5362E-12;
		q6 = 120769.6828E-12;
		q8 = -1075.7667E-12;
		break;
	default:
		break;
	}
	m_x = x;
	m_y = y;
	if (daicha == 3)
	{
		max_y = 667001.3;
		min_y = 332998.73;
	}
	else
	{
		max_y = 834117.86;
		min_y = 165882.141;
	}
	if (m_x >= min_x && m_x <= max_x && m_y >= min_y && m_y <= max_y)
	{
		B0 = m_x*q0;
		Bf = B0 + sin(2 * B0)*(q2 + sin(B0)*sin(B0) *
			(q4 + sin(B0)*sin(B0)*(q6 + q8*sin(B0)*sin(B0))));
		Y = m_y - 500000;
		tf = tan(Bf);
		k = square_e1*cos(Bf)*cos(Bf);
		vf = 1 + k;
		Nf = c / sqrt(vf);
		q = Y / Nf;
		*B = Bf*radian_to_angle + p0*tf*
			(-vf + ((5 + 3 * tf*tf*(1 + (-2 - 3 * k)*k) + 3 * k*(2 - k)) +
			(-(61 + 45 * tf*tf*(2 + tf*tf) + (107 + (-162 - 45 * tf*tf)*tf*tf)*k) +
				(1385 + (3633 + (4095 + 1575 * tf*tf)*tf*tf)*tf*tf)*q*q / 56)*q*q / 30)*q*q / 12)*q*q / 2;
		dL = p0*q / cos(Bf)*(1 + (-(1 + 2 * tf*tf + k) +
			((5 + 4 * tf*tf*(7 + 6 * tf*tf) + 2 * k*(3 + 4 * tf*tf)) -
			(61 + (662 + (1320 + 720 * tf*tf)*tf*tf)*tf*tf)*q*q / 42)*q*q / 20)*q*q / 6);
		*L = L0 + dL;
	}
	return;
}


/********************************************************************************************************
**������Ϣ :unsigned char Calc_GPS_Sum( const char* Buffer )
**�������� :����GPSУ���
**������� :�����ɵ��GPS����
**������� :
*********************************************************************************************************/
unsigned char gps_sum = 0;
unsigned char Calc_GPS_Sum(const char* Buffer)
{
	unsigned char i, j, k, sum;
	sum = 0;
	for (i = 1; i < 255; i++) //i?1?????$???
	{
		if ((Buffer[i] != '*') && (Buffer[i] != 0x00)) //?????
		{
			sum ^= Buffer[i];//GPS??????XOR
		}
		else
		{
			break;
		}
	}
	j = Buffer[i + 1];//?????????
	k = Buffer[i + 2];
	if (isalpha(j)) //???????????,???????????,?????
	{
		if (isupper(j)) //????????????,?????,?????
		{
			j -= 0x37;//?????16??
		}
		else
		{
			j -= 0x57;//?????16??
		}
	}
	else
	{
		if ((j >= 0x30) && (j <= 0x39))
		{
			j -= 0x30;//?????16??
		}
	}
	if (isalpha(k)) //???????????,???????????,?????
	{
		if (isupper(k)) //????????????,?????,?????
		{
			k -= 0x37;//?????16??
		}
		else
		{
			k -= 0x57;//?????16??
		}
	}
	else
	{
		if ((k >= 0x30) && (k <= 0x39))
		{
			k -= 0x30;//?????16??
		}
	}
	j = (j << 4) + k; //?????16??
	gps_sum = j;

	if (sum == j)
	{
		return VALID; //?????
	}
	else
	{
		return INVALID; //?????
	}
}


/********************************************************************************************************
**������Ϣ :void Creat_DH_Index( char* buffer )
**�������� :Ѱ�����ж��ŵ�λ��,��������
**������� :����GPS���ݻ�����
**������� :����ȫ�ֱ��������еĶ�������,ԭGPS�����еĶ��Ž��ᱻ0x00���
*********************************************************************************************************/
unsigned char DH_id_sep[32];
void Creat_DH_Index(char* buffer)
{
	unsigned short i, len;
	unsigned char idj;
	memset(DH_id_sep, 0, sizeof(DH_id_sep));
	len = strlen(buffer);
	for (i = 0, idj = 0; i < len; i++)
	{
		if (buffer[i] == ',')
		{
			DH_id_sep[idj] = i;
			idj++;
			buffer[i] = 0x00;
		}
	}
}

unsigned char DH_id_sep_hw[32];
void Creat_CMD_Index(char* buffer)
{
	unsigned short i, len;
	unsigned char idj;
	memset(DH_id_sep_hw, 0, sizeof(DH_id_sep_hw));
	len = strlen(buffer);
	for (i = 0, idj = 0; i < len; i++)
	{
		if (buffer[i] == ',')
		{
			DH_id_sep_hw[idj] = i;
			idj++;
			buffer[i] = 0x00;
		}
		if (buffer[i] == '*')
		{
			break;
		}
	}
}

/********************************************************************************************************
**������Ϣ :char* Real_Process_DH( char* buffer, unsigned char num )
**�������� :����GPS���ݵ�N��������ƫ��
**������� :����������Ľ���GPS���ݻ�����
**������� :���ص�N��","֮�����Ϣ����Ҫ*buffer��Ч������������ſ���ִ��
*********************************************************************************************************/
char* Real_Process_DH(char* buffer, unsigned char num)
{
	if (num < 1)
		return  &buffer[0];
	return  &buffer[DH_id_sep[num - 1] + 1];
}

char* Real_Process_DH_hw(char* buffer, unsigned char num)
{
	if (num < 1)
		return  &buffer[0];
	return  &buffer[DH_id_sep_hw[num - 1] + 1];
}

/********************************************************************************************************
**������Ϣ :void Real_GPS_Command_Process( void )
**�������� :�������ò������������ݲ�����GPS���ݽṹ��
**������� :
**������� :
//robot2 GNGGA
*********************************************************************************************************/
void Real_GPS_Command_Process(void)
{
	if (strstr(GPS_Real_buf.data, "GPGGA"))//$GPGGA,112118.000,3743.5044,N,11540.5393,E,1,06,1.6,15.3,M,-9.1,M,,0000*7E
	{
		gps.flag_dataarrive = 1;
		GPS_Information.Lat = myStod(Real_Process_DH(GPS_Real_buf.data, 2));
		GPS_Information.Lon = myStod(Real_Process_DH(GPS_Real_buf.data, 4));
		GPS_Information.Qual = (unsigned char)myStod(Real_Process_DH(GPS_Real_buf.data, 6));
		GPS_Information.Use_EPH_Sum = (unsigned char)myStod(Real_Process_DH(GPS_Real_buf.data, 7)); //?7??????????
		GPS_Information.HDOP = myStod(Real_Process_DH(GPS_Real_buf.data, 8)); //?7??????????
		GPS_Information.Lat = (double)((int)(GPS_Information.Lat*0.01)) + (GPS_Information.Lat*0.01 - (double)((int)(GPS_Information.Lat*0.01))) / 60.0*100.0;
		GPS_Information.Lon = (double)((int)(GPS_Information.Lon*0.01)) + (GPS_Information.Lon*0.01 - (double)((int)(GPS_Information.Lon*0.01))) / 60.0*100.0;
		return;
	}
	if (strstr(GPS_Real_buf.data, "GNHDT"))//$GNHDT,67.2188,T*29
	{
		gps.flag_heading_dataarrive = 1;
		gps.flag_headingconfidence = 1;
		GPS_Information.Heading = atof(Real_Process_DH(GPS_Real_buf.data, 1));
		return;
	}
	else
	{
		gps.flag_headingconfidence = 0;
	}
	return;
}


extern void analysisGPS(void)
{
	rx_gps();
	parase_gps();//gps����
	gps_correct();
}


extern char rxHuaweiCmd(void)
{
	char re = 0;
	if (HUAWEI_Cmd_buf.dataarrive == 1)
	{
		HUAWEI_Cmd_buf.dataarrive = 0;
		{
			{
				Creat_CMD_Index(HUAWEI_Cmd_buf.data);
				Real_HUAWEI_Command_Process();
			}
		}
		re = 1;
	}
	return re;
}

void Real_HUAWEI_Command_Process(void)
{
	if (strstr(HUAWEI_Cmd_buf.data, "MOVETO"))//$GPGGA,112118.000,3743.5044,N,11540.5393,E,1,06,1.6,15.3,M,-9.1,M,,0000*7E
	{
		lineplan.heart_beat_rx_rk3288 = 0;
		lineplan.flag_new_cmdframe = 1;
		HUAWEI_cmd.time = myStod(Real_Process_DH_hw(HUAWEI_Cmd_buf.data, 1)); //?7??????????
		HUAWEI_cmd.longitude = myStod(Real_Process_DH_hw(HUAWEI_Cmd_buf.data, 2));
		HUAWEI_cmd.latitude = myStod(Real_Process_DH_hw(HUAWEI_Cmd_buf.data, 3));
		HUAWEI_cmd.speed = myStod(Real_Process_DH_hw(HUAWEI_Cmd_buf.data, 4));
		HUAWEI_cmd.heading = atof(Real_Process_DH_hw(HUAWEI_Cmd_buf.data, 5));
		Gauss_projection(&lineplan.x, &lineplan.y, &gps.z, HUAWEI_cmd.latitude, HUAWEI_cmd.longitude);//��������ϵ->���ƽ������ϵ
		lineplan.x = -(lineplan.x - GPS_STANDARD_X);//���ƽ������ϵ->С������ϵ
		lineplan.y = lineplan.y - GPS_STANDARD_Y;
		lineplan.v = HUAWEI_cmd.speed;
		lineplan.heading = HUAWEI_cmd.heading + SCANER_STANDARD_HEADING;	//�Ƕȱ궨δ��  
		cmd.dataarrive = 1;
		cmd.type = SUB_TRACKING;
		return;
	}
}

extern void initGPSData(void)
{
	gps.flag_confidence = 1;
	gps.kalman_x.p = 0.01;
	gps.kalman_y.p = 0.01;
	gps.kalman_dx.p = 0.01;
	gps.kalman_dy.p = 0.01;
	gps.kalman_heading.p = 0.01;
	gps.kalman_dheading.p = 0.01;
	gps.kalman_x.q = 0.01;
	gps.kalman_y.q = 0.01;
	gps.kalman_dx.q = 0.01;
	gps.kalman_dy.q = 0.01;
	gps.kalman_heading.q = 0.01;
	gps.kalman_dheading.q = 0.01;
	gps.kalman_x.r = 2.0;
	gps.kalman_y.r = 2.0;
	gps.kalman_dx.r = 0.2;
	gps.kalman_dy.r = 0.2;
	gps.kalman_heading.r = 0.2;
	gps.kalman_dheading.r = 0.2;
	gps.piddx.p = 0.1;
	gps.piddx.i = 0.001;
	gps.piddy.p = 0.1;
	gps.piddy.i = 0.001;
	gps.piddheading.p = 1;
	gps.piddheading.i = 0.01;
}

static u8 head_gps;

void USART2_IRQHandler(void) //??2??????
{
	unsigned char Recv;
	if (USART2->SR&(1 << 5))//���յ�����
	{
		Recv = USART2->DR;
		if (GPS_BL_buf.dataarrive == 0 || GPS_Heading_buf.dataarrive == 0)
		{
			if (Recv == '$')
			{
				GPS_Uart_buf.rx_pc = 0;
				head_gps = '$';
			}
			else
			{
				if (GPS_Uart_buf.rx_pc < sizeof(GPS_Uart_buf.data) - 1)
				{
					GPS_Uart_buf.rx_pc++;
				}
			}
			if (head_gps == '$')
			{
				GPS_Uart_buf.data[GPS_Uart_buf.rx_pc] = Recv;
				if (Recv == '\r')    //??????0x0D?????GPS??
				{
					GPS_Uart_buf.dataarrive = 1;
					head_gps = 0;
					if (strstr(GPS_Uart_buf.data, "GPGGA"))
					{
						memcpy(&GPS_BL_buf, &GPS_Uart_buf, sizeof(GPS_REALBUF_STRUCT_TYPE));

					}
					if (strstr(GPS_Uart_buf.data, "GPHDT"))
					{
						memcpy(&GPS_Heading_buf, &GPS_Uart_buf, sizeof(GPS_REALBUF_STRUCT_TYPE));
					}
				}
			}
		}
	}
}
///////���ӿ���̨///////////////////////////////
