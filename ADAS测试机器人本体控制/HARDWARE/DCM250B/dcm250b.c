#include "dcm250b.h"
#include "..\USER\basicfunc.h"
#include "math.h"
#include "delay.h"  
#include "string.h"
#include "stdlib.h"
#include "usart.h"
#include "..\USER\hmc5883.h"
#include "..\USER\robot_action.h"

ROBOT_DCM250B dcm250b;
ROBOT_DCM250B_CMD dcm250b_cmd;


/**************************实现函数********************************************
*函数原型:	  double dcm2f(const char *st) 
*功　　能:	 ascii--->double,for dcm250b
输入参数： 
输出参数：
*******************************************************************************/
char sensor_detection(double gyro_omg,double compass_heading)  
{  
    char re = 0,i;
	  double diffomg,mean_diffomg,diff_1st;
	  compass.omg = (compass_heading - compass.heading)/0.1;
	  compass.heading = compass_heading;
	  diffomg = compass.omg - gyro_omg;
	   for(i=1;i<5;i++)
		{
				compass.diffomg[i-1] = compass.diffomg[i];
		}
		//求陀螺与罗盘变化量差值
		compass.diffomg[4] = diffomg; 
		mean_diffomg = (compass.diffomg[0]+compass.diffomg[1]+compass.diffomg[2]+compass.diffomg[3]+compass.diffomg[4])*0.2;
		//求均值一阶差分
    diff_1st = mean_diffomg - compass.mean_diffomg;
		compass.mean_diffomg = mean_diffomg;
		//求均值二阶差分
    compass.diff_2nd = diff_1st - compass.diff_1st;
		compass.diff_1st = diff_1st;
		
		
//	  double compass_heading_diff,gyro_heading_diff,diff_mean,diff_1st;
//	  compass_heading_diff = compass_heading - compass.heading;
//	  compass.heading = compass_heading;
//    gyro_heading_diff = gyro_heading - gyro.heading;
//	  gyro.heading = gyro_heading;
//    for(i=1;i<5;i++)
//		{
//				compass.diff_compass_gyro[i-1] = compass.diff_compass_gyro[i];
//		}
//		//求陀螺与罗盘变化量差值
//		compass.diff_compass_gyro[4] = compass_heading_diff - gyro_heading_diff;
//    //求差值均值
//    diff_mean = (compass.diff_compass_gyro[0] + compass.diff_compass_gyro[1] + compass.diff_compass_gyro[2]+
//		                  compass.diff_compass_gyro[3] + compass.diff_compass_gyro[4])/5.0f;
//		
////		diff_mean = compass.diff_compass_gyro[4];
//		//求均值一阶差分
//    diff_1st = diff_mean - compass.diff_mean;
//		compass.diff_mean = diff_mean;
//		//求均值二阶差分
//    compass.diff_2nd = diff_1st - compass.diff_1st;
//		compass.diff_1st = diff_1st;
//		
		return re;
}  

/**************************实现函数********************************************
*函数原型:	  double dcm2f(const char *st) 
*功　　能:	 ascii--->double,for dcm250b
输入参数： 
输出参数：
*******************************************************************************/
double dcm2f(char *buff)  
{  
    double s = 0.0;
	  s = (double)((int)(buff[0]%16)*100)
        + (double)(((int)(buff[1]/16))*10) + (double)((int)(buff[1]%16))
				+ ((double)((int)(buff[2]/16)))*0.1 + (double)((int)(buff[2]%16))*0.01;
		if((int)(buff[0]/16) == 1)
		{
		    s = -s;
		}
    return s;  
}  

double dcm2f_mag(char *buff)  
{  
    double s = 0.0;
	  s = (double)((int)(buff[0]%16)*10)
        + (double)(((int)(buff[1]/16))) + (double)((int)(buff[1]%16))*0.1;
		if((int)(buff[0]/16) == 1)
		{
		    s = -s;
		}
    return s;  
} 
/**************************实现函数********************************************
*函数原型:	  char dcm250b_command_process(char *buff)
*功　　能:	 解析dcm数据
输入参数： 
输出参数：
*******************************************************************************/

char dcm250b_command_process(char *buff)
{
    char re = 0;
	  unsigned char type;
	  
    type = buff[3];	
	  switch(type)
		{
			case 0x84:
			    dcm250b.pitch = dcm2f(&buff[4])*PI/180.0;	
			    dcm250b.roll = dcm2f(&buff[7])*PI/180.0;
			    dcm250b.heading = dcm2f(&buff[10])*PI/180.0;
			
			break;
			case 0x87:
			    dcm250b.mag_dec = dcm2f_mag(&buff[4])*PI/180.0;	
			    dcm250b.flag_wait_mag = 0;
			
			break;
			default:
				break;
		
		}
    return re;
}

char rx_dcm250b(void)
{
    char re = 0,i;
	  if(dcm2stm_uart.dataarrive == 1)
	  {
				
			  dcm250b_command_process(&dcm2stm_uart.rdata[0]);
			  compass.angle = -dcm250b.heading;
			  compass.angle = compass.angle + COMPASS_ZEROBIAS;
		    compass.angle = wrapToPiDe(compass.angle);				
		
				for(i=1;i<10;i++)
				{
						compass.buff[i-1] = compass.buff[i];
				}
				compass.buff[10-1] = compass.angle;
				compass.var = get_var(compass.buff,10);	
					
				dcm2stm_uart.dataarrive = 0;  
				re = 1;
				
				
		}
		return re;	
}

char tx_dcm250b(char len,char type,float data)
{
    char re = 0,aa,i;
	  char *p = &dcm250b_cmd.head;;
    dcm250b_cmd.head = 0x68;
	  dcm250b_cmd.length = len;
	  dcm250b_cmd.addr = 0;
	  dcm250b_cmd.type = type;
	  dcm250b_cmd.checksum = 0;
	  switch(type)
		{
			case SET_MAG_DEC:
						dcm250b_cmd.data[0] = (signdata(data)<<4)|((char)(data*0.1f)); 			
						aa = (char)((data*0.1f - (char)(data*0.1f))*100.0f + 0.5f);
						dcm250b_cmd.data[1] = (((char)(aa*0.1f))<<4)|((char)aa%10); 	
						for(i=0;i<dcm250b_cmd.length - 4;i++)
						{
								dcm250b_cmd.checksum += dcm250b_cmd.data[i]; 
						}			
						dcm250b_cmd.checksum = dcm250b_cmd.checksum + dcm250b_cmd.length + dcm250b_cmd.addr
													 + dcm250b_cmd.type;		  			
			break;
			
			case READ_MAG_DEC:
				  dcm250b.flag_wait_mag = 1;
					dcm250b_cmd.checksum = dcm250b_cmd.length + dcm250b_cmd.addr
												 + dcm250b_cmd.type;					
			break;
			case START_CALIBRATION:
					dcm250b_cmd.checksum = dcm250b_cmd.length + dcm250b_cmd.addr
												 + dcm250b_cmd.type;					
			break;
			case SAVE_CALIBRATION:
					dcm250b_cmd.checksum = dcm250b_cmd.length + dcm250b_cmd.addr
												 + dcm250b_cmd.type;					
			break;
			
			
			default:
				break;
		
		}
		for(i = 0;i<dcm250b_cmd.length;i++)
		{
				uart1_send(*(p+i));
		}
		uart1_send(dcm250b_cmd.checksum);

    return re;
}








































///////////////////////////////////end////////////////////////////////


