#include "stargazer.h"
#include "usart.h"
#include "delay.h" 
#include "lcd.h"
#include "robot_action.h"
#include "math.h"
char gazer_str[100], gazer_str_len=0, gazer_str_ok=0;



short mpu_oldg[10], mpu_pos=0, mpu_off=300;
int mpu_sum;

typedef struct TWRPKG{
  unsigned short magic; // 0x5758 2??
  unsigned char type;  // 0x70   1??
  unsigned char body_len; // 16  1??
  unsigned short sequence; // 0~65535 2??
  uint64_t tagid;    // in  8??
  float x; //4??
  float y; //4??
  float z; //4??   // 12 bytes to replace the former 12bytes
  float heading;//4??
  unsigned short crc;//2??
}TWRPKG;

TWRPKG uwb_point;
//unsigned char uwb_data[32];

//void USART2_IRQHandler(void)
//{
//	u8 USART2_Temp=0, i=0;
//	if(USART2->SR&(1<<5))//接收到数据
//	{	 
//		USART2_Temp = USART2->DR;
//		
//		  uwb.led_cnt ++; 
//			if(uwb_ok == 1)   //recv head
//			{
//				if(USART2_Temp == 0x57)
//				{
//					uwb_pos = 0;
//					uwb_data[uwb_pos++] = USART2_Temp;
//				}
//				else if(uwb_data[0] == 0x57 && USART2_Temp == 0x58)
//				{
//					uwb_data[uwb_pos++] = USART2_Temp;
//					uwb_ok = 0;
//				}
//			}
//			else if(uwb_ok == 0 && uwb_pos < 4)   //recv data
//			{
//				uwb_data[uwb_pos++] = USART2_Temp;
//			}
//			else if(uwb_ok == 0 && uwb_data[2] == 0x40 && uwb_pos < uwb_data[3]+2)   //recv data
//			{
//				uwb_data[uwb_pos++] = USART2_Temp;
//			}
//			else
//			{
//				if(uwb_data[2] == 0x40)
//				{				    
//					  uwb.seq = (short)uwb_data[4] | ((short)uwb_data[5] << 8);
//					  uwb.origin_x = (double)((short)uwb_data[8] | ((short)uwb_data[9] << 8));
//					  uwb.origin_y = (double)((short)uwb_data[10] | ((short)uwb_data[11] << 8));
//					  if(uwb.origin_x > 32767)
//            {
//						    uwb.origin_x = uwb.origin_x -65536;
//						}						
//						if(uwb.origin_y > 32767)
//            {
//						    uwb.origin_y = uwb.origin_y -65536;
//						}
//						uwb.origin_x = uwb.origin_x*0.01;
//					  uwb.origin_y = uwb.origin_y*0.01;
//						if(robot_motion.flag_uwbtag_standard == 1 && robot_motion.flag_initial_pos == 1)
//				    {	
//			          uwb.raw_x = uwb.origin_x - robot_motion.uwbtag2robot_r * cos(robot_motion.uwbtag2robot_ang + robot_motion.heading);
//			          uwb.raw_y = uwb.origin_y - robot_motion.uwbtag2robot_r * sin(robot_motion.uwbtag2robot_ang + robot_motion.heading);
//	          }
//						uwb.kalman_x.x[0] = uwb.raw_x;
////					  for(i=0; i<2; i++)
////				    {
////					      ((unsigned char*)(&(uwb_point.x)))[i] = uwb_data[8+i];
////					      ((unsigned char*)(&(uwb_point.y)))[i] = uwb_data[10+i];
////					//((unsigned char*)(&(uwb_point.z)))[i] = uwb_data[22+i];
////					      uwb.raw_x = (double)uwb_point.x;
////					      uwb.raw_y = (double)uwb_point.y;
////					
////				    }
//				}
//				
//				uwb_ok = 1;
//				TIME_IO = ~TIME_IO;
//				uwb_data[0] = uwb_data[1] = 0;	
//			}
//	}
//}

void stargazer_show(char *str)
{
	printf(str);
	LCD_Fill(30, 150, 430, 166, WHITE);
	LCD_ShowString(30,150,400,16,16, str);
}

int stargazer_send(char *str)
{
	//uart3_sendstr(str);
	uart2_sendstr(str);
	stargazer_show(str);

	return 0;
}

int str_compare(char* str1, char* str2)
{
	int i=0;
	for(i=0; str1[i]!=0&&str2[i]!=0; i++)
	{
		if(str1[i]!=str2[i]){return 0;}
	}
	if(i!=0 && str1[i]==0 && str2[i]==0){return 1;}
	return 0;
}

int setcmd(char *str)
{
	char tmp, str_tmp[50];
	int i=0, j=0;
	for(i=0; str[i]!='`' && i<50; i++){str_tmp[i] = str[i];}
	str_tmp[i] = '`'; str_tmp[i+1] = 0;
	tmp = str_tmp[1];
	for(i=0; i<20; i++)
	{
		stargazer_send(str_tmp);
		str_tmp[1] = '!';
		//delay_ms(100);
		for(j=0; j<50; j++)
		{
			if(gazer_str_ok==1 && str_compare(gazer_str, str_tmp))
			{
				gazer_str_ok=0;
				return 1;
			}
			delay_ms(10);
		}
		str_tmp[1] = tmp;
	}
	return 0;
}

int stargazer_init(int num, int refid)
{
	int t=0;
	
	//char str_idnum[15]={0}, str_refid[15]={0};
	//sprintf(str_idnum, "~#IDNum|%d`", num);
	//sprintf(str_idnum, "~#RefID|%d`", refid);
	
	char str_idnum[15]="~#IDNum|00`", str_refid[15]="~#RefID|000`", tmp=0;
	str_idnum[9] = num%10 + 48; num /= 10;
	str_idnum[8] = num%10 + 48;
	str_refid[10] = refid%10 + 48; refid /= 10;
	str_refid[9] = refid%10 + 48; refid /= 10;
	str_refid[8] = refid%10 + 48;
	
	if(setcmd("~#CalcStop`") == 0){return 0;}
	if(setcmd(str_idnum) == 0){return 0;}
	if(setcmd(str_refid) == 0){return 0;}
	if(setcmd("~#MarkType|HLD1S`") == 0){return 0;}
	//if(setcmd("~#HeightFix|No`") == 0){return 0;}
	//if(setcmd("~#MarkHeight|2000`") == 0){return 0;}
	if(setcmd("~#SetEnd`") == 0){return 0;}
	for(t=0; t<500; t++)
	{
		if(gazer_str_ok==1 && str_compare(gazer_str, "~!ParameterUpdate`")){return 1;}
		delay_ms(10);
	}
	if(t>=500){return 0;}
	if(setcmd("~#CalcStart`") == 0){return 0;}

	return 1;
}
//start building map
int map_building()
{
	if(setcmd("~#CalcStop`") == 0){return 0;}
	if(setcmd("~#MarkMode|Map`") == 0){return 0;}
	if(setcmd("~#MapMode|Start`") == 0){return 0;}
	if(setcmd("~#CalcStart`") == 0){return 0;}
	
	return 1;
}
//get stargazer state
int stargazer_state()
{
	if(setcmd("~#CalcStop`") == 0){return 0;}
	if(setcmd("~@IDNum`") == 0){return 0;}
	if(setcmd("~@RefID`") == 0){return 0;}
	if(setcmd("~@MarkType`") == 0){return 0;}
	if(setcmd("~@HeightFix`") == 0){return 0;}
	if(setcmd("~@MarkHeight`") == 0){return 0;}
	if(setcmd("~#SetEnd`") == 0){return 0;}
	if(setcmd("~#CalcStart`") == 0){return 0;}
	
	return 1;
}

POINT gazer_pos;

int stargazer_pos()
{
		int i=0, j=0;
		char type=0;

		if(gazer_str_ok == 1 && gazer_str[1]=='^')
		{
			//get smode
			if(gazer_str[2]=='F'){gazer_pos.smode = 1;}      //building map
			else if(gazer_str[2]=='I'){gazer_pos.smode = 2;} //build complete
			else if(gazer_str[2]=='Z'){gazer_pos.smode = 3;} //hight value
			else {gazer_pos.smode = 0;}                    //no data
			//get sid
			for(gazer_pos.sid=0,i=3; gazer_str[i]!='|'; i++)
			{
				gazer_pos.sid = gazer_pos.sid*10+gazer_str[i]-48;
			}
			//get sa
			i++;
			type = gazer_str[i++];
			for(gazer_pos.sa=0; gazer_str[i]!='.'; i++)
			{
				gazer_pos.sa = gazer_pos.sa*10+gazer_str[i]-48;
			}	
			for(i++,j=0; gazer_str[i]!='|'; i++,j++)
			{
				if(j==0){gazer_pos.sa += (gazer_str[i]-48)/10.0;}
				else if(j==1){gazer_pos.sa += (gazer_str[i]-48)/100.0;}
			}
			if(type=='+'){gazer_pos.sa = 360-gazer_pos.sa;}
			//get sx
			i++;
			type = gazer_str[i++];
			for(gazer_pos.sx=0; gazer_str[i]!='.'; i++)
			{
				gazer_pos.sx = gazer_pos.sx*10+gazer_str[i]-48;
			}	
			for(i++,j=0; gazer_str[i]!='|'; i++,j++)
			{
				if(j==0){gazer_pos.sx += (gazer_str[i]-48)/10.0;}
				else if(j==1){gazer_pos.sx += (gazer_str[i]-48)/100.0;}
			}
			if(type=='-'){gazer_pos.sx *= -1;}
			//get sy
			i++;
			type = gazer_str[i++];
			for(gazer_pos.sy=0; gazer_str[i]!='.'; i++)
			{
				gazer_pos.sy = gazer_pos.sy*10+gazer_str[i]-48;
			}	
			for(i++,j=0; gazer_str[i]!='|'; i++,j++)
			{
				if(j==0){gazer_pos.sy += (gazer_str[i]-48)/10.0;}
				else if(j==1){gazer_pos.sy += (gazer_str[i]-48)/100.0;}
			}
			if(type=='-'){gazer_pos.sy *= -1;}
			//get sz
			i++;
			for(gazer_pos.sz=0; gazer_str[i]!='.'; i++)
			{
				gazer_pos.sz = gazer_pos.sz*10+gazer_str[i]-48;
			}	
			for(i++,j=0; gazer_str[i]!='`'; i++,j++)
			{
				if(j==0){gazer_pos.sz += (gazer_str[i]-48)/10.0;}
				else if(j==1){gazer_pos.sz += (gazer_str[i]-48)/100.0;}
			}
			stargazer_show(gazer_str);
			gazer_str[1] = '-';
			gazer_str_ok = 0;

			return 1;
		}
		return 0;
}
