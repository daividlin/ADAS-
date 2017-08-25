#include "..\USER\robot_action.h"
#include "delay.h"  
#include "string.h"
#include "math.h"
#include "usart.h"
#include "uwb.h" 
#include "canopen.h"

ROBOTUWB uwb;
UWBFDB uwbfdb;

int parase_uwb(void)
{
    u8 i = 0;
	  uwb.xy_buff_num = 5;
	  if(uwb.flag_data_arrive == 1)
		{
		    uwb.flag_data_arrive = 0;
			  uwb.offline_cnt = 0;
			  //���uwb����δ��������
			  if(uwb.origin_x == uwb.pre_origin_x && uwb.origin_y == uwb.pre_origin_y)
				{
				    uwb.unchanged_cnt ++;
					  if(uwb.unchanged_cnt > 10)
						{
						    uwb.unchanged_cnt = 10;
							  robot_motion.flag_stop = 1;
							  uwb.flag_data_normal = 0;
						}					
				}
        else
        {
				    uwb.unchanged_cnt = 0;	
            uwb.buff_cnt ++;
            if(uwb.buff_cnt > uwb.xy_buff_num)
            {
						    uwb.buff_cnt = uwb.xy_buff_num;
							  uwb.flag_data_normal = 1;
						} 							
				}					
			  uwb.x_v = (uwb.origin_x - uwb.pre_origin_x)/TIMER_PERIOD;
			  uwb.y_v = (uwb.origin_y - uwb.pre_origin_y)/TIMER_PERIOD;
			  if(robot_motion.flag_uwbtag_standard == 1 && robot_motion.flag_initial_pos == 1)
				{	
			      if(fabs(uwb.x_v) < 5)
						{
						    uwb.raw_x = uwb.origin_x - robot_motion.uwbtag2robot_r * cos(robot_motion.uwbtag2robot_ang + robot_motion.heading);
							  for(i=1;i<uwb.xy_buff_num;i++)
		            {
		                uwb.x_buff_100ms[i-1] = uwb.x_buff_100ms[i];
		            }
		            uwb.x_buff_100ms[uwb.xy_buff_num-1] = uwb.raw_x;
			      }
            else
            {
						   clear_buff(uwb.x_buff_100ms,uwb.xy_buff_num); 
						}							
					  if(fabs(uwb.x_v) < 5)
						{
							  uwb.raw_y = uwb.origin_y - robot_motion.uwbtag2robot_r * sin(robot_motion.uwbtag2robot_ang + robot_motion.heading);	
								for(i=1;i<uwb.xy_buff_num;i++)
								{
										uwb.y_buff_100ms[i-1] = uwb.y_buff_100ms[i];
								}
								uwb.y_buff_100ms[uwb.xy_buff_num-1] = uwb.raw_y;							
						}
						else
						{
						    clear_buff(uwb.y_buff_100ms,uwb.xy_buff_num);
						}	
				}
//				for(i=1;i<uwb.xy_buff_num;i++)
//		    {
//		        uwb.x_buff_100ms[i-1] = uwb.x_buff_100ms[i];
//		        uwb.y_buff_100ms[i-1] = uwb.y_buff_100ms[i];
//		    }
//		    uwb.x_buff_100ms[uwb.xy_buff_num-1] = uwb.raw_x;
//		    uwb.y_buff_100ms[uwb.xy_buff_num-1] = uwb.raw_y;
								
				//������ʱ����uwb����������
				if(fabs(uwb.x_buff_100ms[0]) > 0 && fabs(uwb.y_buff_100ms[0]) > 0)
	      {			  
				  //��uwb���ݷ���
					  uwb.pre_raw_x_var = uwb.raw_x_var;
				    uwb.pre_raw_y_var = uwb.raw_y_var;
					  uwb.raw_x_var = (double)(get_var(uwb.x_buff_100ms,uwb.xy_buff_num));
				    uwb.raw_y_var = (double)(get_var(uwb.y_buff_100ms,uwb.xy_buff_num));
//					  uwb.kalman_x.x0 = 0;
//					  uwb.kalman_y.x0 = 0;
//					  for(i=0;i<uwb.xy_buff_num;i++)
//		        {
//		            uwb.kalman_x.x0 += uwb.x_buff_100ms[i]/((double)uwb.xy_buff_num);
//		            uwb.kalman_y.x0 += uwb.y_buff_100ms[i]/((double)uwb.xy_buff_num);
//		        }
					  
	      }		 
		}	
		else//����uwb 10s������֡���
		{
		    uwb.offline_cnt ++;
			  if(uwb.offline_cnt > 100)
			  {
				    uwb.offline_cnt = 100;
					  robot_motion.flag_stop = 1;
					  uwb.flag_data_normal = 0;
				}
		}
		return 0;
}	



	
//		
//void tx_uwb(void)
//{
//	  unsigned char aa[14];
//	  uwbfdb.seq ++;
//	  uwbfdb.leftspd = (short)(gHalData.WheelHal[0].WheelVel*4096.0);
//    uwbfdb.rightspd = (short)(gHalData.WheelHal[1].WheelVel*4096.0);
//	  uwbfdb.forwardspd = (short)(robot_motion.omg*10000.0);
//	  aa[0] = 0x57;
//	  aa[1] = 0x58;
//	  aa[2] = 0x46;
//	  aa[3] = 0x08;
//	  aa[4] = (char)(uwbfdb.seq & 0x00ff);
//	  aa[5] = (char)((uwbfdb.seq & 0xff00) >> 8);
//	  aa[6] = (char)(uwbfdb.leftspd & 0x00ff);
//	  aa[7] = (char)((uwbfdb.leftspd & 0xff00) >> 8);
//	  aa[8] = (char)(uwbfdb.rightspd & 0x00ff);
//	  aa[9] = (char)((uwbfdb.rightspd & 0xff00) >> 8);
//	  aa[10] = (char)(uwbfdb.forwardspd & 0x00ff);
//	  aa[11] = (char)((uwbfdb.forwardspd & 0xff00) >> 8);
//	  aa[12] = 0;
//	  aa[13] = 0;
//	  robot2uwb_cmd(aa,14);
//}

void uwb_correct_kalman(void)
{
    int i = 0;
	  double predict_x,predict_y;
	  uwb.heading_buff_num  = 4;
	  uwb.xy_buff_num = 5;
	  uwb.deltaxy_buff_num = 5; 

	  uwb.heading_var_tol = 0.035*PI/180.0;//30-45/1000
	  uwb.updata_dis_tol = 0.1;
	  uwb.creat_buff_dis_tol = 0.1;
	  uwb.raw_xy_var_tol = 0.005;//0.006;0.005
	
	 //�������һ����ͨ�˲��������߼�Ȩ,period=100ms
	  if(robot_motion.flag_timer_100ms == 1)
		{
        robot_motion.flag_timer_100ms = 0;
		  	if(fabs(uwb.x_buff_100ms[0]) > 0 && fabs(uwb.y_buff_100ms[0]) > 0)
	      {	  
				  //��uwb����Ԥ��ֵ
            predict_x = robot_motion.x + robot_motion.v * TIMER_PERIOD * cos(robot_motion.heading );        	
					  predict_y = robot_motion.y + robot_motion.v * TIMER_PERIOD * sin(robot_motion.heading );
					
	        //��uwb���ݾ�����  
					  uwb.rms_x = (double)(get_rms(uwb.x_buff_100ms,uwb.xy_buff_num,predict_x));
					  uwb.rms_y = (double)(get_rms(uwb.y_buff_100ms,uwb.xy_buff_num,predict_y));
					  //�޳��쳣���� ,serios
					  if((fabs(predict_x-uwb.raw_x) < 1.0)&&(fabs(predict_y-uwb.raw_y) < 1.0))
					  {						    
						    //uwb����ƫ���Сʱ�����ݿ���׼ȷ��ֻ�������Ƚϴ�
							  if((fabs(predict_x-uwb.raw_x) > 0.2) || (fabs(predict_y-uwb.raw_y) > 0.2))
								{
//										if(fabs(uwb.pre_raw_x_var) > uwb.raw_xy_var_tol || fabs(uwb.pre_raw_y_var) > uwb.raw_xy_var_tol)
                    if(fabs(uwb.raw_x_var) > uwb.raw_xy_var_tol*10.0 || fabs(uwb.raw_y_var) > uwb.raw_xy_var_tol*10.0)
										{
												uwb.raw_x = predict_x;
        		            uwb.raw_y = predict_y;																								
										}
					    	}						
						}
            else
            {
						    //uwbֵƫ��ϴ�ʱ��������֡�����Сʱ�����ݿɿ�����Ϊuwbƫ��ϴ�ʱ��������uwb������׼
							  if(fabs(uwb.raw_x_var) > uwb.raw_xy_var_tol*5.0 || fabs(uwb.raw_y_var) > uwb.raw_xy_var_tol*5.0 
									|| fabs(uwb.pre_raw_x_var) > uwb.raw_xy_var_tol*5.0 || fabs(uwb.pre_raw_y_var) > uwb.raw_xy_var_tol*5.0)
							  {
								    uwb.raw_x = predict_x;
        		        uwb.raw_y = predict_y;
                } 		
						}								
            //��uwb����kalman�˲�	
//				    					  
//	          if(uwb.raw_x_var > 0.01)
//						{
//						    uwb.kalman_x.var = 0.01;
//						}	
//					  if(uwb.raw_y_var > 0.01)
//						{
//						    uwb.kalman_y.var = 0.01;
//						}	
						if(uwb.raw_x_var < 0.001)
						{
						    uwb.kalman_x.r = 0.1;
						}	
						else
						{
						    uwb.kalman_x.r = 0.1 + uwb.raw_x_var*1000.0;					  
						}
					  if(uwb.raw_y_var < 0.001)
						{
						    uwb.kalman_y.r = 0.1;
						}	
						else
            {
						    uwb.kalman_y.r = 0.1 + uwb.raw_y_var*1000.0;
						}
            uwb.kalman_x.z = uwb.raw_x;
				    kalman(&uwb.kalman_x);	
            uwb.kalman_y.z = uwb.raw_y;
				    kalman(&uwb.kalman_y);			
				    uwb.now_x1 = uwb.now_x;
            uwb.now_y1 = uwb.now_y;
					  	
            uwb.now_x = uwb.kalman_x.x;    
				    uwb.now_y = uwb.kalman_y.x;									        					
		
				}
        else
        {
				    uwb.kalman_x.z = robot_motion.x;
            uwb.kalman_y.z = robot_motion.y;
					  uwb.kalman_x.x = robot_motion.x;
            uwb.kalman_y.x = robot_motion.y;
				}   					
		}
    uwb.stable_dis += robot_motion.v*TIMER_PERIOD;		
	  //uwb�����ȶ�����λ����Ϣ
    if(fabs(uwb.raw_x_var)<uwb.raw_xy_var_tol && fabs(uwb.raw_y_var)<uwb.raw_xy_var_tol 
			&& fabs(uwb.x_buff_100ms[0]) > 0 && fabs(uwb.y_buff_100ms[0]) > 0)//�����ж���Ϊ�˽������ʱ�������ݲ��õ����⡣
    {
    //��uwb��С��λ�ü�Ĳ�ֵ�����ڶ�С��λ�ù���Ĳ���,0.1m����һ�Σ�
		    uwb.delta_x = uwb.now_x - robot_motion.x;
		    uwb.delta_y = uwb.now_y - robot_motion.y;				   

	      uwb.kalman_dx.r = 10.0;	
        uwb.kalman_dx.z = uwb.delta_x;
				kalman(&uwb.kalman_dx);				
	      uwb.kalman_dy.r = 10.0;	
        uwb.kalman_dy.z = uwb.delta_y;
				kalman(&uwb.kalman_dy);
			
			  uwb.delta_now_x = uwb.kalman_dx.x;
			  uwb.delta_now_y = uwb.kalman_dy.x;
		    //����λ��
				
				if(fabs(uwb.stable_dis) > uwb.creat_buff_dis_tol)
				{	
				    uwb.stable_dis = 0;
					  //��uwb����ֵ�ȶ�������������֡����ɿ�ʱ��������
					  if(fabs(uwb.now_x1 - uwb.now_x) < 0.4 && fabs(uwb.now_y1 - uwb.now_y) < 0.4 
							&& fabs(uwb.pre_raw_x_var) < uwb.raw_xy_var_tol && fabs(uwb.pre_raw_y_var) < uwb.raw_xy_var_tol)
				    {
				  	    uwb.getxy_cnt ++;					  
                robot_motion.x = robot_motion.x + uwb.delta_now_x*0.1;	//0.5	
                robot_motion.y = robot_motion.y + uwb.delta_now_y*0.1;  //0.5
			      }
			  }				
 	  }
		else
		{
		    uwb.stable_dis = 0;
		}	
		//100mm���ڣ���С��ʵʱ�Ƕȣ������ȶ��󣬸��½Ƕ�
		uwb.creat_buff_dis += robot_motion.v*TIMER_PERIOD;
	  if(fabs(uwb.creat_buff_dis) > uwb.creat_buff_dis_tol)
	  {
		    uwb.creat_buff_dis = 0;
				//��������
		    for(i=1;i<uwb.xy_buff_num;i++)
		    {
		        uwb.x_buff_100mm[i-1] = uwb.x_buff_100mm[i];
		        uwb.y_buff_100mm[i-1] = uwb.y_buff_100mm[i];
		    }
        uwb.x_buff_100mm[uwb.xy_buff_num-1] = robot_motion.x;
		    uwb.y_buff_100mm[uwb.xy_buff_num-1] = robot_motion.y;
		    //����Ƕȵ��������Ϊ900mm������Ϊ100mm��һ�Ρ�
				if(fabs(uwb.y_buff_100mm[uwb.xy_buff_num-1])>0 && fabs(uwb.y_buff_100mm[0])>0 
				&& fabs(uwb.x_buff_100mm[uwb.xy_buff_num-1])>0 && fabs(uwb.x_buff_100mm[0])>0)
		  	{
				    uwb.now_heading = atan2(uwb.y_buff_100mm[uwb.xy_buff_num-1] - uwb.y_buff_100mm[0],
    		    uwb.x_buff_100mm[uwb.xy_buff_num-1] - uwb.x_buff_100mm[0]);
        }
				//�����ǶȻ��� 
				for(i=1;i<uwb.heading_buff_num;i++)
		    {
		        uwb.heading_buff[i-1] = uwb.heading_buff[i];
			  }  
	      uwb.heading_buff[uwb.heading_buff_num-1] = uwb.now_heading; 
        if(fabs(uwb.heading_buff[0]) > 0)
	      {
			      uwb.now_heading_var = get_var(uwb.heading_buff,uwb.heading_buff_num);
	      //updata current pos and heading;
            if(uwb.now_heading_var < uwb.heading_var_tol)
            {                   			      
					  		uwb.getheading_cnt ++;
								robot_motion.heading = robot_motion.heading + getanglediff(uwb.now_heading,robot_motion.heading)*0.05;     
            }		            	
	      }			
		}
}

//�ǵ���ջ��������
void uwb_correct(void)
{
    int i = 0;
	  double predict_x,predict_y;
	  uwb.heading_buff_num  = 4;
	  uwb.xy_buff_num = 10;
	  uwb.deltaxy_buff_num = 5; 

	  uwb.heading_var_tol = 0.035*PI/180.0;//30-45/1000
	  uwb.updata_dis_tol = 0.1;
	  uwb.creat_buff_dis_tol = 0.1;
	  uwb.raw_xy_var_tol = 0.005;//0.006;0.005
	
	  uwb.offset_alfa = PI/180.0;
	  uwb.offset_r = 0.12;
	 //�������һ����ͨ�˲��������߼�Ȩ,period=100ms
	  if(robot_motion.flag_timer_100ms == 1)
		{
        robot_motion.flag_timer_100ms = 0;
        
			  
			  for(i=1;i<uwb.xy_buff_num;i++)
		    {
		        uwb.x_buff_100ms[i-1] = uwb.x_buff_100ms[i];
		        uwb.y_buff_100ms[i-1] = uwb.y_buff_100ms[i];
		    }
		    uwb.x_buff_100ms[uwb.xy_buff_num-1] = uwb.raw_x;
		    uwb.y_buff_100ms[uwb.xy_buff_num-1] = uwb.raw_y;
				
				
				//������ʱ����uwb����������
				if(fabs(uwb.x_buff_100ms[0]) > 0 && fabs(uwb.y_buff_100ms[0]) > 0)
	      {			  
				  //��uwb����Ԥ��ֵ
            predict_x = robot_motion.x + robot_motion.v * TIMER_PERIOD * cos(robot_motion.heading );        	
					  predict_y = robot_motion.y + robot_motion.v * TIMER_PERIOD * sin(robot_motion.heading );
					//��uwb���ݷ���
					  uwb.pre_raw_x_var = uwb.raw_x_var;
				    uwb.pre_raw_y_var = uwb.raw_y_var;
					  uwb.raw_x_var = (double)(get_var(uwb.x_buff_100ms,uwb.xy_buff_num));
				    uwb.raw_y_var = (double)(get_var(uwb.y_buff_100ms,uwb.xy_buff_num));
	        //��uwb���ݾ�����  
					  uwb.rms_x = (double)(get_rms(uwb.x_buff_100ms,uwb.xy_buff_num,predict_x));
					  uwb.rms_y = (double)(get_rms(uwb.y_buff_100ms,uwb.xy_buff_num,predict_y));
					  //�޳��쳣���� ,serios
					  if((fabs(predict_x-uwb.raw_x) < 1.0)&&(fabs(predict_y-uwb.raw_y) < 1.0))
					  {						    
						    //uwb����ƫ���Сʱ�����ݿ���׼ȷ��ֻ�������Ƚϴ�
							  if((fabs(predict_x-uwb.raw_x) > 0.2) || (fabs(predict_y-uwb.raw_y) > 0.2))
								{
//										if(fabs(uwb.pre_raw_x_var) > uwb.raw_xy_var_tol || fabs(uwb.pre_raw_y_var) > uwb.raw_xy_var_tol)
                    if(fabs(uwb.raw_x_var) > uwb.raw_xy_var_tol*10.0 || fabs(uwb.raw_y_var) > uwb.raw_xy_var_tol*10.0)
										{
												uwb.x_buff_100ms[uwb.xy_buff_num-1] = predict_x;
        		            uwb.y_buff_100ms[uwb.xy_buff_num-1] = predict_y;																								
										}
					    	}						
						}
            else
            {
						    //uwbֵƫ��ϴ�ʱ��������֡�����Сʱ�����ݿɿ�����Ϊuwbƫ��ϴ�ʱ��������uwb������׼
							  if(fabs(uwb.raw_x_var) > uwb.raw_xy_var_tol*5.0 || fabs(uwb.raw_y_var) > uwb.raw_xy_var_tol*5.0 
									|| fabs(uwb.pre_raw_x_var) > uwb.raw_xy_var_tol*5.0 || fabs(uwb.pre_raw_y_var) > uwb.raw_xy_var_tol*5.0)
							  {
								    uwb.x_buff_100ms[uwb.xy_buff_num-1] = predict_x;
        		        uwb.y_buff_100ms[uwb.xy_buff_num-1] = predict_y;
                } 									
						}	
            //��uwb���ݼ�Ȩ�˲�	
				    
					  uwb.now_x1 = uwb.now_x;
  					uwb.now_y1 = uwb.now_y;
					  
					  uwb.now_x = (uwb.x_buff_100ms[9]*10.0 + uwb.x_buff_100ms[8]*9.0 + uwb.x_buff_100ms[7]*8.0 + uwb.x_buff_100ms[6]*7.0 
				           + uwb.x_buff_100ms[5]*6.0 + uwb.x_buff_100ms[4]*5.0 + uwb.x_buff_100ms[3]*4.0 + uwb.x_buff_100ms[2]*3.0 
				            + uwb.x_buff_100ms[1]*2.0 + uwb.x_buff_100ms[0]*1.0)/55.0;
            uwb.now_y = (uwb.y_buff_100ms[9]*10.0 + uwb.y_buff_100ms[8]*9.0 + uwb.y_buff_100ms[7]*8.0 + uwb.y_buff_100ms[6]*7.0 
				           + uwb.y_buff_100ms[5]*6.0 + uwb.y_buff_100ms[4]*5.0 + uwb.y_buff_100ms[3]*4.0 + uwb.y_buff_100ms[2]*3.0 
				            + uwb.y_buff_100ms[1]*2.0 + uwb.y_buff_100ms[0]*1.0)/55.0;		
            
//						uwb.kalman_x.z = uwb.raw_x;
//            kalman(&uwb.kalman_x);
						
				}											
		}		
		

	  //uwb�����ȶ�����λ����Ϣ
    if(fabs(uwb.raw_x_var)<uwb.raw_xy_var_tol && fabs(uwb.raw_y_var)<uwb.raw_xy_var_tol 
			&& fabs(uwb.x_buff_100ms[0]) > 0 && fabs(uwb.y_buff_100ms[0]) > 0)//�����ж���Ϊ�˽������ʱ�������ݲ��õ����⡣
    {
    //��uwb��С��λ�ü�Ĳ�ֵ�����ڶ�С��λ�ù���Ĳ���,0.1m����һ�Σ�
		    uwb.delta_x = uwb.now_x - robot_motion.x;
		    uwb.delta_y = uwb.now_y - robot_motion.y;
		
		    for(i=1;i<uwb.deltaxy_buff_num;i++)
		    {
		        uwb.delta_x_buff[i-1] = uwb.delta_x_buff[i];
		        uwb.delta_y_buff[i-1] = uwb.delta_y_buff[i];
		    }
		    uwb.delta_x_buff[uwb.deltaxy_buff_num-1] = uwb.delta_x;
		    uwb.delta_y_buff[uwb.deltaxy_buff_num-1] = uwb.delta_y;
									
				if(fabs(uwb.delta_x_buff[0]) > 0 && fabs(uwb.delta_y_buff[0]) > 0)
				{	
		        uwb.delta_now_x = (uwb.delta_x_buff[4]*5.0 + uwb.delta_x_buff[3]*3.0 + uwb.delta_x_buff[2]*1.0 
				            + uwb.delta_x_buff[1]*0.8 + uwb.delta_x_buff[0]*0.2)/10.0;
            uwb.delta_now_y = (uwb.delta_y_buff[4]*5.0 + uwb.delta_y_buff[3]*3.0 + uwb.delta_y_buff[2]*1.0 
				            + uwb.delta_y_buff[1]*0.8 + uwb.delta_y_buff[0]*0.2)/10.0;									 					  
				}
//        uwb.kalman_x.q = 0.01;
//	      uwb.kalman_x.r = 10.0;	
//        uwb.kalman_x.p = 0.01;
//        uwb.kalman_x.z = uwb.delta_x;
//				kalman(&uwb.kalman_x);
//				uwb.kalman_y.q = 0.01;
//	      uwb.kalman_y.r = 10.0;	
//        uwb.kalman_y.p = 0.01;
//        uwb.kalman_y.z = uwb.delta_y;
//				kalman(&uwb.kalman_y);
		    //����λ��
				uwb.stable_dis += robot_motion.v*TIMER_PERIOD;
				if(fabs(uwb.stable_dis) > uwb.creat_buff_dis_tol)
				{	
				    uwb.stable_dis = 0;
					  //��uwb����ֵ�ȶ�������������֡����ɿ�ʱ��������
					  if(fabs(uwb.now_x1 - uwb.now_x) < 0.4 && fabs(uwb.now_y1 - uwb.now_y) < 0.4 
							&& fabs(uwb.pre_raw_x_var) < uwb.raw_xy_var_tol && fabs(uwb.pre_raw_y_var) < uwb.raw_xy_var_tol)
				    {
				  	    uwb.getxy_cnt ++;					  
                robot_motion.x = robot_motion.x + uwb.delta_now_x*0.1;	//0.5	
                robot_motion.y = robot_motion.y + uwb.delta_now_y*0.1;  //0.5
			      }
			  }				
 	  }
		//100mm���ڣ���С��ʵʱ�Ƕȣ������ȶ��󣬸��½Ƕ�
		uwb.creat_buff_dis += robot_motion.v*TIMER_PERIOD;
	  if(fabs(uwb.creat_buff_dis) > uwb.creat_buff_dis_tol)
	  {
		    uwb.creat_buff_dis = 0;
				//��������
		    for(i=1;i<uwb.xy_buff_num;i++)
		    {
		        uwb.x_buff_100mm[i-1] = uwb.x_buff_100mm[i];
		        uwb.y_buff_100mm[i-1] = uwb.y_buff_100mm[i];
		    }
        uwb.x_buff_100mm[uwb.xy_buff_num-1] = robot_motion.x;
		    uwb.y_buff_100mm[uwb.xy_buff_num-1] = robot_motion.y;
		    //����Ƕȵ��������Ϊ900mm������Ϊ100mm��һ�Ρ�
				if(fabs(uwb.y_buff_100mm[uwb.xy_buff_num-1])>0 && fabs(uwb.y_buff_100mm[0])>0 
				&& fabs(uwb.x_buff_100mm[uwb.xy_buff_num-1])>0 && fabs(uwb.x_buff_100mm[0])>0)
		  	{
				    uwb.now_heading = atan2(uwb.y_buff_100mm[uwb.xy_buff_num-1] - uwb.y_buff_100mm[0],
    		    uwb.x_buff_100mm[uwb.xy_buff_num-1] - uwb.x_buff_100mm[0]);
        }
				//�����ǶȻ��� 
				for(i=1;i<uwb.heading_buff_num;i++)
		    {
		        uwb.heading_buff[i-1] = uwb.heading_buff[i];
			  }  
	      uwb.heading_buff[uwb.heading_buff_num-1] = uwb.now_heading; 
        if(fabs(uwb.heading_buff[0]) > 0)
	      {
			      uwb.now_heading_var = get_var(uwb.heading_buff,uwb.heading_buff_num);
	      //updata current pos and heading;
            if(uwb.now_heading_var < uwb.heading_var_tol)
            {                   			      
					  		uwb.getheading_cnt ++;
								robot_motion.heading = robot_motion.heading + getanglediff(uwb.now_heading,robot_motion.heading)*0.05;     
            }		            	
	      }			
		}
}










































