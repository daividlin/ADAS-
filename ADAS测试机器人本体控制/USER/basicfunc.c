#include "basicfunc.h"
#include "delay.h"  
#include "string.h"
#include "math.h"
#include "ctype.h"
int signH(float v)
{
		if(v >35)
		{
			return 1;
		}
		else if(v < -35)
		{
			return -1;
		}
		else
		{
			return 0;
		}
		
}



u32 GetMedianNum1(u32 *ibArray, int iFilterLen)  
{  
    int i,j;//   
    u32 bTemp = 0;  
    u32 bArray[5];
		memcpy(&bArray[0],ibArray,sizeof(int)*iFilterLen);
    for (j = 0; j < iFilterLen - 1; j ++)  
    {  
        for (i = 0; i < iFilterLen - j - 1; i ++)  
        {  
            if (bArray[i] > bArray[i + 1])  
            {  
                // 
                bTemp = bArray[i];  
                bArray[i] = bArray[i + 1];  
                bArray[i + 1] = bTemp;  
            }  
        }  
    }  
      
    // 
    if ((iFilterLen & 1) > 0)  
    {  
        bTemp = bArray[(iFilterLen + 1) / 2-1];  
    }  
    else  
    {  
       // bTemp = (bArray[iFilterLen / 2] + bArray[iFilterLen / 2 + 1]) / 2;  
    }  
  
    return bTemp;  
}  		

float GetMedianNum(float *array,unsigned char length)
{
	unsigned char i,j;
	float tempData;
	for(j = 0;j < length - 1; j++)
	{
		for(i = 0;i < length - j - 1;i ++)
		{
			if(array[i] > array[i + 1])
			{
				tempData = array[i];
				array[i] = array[i + 1];
				array[i + 1] = tempData;
			}
		}
	}
	if((length & 0x01) > 0)
		tempData = array[((length + 1) / 2) - 1];
	else
		tempData = (array[(length / 2) - 1] + array[(length / 2 + 1 - 1)]) / 2;
	return tempData;
}


int get_weigtedMean(WEIGTEDMEAN *weigtedmean_interface)
{
		int re = 0;
	  WEIGTEDMEAN weigtedmean;
	  memcpy(&weigtedmean,weigtedmean_interface,sizeof(WEIGTEDMEAN));
	
		weigtedmean.dat[0] = weigtedmean.dat[1];
	  weigtedmean.dat[1] = weigtedmean.dat[2];	  
	  weigtedmean.dat[2] = weigtedmean.dat[3];	  
	  weigtedmean.dat[3] = weigtedmean.fDat;
	  weigtedmean.fDat = weigtedmean.dat[3]*0.4f + weigtedmean.dat[2]*0.3f + weigtedmean.dat[1]*0.2f + weigtedmean.dat[0]*0.1f;
	  	
	  memcpy(weigtedmean_interface,&weigtedmean,sizeof(WEIGTEDMEAN));
	  return re;
}

char signdata(float data)
{
    if(data<0)
			return 1;
		else
			return 0;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:	  double myStod(const char *st) 
*��������:	 ascii--->double
��������� 
���������
*******************************************************************************/
double myStod(const char *st)  
{  
    double s = 0.0;
	  double order = 1.0; 
	  size_t len = strlen(st);  
    size_t i = 0;  
    if (st[0] == '-')  i++;  
	  
    while (i < len && isdigit(st[i]))  
    {  
        s = s * 10 + (double)(st[i] - '0');  
        i++;  
    }  
      
    if (st[i] == '.') i++;  
    else return s;  
  
     
    while (i < len && isdigit(st[i]))  
    {  
        order = order / 10;  
        s = s + (double)(st[i] - '0') * order;  
        i++;  
    }  
  
    if (st[i] == '-') s = -1 * s;  
  
    return s;  
}  



/**************************ʵ�ֺ���********************************************
*����ԭ��:	   float fuzzy_pid(double *p,double pid[],double boundary[],double offset,int num)
*��������:	 ��ֶ�pid����
��������� ���飬Ԫ�ظ���
���������
*******************************************************************************/
int fuzzy_pid(double *p,double pid[],double boundary[],double offset,int num)
{
	
	  int re = 0,i = 0;
	  for(i=0;i<num;i++)
    {
				if( fabs(offset)< boundary[i]) 
				{
				    *p=pid[i];
					   break;
				}	 
        else
					*p = pid[i+1];
    }
		return re;
		
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   float get_var
*��������:	 ��ָ����С����ķ���
��������� ���飬Ԫ�ظ���
���������
*******************************************************************************/
float get_var(double a[],int num)
{
    int i = 0;double average = 0,sum = 0,max = 0,min = 0;
	  float var = 0.0;
	  max = a[0],min = a[0];
	  for(i=0;i<num;i++)
    {
				if(max < a[i]) 
					 max=a[i];
        if(min > a[i])
           min=a[i];
				sum = sum + a[i];
    }
		average = (sum - max - min)/((float)(num - 2)); 

    for (i=0;i<num;i++)
		{
		    var += (a[i] - average)*(a[i] - average)/((float)(num - 2));
		}
		var = (var -(max - average)*(max - average)/((float)(num - 2)) - (min - average)*(min - average)/((float)(num - 2)));

    return var; 
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:	   int kalman
*��������:	 �������˲�
���������   �������ṹ��
����������������ṹ��
*******************************************************************************/
int kalman(KALMAN *kalman_interface)
{
   
	  int re = 0;
	  KALMAN kalman;
	  memcpy(&kalman,kalman_interface,sizeof(KALMAN));
	  
	  kalman.p = kalman.p + kalman.q;
	  kalman.k = kalman.p / (kalman.p + kalman.r); 
    kalman.x = kalman.x + kalman.k * (kalman.z - kalman.x);
	  kalman.p = (1 - kalman.k)*kalman.p;
	
	  memcpy(kalman_interface,&kalman,sizeof(KALMAN));
	  return re;
}   

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   float get_rms
*��������:	 ��һ������ľ�����
���������   ָ����С�����飬����Ԫ�ظ���������
���������   
*******************************************************************************/
float get_rms(double a[],int num,double predict)
{
    int i = 0;
  	float rms = 0.0;
	 
    for (i=0;i<num;i++)
		{
		    rms += (a[i] - predict)*(a[i] - predict)/((float)num);
		}

    return rms; 
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   double sign
*��������:	 �����
���������  ��ֵ
���������  
*******************************************************************************/
double sign(double indata)
{
	if (indata>0)
	return 1;
	else if (indata<0)
	return -1;
	else
	return 0;
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:	 double wrapTo2PiDe
*��������:	 �Ƕ�Բ����2pi��Χ
���������   �Ƕ�
���������  
*******************************************************************************/
double wrapTo2PiDe(double inrad)
{
 // positiveInput = (inrad > 0);
  //double ct = 2.0*PI;
  inrad = fmod(inrad, 2.0*PI);
  if (inrad < 0)
     inrad = 2.0*PI + inrad;
  return inrad;
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:	 double wrapToPiDe
*��������:	 �Ƕ�Բ����pi��Χ
���������   �Ƕ�
���������   
*******************************************************************************/
double wrapToPiDe(double inrad)
{
	
	double r;
	r = wrapTo2PiDe(inrad);
	if (r > PI)
	   r = r - 2*PI;
	return r;
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:	 double getanglediff
*��������:	 �Ƕ�����ΧԼ����2pi��Χ�ڡ�
���������   �Ƕ�
��������� 
*******************************************************************************/
double getanglediff(double target, double now)
{

  double dif;
  target = wrapTo2PiDe(target);
  now = wrapTo2PiDe(now);
  dif = target - now;
  if (dif > PI)
    dif = -2*PI + dif;
  else if (dif<-PI)
    dif = 2*PI + dif;
  return dif;
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:	 double wrapToHalfPiDe
*��������:	 �Ƕ�Բ����pi/2��Χ
���������   �Ƕ�
���������   
*******************************************************************************/  
double wrapToHalfPiDe(double inrad)
{
 
    inrad = wrapToPiDe(inrad);
    if(inrad > -PI*0.25 && inrad < PI*0.25)
	   inrad = inrad;
    else if(inrad >= PI*0.25 && inrad < PI*0.75)
	   inrad =  inrad - PI*0.5;
    else if(inrad <= PI && inrad >= PI*0.75)
	   inrad =  inrad - PI;
    else  if(inrad <= -PI*0.25 && inrad > -PI*0.75)
	   inrad =  inrad + PI*0.5;
	else  if(inrad <= -PI*0.75 && inrad >= -PI)
	   inrad =  inrad + PI;

  return inrad; 
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:	 int GetNominalHeading
*��������:	 �Ƕ�Բ��Ϊ����ֵ
���������   �Ƕ�
���������   �Ƕ�
*******************************************************************************/  
int GetNominalHeading(double in, double* out)
{
	int re = 1;
	if (fabs(getanglediff(in,0))<=PI/4.0)
		*out = 0;
	else if (fabs(getanglediff(in,PI/2.0))<=PI/4.0)
		*out = PI/2.0;
	else if (fabs(getanglediff(in,-PI/2.0))<=PI/4.0)
		*out = -PI/2.0;
	else if (fabs(getanglediff(in,PI))<=PI/4.0)
		*out = PI;
    else
        re = 0;
	return re;
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:	 void clear_buff
*��������:	 �������
���������   �Ƕ�
���������   
*******************************************************************************/ 
void clear_buff(double a[],int num)
{
    int i;  
	  for(i=0;i<num;i++)
    {
			 a[i] = 0;	
    }
}












































