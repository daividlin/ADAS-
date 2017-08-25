#ifndef __BASICFUNC_H
#define __BASICFUNC_H 
#include "sys.h"
#include "stdio.h"


#define PI 3.1415926535898

typedef struct kalman_struct
{
    double x,z,r,p,q,k;
}KALMAN;

typedef struct weigtedmean_struct
{
    float dat[4];
		float fDat;
}WEIGTEDMEAN;

int signH(float v);
u32 GetMedianNum1(u32 * bArray, int iFilterLen); 
float GetMedianNum(float *array,unsigned char length);
int get_weigtedMean(WEIGTEDMEAN *weigtedmean_interface);
double sign(double indata);
double wrapTo2PiDe(double inrad);
double wrapToPiDe(double inrad);

double getanglediff(double target, double now);

   
double wrapToHalfPiDe(double inrad);

int GetNominalHeading(double in, double* out);

void clear_buff(double a[],int num);

int kalman(KALMAN *kalman_interface);

float get_rms(double a[],int num,double predict);
float get_var(double a[],int num);

int fuzzy_pid(double *p,double pid[],double boundary[],double offset,int num);

double myStod(const char *st);  

char signdata(float data);


















#endif
