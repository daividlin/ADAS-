#ifndef __BASICFUNC_H
#define __BASICFUNC_H 
#include "sys.h"
#include "stdio.h"
#include "typeinclude.h"


int signH(float v);
u32 GetMedianNum1(u32 * bArray, int iFilterLen);
float GetMedianNum(float *array, unsigned char length);
int get_weigtedMean(WEIGTEDMEAN *weigtedmean_interface);
double getSign(double indata);
double angle2FullRadian(double inrad);
double angle2HalfRadian(double inrad);
double getAngleDiff(double target, double now);
double wrapToHalfPiDe(double inrad);
int GetNominalHeading(double in, double* out);
void clearBufByLen(double a[], int num);
int kalman(KALMAN *kalman_interface);
float getMeanVariance(double a[], int num, double predict);
float get_var(double a[], int num);
int fuzzy_pid(double *p, double pid[], double boundary[], double offset, int num);
double myStod(const char *st);
char signdata(float data);

#endif
