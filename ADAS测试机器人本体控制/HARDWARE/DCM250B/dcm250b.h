#ifndef __DCM250B_H
#define __DCM250B_H

#define SET_MAG_DEC 0x06
#define READ_MAG_DEC 0x07
#define START_CALIBRATION 0x08
#define SAVE_CALIBRATION 0x0a


typedef struct dcm250b_struct
{	
    float pitch,roll,heading,mag_dec;
	  char type,flag_wait_mag;
	
}ROBOT_DCM250B;


typedef struct dcm250b_cmd_struct
{
    char head;
	  char length;
	  char addr;
	  char type;
	  char data[4];
	  char checksum;
}ROBOT_DCM250B_CMD;

extern ROBOT_DCM250B_CMD dcm250b_cmd;
extern ROBOT_DCM250B dcm250b;

char sensor_detection(double gyro_heading,double compass_heading);
char dcm250b_command_process(char *buff);
char* Real_Process_Dcm(char* buffer, unsigned char num );
void Creat_dcm_Index(char* buffer);
char tx_dcm250b(char len,char type,float data);
char rx_dcm250b(void);
#endif


