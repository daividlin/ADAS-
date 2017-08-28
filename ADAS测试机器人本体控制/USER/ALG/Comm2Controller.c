#include "Comm2Controller.h"

GPS_REAL_BUFTYPE HUAWEI_Cmd_buf;
GPS_REAL_BUFTYPE HUAWEI_Cmd_buf_uart;
HUAWEI_CMDTYPE HUAWEI_cmd;
HUAWEI_CMDTYPE HUAWEI_status;


void is3288CmdLost(int *heart_count, int* flag)
{
	if (*heart_count>= 5)
	{
		*heart_count = 5;
		*flag = 0;
	}
	else
	{
		(*heart_count)++;
	}
}

void checkRK3288Msg(void)
{
	CTRLCMD_STRUCT_TYPE ctrl_cmd;
	if (PC2STUsart.dataarrive == 1)
	{
		PC2STUsart.dataarrive = 0;
		cmd.dataarrive = 1;
		memcpy(&cmd.type, &PC2STUsart.rdata[4], 1);
		memcpy(&ctrl_cmd.targetX, &PC2STUsart.rdata[5], 4);
		memcpy(&ctrl_cmd.targetY, &PC2STUsart.rdata[9], 4);
		memcpy(&ctrl_cmd.targetAngle, &PC2STUsart.rdata[13], 4);
		//			  cmd.type = ctrlcmd.type;
		cmd.target_x = ((double)ctrl_cmd.targetX)*0.001;
		cmd.target_y = ((double)ctrl_cmd.targetY)*0.001;
		cmd.target_heading = ((double)ctrl_cmd.targetAngle / 1000.0) / 180.0*PI;
	}
}

