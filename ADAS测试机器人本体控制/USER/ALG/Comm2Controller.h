#ifndef _COMM_2_CONTRLLLER_H
#define _COMM_2_CONTRLLLER_H

#include "robot_action.h"
#include "usart.h"
#include <string.h>

void is3288CmdLost(int *heart_count, int* flag);
void checkRK3288Msg(void);


#endif // !_COMM_2_CONTRLLLER_H
