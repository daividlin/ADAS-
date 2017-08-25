#ifndef __STARGAZER_H
#define __STARGAZER_H

typedef struct point_struct
{
	float smode,sid,sa,sx,sy,sz;
}POINT;

extern POINT gazer_pos;   //stargazer current pos
extern char gazer_str[100], gazer_str_len, gazer_str_ok;   //recved stargazer data


/*
函数功能：获取stargazer相关参数信息
返回值：1---成功，0---失败
*/
int stargazer_state(void);

/*
函数功能：stargazer初始化
参数相关配置，num：landmark个数，refid：坐标原点landmark的id号
返回值：1---成功，0---失败
*/
int stargazer_init(int num, int refid);

/*
函数功能：使stargazer进入建立地图模式
返回值：1---成功，0---失败
*/
int map_building(void);

/*
函数功能：返回当前坐标点
返回值：1---成功，0---失败
备注：使用全局变量gazer_pos保存坐标点
*/
int stargazer_pos(void);


#endif
