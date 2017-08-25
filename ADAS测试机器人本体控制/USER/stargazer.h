#ifndef __STARGAZER_H
#define __STARGAZER_H

typedef struct point_struct
{
	float smode,sid,sa,sx,sy,sz;
}POINT;

extern POINT gazer_pos;   //stargazer current pos
extern char gazer_str[100], gazer_str_len, gazer_str_ok;   //recved stargazer data


/*
�������ܣ���ȡstargazer��ز�����Ϣ
����ֵ��1---�ɹ���0---ʧ��
*/
int stargazer_state(void);

/*
�������ܣ�stargazer��ʼ��
����������ã�num��landmark������refid������ԭ��landmark��id��
����ֵ��1---�ɹ���0---ʧ��
*/
int stargazer_init(int num, int refid);

/*
�������ܣ�ʹstargazer���뽨����ͼģʽ
����ֵ��1---�ɹ���0---ʧ��
*/
int map_building(void);

/*
�������ܣ����ص�ǰ�����
����ֵ��1---�ɹ���0---ʧ��
��ע��ʹ��ȫ�ֱ���gazer_pos���������
*/
int stargazer_pos(void);


#endif
