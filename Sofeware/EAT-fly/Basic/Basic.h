#pragma once

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define SYSTIMERCLK 80e+6

void init_Basic();

#define  INT_PRIO_0  0x00
#define  INT_PRIO_1  0x20
#define  INT_PRIO_2  0x40
#define  INT_PRIO_3  0x60
#define  INT_PRIO_4  0x80
#define  INT_PRIO_5  0xA0
#define  INT_PRIO_6  0xD0
#define  INT_PRIO_7  0xE0

#pragma region TIME

	extern const float TIM2sec_scale;

	//时间均为秒s单位
	typedef struct 
	{
		uint64_t t;
	}TIME;

	//获取当前时间
	TIME get_TIME_now();
	
	//计算距离last_time的时间
	float get_pass_time( TIME last_time );
	//计算距离time_a和time_b相间隔的时间
	float get_time_difference( TIME time_a , TIME time_b );
	//计算系统运行时间
	float get_System_Run_Time();
	
	//计算距离last_time的时间
	//并将last_time设置为当前时间
	//用于精确计时
	float get_pass_time_st( TIME* last_time );
	
	//将时间设置为不可用
	//主要用于判断
	void Time_set_inValid( TIME* t );
	bool Time_isValid( TIME t );
	
	//延时
	void delay( float t );

#pragma endpragma