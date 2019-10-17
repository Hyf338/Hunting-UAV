#pragma once

#include <stdbool.h>

typedef struct
{
	float mode_frequency;	//模式运行频率
	void (*mode_enter)();	//进入模式触发函数
	void (*mode_exit)();	//离开模式触发函数
	void (*mode_main_func)();	//模式主函数（运行频率执行）
}Mode;

void init_Modes();

bool change_Mode( unsigned char mode );
unsigned char get_current_Mode();

#define Is_Calibration_Mode(m) ( m>=10 && m<=19 )
#define Is_Flight_Mode(m) ( m>=30 && m<=39 )