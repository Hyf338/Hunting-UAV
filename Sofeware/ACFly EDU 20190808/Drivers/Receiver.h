#pragma once

#include "Basic.h"
#include <stdbool.h>

//接收机定义
typedef struct
{
	bool present;	//是否存在
	bool connected;	//是否已连接
	bool available;	//是否可用
	TIME last_update_time;	//上次更新时间
	float update_time;	//更新时间间隔
	
	float raw_data[16];	//原始数据
	float data[8];	//校准后的数据
}Receiver;

#define Receivers_Count 3
typedef enum
{
	RC_Type_Sbus = 0 ,
	RC_Type_PPM = 1 ,
}RC_Type;

//获取指定的接收机
const Receiver* get_Receiver( RC_Type rc );
//获取当前使用的接收机
const Receiver* get_current_Receiver();
//获取当前使用的接收机
RC_Type get_current_Receiver_Type();