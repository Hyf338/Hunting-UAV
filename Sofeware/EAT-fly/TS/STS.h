#pragma once

#include "Basic.h"

//Simple Task Scheduling
//简易任务调度器
//朱文杰 20181224

//最大任务个数
#define STS_MAX_TASK_COUNT 20

typedef enum
{
	STS_Task_Trigger_Mode_RoughTime ,	//粗略时间触发任务
	STS_Task_Trigger_Mode_PreciseTime ,	//精确时间触发任务
	
	STS_Task_Trigger_Mode_Custom ,	//自定义函数触发任务
}STS_Task_Trigger_Mode;

//添加任务函数，返回任务ID
//如任务模式为时间触发，t为触发时间间隔
//如任务模式为自定义函数触发，trigger_func为自定义函数
//mainfunc为任务主函数
unsigned int STS_Add_Task( STS_Task_Trigger_Mode mode , float t , bool (*trigger_func)( unsigned int Task_ID ) , void (*mainfunc)( unsigned int Task_ID ) );

//更改任务触发模式
bool STS_Change_Task_Mode( unsigned int Task_ID , STS_Task_Trigger_Mode mode , float t , bool (*trigger_func)( unsigned int Task_ID ) );

//更改任务主函数
bool STS_Change_Task_MainFunc( unsigned int Task_ID , void (*mainfunc)( unsigned int Task_ID ) );

//删除任务
bool STS_Remove_Task( unsigned int Task_ID );

//遮蔽任务
bool STS_Shield_Task( unsigned int Task_ID );

//复现遮蔽的任务
bool STS_Manifest_Task( unsigned int Task_ID );

void STS_Run();