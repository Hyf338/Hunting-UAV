#include "STS.h"

typedef struct
{
	STS_Task_Trigger_Mode mode;	//任务触发模式
	
	TIME last_trigger_time;	//上次触发任务时间
	float t;
	bool (*trigger_func)(  unsigned int Task_ID );	//自定义函数触发 函数指针
	
	bool shielded;	//任务是否被屏蔽
	
	void (*mainfunc)( unsigned int Task_ID );	//任务执行函数指针
}STS_Task;

static STS_Task tasks[ STS_MAX_TASK_COUNT ] = {0};
static int16_t STS_last_task = -1;

//添加任务函数，返回任务ID
//如任务模式为时间触发，t为触发时间间隔
//如任务模式为自定义函数触发，trigger_func为自定义函数
//mainfunc为任务主函数
unsigned int STS_Add_Task( STS_Task_Trigger_Mode mode , float t , bool (*trigger_func)( unsigned int Task_ID ) , void (*mainfunc)( unsigned int Task_ID ) )
{
	int16_t first_available_id = -1;
	for( uint16_t i = 0 ; i < STS_MAX_TASK_COUNT ; ++i )
	{
		if( tasks[ i ].mainfunc == 0 )
		{
			first_available_id = i;
			break;
		}
	}
	//任务满，不能再加，返回0
	if( first_available_id < 0 )
		return 0;
	
	if( mode == STS_Task_Trigger_Mode_Custom && trigger_func == 0 )
		return 0;
	else if( t < 0 )
		return 0;
	
	if( STS_last_task < first_available_id )
		STS_last_task = first_available_id;
	
	tasks[ first_available_id ].mode = mode;
	tasks[ first_available_id ].t = t;
	tasks[ first_available_id ].trigger_func = trigger_func;
	tasks[ first_available_id ].mainfunc = mainfunc;
	tasks[ first_available_id ].shielded = false;
	return first_available_id;
}

//更改任务触发模式
bool STS_Change_Task_Mode( unsigned int Task_ID , STS_Task_Trigger_Mode mode , float t , bool (*trigger_func)( unsigned int Task_ID ) )
{
	if( Task_ID >= STS_MAX_TASK_COUNT )
		return false;
	if( tasks[ Task_ID ].mainfunc == 0 )
		return false;
	
	tasks[ Task_ID ].mode = mode;
	tasks[ Task_ID ].t = t;
	tasks[ Task_ID ].trigger_func = trigger_func;
	tasks[ Task_ID ].shielded = false;
	return true;
}

//更改任务主函数
bool STS_Change_Task_MainFunc( unsigned int Task_ID , void (*mainfunc)( unsigned int Task_ID ) )
{
	if( Task_ID >= STS_MAX_TASK_COUNT )
		return false;
	if( tasks[ Task_ID ].mainfunc == 0 )
		return false;
	
	tasks[ Task_ID ].mainfunc = mainfunc;
	return true;
}

//删除任务
bool STS_Remove_Task( unsigned int Task_ID )
{
	if( Task_ID >= STS_MAX_TASK_COUNT )
		return false;
	if( tasks[ Task_ID ].mainfunc == 0 )
		return false;
	
	if( STS_last_task == Task_ID )
	{
		int16_t last_task = -1;
		for( uint16_t i = 0 ; i < STS_MAX_TASK_COUNT ; ++i )
		{
			if( tasks[ i ].mainfunc == 0 )
			{
				last_task = i;
			}
		}
		STS_last_task = last_task;
	}
	
	tasks[ Task_ID ].mainfunc = 0;
	return true;	
}

//遮蔽任务
bool STS_Shield_Task( unsigned int Task_ID )
{
	if( Task_ID >= STS_MAX_TASK_COUNT )
		return false;
	if( tasks[ Task_ID ].mainfunc == 0 )
		return false;
	tasks[ Task_ID ].shielded = true;
	return true;	
}

//复现遮蔽的任务
bool STS_Manifest_Task( unsigned int Task_ID )
{
	if( Task_ID >= STS_MAX_TASK_COUNT )
		return false;
	if( tasks[ Task_ID ].mainfunc == 0 )
		return false;
	tasks[ Task_ID ].shielded = false;
	return true;	
}

void STS_Run()
{
	while(1)
	{
		for( uint16_t i = 0 ; i <= STS_last_task ; ++i )
		{
			if( (tasks[ i ].mainfunc != 0) && (tasks[ i ].shielded == false) )
			{
				switch( tasks[i].mode )
				{
					//粗略时间触发
					case STS_Task_Trigger_Mode_RoughTime:
					{
						float pass_time = get_pass_time( tasks[i].last_trigger_time );
						if( pass_time >= tasks[i].t )
						{
							tasks[i].mainfunc( i );
							tasks[i].last_trigger_time = get_TIME_now();
						}
						break;
					}
					
					//精确时间触发
					case STS_Task_Trigger_Mode_PreciseTime:
					{
						float pass_time = get_pass_time_st( &tasks[i].last_trigger_time );
						if( pass_time >= tasks[i].t )
							tasks[i].mainfunc( i );
						break;
					}
					
					//自定义函数触发
					case STS_Task_Trigger_Mode_Custom:
					{
						if( tasks[i].trigger_func( i ) )
						{
							tasks[i].mainfunc( i );
							tasks[i].last_trigger_time = get_TIME_now();
						}
						break;
					}
				}
			}
		}
	}
}