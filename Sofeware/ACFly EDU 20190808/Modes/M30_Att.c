#include "Modes.h"
#include "Basic.h"
#include "stdlib.h"
#include "M30_Att.h"

#include "AC_Math.h"
#include "Receiver.h"
#include "InteractiveInterface.h"
#include "ControlSystem.h"

static void M30_Att_MainFunc();
static void M30_Att_enter();
static void M30_Att_exit();
const Mode M30_Att = 
{
	50 , //mode frequency
	M30_Att_enter , //enter
	M30_Att_exit ,	//exit
	M30_Att_MainFunc ,	//mode main func
};

typedef struct
{
	//退出模式计数器
	uint16_t exit_mode_counter;
}MODE_INF;
static MODE_INF* Mode_Inf;

static void M30_Att_enter()
{
	Led_setStatus( LED_status_running1 );
	
	//初始化模式变量
	Mode_Inf = malloc( sizeof( MODE_INF ) );
	Attitude_Control_Enable();
}

static void M30_Att_exit()
{
	Attitude_Control_Disable();
	
	free( Mode_Inf );
}

static void M30_Att_MainFunc()
{
	const Receiver* rc = get_current_Receiver();
		
	if( rc->available == false )
	{
		//接收机不可用
		Attitude_Control_set_Throttle( 0 );
		return;
	}
	
	//获取遥控摇杆位置
	float throttle_stick = rc->data[0];
	float yaw_stick = rc->data[1];
	float pitch_stick = rc->data[2];
	float roll_stick = rc->data[3];
	
	/*判断退出模式*/
		if( throttle_stick < 5 && yaw_stick < 5 && pitch_stick < 5 && roll_stick > 95 )
		{
			if( ++Mode_Inf->exit_mode_counter >= 50 )
			{
				change_Mode( 1 );
				return;
			}
		}
		else
			Mode_Inf->exit_mode_counter = 0;
	/*判断退出模式*/
	
	//油门杆直接控油门
	Attitude_Control_set_Throttle( throttle_stick );
	//俯仰横滚杆控俯仰横滚
	Attitude_Control_set_Target_RollPitch( ( roll_stick - 50.0f )*0.015f , ( pitch_stick - 50.0f )*0.015f );
	//偏航杆在中间锁偏航
	//不在中间控制偏航速度
	if( in_symmetry_range_offset_float( yaw_stick , 5 , 50 ) )
		Attitude_Control_set_YawLock();
	else
		Attitude_Control_set_Target_YawRate( ( 50.0f - yaw_stick )*0.05f );
}