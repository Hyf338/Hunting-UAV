#include "Modes.h"
#include "Basic.h"
#include "stdlib.h"
#include <stdio.h>
#include "M35_Auto1.h"

#include "AC_Math.h"
#include "Receiver.h"
#include "InteractiveInterface.h"
#include "ControlSystem.h"
#include "MeasurementSystem.h"

static void M35_Auto1_MainFunc();
static void M35_Auto1_enter();
static void M35_Auto1_exit();
const Mode M35_Auto1 = 
{
	50 , //mode frequency
	M35_Auto1_enter , //enter
	M35_Auto1_exit ,	//exit
	M35_Auto1_MainFunc ,	//mode main func
};

typedef struct
{
	//退出模式计数器
	uint16_t exit_mode_counter;
	
	//自动飞行状态机
	uint8_t auto_step1;	//0-记录按钮位置
											//1-等待按钮按下起飞 
											//2-等待起飞完成 
											//3-等待2秒
											//4-降落
											//5-等待降落完成
	uint16_t auto_counter;
	float last_button_value;
	
	float last_height;
}MODE_INF;
static MODE_INF* Mode_Inf;

static void M35_Auto1_enter()
{
	Led_setStatus( LED_status_running1 );
	
	//初始化模式变量
	Mode_Inf = malloc( sizeof( MODE_INF ) );
	Mode_Inf->exit_mode_counter = 0;
	Mode_Inf->auto_step1 = Mode_Inf->auto_counter = 0;
	Altitude_Control_Enable();
}

static void M35_Auto1_exit()
{
	Altitude_Control_Disable();
	Attitude_Control_Disable();
	
	free( Mode_Inf );
}


/******我们的国一在这里实现******/
static void M35_Auto1_MainFunc()
{
	/*外部数据拉取*/
	extern uint8_t Uart0_mode_contrl;//外扩版模式数据发送
	extern uint8_t SDI_mode_contrl;//openmv工作模式修改
	
	extern uint16_t uart0_msg_pack[31];//地面站调参数据接收
	extern uint8_t uart0_num;//调参的序号
	
	extern uint8_t SDI_area_central[11];//openmv五个区域中心点坐标
	extern uint8_t SDI_blob_central[3];//openmv色块中心点坐标
	
	extern uint16_t UWB_data;//UWB距离
	
	extern float Primitive_Height;//激光测距原始高度》》》无人机对地高度
	/*外部数据拉取*/
	
	/*参数区*/
	static float SDI_fly_1 = 0.0f;//
	static float SDI_fly_2 = 0.0f;//
	static float SDI_fly_3 = 1.0f;//
	static float SDI_fly_4 = 0.0f;//
	static float SDI_fly_5 = 0.0f;//
	static float SDI_fly_6 = 0.0f;//
	static float SDI_fly_7 = 0.0f;//
	static float SDI_fly_8 = 0.0f;//
	static float SDI_fly_9 = 0.0f;//
	static float SDI_fly_10 = 0.0f;//
	
	
	static float M35_init_yaw = 0.0f;//起飞偏航角
	static float M35_last_yaw = 0.0f;//上一次偏航角
	static float M35_now_yaw = 0.0f;//现在的偏航角
	
	
	char oled_srr[5];//OLED显示转换
	/*参数区*/
	
	/*远程参数处理*/
	if(uart0_msg_pack[0] == 1)//巡线模式参数更新
	{
		uart0_msg_pack[0] = 0;//赋值允许清零
		switch(uart0_num)
		{
			case 1:
				SDI_fly_1 = uart0_msg_pack[1]/100.0f;
			break;
			case 2:
				SDI_fly_2 = uart0_msg_pack[2]/100.0f;
			break;
			case 3:
				SDI_fly_3 = uart0_msg_pack[3]/100.0f;
			break;
			case 4:
				SDI_fly_4 = uart0_msg_pack[4]/100.0f;
			break;
			case 5:
				SDI_fly_5 = uart0_msg_pack[5]/100.0f;
			break;
			case 6:
				SDI_fly_6 = uart0_msg_pack[6]/100.0f;
			break;
			case 7:
				SDI_fly_7 = uart0_msg_pack[7]/100.0f; 
			break;
			case 8:
				SDI_fly_8 = uart0_msg_pack[8]/100.0f;
			break;
			case 9:
				SDI_fly_9 = uart0_msg_pack[9]/100.0f;
			break;
			case 10:
				SDI_fly_10 = uart0_msg_pack[10]/100.0f;
			break;
			default:;
		}
		
		OLED_Clear();
		OLED_Draw_Str8x6( "M_1:" , 2 , 0 );
		sprintf( oled_srr , "%5.2f" , SDI_fly_1 );
		OLED_Draw_Str8x6( oled_srr , 2 , 30 );
	
		OLED_Draw_Str8x6( "M_2:" , 3 , 0 );
		sprintf( oled_srr , "%5.2f" , SDI_fly_2 );
		OLED_Draw_Str8x6( oled_srr , 3 , 30 );
	
		OLED_Draw_Str8x6( "M_3:" , 4 , 0 );
		sprintf( oled_srr , "%5.2f" , SDI_fly_3 );
		OLED_Draw_Str8x6( oled_srr , 4 , 30 );
	
		OLED_Draw_Str8x6( "M_4:" , 5 , 0 );
		sprintf( oled_srr , "%5.2f" , SDI_fly_4 );
		OLED_Draw_Str8x6( oled_srr , 5 , 30 );
	
		OLED_Draw_Str8x6( "M_5:" , 6 , 0 );
		sprintf( oled_srr , "%5.2f" , SDI_fly_5 );
		OLED_Draw_Str8x6( oled_srr , 6 , 30 );
	
		OLED_Draw_Str8x6( "M_6:" , 7 , 0 );
		sprintf( oled_srr , "%5.2f" , SDI_fly_6 );
		OLED_Draw_Str8x6( oled_srr , 7 , 30 );
	
		OLED_Draw_Str8x6( "M_7:" , 2 , 64 );
		sprintf( oled_srr , "%5.2f" , SDI_fly_7 );
		OLED_Draw_Str8x6( oled_srr , 2 , 94 );
	
		OLED_Draw_Str8x6( "M_8:" , 3 , 64 );
		sprintf( oled_srr , "%5.2f" , SDI_fly_8 );
		OLED_Draw_Str8x6( oled_srr , 3 , 94 );
	
		OLED_Draw_Str8x6( "M_9:" , 4 , 64 );
		sprintf( oled_srr , "%5.2f" , SDI_fly_9 );
		OLED_Draw_Str8x6( oled_srr , 4 , 94 );
		
		OLED_Draw_Str8x6( "M_10:" , 5 , 64 );
		sprintf( oled_srr , "%5.2f" , SDI_fly_10 );
		OLED_Draw_Str8x6( oled_srr , 5 , 94 );
	}
	/*远程参数处理*/
	
	/*test区*/
	//显示当前运行状态
	static uint8_t SDK_auto_step1 = 0;
	if(Mode_Inf->auto_step1 != 0)SDK_auto_step1 = Mode_Inf->auto_step1;
	OLED_Draw_Str8x6( "c" , 0 , 90 );
	sprintf( oled_srr , "%3d" , SDK_auto_step1);
	OLED_Draw_Str8x6( oled_srr , 0 , 100 );
	
	//获取当前机体姿态信息
	Quaternion _M35Auto = get_attitude();
	
	
	OLED_Update();
	/*test区*/
	
	
	const Receiver* rc = get_current_Receiver();
		
	if( rc->available == false )
	{
		//接收机不可用
		//降落
		Position_Control_set_XYLock();
		Position_Control_set_TargetVelocityZ( -50 );
		return;
	}
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
		
	//判断摇杆是否在中间
	bool sticks_in_neutral = 
		in_symmetry_range_offset_float( throttle_stick , 5 , 50 ) && \
		in_symmetry_range_offset_float( yaw_stick , 5 , 50 ) && \
		in_symmetry_range_offset_float( pitch_stick , 5 , 50 ) && \
		in_symmetry_range_offset_float( roll_stick , 5 , 50 );
	
	
//	
//	if( sticks_in_neutral && get_Position_Measurement_System_Status() == Measurement_System_Status_Ready )
	if( sticks_in_neutral && get_Altitude_Measurement_System_Status() == Measurement_System_Status_Ready )
	{
		//摇杆在中间
		//执行自动飞行		
		//只有在位置有效时才执行自动飞行
		
		//打开水平位置控制
		Position_Control_Disable();//【修改】关闭水平控制
		switch( Mode_Inf->auto_step1 )
		{
			case 0:
			{
				Uart0_mode_contrl = 1;//关闭激光
				M35_init_yaw = Quaternion_getYaw(_M35Auto);//记录起飞前的地理坐标系偏航角
				Mode_Inf->last_button_value = rc->data[5];
				++Mode_Inf->auto_step1;
				Mode_Inf->auto_counter = 0;
			}break;
			
			case 1:
			{//等待按钮按下起飞
				if( get_is_inFlight() == false && fabsf( rc->data[5] - Mode_Inf->last_button_value ) > 15 )
				{
					Mode_Inf->last_button_value = rc->data[5];//记录按钮初始状态
					Position_Control_Takeoff_HeightRelative( 90.0f );
					++Mode_Inf->auto_step1;
					Mode_Inf->auto_counter = 0;
				}
			}break;
				
			case 2:
			{//等待起飞完成
				if( get_Altitude_ControlMode() == Position_ControlMode_Position )
				{
					Uart0_mode_contrl = 2;//开启激光
					
					Mode_Inf->auto_step1 = 3;//高度矫正
					Mode_Inf->auto_counter = 0;
				}
			}break;
			
			case 3:
			{
				Position_Control_set_TargetVelocityZ(90-Primitive_Height);//控制高度，维持在100cm
				
				if(fabs(90-Primitive_Height) < 5)//高度矫正完成
				{
					Position_Control_set_TargetVelocityZ(0);
					Position_Control_set_ZLock();//锁高度
					Mode_Inf->auto_step1 = 10;//进入工作状态
				}
				
				if(fabsf( rc->data[5] - Mode_Inf->last_button_value ) > 15 )
				{
				 	Mode_Inf->last_button_value = rc->data[5];//记录当前按钮状态
					Mode_Inf->auto_step1 = 200;//下降按键按下，返回状态 0
					Mode_Inf->auto_counter = 0;
				}
			}break;
			
			
			
			
			
	/*******
			case 10:
			{
				Position_Control_set_TargetPositionXYRelativeBodyHeading(130,0);
				++Mode_Inf->auto_step1;
				if(fabsf( rc->data[5] - Mode_Inf->last_button_value ) > 15 )
				{
				 	Mode_Inf->last_button_value = rc->data[5];//记录当前按钮状态
					Mode_Inf->auto_step1 = 200;//下降按键按下，返回状态 0
					Mode_Inf->auto_counter = 0;
				}
			}break;
			
			case 11:
			{
				Attitude_Control_set_Target_YawRelative(degree2rad(90));
				++Mode_Inf->auto_step1;
				if(fabsf( rc->data[5] - Mode_Inf->last_button_value ) > 15 )
				{
				 	Mode_Inf->last_button_value = rc->data[5];//记录当前按钮状态
					Mode_Inf->auto_step1 = 200;//下降按键按下，返回状态 0
					Mode_Inf->auto_counter = 0;
				}
			}
			
			case 12:
			{
				if( get_Position_ControlMode() == Position_ControlMode_Position )
				{
					++Mode_Inf->auto_step1;
					Mode_Inf->auto_counter = 0;
				}
				
				if(fabsf( rc->data[5] - Mode_Inf->last_button_value ) > 15 )
				{
				 	Mode_Inf->last_button_value = rc->data[5];//记录当前按钮状态
					Mode_Inf->auto_step1 = 200;//下降按键按下，返回状态 0
					Mode_Inf->auto_counter = 0;
				}
			}break;
			
********************************/
			
			case 200:
			{//降落
				//Position_Control_set_XYLock();【修改】关闭水平锁定
				Attitude_Control_set_YawLock(); 
				++Mode_Inf->auto_step1;
			}break;		
				
			case 201:
			{//等待位置锁定完成
				if( get_Altitude_ControlMode() == Position_ControlMode_Position )
				{
					Position_Control_set_TargetVelocityZ( -40 );
					++Mode_Inf->auto_step1;
					Mode_Inf->auto_counter = 0;
				}
			}break;
			
			/******新增下降状态，判断高度使其平缓降落************************************/			
			case 202:
			{
				//Position_Control_set_XYLock();【修改】关闭水平锁定
				Attitude_Control_set_YawLock();
				Position_Control_set_TargetVelocityZ( -50 );//疑问：是否需要添加此句
				if(Primitive_Height<50)
				{
					Position_Control_set_TargetVelocityZ( -10 );
					++Mode_Inf->auto_step1;
					Mode_Inf->auto_counter = 0;
				}
			}break;
			
			/******************************************/
			case 203:
			{
				//等待降落完成
				if( get_is_inFlight() == false )
				{
					Mode_Inf->auto_step1 = 0;
					Mode_Inf->auto_counter = 0;
				}
			}break;
			default:break;
		}
	}
	else
	{
		ManualControl:
		//摇杆不在中间
		//手动控制
		Mode_Inf->auto_step1 = Mode_Inf->auto_counter = 0;
		
		
		//关闭水平位置控制
		Position_Control_Disable();
		
		//高度控制输入
		if( in_symmetry_range_offset_float( throttle_stick , 5 , 50 ) )
			Position_Control_set_ZLock();
		else
		{
			float throttle_stick_test=throttle_stick - 50.0f;
			if(throttle_stick_test>0) throttle_stick_test=0;
			
			Position_Control_set_TargetVelocityZ( ( throttle_stick_test )* 6 );
		}
		//偏航控制输入
		if( in_symmetry_range_offset_float( yaw_stick , 5 , 50 ) )
			Attitude_Control_set_YawLock();
		else
			Attitude_Control_set_Target_YawRate( ( 50.0f - yaw_stick )*0.05f );
		
		//Roll Pitch控制输入
		Attitude_Control_set_Target_RollPitch( \
			( roll_stick 	- 50.0f )*0.015f, \
			( pitch_stick - 50.0f )*0.015f );
	}
}

