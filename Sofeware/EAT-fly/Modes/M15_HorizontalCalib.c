#include "Modes.h"
#include "Basic.h"
#include "M15_HorizontalCalib.h"
#include <stdlib.h>

#include "MeasurementSystem.h"
#include "InteractiveInterface.h"
#include "Sensors.h"
#include "AC_Math.h"
#include "Configurations.h"

//水平校准
//将飞机放平（不是飞控）
//静止一段时间

static void M15_HorizontalCalib_MainFunc();
static void M15_HorizontalCalib_enter();
static void M15_HorizontalCalib_exit();
const Mode M15_HorizontalCalib = 
{
	50 , //mode frequency
	M15_HorizontalCalib_enter , //enter
	M15_HorizontalCalib_exit ,	//exit
	M15_HorizontalCalib_MainFunc ,	//mode main func
};

//定义静止等待时间
#define Calibration_Time 100

typedef struct
{
	//判断板子是否静止
	vector3_float Calibration_Acc_Max , Calibration_Acc_Min;
	vector3_float Calibration_Gyro_Max , Calibration_Gyro_Min;
	//校准计数器
	unsigned short Calibration_Counter;
}MODE_INF;
static MODE_INF* Mode_Inf;

static void M15_HorizontalCalib_enter()
{
	//设置状态灯
	Led_setProgress(0);
	
	//初始化模式变量
	Mode_Inf = malloc( sizeof( MODE_INF ) );
	Mode_Inf->Calibration_Acc_Max.x = Mode_Inf->Calibration_Acc_Max.y = Mode_Inf->Calibration_Acc_Max.z = 0;	
	Mode_Inf->Calibration_Acc_Min.x = Mode_Inf->Calibration_Acc_Min.y = Mode_Inf->Calibration_Acc_Min.z = 0;
	Mode_Inf->Calibration_Gyro_Max.x = Mode_Inf->Calibration_Gyro_Max.y = Mode_Inf->Calibration_Gyro_Max.z = 0;	
	Mode_Inf->Calibration_Gyro_Min.x = Mode_Inf->Calibration_Gyro_Min.y = Mode_Inf->Calibration_Gyro_Min.z = 0;
	Mode_Inf->Calibration_Counter = 0;
}

static void M15_HorizontalCalib_exit()
{
	free( Mode_Inf );
}

static void M15_HorizontalCalib_MainFunc()
{
	vector3_float acc_filted = get_AccelerationFilted();
	vector3_float gyro_filted = get_AngularRateCtrl();
	
	//判断飞控是否静止
	if( acc_filted.x > Mode_Inf->Calibration_Acc_Max.x ) Mode_Inf->Calibration_Acc_Max.x = acc_filted.x;
	else if( acc_filted.x < Mode_Inf->Calibration_Acc_Min.x ) Mode_Inf->Calibration_Acc_Min.x = acc_filted.x;
	if( acc_filted.y > Mode_Inf->Calibration_Acc_Max.y ) Mode_Inf->Calibration_Acc_Max.y = acc_filted.y;
	else if( acc_filted.y < Mode_Inf->Calibration_Acc_Min.y ) Mode_Inf->Calibration_Acc_Min.y = acc_filted.y;
	if( acc_filted.z > Mode_Inf->Calibration_Acc_Max.z ) Mode_Inf->Calibration_Acc_Max.z = acc_filted.z;
	else if( acc_filted.z < Mode_Inf->Calibration_Acc_Min.z ) Mode_Inf->Calibration_Acc_Min.z = acc_filted.z;
	
	if( gyro_filted.x > Mode_Inf->Calibration_Gyro_Max.x ) Mode_Inf->Calibration_Gyro_Max.x = gyro_filted.x;
	else if( gyro_filted.x < Mode_Inf->Calibration_Gyro_Min.x ) Mode_Inf->Calibration_Gyro_Min.x = gyro_filted.x;
	if( gyro_filted.y > Mode_Inf->Calibration_Gyro_Max.y ) Mode_Inf->Calibration_Gyro_Max.y = gyro_filted.y;
	else if( gyro_filted.y < Mode_Inf->Calibration_Gyro_Min.y ) Mode_Inf->Calibration_Gyro_Min.y = gyro_filted.y;
	if( gyro_filted.z > Mode_Inf->Calibration_Gyro_Max.z ) Mode_Inf->Calibration_Gyro_Max.z = gyro_filted.z;
	else if( gyro_filted.z < Mode_Inf->Calibration_Gyro_Min.z ) Mode_Inf->Calibration_Gyro_Min.z = gyro_filted.z;
	
	float acc_fluctuation_range;	float gyro_fluctuation_range;
	arm_sqrt_f32( vector3_float_square( vector3_float_subtract( Mode_Inf->Calibration_Acc_Max , Mode_Inf->Calibration_Acc_Min ) ), &acc_fluctuation_range );
	arm_sqrt_f32( vector3_float_square( vector3_float_subtract( Mode_Inf->Calibration_Gyro_Max , Mode_Inf->Calibration_Gyro_Min) ) , &gyro_fluctuation_range );
	if( ( acc_fluctuation_range > 10.0f ) || ( gyro_fluctuation_range > 0.03f ) )
	{
		//板子不是静止的
		Led_setProgress(0);
		Mode_Inf->Calibration_Acc_Max.x = Mode_Inf->Calibration_Acc_Min.x = acc_filted.x;
		Mode_Inf->Calibration_Acc_Max.y = Mode_Inf->Calibration_Acc_Min.y = acc_filted.y;
		Mode_Inf->Calibration_Acc_Max.z = Mode_Inf->Calibration_Acc_Min.z = acc_filted.z;
		Mode_Inf->Calibration_Gyro_Max.x = Mode_Inf->Calibration_Gyro_Min.x = gyro_filted.x;
		Mode_Inf->Calibration_Gyro_Max.y = Mode_Inf->Calibration_Gyro_Min.y = gyro_filted.y;
		Mode_Inf->Calibration_Gyro_Max.z = Mode_Inf->Calibration_Gyro_Min.z = gyro_filted.z;
		Mode_Inf->Calibration_Counter = 0;
		return;
	}
	else
		Led_setProgress( Mode_Inf->Calibration_Counter * 100 / Calibration_Time );
	
	++Mode_Inf->Calibration_Counter;
	
	if( Mode_Inf->Calibration_Counter >= Calibration_Time )
	{
		Led_setSignal( LED_signal_success );
		Quaternion quat = get_attitude();
		quat = Quaternion_conjugate( Quaternion_get_PRQuat( quat ) );
		Cfg_update_Horizontal_Calibration( quat );
		change_Mode( 01 );
	}
}

