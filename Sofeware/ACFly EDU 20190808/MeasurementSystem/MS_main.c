#include "MS_main.h"
#include "MeasurementSystem.h"

#include "Configurations.h"
#include "Sensors.h"
#include "MS_Attitude.h"
#include "MS_Position.h"
#include "Sensors_Backend.h"

#include "vector_3.h"
#include "Filters_Butter.h"

/*解算系统状态*/
	MeasurementSystem_Status Attitude_Measurement_System_Status = Measurement_System_Status_Initializing;
	MeasurementSystem_Status Altitude_Measurement_System_Status = Measurement_System_Status_Initializing;
	MeasurementSystem_Status Position_Measurement_System_Status = Measurement_System_Status_Initializing;
	MeasurementSystem_Status get_Attitude_Measurement_System_Status()
	{
		return Attitude_Measurement_System_Status;
	}
	MeasurementSystem_Status get_Altitude_Measurement_System_Status()
	{
		return Altitude_Measurement_System_Status;
	}
	MeasurementSystem_Status get_Position_Measurement_System_Status()
	{
		return Position_Measurement_System_Status;
	}
/*解算系统状态*/

/*滤波数据*/
	static Filter_Butter4_LP_float AngularRateLP_Filters[3];
	static vector3_float AngularRateFilted;
	
	static Filter_Butter4_LP_float Acceleration_Filters[3];
	static Filter_Butter4_LP_float AccelerationCtrl_Filters[3];
	static Filter_Butter8_LP_float AngularRateCtrl_Filters[3];
	static vector3_float AccelerationFilted;
	static vector3_float AccelerationCtrl;
	static vector3_float AngularRateCtrl;
	vector3_float get_AngularRateFilted()
	{
		return AngularRateFilted;
	}
	vector3_float get_AccelerationFilted()
	{
		return AccelerationFilted;
	}
	vector3_float get_AccelerationCtrl()
	{
		return AccelerationCtrl;
	}
	vector3_float get_AngularRateCtrl()
	{
		return AngularRateCtrl;
	}
/*滤波数据*/

/*解算数据*/
	//获取倾斜角的cos
	float get_lean_angle_cosin()
	{
		Quaternion Airframe_Quat = get_Airframe_attitude();
		return 	Airframe_Quat.qw*Airframe_Quat.qw - \
						Airframe_Quat.qx*Airframe_Quat.qx - \
						Airframe_Quat.qy*Airframe_Quat.qy + \
						Airframe_Quat.qz*Airframe_Quat.qz;
	}
/*解算数据*/
	
//解算系统主函数
//由传感器驱动在获取传感器数据后调用
void MS_main()
{
	TIME t = get_TIME_now();
	
	/*滤波*/
		const IMU_Sensor* Accelerometer = GetAccelerometer(0);
		const IMU_Sensor* Gyroscope = GetGyroscope(0);
	
		AngularRateFilted.x = Filter_Butter4_LP_float_run( &AngularRateLP_Filters[0] , Gyroscope->data.x );
		AngularRateFilted.y = Filter_Butter4_LP_float_run( &AngularRateLP_Filters[1] , Gyroscope->data.y );
		AngularRateFilted.z = Filter_Butter4_LP_float_run( &AngularRateLP_Filters[2] , Gyroscope->data.z );
	
		AccelerationFilted.x = Filter_Butter4_LP_float_run( &Acceleration_Filters[0] , Accelerometer->data.x );
		AccelerationFilted.y = Filter_Butter4_LP_float_run( &Acceleration_Filters[1] , Accelerometer->data.y );
		AccelerationFilted.z = Filter_Butter4_LP_float_run( &Acceleration_Filters[2] , Accelerometer->data.z );
	
		AccelerationCtrl.x = Filter_Butter4_LP_float_run( &AccelerationCtrl_Filters[0] , get_AccelerationENU().x );
		AccelerationCtrl.y = Filter_Butter4_LP_float_run( &AccelerationCtrl_Filters[1] , get_AccelerationENU().y );
		AccelerationCtrl.z = Filter_Butter4_LP_float_run( &AccelerationCtrl_Filters[2] , get_AccelerationENU().z );
	
		AngularRateCtrl.x = Filter_Butter8_LP_float_run( &AngularRateCtrl_Filters[0] , Gyroscope->data.x );
		AngularRateCtrl.y = Filter_Butter8_LP_float_run( &AngularRateCtrl_Filters[1] , Gyroscope->data.y );
		AngularRateCtrl.z = Filter_Butter8_LP_float_run( &AngularRateCtrl_Filters[2] , Gyroscope->data.z );	
	/*滤波*/
	
	/*失能无效传感器*/
		for( uint8_t i = 0 ; i < Position_Sensors_Count ; ++i )
		{
			const Position_Sensor* sensor = GetPositionSensor(i);
			if( sensor->available )
			{
				float last_update_time = get_pass_time( sensor->last_update_time );
				if( last_update_time > 1.2f )
					PositionSensorSetInavailable( i );
			}
		}
	/*失能无效传感器*/
	
	MS_Attitude();
	MS_Position();
	
	static float tt;
	tt = get_pass_time( t );
}

void init_MS()
{
	Filter_Butter4_LP_float_init( &AngularRateLP_Filters[0] , 1000 , 10 );
	Filter_Butter4_LP_float_init( &AngularRateLP_Filters[1] , 1000 , 10 );
	Filter_Butter4_LP_float_init( &AngularRateLP_Filters[2] , 1000 , 10 );
	
	Filter_Butter4_LP_float_init( &Acceleration_Filters[0] , 1000 , 10 );
	Filter_Butter4_LP_float_init( &Acceleration_Filters[1] , 1000 , 10 );
	Filter_Butter4_LP_float_init( &Acceleration_Filters[2] , 1000 , 10 );
	
	Filter_Butter4_LP_float_init( &AccelerationCtrl_Filters[0] , 1000 , 10 );
	Filter_Butter4_LP_float_init( &AccelerationCtrl_Filters[1] , 1000 , 10 );
	Filter_Butter4_LP_float_init( &AccelerationCtrl_Filters[2] , 1000 , 10 );
	
	Filter_Butter8_LP_float_init( &AngularRateCtrl_Filters[0] , 1000 , 50 );
	Filter_Butter8_LP_float_init( &AngularRateCtrl_Filters[1] , 1000 , 50 );
	Filter_Butter8_LP_float_init( &AngularRateCtrl_Filters[2] , 1000 , 50 );
	init_MS_Attitude();
	init_MS_Position();
	
	Quaternion initial_Horizontal_Calibration_Quat = { 1 , 0 , 0 , 0 };
	Cfg_set_initial_Horizontal_Calibration( initial_Horizontal_Calibration_Quat );
}
