#pragma once

#include <stdbool.h>
#include "Quaternion.h"

/*解算系统状态*/
	typedef enum
	{
		Measurement_System_Status_Initializing ,
		
		Measurement_System_Status_Ready ,
	}MeasurementSystem_Status;
	
	MeasurementSystem_Status get_Attitude_Measurement_System_Status();
	MeasurementSystem_Status get_Altitude_Measurement_System_Status();
	MeasurementSystem_Status get_Position_Measurement_System_Status();
/*解算系统状态*/

/*获取滤波数据*/
	vector3_float get_AngularRateFilted();
	vector3_float get_AccelerationFilted();
	vector3_float get_AccelerationCtrl();
	vector3_float get_AngularRateCtrl();
/*获取滤波数据*/

/*解算数据*/
	//获取倾斜角的cos
	float get_lean_angle_cosin();
	//获取指定传感器的估计位置
	bool get_Estimated_Sensor_Position_z( float* pos_z , unsigned char index );
	bool get_Estimated_Sensor_Position_xy( float* pos_x , float* pos_y , unsigned char index );
	bool get_Point_From_LatLon( float* pos_x , float* pos_y , double Lat , double Lon );
	bool get_LatLon_From_Point( float pos_x , float pos_y , double* Lat , double* Lon );
/*解算数据*/
	
/*获取历史数据*/
	Quaternion get_attitude();
	Quaternion get_Airframe_attitude();
	
	vector3_float get_Position();
	vector3_float get_VelocityENU();
	vector3_float get_AccelerationENU();
	
	Quaternion get_history_attitude( float t );
	Quaternion get_history_Airframe_attitude( float t );
	vector3_float get_history_acceleration( float t );
	
	vector3_float get_history_position( float delay );
	vector3_float get_history_velocityENU( float delay );
/*获取历史数据*/
	
	
/*获取正版验证*/
	//获取正版验证是否成功
	bool MS_Attitude_WGA();
	//获取正版验证标识符
	unsigned int MS_Attitude_get_WGA( unsigned char index );
	
	//获取正版验证是否成功
	bool MS_Position_WGA();
	//获取正版验证标识符
	unsigned int MS_Position_get_WGA( unsigned char index );
/*获取正版验证*/