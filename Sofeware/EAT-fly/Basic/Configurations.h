#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "Quaternion.h"
#include "Receiver.h"
#include "mavlink.h"

//存储版本号
//更改此值将会擦除EEProm所有内容
//并重新写入初始配置信息
#define CFG_Version 12

//本页设置见excel表：ACFly EDU EEPROM分配
//每种设置分为三个函数：
//Cfg_set_initial_xxx ：用于初始化设置
//Cfg_get_xxx ：用于读取设置
//Cfg_update_xxx ：用于更新设置并写入eeprom

/*加速度计校准*/
	void Cfg_set_initial_AccelerometerOffset( uint8_t sensor , vector3_float offset );
	vector3_float Cfg_get_AccelerometerOffset( uint8_t sensor );
	bool Cfg_update_AccelerometerOffset( uint8_t sensor , vector3_float offset );
	
	void Cfg_set_initial_AccelerometerSensivitity( uint8_t sensor , vector3_float Sensivitity );
	vector3_float Cfg_get_AccelerometerSensivitity( uint8_t sensor );
	bool Cfg_update_AccelerometerSensivitity( uint8_t sensor , vector3_float Sensivitity );
/*加速度计校准*/

/*陀螺校准*/
	void Cfg_set_initial_GyroscopeOffset( uint8_t sensor , vector3_float offset );
	vector3_float Cfg_get_GyroscopeOffset( uint8_t sensor );
	bool Cfg_update_GyroscopeOffset( uint8_t sensor , vector3_float offset );
	
	void Cfg_set_initial_GyroscopeSensivitity( uint8_t sensor , vector3_float Sensivitity );
	vector3_float Cfg_get_GyroscopeSensivitity( uint8_t sensor );
	bool Cfg_update_GyroscopeSensivitity( uint8_t sensor , vector3_float Sensivitity );
/*陀螺校准*/

/*磁罗盘校准*/
	void Cfg_set_initial_MagnetometerOffset( uint8_t sensor , vector3_float offset );
	vector3_float Cfg_get_MagnetometerOffset( uint8_t sensor );
	bool Cfg_update_MagnetometerOffset( uint8_t sensor , vector3_float offset );
	
	void Cfg_set_initial_MagnetometerSensivitity( uint8_t sensor , vector3_float Sensivitity );
	vector3_float Cfg_get_MagnetometerSensivitity( uint8_t sensor );
	bool Cfg_update_MagnetometerSensivitity( uint8_t sensor , vector3_float Sensivitity );
/*磁罗盘校准*/

/*接收机校准*/
	void Cfg_set_initial_Channel_Reflection( RC_Type rc , uint8_t assign[8] );
	const uint8_t* Cfg_get_Channel_Reflection( RC_Type rc );
	bool Cfg_update_Channel_Reflection( RC_Type rc , uint8_t assign[8] );
	
	void Cfg_set_initial_Channel_Min( RC_Type rc , float channels_Min[8] );
	const float* Cfg_get_Channel_Min( RC_Type rc );
	bool Cfg_update_Channel_Min( RC_Type rc , float channels_Min[8] );
	
	void Cfg_set_initial_Channel_Scale( RC_Type rc , float channels_Scale[8] );
	const float* Cfg_get_Channel_Scale( RC_Type rc );
	bool Cfg_update_Channel_Scale( RC_Type rc , float channels_Scale[8] );
/*接收机校准*/

/*水平校准*/
	void Cfg_set_initial_Horizontal_Calibration( Quaternion quat );
	Quaternion Cfg_get_Horizontal_Calibration();
	bool Cfg_update_Horizontal_Calibration( Quaternion quat );
/*水平校准*/


/*飞行器参数*/
	//飞行器类型
	typedef enum
	{
		UAVType_Rotor4_X = 10 ,	//四旋翼X型
		UAVType_Rotor6_X = 11 ,	//六旋翼X型
		UAVType_Rotor8_X = 12 ,	//八旋翼X型
		
		UAVType_Rotor4_C = 15 ,	//四旋翼十字型
		
		UAVType_Rotor42_C = 20 ,	//四旋翼Double十字型
	}UAVType;	
	void Cfg_set_initial_UAVType( UAVType type );
	UAVType Cfg_get_UAVType();
	bool Cfg_update_UAVType( UAVType type );
/*飞行器参数*/
	
/*电机参数*/
	//电机起转油门
	void Cfg_set_initial_MotorStartingThrottle( uint8_t thr );
	uint8_t Cfg_get_MotorStartingThrottle();
	bool Cfg_update_MotorStartingThrottle( uint8_t thr );
	
	//电机非线性修正系数
	void Cfg_set_initial_MotorNonlineFactor( float factor );
	float Cfg_get_MotorNonlineFactor();
	bool Cfg_update_MotorNonlineFactor( float factor );
	
	//电机满输出油门比例
	void Cfg_set_initial_MotorFullThrottleRatio( float ratio );
	float Cfg_get_MotorFullThrottleRatio();
	bool Cfg_update_MotorFullThrottleRatio( float ratio );
	
	//电机加速惯性时间T
	void Cfg_set_initial_MotorT( float T );
	float Cfg_get_MotorT();
	bool Cfg_update_MotorT( float T );
/*电机参数*/

/*BAT---电池参数*/
	//电池基准电压
	void Cfg_set_initial_BatSTVoltage( float T );
	float Cfg_get_BatSTVoltage();
	bool Cfg_update_BatSTVoltage( float T );
	
	//电源ADC放大倍数
	void Cfg_set_initial_BatVoltageADCMag( float T );
	float Cfg_get_BatVoltageADCMag();
	bool Cfg_update_BatVoltageADCMag( float T );
/*BAT---电池参数*/

/*姿态控制参数*/
	//姿态三轴感度及增益
	//RPY：0-Roll 1-Pitch 2-Yaw
	void Cfg_set_initial_RPYCtrl_b( uint8_t RPY , float b );
	float Cfg_get_RPYCtrl_b( uint8_t RPY );
	bool Cfg_update_RPYCtrl_b( uint8_t RPY , float b );
	
	void Cfg_set_initial_RPYCtrl_Pn( uint8_t RPY , float P , uint8_t n );
	float Cfg_get_RPYCtrl_Pn( uint8_t RPY , uint8_t n );
	bool Cfg_update_RPYCtrl_Pn( uint8_t RPY , float P , uint8_t n );
	
	void Cfg_set_initial_RPYCtrl_TD4Pn( uint8_t RPY , float P , uint8_t n );
	float Cfg_get_RPYCtrl_TD4Pn( uint8_t RPY , uint8_t n );
	bool Cfg_update_RPYCtrl_TD4Pn( uint8_t RPY , float P , uint8_t n );
/*姿态控制参数*/

/*正版验证*/	
	uint32_t Cfg_get_WGA( uint8_t index );
	bool Cfg_update_WGA( uint32_t wga[3] );
/*正版验证*/



/*Params*/
	//用户可通过上位机等更改的参数
	typedef struct
	{
		//参数类型
		MAV_PARAM_TYPE type;
		//参数名称（最多16字符）
		char name[17];
		//更新参数的函数指针
		bool (*update_func)( float param );
		//读取参数的函数指针
		float (*get_param)();
	}Param;
	
	//参数列表
	extern const Param Params[];
	//参数个数
	extern const uint16_t Params_Count;
	//用名称寻找参数
	int16_t Params_Find( char name[17] );
/*Params*/

void init_Configurations();