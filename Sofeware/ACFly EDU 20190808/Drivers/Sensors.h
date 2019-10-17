#pragma once

#include "Basic.h"
#include "AC_Math.h"
#include "map_projection.h"
#include "vector_3.h"

#define IMU_Sensors_Count 3
#define Position_Sensors_Count 8

/*IMU传感器定义*/
	typedef struct
	{
		bool present;	//传感器是否存在
		TIME last_update_time;	//上次更新时间
		float sample_time;	//采样时间
		
		float sensitivity;	//灵敏度（原始数据->实际单位 陀螺：rad/s 加速度：cm/s^2 磁场：gauss）
		
		vector3_int data_raw;	//原始数据
		vector3_float data;	//实际单位数据
	}IMU_Sensor;
/*IMU传感器定义*/

/*位置传感器定义*/
	typedef enum
	{
		Position_Sensor_Type_GlobalPositioning ,	//全球定位（经纬度定位，如GPS）
		Position_Sensor_Type_RelativePositioning ,	//相对定位（如气压计，参照物不会改变）
		Position_Sensor_Type_RangePositioning ,	//距离定位（测距定位，如超声波，参照物可能会变化）
	}Position_Sensor_Type;
	typedef enum
	{
		//s-位置 v-速度
		//如sv_xy表示该传感器具有：位置速度的xy数据
		Position_Sensor_DataType_s_xy ,
		Position_Sensor_DataType_s_z ,
		Position_Sensor_DataType_s_xyz ,
		
		Position_Sensor_DataType_v_xy ,
		Position_Sensor_DataType_v_z ,
		Position_Sensor_DataType_v_xyz ,
		
		Position_Sensor_DataType_sv_xy ,
		Position_Sensor_DataType_sv_z ,
		Position_Sensor_DataType_sv_xyz ,
	}Position_Sensor_DataType;
	typedef enum
	{
		Position_Sensor_frame_ENU ,	//速度数据在ENU坐标系下
		Position_Sensor_frame_BodyHeading ,	//速度数据x为机头朝向（与地面平行），y为朝向机头左方（与地面平行），z为上方
	}Position_Sensor_frame;
	typedef struct
	{
		bool publishing;	//是否正在更新
		
		bool present;	//传感器是否存在
		bool available;	//传感器是否可用
		TIME last_update_time;	//上次更新时间
		TIME inavailable_start_time;	//传感器不可用开始时间
		float delay;	//传感器延时
		float sample_time;	//采样时间
		
		bool safe;	//传感器是否安全（数据缓慢变化不会发生跳变 ！！注意！！如不确定不要设置为safe）
		Position_Sensor_Type sensor_type;	//传感器类型（见枚举注释）
		Position_Sensor_DataType sensor_DataType;	//传感器数据类型（见枚举注释）
		Position_Sensor_frame velocity_data_frame;	//速度数据坐标系（见枚举注释）
		
		vector3_double position_Global;	//经纬度
		vector3_float position;	//位置(cm)
		vector3_float velocity;	//速度(cm/s)
	}Position_Sensor;

	//获取当前经纬度定位可不可用
	bool get_MP_Available();
	//获取当前经纬度转平面信息
	const Map_Projection* get_MP();
	
	extern const unsigned char default_ultrasonic_sensor_index;
	
	extern const unsigned char default_GY53_sensor_index;
	
	extern const unsigned char  default_optical_flow_index;
	extern const unsigned char  internal_baro_sensor_index;
	extern const unsigned char  default_gps_sensor_index;
/*位置传感器定义*/
	
/*IMU*/

	/*IMU传感器读取函数*/
		const IMU_Sensor* GetAccelerometer( unsigned char index );
		const IMU_Sensor* GetGyroscope( unsigned char index );
		const IMU_Sensor* GetMagnetometer( unsigned char index );
	/*IMU传感器注册函数*/
	
	extern const unsigned char External_Magnetometer_Index;
	extern const unsigned char Internal_Magnetometer_Index;
	
/*IMU*/
	
/*位置传感器*/
	
	/*位置传感器读取函数*/
		const Position_Sensor* GetPositionSensor( unsigned char index );
	/*位置传感器读取函数*/
		
/*位置传感器*/

/*电池*/
	float getBatteryVoltage();
/*电池*/