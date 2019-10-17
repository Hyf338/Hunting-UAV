#pragma once

#include "Sensors.h"

extern IMU_Sensor Accelerometer[IMU_Sensors_Count];
extern IMU_Sensor Gyroscope[IMU_Sensors_Count];
extern IMU_Sensor Magnetometer[IMU_Sensors_Count];

extern Position_Sensor Position_Sensors[ Position_Sensors_Count ];

/*IMU*/

	/*IMU传感器注册函数*/
		bool IMUAccelerometerRegister( unsigned char index , float sensitivity );
		bool IMUAccelerometerUnRegister( unsigned char index );

		bool IMUGyroscopeRegister( unsigned char index , float sensitivity );
		bool IMUGyroscopeUnRegister( unsigned char index );
		
		bool IMUMagnetometerRegister( unsigned char index , float sensitivity );
		bool IMUMagnetometerUnRegister( unsigned char index );
	/*IMU传感器注册函数*/
	
	/*IMU传感器更新函数*/
		bool IMUAccelerometerUpdate( unsigned char index , vector3_int data );
		bool IMUGyroscopeUpdate( unsigned char index , vector3_int data );
		bool IMUMagnetometerUpdate( unsigned char index , vector3_int data );
	/*IMU传感器更新函数*/

/*IMU*/

/*位置传感器*/
	//参数详细定义见Sensors.h中位置传感器注释
		
	/*位置传感器注册函数*/
		//safe：传感器是否安全（数据缓慢变化不会发生跳变 ！！注意！！如不确定不要设置为safe）
		bool PositionSensorRegister( 
			unsigned char index ,\
			Position_Sensor_Type sensor_type ,\
			Position_Sensor_DataType sensor_data_type ,\
			Position_Sensor_frame sensor_vel_frame ,\
			float delay ,\
			bool safe \
		);
		//注销传感器
		bool PositionSensorUnRegister( unsigned char index );
	/*位置传感器注册函数*/
				
	//更改位置传感器DataType
	bool PositionSensorChangeDataType( unsigned char index , Position_Sensor_DataType datatype );
			
	/*位置传感器更新函数*/
		//delay参数小于0则不会改变delay		
		bool PositionSensorUpdatePositionGlobal( unsigned char index , vector3_double position_Global , bool available , float delay );
		bool PositionSensorUpdatePosition( unsigned char index , vector3_float position , bool available , float delay );
		bool PositionSensorUpdatePositionGlobalVel( unsigned char index , vector3_double position_Global , vector3_float vel , bool available , float delay );
		bool PositionSensorUpdatePositionVel( unsigned char index , vector3_float position , vector3_float vel , bool available , float delay );
		bool PositionSensorUpdateVel( unsigned char index , vector3_float vel , bool available , float delay );
		//位置传感器失效函数
		bool PositionSensorSetInavailable( unsigned char index );
	/*IMU传感器更新函数*/
		
/*位置传感器*/	