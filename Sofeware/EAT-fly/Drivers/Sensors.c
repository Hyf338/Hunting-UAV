#include "Sensors_Backend.h"

#include "Configurations.h"

//IMU传感器定义
IMU_Sensor Accelerometer[IMU_Sensors_Count];
IMU_Sensor Gyroscope[IMU_Sensors_Count];
IMU_Sensor Magnetometer[IMU_Sensors_Count];

//经纬度转平面坐标变量
static bool MP_Available = false;
static Map_Projection MP;
//获取当前经纬度定位可不可用
bool get_MP_Available()
{
	return MP_Available;
}
//获取当前经纬度转平面信息
const Map_Projection* get_MP()
{
	return &MP;
}

//位置传感器定义
Position_Sensor Position_Sensors[ Position_Sensors_Count ];

/*IMU*/

	const unsigned char External_Magnetometer_Index = 0;
	const unsigned char Internal_Magnetometer_Index = 2;

	/*IMU传感器注册函数*/
		bool IMUAccelerometerRegister( unsigned char index , float sensitivity )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			if( Accelerometer[index].present )
				return false;
			
			Accelerometer[index].present = true;
			Accelerometer[index].last_update_time = get_TIME_now();
			Accelerometer[index].sensitivity = sensitivity;
			
			return true;
		}
		bool IMUAccelerometerUnRegister( unsigned char index )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			if( Accelerometer[index].present == false )
				return false;
			
			Accelerometer[index].present = false;
			return true;
		}
		
		bool IMUGyroscopeRegister( unsigned char index , float sensitivity )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			if( Gyroscope[index].present )
				return false;
			
			Gyroscope[index].present = true;
			Gyroscope[index].last_update_time = get_TIME_now();
			Gyroscope[index].sensitivity = sensitivity;
			
			return true;
		}
		bool IMUGyroscopeUnRegister( unsigned char index )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			if( Gyroscope[index].present == false )
				return false;
			
			Gyroscope[index].present = false;
			return true;
		}
		
		bool IMUMagnetometerRegister( unsigned char index , float sensitivity )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			if( Magnetometer[index].present )
				return false;
			
			Magnetometer[index].present = true;
			Magnetometer[index].last_update_time = get_TIME_now();
			Magnetometer[index].sensitivity = sensitivity;
			
			return true;
		}
		bool IMUMagnetometerUnRegister( unsigned char index )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			if( Magnetometer[index].present == false )
				return false;
			
			Magnetometer[index].present = false;
			return true;
		}
	/*IMU传感器注册函数*/
		
	/*IMU传感器更新函数*/
		bool IMUAccelerometerUpdate( unsigned char index , vector3_int data )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			if( Accelerometer[index].present == false )
				return false;
			
			//写入更新时间
			Accelerometer[index].sample_time = get_pass_time_st( &Accelerometer[index].last_update_time );
			//写入传感器数据
			Accelerometer[index].data_raw = data;
			vector3_float rdata;
			vector3_float offset = Cfg_get_AccelerometerOffset( index );
			vector3_float scale = Cfg_get_AccelerometerSensivitity( index );
			rdata.x = ( data.x - offset.x ) * scale.x;
			rdata.y = ( data.y - offset.y ) * scale.y;
			rdata.z = ( data.z - offset.z ) * scale.z;
			rdata = vector3_float_mult( rdata , Accelerometer[index].sensitivity );
			Accelerometer[index].data = rdata;
			
			return true;
		}
		bool IMUGyroscopeUpdate( unsigned char index , vector3_int data )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			if( Gyroscope[index].present == false )
				return false;
			
			//写入更新时间
			Gyroscope[index].sample_time = get_pass_time_st( &Gyroscope[index].last_update_time );
			//写入传感器数据
			Gyroscope[index].data_raw = data;
			vector3_float rdata;
			vector3_float offset = Cfg_get_GyroscopeOffset( index );
			rdata.x = data.x - offset.x;
			rdata.y = data.y - offset.y;
			rdata.z = data.z - offset.z;
			rdata = vector3_float_mult( rdata , Gyroscope[index].sensitivity );
			Gyroscope[index].data = rdata;
			
			return true;
		}
		bool IMUMagnetometerUpdate( unsigned char index , vector3_int data )
		{
			if( Magnetometer[index].present == false || index >= IMU_Sensors_Count )
				return false;
			
			//写入更新时间
			Magnetometer[index].sample_time = get_pass_time_st( &Magnetometer[index].last_update_time );
			//写入传感器数据
			Magnetometer[index].data_raw = data;
			vector3_float rdata;
			vector3_float offset = Cfg_get_MagnetometerOffset( index );
			vector3_float scale = Cfg_get_MagnetometerSensivitity( index );
			rdata.x = ( data.x - offset.x ) * scale.x;
			rdata.y = ( data.y - offset.y ) * scale.y;
			rdata.z = ( data.z - offset.z ) * scale.z;
			rdata = vector3_float_mult( rdata , Magnetometer[index].sensitivity );
			Magnetometer[index].data = rdata;
			
			return true;
		}
	/*IMU传感器更新函数*/
	
	/*IMU传感器读取函数*/
		const IMU_Sensor* GetAccelerometer( unsigned char index )
		{
			if( index >= IMU_Sensors_Count )
				return 0;
			
			return &Accelerometer[index];
		}
		const IMU_Sensor* GetGyroscope( unsigned char index )
		{
			if( index >= IMU_Sensors_Count )
				return 0;
			
			return &Gyroscope[index];
		}
		const IMU_Sensor* GetMagnetometer( unsigned char index )
		{
			if( index >= IMU_Sensors_Count )
				return 0;
			
			return &Magnetometer[index];
		}
	/*IMU传感器读取函数*/
		
/*IMU*/
		
/*位置传感器*/
		
	const unsigned char default_GY53_sensor_index = 0;
	const unsigned char default_ultrasonic_sensor_index = 1;
	const unsigned char default_optical_flow_index = 6;
	const unsigned char internal_baro_sensor_index = 2;
	const unsigned char default_gps_sensor_index = 5;
		
	/*位置传感器注册函数*/
		bool PositionSensorRegister( 
			unsigned char index ,\
			Position_Sensor_Type sensor_type ,\
			Position_Sensor_DataType sensor_data_type ,\
			Position_Sensor_frame sensor_vel_frame ,\
			float delay ,\
			bool safe \
		)
		{
			if( index >= Position_Sensors_Count )
				return false;			
			Position_Sensor* sensor = &Position_Sensors[index];
			if( sensor->present )
				return false;			
			if( delay < 0 )
				return false;
			if( sensor->publishing )
				return false;
			
			sensor->publishing = true;
			
			sensor->present = true;
			sensor->available = false;
			sensor->inavailable_start_time = get_TIME_now();
			sensor->last_update_time = get_TIME_now();
			sensor->sensor_type = sensor_type;
			sensor->sensor_DataType = sensor_data_type;
			sensor->velocity_data_frame = sensor_vel_frame;
			sensor->delay = delay;
			sensor->safe = safe;
			sensor->sample_time = 0;
			
			sensor->publishing = false;
			
			return true;
		}
		bool PositionSensorUnRegister( unsigned char index )
		{
			if( index >= Position_Sensors_Count )
				return false;			
			Position_Sensor* sensor = &Position_Sensors[index];
			if( sensor->present == false )
				return false;
			if( sensor->publishing )
				return false;
			
			bool NoGlobalPosSensor = true;
			for( uint8_t i = 0 ; i < Position_Sensors_Count ; ++i )
			{
				const Position_Sensor* sensor = GetPositionSensor(i);
				if( sensor->available )
				{
					if( sensor->sensor_type == Position_Sensor_Type_GlobalPositioning )
					{
						NoGlobalPosSensor = false;
						break;
					}
				}
			}
			if( NoGlobalPosSensor )
				MP_Available = false;
			
			sensor->present = sensor->available = false;
			return true;
		}
	/*位置传感器注册函数*/
	
	//更改位置传感器DataType
	bool PositionSensorChangeDataType( unsigned char index , Position_Sensor_DataType datatype )
	{
		if( index >= Position_Sensors_Count )
			return false;			
		Position_Sensor* sensor = &Position_Sensors[index];
		if( sensor->present == false )
			return false;
		if( sensor->publishing )
				return false;
		
		sensor->sensor_DataType = datatype;
		return true;
	}
		
	/*位置传感器更新函数*/
		
		//失能位置传感器
		bool PositionSensorSetInavailable( unsigned char index )
		{
			if( index >= Position_Sensors_Count )
				return false;			
			Position_Sensor* sensor = &Position_Sensors[index];
			if( sensor->present == false )
				return false;
			if( sensor->publishing )
				return false;
			
			sensor->publishing = true;
			
			if( Time_isValid( sensor->inavailable_start_time ) == false )
				sensor->inavailable_start_time = get_TIME_now();
			sensor->sample_time = get_pass_time_st( &sensor->last_update_time );
			
			sensor->available = false;
			
			bool NoGlobalPosSensor = true;
			for( uint8_t i = 0 ; i < Position_Sensors_Count ; ++i )
			{
				const Position_Sensor* sensor = GetPositionSensor(i);
				if( sensor->available )
				{
					if( sensor->sensor_type == Position_Sensor_Type_GlobalPositioning )
					{
						NoGlobalPosSensor = false;
						break;
					}
				}
			}
			if( NoGlobalPosSensor )
				MP_Available = false;			
			
			sensor->publishing = false;
			
			return true;
		}
	
		bool PositionSensorUpdatePositionGlobal( unsigned char index , vector3_double position_Global , bool available , float delay )
		{
			if( index >= Position_Sensors_Count )
				return false;			
			Position_Sensor* sensor = &Position_Sensors[index];
			if( sensor->present == false )
				return false;
			if( sensor->sensor_type != Position_Sensor_Type_GlobalPositioning )
				return false;
			if( sensor->publishing )
				return false;
			
			//判断传感器类型、数据是否正确
			bool data_effective;
			switch( sensor->sensor_DataType )
			{
				case Position_Sensor_DataType_s_xy:
					if( __ARM_isnan( position_Global.x ) || __ARM_isnan( position_Global.y ) || \
							__ARM_isinf( position_Global.x ) || __ARM_isinf( position_Global.y ) )
						data_effective = false;
					else
						data_effective = true;
					break;
					
				case Position_Sensor_DataType_s_z:
					if( __ARM_isnan( position_Global.z ) || __ARM_isinf( position_Global.z ) )
						data_effective = false;
					else
						data_effective = true;
					
				case Position_Sensor_DataType_s_xyz:
					if( __ARM_isnan( position_Global.x ) || __ARM_isnan( position_Global.y ) || __ARM_isnan( position_Global.z ) || \
							__ARM_isinf( position_Global.x ) || __ARM_isinf( position_Global.y ) || __ARM_isinf( position_Global.z ) )
						data_effective = false;
					else
						data_effective = true;
					break;
					
				default:
					data_effective = false;
					break;
			}
			if( !data_effective )
				return false;
			
			//传感器数据正在更新
			sensor->publishing = true;
			
			sensor->position_Global = position_Global;
			if( available )
			{	
				Time_set_inValid( &sensor->inavailable_start_time );
				
				if( MP_Available == false )
				{
					MP_Available = true;
					map_projection_init( position_Global.x , position_Global.y , &MP );
				}
				double pos_x , pos_y;
				map_projection_project( position_Global.x , position_Global.y , &pos_y , &pos_x , &MP );
				sensor->position.x = pos_x * 100;
				sensor->position.y = pos_y * 100;
				sensor->position.z = position_Global.z;
			}
			else if( Time_isValid( sensor->inavailable_start_time ) == false )
			{
				sensor->inavailable_start_time = get_TIME_now();
				bool NoGlobalPosSensor = true;
				for( uint8_t i = 0 ; i < Position_Sensors_Count ; ++i )
				{
					const Position_Sensor* sensor = GetPositionSensor(i);
					if( sensor->available )
					{
						if( sensor->sensor_type == Position_Sensor_Type_GlobalPositioning )
						{
							NoGlobalPosSensor = false;
							break;
						}
					}
				}
				if( NoGlobalPosSensor )
					MP_Available = false;
			}
				
			if( sensor->available )
				sensor->sample_time = get_pass_time_st( &sensor->last_update_time );
			else
				sensor->sample_time = 0;
			
			sensor->available = available;
			if( delay > 0 )
				sensor->delay = delay;
			
			//传感器数据更新完成
			sensor->publishing = false;
			
			return true;
		}
		bool PositionSensorUpdatePosition( unsigned char index , vector3_float position , bool available , float delay )
		{
			if( index >= Position_Sensors_Count )
				return false;			
			Position_Sensor* sensor = &Position_Sensors[index];
			if( sensor->present == false )
				return false;
			if( sensor->sensor_type == Position_Sensor_Type_GlobalPositioning )
				return false;
			if( sensor->publishing )
				return false;
			
			//判断传感器类型、数据是否正确
			bool data_effective;
			switch( sensor->sensor_DataType )
			{
				case Position_Sensor_DataType_s_xy:
					if( __ARM_isnanf( position.x ) || __ARM_isnanf( position.y ) || \
							__ARM_isinff( position.x ) || __ARM_isinff( position.y ) )
						data_effective = false;
					else
						data_effective = true;
					break;
					
				case Position_Sensor_DataType_s_z:
					if( __ARM_isnanf( position.z ) || __ARM_isinff( position.z ) )
						data_effective = false;
					else
						data_effective = true;
					
				case Position_Sensor_DataType_s_xyz:
					if( __ARM_isnanf( position.x ) || __ARM_isnanf( position.y ) || __ARM_isnanf( position.z ) || \
							__ARM_isinff( position.x ) || __ARM_isinff( position.y ) || __ARM_isinff( position.z ) )
						data_effective = false;
					else
						data_effective = true;
					break;
					
				default:
					data_effective = false;
					break;
			}
			if( !data_effective )
				return false;
			
			//传感器数据正在更新
			sensor->publishing = true;
			
			if( available )
			{		
				Time_set_inValid( &sensor->inavailable_start_time );	
				
				sensor->position = position;			
			}
			else if( Time_isValid( sensor->inavailable_start_time ) == false )
				sensor->inavailable_start_time = get_TIME_now();
			
			if( sensor->available )
				sensor->sample_time = get_pass_time_st( &sensor->last_update_time );
			else
				sensor->sample_time = 0;
			
			sensor->available = available;
			if( delay > 0 )
				sensor->delay = delay;
			
			//传感器数据更新完成
			sensor->publishing = false;
			
			return true;
		}
		bool PositionSensorUpdatePositionGlobalVel( unsigned char index , vector3_double position_Global , vector3_float vel , bool available , float delay )
		{
			if( index >= Position_Sensors_Count )
				return false;			
			Position_Sensor* sensor = &Position_Sensors[index];
			if( sensor->present == false )
				return false;
			if( sensor->sensor_type != Position_Sensor_Type_GlobalPositioning )
				return false;
			if( sensor->publishing )
				return false;
			
			//判断传感器类型、数据是否正确
			bool data_effective;
			switch( sensor->sensor_DataType )
			{
				case Position_Sensor_DataType_sv_xy:
					if( __ARM_isnan( position_Global.x ) || __ARM_isnan( position_Global.y ) || \
							__ARM_isinf( position_Global.x ) || __ARM_isinf( position_Global.y ) || \
							__ARM_isnanf( vel.x ) || __ARM_isnanf( vel.y ) || \
							__ARM_isinff( vel.x ) || __ARM_isinff( vel.y ) )
						data_effective = false;
					else
						data_effective = true;
					break;
					
				case Position_Sensor_DataType_sv_z:
					if( __ARM_isnan( position_Global.z ) || __ARM_isinf( position_Global.z ) || \
							__ARM_isnanf( vel.z ) || __ARM_isinff( vel.z ) )
						data_effective = false;
					else
						data_effective = true;
					
				case Position_Sensor_DataType_sv_xyz:
					if( __ARM_isnan( position_Global.x ) || __ARM_isnan( position_Global.y ) || __ARM_isnan( position_Global.z ) || \
							__ARM_isinf( position_Global.x ) || __ARM_isinf( position_Global.y ) || __ARM_isinf( position_Global.z ) || \
							__ARM_isnanf( vel.x ) || __ARM_isnanf( vel.y ) || __ARM_isnanf( vel.z ) || \
							__ARM_isinff( vel.x ) || __ARM_isinff( vel.y ) || __ARM_isinff( vel.z ) )
						data_effective = false;
					else
						data_effective = true;
					break;
					
				default:
					data_effective = false;
					break;
			}
			if( !data_effective )
				return false;
			
			//传感器数据正在更新
			sensor->publishing = true;
			
			sensor->position_Global = position_Global;
			if( available )
			{
				Time_set_inValid( &sensor->inavailable_start_time );
				
				if( MP_Available == false )
				{
					MP_Available = true;
					map_projection_init( position_Global.x , position_Global.y , &MP );
				}
				double pos_x , pos_y;
				map_projection_project( position_Global.x , position_Global.y , &pos_y , &pos_x , &MP );
				sensor->position.x = pos_x * 100;
				sensor->position.y = pos_y * 100;
				sensor->position.z = position_Global.z;
				sensor->velocity = vel;
			}
			else if( Time_isValid( sensor->inavailable_start_time ) == false )
			{
				sensor->inavailable_start_time = get_TIME_now();
				bool NoGlobalPosSensor = true;
				for( uint8_t i = 0 ; i < Position_Sensors_Count ; ++i )
				{
					const Position_Sensor* sensor = GetPositionSensor(i);
					if( sensor->available )
					{
						if( sensor->sensor_type == Position_Sensor_Type_GlobalPositioning )
						{
							NoGlobalPosSensor = false;
							break;
						}
					}
				}
				if( NoGlobalPosSensor )
					MP_Available = false;
			}
			
			if( sensor->available )
				sensor->sample_time = get_pass_time_st( &sensor->last_update_time );
			else
				sensor->sample_time = 0;
			
			sensor->available = available;
			if( delay > 0 )
				sensor->delay = delay;
			
			//传感器数据更新完成
			sensor->publishing = false;
			
			return true;
		}
		bool PositionSensorUpdatePositionVel( unsigned char index , vector3_float position , vector3_float vel , bool available , float delay )
		{
			if( index >= Position_Sensors_Count )
				return false;			
			Position_Sensor* sensor = &Position_Sensors[index];
			if( sensor->present == false )
				return false;
			if( sensor->sensor_type == Position_Sensor_Type_GlobalPositioning )
				return false;
			if( sensor->publishing )
				return false;
			
			//判断传感器类型、数据是否正确
			bool data_effective;
			switch( sensor->sensor_DataType )
			{
				case Position_Sensor_DataType_sv_xy:
					if( __ARM_isnanf( position.x ) || __ARM_isnanf( position.y ) || \
							__ARM_isinff( position.x ) || __ARM_isinff( position.y ) || \
							__ARM_isnanf( vel.x ) || __ARM_isnanf( vel.y ) || \
							__ARM_isinff( vel.x ) || __ARM_isinff( vel.y ) )
						data_effective = false;
					else
						data_effective = true;
					break;
					
				case Position_Sensor_DataType_sv_z:
					if( __ARM_isnanf( position.z ) || __ARM_isinff( position.z ) || \
							__ARM_isnanf( vel.z ) || __ARM_isinff( vel.z ) )
						data_effective = false;
					else
						data_effective = true;
					
				case Position_Sensor_DataType_sv_xyz:
					if( __ARM_isnanf( position.x ) || __ARM_isnanf( position.y ) || __ARM_isnanf( position.z ) || \
							__ARM_isinff( position.x ) || __ARM_isinff( position.y ) || __ARM_isinff( position.z ) || \
							__ARM_isnanf( vel.x ) || __ARM_isnanf( vel.y ) || __ARM_isnanf( vel.z ) || \
							__ARM_isinff( vel.x ) || __ARM_isinff( vel.y ) || __ARM_isinff( vel.z ) )
						data_effective = false;
					else
						data_effective = true;
					break;
					
				default:
					data_effective = false;
					break;
			}
			if( !data_effective )
				return false;
			
			//传感器数据正在更新
			sensor->publishing = true;
			
			if( available )
			{
				Time_set_inValid( &sensor->inavailable_start_time );
				
				sensor->position = position;
				sensor->velocity = vel;
			}
			else if( Time_isValid( sensor->inavailable_start_time ) == false )
				sensor->inavailable_start_time = get_TIME_now();
			
			if( sensor->available )
				sensor->sample_time = get_pass_time_st( &sensor->last_update_time );
			else
				sensor->sample_time = 0;
			
			sensor->available = available;
			if( delay > 0 )
				sensor->delay = delay;
			
			//传感器数据更新完成
			sensor->publishing = false;
			
			return true;
		}
		bool PositionSensorUpdateVel( unsigned char index , vector3_float vel , bool available , float delay )
		{
			if( index >= Position_Sensors_Count )
				return false;			
			Position_Sensor* sensor = &Position_Sensors[index];
			if( sensor->present == false )
				return false;
			if( sensor->publishing )
				return false;
			
			//判断传感器类型、数据是否正确
			bool data_effective;
			switch( sensor->sensor_DataType )
			{
				case Position_Sensor_DataType_v_xy:
					if( __ARM_isnanf( vel.x ) || __ARM_isnanf( vel.y ) || \
							__ARM_isinff( vel.x ) || __ARM_isinff( vel.y ) )
						data_effective = false;
					else
						data_effective = true;
					break;
					
				case Position_Sensor_DataType_v_z:
					if( __ARM_isnanf( vel.z ) || __ARM_isinff( vel.z ) )
						data_effective = false;
					else
						data_effective = true;
					
				case Position_Sensor_DataType_v_xyz:
					if( __ARM_isnanf( vel.x ) || __ARM_isnanf( vel.y ) || __ARM_isnanf( vel.z ) || \
							__ARM_isinff( vel.x ) || __ARM_isinff( vel.y ) || __ARM_isinff( vel.z ) )
						data_effective = false;
					else
						data_effective = true;
					break;
					
				default:
					data_effective = false;
					break;
			}
			if( !data_effective )
				return false;
			
			//传感器数据正在更新
			sensor->publishing = true;
			
			if( available )
			{
				Time_set_inValid( &sensor->inavailable_start_time );
				
				sensor->velocity = vel;
			}
			else if( Time_isValid( sensor->inavailable_start_time ) == false )
				sensor->inavailable_start_time = get_TIME_now();
			
			if( sensor->available )
				sensor->sample_time = get_pass_time_st( &sensor->last_update_time );
			else
				sensor->sample_time = 0;
			
			sensor->available = available;
			if( delay > 0 )
				sensor->delay = delay;
			
			//传感器数据更新完成
			sensor->publishing = false;
			
			return true;
		}
	/*位置传感器更新函数*/
		
	/*位置传感器读取函数*/
		const Position_Sensor* GetPositionSensor( unsigned char index )
		{
			if( index >= Position_Sensors_Count )
				return false;			
			return &Position_Sensors[index];
		}
	/*位置传感器读取函数*/	
	
/*位置传感器*/		