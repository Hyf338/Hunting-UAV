#include "DebugComm.h"
#include "STS.h"

#include "Sensors.h"
#include "drv_USB.h"
#include "MeasurementSystem.h"
#include "drv_Uart2.h"
#include "mavlink.h"
#include "Commulink.h"

static void debug_server( unsigned int Task_ID );
void init_Debug()
{
	STS_Add_Task( STS_Task_Trigger_Mode_RoughTime , 0.05f , 0 , debug_server );
}

float debug_test[10];

static void debug_server( unsigned int Task_ID )
{
	mavlink_message_t msg_sd;
	
	uint8_t port_id = 0;
	const Port* port = get_Port( port_id );
	
	if( port->write )
	{
		float Yaw = Quaternion_getYaw( get_Airframe_attitude() );
		float Yaw_sin , Yaw_cos;
		arm_sin_cos_f32( rad2degree(Yaw) , &Yaw_sin , &Yaw_cos );
		
		float ultra_height;
		bool ultra_height_available = get_Estimated_Sensor_Position_z( &ultra_height , default_ultrasonic_sensor_index );
		mavlink_msg_debug_vect_pack_chan( 
			1 ,	//system id
			MAV_COMP_ID_AUTOPILOT1 ,	//component id
			port_id , 	//chan
			&msg_sd ,
			"JiTiSuDu" ,	//name
			get_System_Run_Time() * 1e6 , 	//boot ms
			getBatteryVoltage() ,
			debug_test[6] ,
			GetPositionSensor( internal_baro_sensor_index )->position.z * 0.01f - 50 );
		mavlink_msg_to_send_buffer(port->write, &msg_sd);
		
		mavlink_msg_debug_vect_pack_chan( 
			1 ,	//system id
			MAV_COMP_ID_AUTOPILOT1 ,	//component id
			port_id , 	//chan
			&msg_sd ,
			"GuangLiu" ,	//name
			get_System_Run_Time() * 1e6 , 	//boot ms
			debug_test[0] ,
			debug_test[1] ,
			GetPositionSensor(default_ultrasonic_sensor_index)->position.z*0.01f );
		mavlink_msg_to_send_buffer(port->write, &msg_sd);
		
		mavlink_msg_debug_vect_pack_chan( 
			1 ,	//system id
			MAV_COMP_ID_AUTOPILOT1 ,	//component id
			port_id , 	//chan
			&msg_sd ,
			"Mag" ,	//name
			get_System_Run_Time() * 1e6 , 	//boot ms
			GetMagnetometer(0)->data.x ,
			GetMagnetometer(0)->data.y ,
			GetMagnetometer(0)->data.z );
		mavlink_msg_to_send_buffer(port->write, &msg_sd);
	}
	
//	unsigned char checksum=0;
//	uint8_t buf[30];
//	int i = 0;
//	signed short temp;
//	
//		buf[i] = 0xaa;
//		checksum += buf[i++];
//		buf[i] = 0xaa;
//		checksum += buf[i++];
//		buf[i] = 0x02;
//		checksum += buf[i++];
//		buf[i] = 18;
//		checksum += buf[i++];
//		
//		temp = GetPositionSensor( default_optical_flow_index )->velocity.x;
//		buf[i] = temp >> 8;
//		checksum += buf[i++];
//		buf[i] = temp;
//		checksum += buf[i++];
//		
//		//temp = debug_test[5];	
//		float h = 0;
//		//get_Estimated_Sensor_Position_z( &h  , default_ultrasonic_sensor_index );
//		temp = GetPositionSensor( default_optical_flow_index )->velocity.y;
//		buf[i] = temp >> 8;
//		checksum += buf[i++];
//		buf[i] = temp;
//		checksum += buf[i++];
//		
//		temp = debug_test[6];
//		buf[i] = temp >> 8;
//		checksum += buf[i++];
//		buf[i] = temp;
//		checksum += buf[i++];
//		
//		temp = debug_test[7];
//		buf[i] = temp >> 8;
//		checksum += buf[i++];
//		buf[i] = temp;
//		checksum += buf[i++];
//		
//		temp = debug_test[8];
//		buf[i] = temp >> 8;
//		checksum += buf[i++];
//		buf[i] = temp;
//		checksum += buf[i++];
//		
//		//temp = debug_test[9];
//		temp = get_Position().x;
//		buf[i] = temp >> 8;
//		checksum += buf[i++];
//		buf[i] = temp;
//		checksum += buf[i++];
//		
//		temp = get_Position().y;
//		buf[i] = temp >> 8;
//		checksum += buf[i++];
//		buf[i] = temp;
//		checksum += buf[i++];
//		
//		float Yaw = Quaternion_getYaw( get_Airframe_attitude() );
//		float Yaw_sin , Yaw_cos;
//		arm_sin_cos_f32( rad2degree(Yaw) , &Yaw_sin , &Yaw_cos );
//		
//		//temp = get_VelocityENU().z;
//		temp = map_ENU2BodyHeading_x( get_VelocityENU().x , get_VelocityENU().y , Yaw_sin , Yaw_cos  );
//		buf[i] = temp >> 8;
//		checksum += buf[i++];
//		buf[i] = temp;
//		checksum += buf[i++];
//				
//		temp = map_ENU2BodyHeading_y( get_VelocityENU().x , get_VelocityENU().y , Yaw_sin , Yaw_cos  );
//		//temp = get_Position().z;
//		//temp = rad2degree( get_AngularRateCtrl().y );
//		buf[i] = temp >> 8;
//		checksum += buf[i++];
//		buf[i] = temp;
//		checksum += buf[i++];
//		
//		buf[i++] = checksum;
//		write_UsbUart( buf , i );
//		Uart2_Send( buf , i );
//	/*acc gyro mag*/
//	
//	i = 0;
//	checksum = 0;
//	buf[i] = 0xaa;
//	checksum += buf[i++];
//	buf[i] = 0xaa;
//	checksum += buf[i++];
//	buf[i] = 0x01;
//	checksum += buf[i++];
//	buf[i] = 6;
//	checksum += buf[i++];
//	
//	temp = debug_test[0]*5729.578f;
//	buf[i] = temp >> 8;
//	checksum += buf[i++];
//	buf[i] = temp;
//	checksum += buf[i++];
//	
//	temp = debug_test[1]*5729.578f;
//	buf[i] = temp >> 8;
//	checksum += buf[i++];
//	buf[i] = temp;
//	checksum += buf[i++];
//	
//	temp = debug_test[2]*5729.578f;
//	buf[i] = temp >> 8;
//	checksum += buf[i++];
//	buf[i] = temp;
//	checksum += buf[i++];
//	
//	buf[i++] = checksum;
//	write_UsbUart( buf , i );
//	Uart2_Send( buf , i );
}