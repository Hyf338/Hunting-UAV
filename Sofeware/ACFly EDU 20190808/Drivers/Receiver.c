#include "Receiver.h"

#include "AC_Math.h"
#include "Configurations.h"

static RC_Type current_Receiver = RC_Type_Sbus;
static Receiver Receivers[ Receivers_Count ];

//获取接收机
Receiver* get_Receiver_NC( RC_Type rc )
{
	return &Receivers[rc];
}

//更新接收机数据
void Receiver_Update( RC_Type _rc , bool connected , float raw_data[16] , uint8_t channel_count )
{	
	Receiver* rc = &Receivers[ _rc ];
	rc->present = true;
	rc->update_time = get_pass_time_st( &rc->last_update_time );
	rc->connected = rc->available = connected;
	if( connected )
	{
		memcpy( rc->raw_data , raw_data , channel_count*sizeof(float) );
		for( uint8_t k = channel_count ; k < 16 ; ++k )
			rc->raw_data[k] = 200;
	
		const uint8_t* channel_Reflection = Cfg_get_Channel_Reflection( _rc );
		const float* channels_Min_RC = Cfg_get_Channel_Min( _rc );
		const float* channels_RC_Scale = Cfg_get_Channel_Scale( _rc );
		for( unsigned char i = 0 ; i < 8 ; ++i )
		{
			unsigned char ch_reflection = channel_Reflection[i];

			if( ch_reflection < 16 && rc->raw_data[ ch_reflection ] < 120.0f )
			{
				rc->data[i] = constrain_range_float( ( rc->raw_data[ ch_reflection ] - channels_Min_RC[i] ) * channels_RC_Scale[i] , 100 , 0 );
			}
			else if( i < 6 )
			{
				rc->available = false;
				rc->data[i] = 200.0f;
			}
			else
				rc->data[i] = 200.0f;
		}
		
		//切换接收机
		if( Receivers[current_Receiver].available == false )
		{
			current_Receiver = _rc;
		}
	}
}

//获取指定的接收机
const Receiver* get_Receiver( RC_Type rc )
{
	return &Receivers[rc];
}
//获取当前使用的接收机
RC_Type get_current_Receiver_Type()
{
	return current_Receiver;
}
//获取当前使用的接收机
const Receiver* get_current_Receiver()
{
	return &Receivers[current_Receiver];
}