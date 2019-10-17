#pragma once

#include <stdint.h>
#include <stdbool.h>

//端口定义
typedef struct
{
	//写端口函数
	void (*write)( const uint8_t* data , uint16_t length );
	//读端口函数
	uint16_t (*read)( uint8_t* data , uint16_t length );
	//获取端口待读字节数量函数
	uint16_t (*DataAvailable)();
}Port;

//注册端口用于协议通信
bool PortRegister( Port port );

//获取端口
const Port* get_Port( uint8_t port );

/*发送状态*/
	//发送参数列表
	void Send_Param_List();

	//设置发送消息
	//port_index:端口序号
	//Msg:消息序号
	//Rate:发送频率(hz) 0表示不发送
	//返回是否设置成功
	bool SetMsgRate( uint8_t port_index , uint16_t Msg , uint16_t Rate );
/*发送状态*/


void init_CommuLink();