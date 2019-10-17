#pragma once

void init_drv_USB();

//向端口写入数据
void write_UsbUart( const uint8_t* data , uint16_t length );

//读出端口数据
//返回值：实际读出的数据字节数
uint16_t read_UsbUart( uint8_t* data , uint16_t length );

//返回端口有多少字节待接收数据
uint16_t UsbUart_DataAvailable();