#pragma once

#include <stdint.h>

void init_drv_Uart2();

void Uart2_Send( const uint8_t* data , uint16_t length );
uint16_t read_UART2( uint8_t* data , uint16_t length );
uint16_t UART2_DataAvailable();