#pragma once

#include <stdint.h>

void Uart3_Send( const uint8_t* data , uint16_t length );
uint16_t read_Uart3( uint8_t* data , uint16_t length );
uint16_t Uart3_DataAvailable();

void init_drv_SDI();