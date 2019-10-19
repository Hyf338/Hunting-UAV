#pragma once

#include <stdint.h>

void Uart0_Send( const uint8_t* data , uint16_t length );
uint16_t read_Uart0( uint8_t* data , uint16_t length );
uint16_t Uart0_DataAvailable();

void init_drv_Uart0();