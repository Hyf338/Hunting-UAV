#include "Basic.h"
#include "drv_OpticalFlow.h"
#include "Sensors_Backend.h"
#include "MeasurementSystem.h"

#include "TM4C123GH6PM.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "hw_types.h"
#include "hw_ints.h"
#include "debug.h"
#include "fpu.h"
#include "gpio.h"
#include "pin_map.h"
#include "sysctl.h"
#include "uart.h"
#include "interrupt.h"
#include "timer.h"
#include "hw_gpio.h"

static void OpticalFlow_Handler();

void init_drv_OpticalFlow()
{
	//使能Uart1引脚（Rx:PB0 Tx:PB1）
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	//使能UART1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	
	//配置GPIO
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	GPIOPinTypeUART(GPIOB_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO的UART模式配置
	
	//配置Uart
	//UART协议配置 波特率115200 8位 1停止位  无校验位	
	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet() , 115200,			
	                   (UART_CONFIG_WLEN_8 
										| UART_CONFIG_STOP_ONE 
										|	UART_CONFIG_PAR_NONE));	
	
	UARTIntEnable(UART1_BASE,UART_INT_RX);//使能UART0接收中断		
	UARTIntRegister(UART1_BASE,OpticalFlow_Handler);//UART中断地址注册	
	IntPrioritySet(INT_UART1, INT_PRIO_7);
	
	//注册传感器
	PositionSensorRegister( default_optical_flow_index , \
													Position_Sensor_Type_RelativePositioning , \
													Position_Sensor_DataType_v_xy , \
													Position_Sensor_frame_BodyHeading , \
													0.1f , \
													false );
}

typedef struct
{
	unsigned char  Reserved;
	unsigned char  Quality;
	signed char Delta_X;
	signed char Delta_Y;
	unsigned char  Reference;
	unsigned char  Lightness;
	signed char Delta_Z;
	signed char Sub_X;
	signed char Sub_Y;
	unsigned char  Timestamp[4];
	signed char  Checksum;
	unsigned char  Packet_footer[2];
}__PACKED _Flow;
static const unsigned char packet_ID[4] = { '#' , 'J' , 'B' , '#' };
static void OpticalFlow_Handler()
{
	uint32_t sbus_err = UARTRxErrorGet( UART1_BASE );
	UARTIntClear( UART1_BASE , UART_INT_OE );
	UARTRxErrorClear( UART1_BASE );
	
	//状态机接收数据
	while( ( UART1->FR & (1<<4) ) == false	)
	{
		//接收
		uint8_t rdata = UART1->DR;
		
		/*状态机*/
			static _Flow  Flow;
			static unsigned char rc_counter = 0;
			static signed char sum = 0;
		/*状态机*/
		
		if( rc_counter < 4 )
		{
			//接收包头
			if( rdata != packet_ID[ rc_counter ] )
				rc_counter = 0;
			else
			{
				++rc_counter;
				sum = 0;
			}
		}
		else if( rc_counter <= 17 )
		{
			( (unsigned char*)&Flow )[ rc_counter - 4 ] = rdata;
			sum += (signed char)rdata;
			++rc_counter;
		}
		else if( rc_counter == 18 )
		{
			if( sum != 0 || rdata != '\r' )
				rc_counter = 0;
			else
				++rc_counter;
		}
		else
		{
			if( rdata == '\n' )
			{
				vector3_float vel;
				float ultra_height;
				bool ultra_height_available = get_Estimated_Sensor_Position_z( &ultra_height , default_ultrasonic_sensor_index );
				if( ultra_height_available )
				{
					float ultra_deadband = ultra_height - 25;
          if( ultra_deadband < 0 )
						ultra_deadband = 0;
					float rotation_compensation_x = -constrain_float( rad2degree( get_AngularRateCtrl().y ) * 0.3f , 45 );
					float rotation_compensation_y = constrain_float( rad2degree( get_AngularRateCtrl().x ) * 0.3f , 45 );
					vel.x = ( (float)Flow.Delta_X - rotation_compensation_x ) * 2.0f * (1 + ultra_deadband/40) ;
					vel.y = ( (float)Flow.Delta_Y - rotation_compensation_y ) * 2.0f * (1 + ultra_deadband/40) ;
					PositionSensorUpdateVel( default_optical_flow_index , vel , true , -1 );
				}
				else
					PositionSensorSetInavailable( default_ultrasonic_sensor_index );
			}
			rc_counter = 0;
		}
	}
}