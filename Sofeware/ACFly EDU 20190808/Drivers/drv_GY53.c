#include "Basic.h"
#include "STS.h"
#include "drv_GY53.h"
#include "Sensors_Backend.h"
#include "MeasurementSystem.h"
#include "Sensors.h"

#include "TM4C123GH6PM.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "debug.h"
#include "fpu.h"
#include "gpio.h"
#include "pin_map.h"
#include "pwm.h"
#include "rom.h"
#include "sysctl.h"
#include "uart.h"
#include "interrupt.h"
#include "timer.h"
#include "hw_gpio.h"

static void GY53_Handler();
//static void Ultrasonic_Server( unsigned int Task_ID );
/*状态机*/
	//超声波测高状态
	//255：未开始
	//0：已发送开始信号
	//1：已记录第一个边沿
	//static uint8_t Ultrasoinc_counter = 255;
/*状态机*/

void init_drv_GY53()
{
	//使能Uart1引脚（Rx:PE0 Tx:PE1）
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	//使能UART1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
	
	//配置GPIO
	GPIOPinConfigure(GPIO_PE0_U7RX);
	GPIOPinConfigure(GPIO_PE1_U7TX);
	GPIOPinTypeUART(GPIOE_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO的UART模式配置
	
	//配置Uart
	//UART协议配置 波特率115200 8位 1停止位  无校验位	
	UARTConfigSetExpClk(UART7_BASE, SysCtlClockGet() , 115200,			
	                   (UART_CONFIG_WLEN_8 
										| UART_CONFIG_STOP_ONE 
										|	UART_CONFIG_PAR_NONE));	
	
	UARTFIFOEnable( UART7_BASE );
	UARTIntEnable(UART7_BASE,UART_INT_RX|UART_INT_RT);//使能UART0接收中断		
	UARTIntRegister(UART7_BASE,GY53_Handler);//UART中断地址注册	
	IntPrioritySet(INT_UART7, INT_PRIO_7);
	
	//注册传感器
	PositionSensorRegister( default_GY53_sensor_index , \
													Position_Sensor_Type_RangePositioning , \
													Position_Sensor_DataType_s_z , \
													Position_Sensor_frame_BodyHeading , \
													0.05f , \
													false );
}

float Primitive_Height = 0;//原始高度数据截取
static void GY53_Handler()
{
	uint32_t uart_err = UARTRxErrorGet( UART7_BASE );
	UARTIntClear( UART7_BASE , UART_INT_OE | UART_INT_RT );
	UARTRxErrorClear( UART7_BASE );
	
	//状态机接收数据
	while( ( UART7->FR & (1<<4) ) == false	)
	{
		//接收
		uint8_t rdata = UART7->DR & 0xff;
		
		static uint8_t vl53_sum=0;
		static uint8_t vl53_step = 0;//0，帧头 0x5A
																//1，帧头 0x5A
																//2，本帧数据类型 0x15
																//3, 数据量
																//4，数据高8位
																//5，数据低8位
																//6，模块测量模式
																//7，校验和
		static uint8_t vl53_buf[2] = {0};
		static uint8_t vl53_byte6=0;
		
		
		
		switch(vl53_step)
		{
			case 0:
			{
				vl53_buf[0] = 0;
				vl53_buf[1] = 0;
				vl53_sum = 0;
				vl53_byte6 = 0;
				if(rdata == 0x5a)
				{
					vl53_sum += rdata;
					vl53_step++;
				}	
				else
					vl53_step = 0;
			}break;
				
			case 1:
			{
				if(rdata == 0x5a)
				{
					vl53_sum += rdata;
					vl53_step++;
				}	
				else
					vl53_step = 0;
			}break;
				
			case 2:
			{
				if(rdata == 0x15)
				{
					vl53_sum += rdata;
					vl53_step++;
				}	
				else
					vl53_step = 0;
			}break;
				
			case 3:
			{
				if(rdata == 0x03)
				{
					vl53_sum += rdata;
					vl53_step++;
				}
				else
					vl53_step = 0;
			}break;
			
			case 4:
				vl53_buf[0] = rdata;
				vl53_sum += rdata;
				vl53_step++;
				break;
			
			case 5:
			{
				vl53_buf[1] = rdata;
				vl53_sum += rdata;
				++ vl53_step;
			}break;
			
			case 6:
			{
				vl53_byte6 = rdata;
				vl53_sum += rdata;
				vl53_step++;
			}break;
			
			case 7:
			{
				vector3_float vl53_position;
				if(vl53_sum == rdata)
				{
					if(((vl53_byte6 >> 4) & 0x0f) == 0)//距离可靠
					{
						vl53_position.z = ((vl53_buf[0]<<8) | vl53_buf[1]) / 10.0f;//单位厘米
						if(vl53_position.z > 3 && vl53_position.z < 300)//中距离
						{
							float lean_cosin = get_lean_angle_cosin();
							vl53_position.z *= lean_cosin;
							
							Primitive_Height = vl53_position.z;//截取乘上余弦角的原始高度数据，用来矫正起飞高度不稳的情况
							
							//vl53_position.z = 340 - vl53_position.z;//对天距离进行融合，340为天花板到地距离340cm
							
							PositionSensorUpdatePosition( default_GY53_sensor_index , vl53_position , true , -1 );
						}
					}
					else
						PositionSensorSetInavailable( default_GY53_sensor_index );
				}
				else
					PositionSensorSetInavailable( default_GY53_sensor_index );
				
				
				vl53_step = 0;
			}break;
			
				
			default:break;
		}
	}
}