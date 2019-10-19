#include "Basic.h"
#include "drv_Uart2.h"

#include "Quaternion.h"
#include "MeasurementSystem.h"

#include "Commulink.h"
#include "STS.h"
#include "Sensors_Backend.h"
#include "RingBuf.h"

#include "TM4C123GH6PM.h"
#include "uart.h"
#include "sysctl.h"
#include "gpio.h"
#include "pin_map.h"
#include "interrupt.h"
#include "hw_ints.h"
#include "hw_gpio.h"
#include "Timer.h"
#include "udma.h"

//串口中断
static void UART2_Handler();

/*发送缓冲区*/
	#define TX_BUFFER_SIZE 200
	static uint8_t tx_buffer[TX_BUFFER_SIZE];
	static RingBuf_uint8_t Tx_RingBuf;
/*发送缓冲区*/

/*接收缓冲区*/
	#define RX_BUFFER_SIZE 24
	static uint8_t rx_buffer[RX_BUFFER_SIZE];
	static RingBuf_uint8_t Rx_RingBuf;
/*接收缓冲区*/

void init_drv_Uart2()
{
	//使能UART2引脚（Rx:PD6 Tx:PD7）
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	//使能UART2
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
	
	//PD7解锁
	GPIOD->LOCK = GPIO_LOCK_KEY;
	GPIOD->CR |= (1<<7);
	GPIOD->LOCK = 0;
	
	//配置GPIO
	GPIOPinConfigure(GPIO_PD6_U2RX);
	GPIOPinConfigure(GPIO_PD7_U2TX);
	GPIOPinTypeUART(GPIOD_BASE, GPIO_PIN_6 | GPIO_PIN_7);//GPIO的UART模式配置
		
	//配置Uart
	UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet() , 115200,			
	                   (UART_CONFIG_WLEN_8 
										| UART_CONFIG_STOP_ONE 
										|	UART_CONFIG_PAR_NONE));	
	UARTFIFOEnable( UART2_BASE );
	
	//初始化缓冲区
	RingBuf_uint8_t_init( &Tx_RingBuf , tx_buffer , TX_BUFFER_SIZE );
	RingBuf_uint8_t_init( &Rx_RingBuf , rx_buffer , RX_BUFFER_SIZE );
	
	//配置串口接收中断
	UARTIntEnable( UART2_BASE , UART_INT_RX | UART_INT_RT );
	UARTIntRegister( UART2_BASE , UART2_Handler );
	
	//配置DMA发送
	uDMAChannelControlSet( UDMA_PRI_SELECT | UDMA_CH13_UART2TX , \
		UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_1 );
	UARTDMAEnable( UART2_BASE , UART_DMA_TX );
	UARTIntRegister( UART2_BASE , UART2_Handler );	
	uDMAChannelAssign(UDMA_CH13_UART2TX  );	
	
	//打开中断
	IntPrioritySet( INT_UART2 , INT_PRIO_5 );
	IntEnable( INT_UART2 );
	
	//注册端口
	Port uart_port;
	uart_port.read = read_UART2;
	uart_port.DataAvailable = UART2_DataAvailable;
	uart_port.write = Uart2_Send;
	PortRegister( uart_port );
}




void Uart2_Send( const uint8_t* data , uint16_t length )
{
	IntDisable( INT_UART2 );
	
	//获取剩余的缓冲区空间
	int16_t buffer_space = RingBuf_uint8_t_get_Freesize( &Tx_RingBuf );
	//获取DMA中待发送的字节数
	int16_t DMA_Remain = uDMAChannelSizeGet( UDMA_CH13_UART2TX );
	
	//计算要发送的字节数
	int16_t max_send_count = buffer_space - DMA_Remain;
	if( max_send_count < 0 )
		max_send_count = 0;
	uint16_t send_count = ( length < max_send_count ) ? length : max_send_count;
	
	//将待发送字节压入缓冲区
	RingBuf_uint8_t_push_length( &Tx_RingBuf , data , send_count );
//	for( uint8_t i = 0 ; i < send_count ; ++i )
//		RingBuf_uint8_t_push( &Tx_RingBuf , data[i] );
	
	//获取DMA发送是否完成
	if( uDMAChannelIsEnabled( UDMA_CH13_UART2TX ) == false )
	{
		//DMA已完成
		//可以继续发送
		uint16_t length;
		uint8_t* p = RingBuf_uint8_t_pop_DMABuf( &Tx_RingBuf , &length );
		if( length )
		{
			uDMAChannelTransferSet( UDMA_PRI_SELECT | UDMA_CH13_UART2TX , \
				UDMA_MODE_BASIC , p , (void*)&UART2->DR , length );
			uDMAChannelEnable( UDMA_CH13_UART2TX );
		}
	}
	IntEnable( INT_UART2 );
}
static void UART2_Handler()
{
	UARTIntClear( UART2_BASE , UART_INT_OE | UART_INT_RT );
	UARTRxErrorClear( UART2_BASE );
	while( ( UART2->FR & (1<<4) ) == false	)
	{
		//接收
		uint8_t rdata = UART2->DR & 0xff;
		RingBuf_uint8_t_push( &Rx_RingBuf , rdata );
	}
	
	if( uDMAChannelIsEnabled( UDMA_CH13_UART2TX ) == false )
	{
		uint16_t length;
		uint8_t* p = RingBuf_uint8_t_pop_DMABuf( &Tx_RingBuf , &length );
		if( length )
		{
			uDMAChannelTransferSet( UDMA_PRI_SELECT | UDMA_CH13_UART2TX , \
				UDMA_MODE_BASIC , p , (void*)&UART2->DR , length );
			uDMAChannelEnable( UDMA_CH13_UART2TX );
		}
	}
}
uint16_t read_UART2( uint8_t* data , uint16_t length )
{
	IntDisable( INT_UART2 );
	uint8_t read_bytes = RingBuf_uint8_t_pop_length( &Rx_RingBuf , data , length );
	IntEnable( INT_UART2 );
	return read_bytes;
}
uint16_t UART2_DataAvailable()
{
	IntDisable( INT_UART2 );
	uint16_t bytes2read = RingBuf_uint8_t_get_Bytes2read( &Rx_RingBuf );
	IntEnable( INT_UART2 );
	return bytes2read;
}