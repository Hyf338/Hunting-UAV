#include "Basic.h"
#include "drv_SDI.h"

#include "Quaternion.h"
#include "MeasurementSystem.h"

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

#define BUFFER_SIZE 64

//串口中断
static void UART3_Handler();

/*发送缓冲区*/
	static uint8_t tx_buffer[BUFFER_SIZE];
	static RingBuf_uint8_t Tx_RingBuf;
/*发送缓冲区*/

/*接收缓冲区*/
	static uint8_t rx_buffer[BUFFER_SIZE];
	static RingBuf_uint8_t Rx_RingBuf;
/*接收缓冲区*/

static bool SDI_RCTrigger( unsigned int Task_ID );
static void SDI_Server( unsigned int Task_ID );

static bool SDI_TXTrigger( unsigned int Task_ID );
static void SDI_Tx_model_contrl( unsigned int Task_ID );

void init_drv_SDI()
{
	//使能Uart3引脚（Rx:PC6 Tx:PC7）
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	//使能UART3
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
	
	//配置GPIO
	GPIOPinConfigure(GPIO_PC6_U3RX);
	GPIOPinConfigure(GPIO_PC7_U3TX);
	GPIOPinTypeUART(GPIOC_BASE, GPIO_PIN_6 | GPIO_PIN_7);//GPIO的UART模式配置
		
	//配置Uart
	UARTConfigSetExpClk(UART3_BASE, SysCtlClockGet() , 115200,			
	                   (UART_CONFIG_WLEN_8 
										| UART_CONFIG_STOP_ONE 
										|	UART_CONFIG_PAR_NONE));	
	
	//初始化缓冲区
	RingBuf_uint8_t_init( &Tx_RingBuf , tx_buffer , BUFFER_SIZE );
	RingBuf_uint8_t_init( &Rx_RingBuf , rx_buffer , BUFFER_SIZE );
	
	//配置串口接收中断
	UARTIntEnable( UART3_BASE , UART_INT_RX | UART_INT_RT );
	UARTIntRegister( UART3_BASE , UART3_Handler );
	
	//配置DMA发送
	uDMAChannelControlSet( UDMA_PRI_SELECT | UDMA_CH17_UART3TX , \
		UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_1 );
	UARTDMAEnable( UART3_BASE , UART_DMA_TX );
	UARTIntRegister( UART3_BASE , UART3_Handler );	
	uDMAChannelAssign(UDMA_CH17_UART3TX  );	
	
	//打开中断
	IntPrioritySet( INT_UART3 , INT_PRIO_5 );
	IntEnable( INT_UART3 );
	
	
//	//注册传感器
//	PositionSensorRegister(3 , Position_Sensor_Type_RelativePositioning , Position_Sensor_DataType_s_xy , Position_Sensor_frame_ENU , 0.1f , false );
	//添加简单二次开发协议解析任务
	STS_Add_Task( STS_Task_Trigger_Mode_Custom , 0 , SDI_RCTrigger , SDI_Server );//接收openmv串口数据任务
	STS_Add_Task( STS_Task_Trigger_Mode_Custom , 0 , SDI_TXTrigger , SDI_Tx_model_contrl );//发送给openmv串口数据任务
}


uint8_t SDI_mode_contrl = 0;//状态外部输出，控制openmv识别模式  由M35 控制
uint8_t SDI_tc_mode = 0;//暂存 SDI_mode_contrl
static bool SDI_TXTrigger( unsigned int Task_ID )
{
	if( SDI_mode_contrl != 0 )
	{
		SDI_tc_mode = SDI_mode_contrl;
		SDI_mode_contrl = 0;//执行一次发送任务即关闭发送
		return true;
	}
	return false;
}

static void SDI_Tx_model_contrl( unsigned int Task_ID )
{//openmv工作模式控制
	uint8_t tc_buf[8] = {0};
	uint8_t tc_cnt = 0;
	uint8_t sum = 0;
	uint16_t tc_data_buf = 0;
	
	tc_buf[0] = 0xaa;//包头1
	tc_buf[1] = 0x55;//包头2
	
	tc_buf[2] = 0x13;//模式位
	
	switch(SDI_tc_mode)
	{//openmv指令控制
		case 0xff:
		{
			tc_buf[3] = 1;//数据长度
			tc_buf[4] = 0xff;//不识别模式
		}break;
		
		case 1:
		{
			tc_buf[3] = 1;//数据长度
			tc_buf[4] = 1;//巡线模式
		}break;
		
		case 2:
		{
			tc_buf[3] = 1;//数据长度
			tc_buf[4] = 2;//色块模式
		}break;
		default:;
	}
	tc_buf[5] = tc_buf[0] + tc_buf[1] + tc_buf[2] + tc_buf[3] + tc_buf[4];
	Uart3_Send(tc_buf,6);
}




static bool SDI_RCTrigger( unsigned int Task_ID )
{
	if( Uart3_DataAvailable() )
		return true;
	return false;
}


uint8_t SDI_area_central[11] = {0};//前面10位分别为1~5区的xy坐标，最后一位为3区的色块宽度
uint8_t SDI_blob_central[3] = {0};//第0位表示有没有找到色块   第1为x坐标   第2为y坐标   
static void SDI_Server ( unsigned int Task_ID )
{
	//简单二次开发协议解析
	
	/*状态机变量*/
		static uint8_t rc_step1 = 0;	//0：接收包头'A' 'C'
																	//1：接收1字节消息类别
																	//2：接收1字节消息长度
																	//3：接收数据包内容
																	//4：接收2字节校验
		static uint8_t rc_step2 = 0;
	
		#define MAX_SDI_PACKET_SIZE 11
		static uint8_t msg_type;
		static uint8_t msg_length;
		ALIGN4 static uint8_t msg_pack[MAX_SDI_PACKET_SIZE];
		static uint8_t sumB;
		
		#define reset_SDI_RC ( rc_step1 = rc_step2 = 0 )
	/*状态机变量*/
	
	uint8_t rc_buf[20];
	uint8_t length = read_Uart3( rc_buf , 20 );
	for( uint8_t i = 0 ; i < length ; ++i )
	{
		uint8_t r_data = rc_buf[i];
		
		switch( rc_step1 )
		{
			case 0 :
				//接收包头AA55
				sumB=0;
				if(r_data == 0xAA)
				{
					sumB += r_data;
					rc_step1 = 1;
				}
				break;
			case 1:
				if(r_data == 0x55)
				{
					sumB += r_data;
					rc_step1 = 2;
				}
				else reset_SDI_RC;
				break;
			case 2:
				//接收消息类别
				if(r_data > 0x10 && r_data < 0x20)//openmv模式
				{
					msg_type = r_data;
					sumB += r_data;
					rc_step1 = 3;
					rc_step2 = 0;
				}
				else reset_SDI_RC;
				break;
			
			case 3:
				//接收消息长度
				if( r_data > MAX_SDI_PACKET_SIZE )
				{
					reset_SDI_RC;
					break;
				}
				msg_length = r_data;
				sumB += r_data;
				rc_step1 = 4;
				rc_step2 = 0;
				break;
				
			case 4:
				//接收数据包
				msg_pack[ rc_step2 ++] = r_data;
				sumB += r_data;
				if( rc_step2 >= msg_length )
				{
					rc_step1 = 5;
					rc_step2 = 0;
				}
				break;
				
			case 5:
				//接收校验位
				if(sumB == r_data)
				{
					if(msg_type == 0x12)//巡线模式数据处理
					{
						for(rc_step2 = 0;rc_step2 < 11;rc_step2++)
						{
							SDI_area_central[rc_step2] = msg_pack[ rc_step2 ];
						}
					}
					
					else if(msg_type == 0x11)//巡色块数据处理
					{
						SDI_blob_central[0] = msg_pack[6];
						SDI_blob_central[1] = msg_pack[4] - 80;
						SDI_blob_central[2] = 60 - msg_pack[5];
					}
				}
				
				reset_SDI_RC;		
					
				break;
		}
	}
}





void Uart3_Send( const uint8_t* data , uint16_t length )
{
	IntDisable( INT_UART3 );
	
	//获取剩余的缓冲区空间
	int16_t buffer_space = RingBuf_uint8_t_get_Freesize( &Tx_RingBuf );
	//获取DMA中待发送的字节数
	int16_t DMA_Remain = uDMAChannelSizeGet( UDMA_CH17_UART3TX );
	
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
	if( uDMAChannelIsEnabled( UDMA_CH17_UART3TX ) == false )
	{
		//DMA已完成
		//可以继续发送
		uint16_t length;
		uint8_t* p = RingBuf_uint8_t_pop_DMABuf( &Tx_RingBuf , &length );
		if( length )
		{
			uDMAChannelTransferSet( UDMA_PRI_SELECT | UDMA_CH17_UART3TX , \
				UDMA_MODE_BASIC , p , (void*)&UART3->DR , length );
			uDMAChannelEnable( UDMA_CH17_UART3TX );
		}
	}
	IntEnable( INT_UART3 );
}
static void UART3_Handler()
{
	UARTIntClear( UART3_BASE , UART_INT_OE );
	UARTRxErrorClear( UART3_BASE );
	while( ( UART3->FR & (1<<4) ) == false	)
	{
		//接收
		uint8_t rdata = UART3->DR;
		RingBuf_uint8_t_push( &Rx_RingBuf , rdata );
	}
	
	if( uDMAChannelIsEnabled( UDMA_CH17_UART3TX ) == false )
	{
		uint16_t length;
		uint8_t* p = RingBuf_uint8_t_pop_DMABuf( &Tx_RingBuf , &length );
		if( length )
		{
			uDMAChannelTransferSet( UDMA_PRI_SELECT | UDMA_CH17_UART3TX , \
				UDMA_MODE_BASIC , p , (void*)&UART3->DR , length );
			uDMAChannelEnable( UDMA_CH17_UART3TX );
		}
	}
}
uint16_t read_Uart3( uint8_t* data , uint16_t length )
{
	IntDisable( INT_UART3 );
	uint8_t read_bytes = RingBuf_uint8_t_pop_length( &Rx_RingBuf , data , length );
	IntEnable( INT_UART3 );
	return read_bytes;
}
uint16_t Uart3_DataAvailable()
{
	IntDisable( INT_UART3 );
	uint16_t bytes2read = RingBuf_uint8_t_get_Bytes2read( &Rx_RingBuf );
	IntEnable( INT_UART3 );
	return bytes2read;
}