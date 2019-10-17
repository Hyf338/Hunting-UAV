#include "Basic.h"
#include "drv_Uart0.h"

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

#define Uart0_BUFFER_SIZE 64

//串口中断
static void UART0_Handler();

/*发送缓冲区*/
	static uint8_t Uart0_tx_buffer[Uart0_BUFFER_SIZE];
	static RingBuf_uint8_t Uart0_Tx_RingBuf;
/*发送缓冲区*/

/*接收缓冲区*/
	static uint8_t Uart0_rx_buffer[Uart0_BUFFER_SIZE];
	static RingBuf_uint8_t Uart0_Rx_RingBuf;
/*接收缓冲区*/

static bool Uart0_RCTrigger( unsigned int Task_ID );
static void Uart0_Server( unsigned int Task_ID );

static bool Uart0_TCTrigger( unsigned int Task_ID );
static void Uart0_Server_Send( unsigned int Task_ID );

void init_drv_Uart0()
{
	//使能Uart0引脚（Rx:PA0 Tx:PA1）
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	//使能UART0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	
	//配置GPIO
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIOA_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO的UART模式配置
		
	//配置Uart
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet() , 115200,			
	                   (UART_CONFIG_WLEN_8 
										| UART_CONFIG_STOP_ONE 
										|	UART_CONFIG_PAR_NONE));	
	
	//初始化缓冲区
	RingBuf_uint8_t_init( &Uart0_Tx_RingBuf , Uart0_tx_buffer , Uart0_BUFFER_SIZE );
	RingBuf_uint8_t_init( &Uart0_Rx_RingBuf , Uart0_rx_buffer , Uart0_BUFFER_SIZE );
	
	//配置串口接收中断
	UARTIntEnable( UART0_BASE , UART_INT_RX | UART_INT_RT);
	UARTIntRegister( UART0_BASE , UART0_Handler );
	
	//配置DMA发送
	uDMAChannelControlSet( UDMA_PRI_SELECT | UDMA_CH9_UART0TX , \
		UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_1 );
	UARTDMAEnable( UART0_BASE , UART_DMA_TX );
	UARTIntRegister( UART0_BASE , UART0_Handler );	
	uDMAChannelAssign(UDMA_CH9_UART0TX  );	
	
	//打开中断
	IntPrioritySet( INT_UART0 , INT_PRIO_7 );
	IntEnable( INT_UART0 );
	
	
//	//注册传感器
//	PositionSensorRegister( 3 , Position_Sensor_Type_RelativePositioning , Position_Sensor_DataType_s_xy , Position_Sensor_frame_ENU , 0.1f , false );
	//添加简单二次开发协议解析任务
	STS_Add_Task( STS_Task_Trigger_Mode_Custom , 0 , Uart0_RCTrigger , Uart0_Server );//接收任务添加，只有收到串口信息才会被触发
	STS_Add_Task( STS_Task_Trigger_Mode_Custom , 0 , Uart0_TCTrigger , Uart0_Server_Send );//发送任务添加，由 M35 控制（Uart0_mode_contrl）
}



uint8_t Uart0_mode_contrl = 0;//状态外部输出，控制多功能执行端
uint8_t Uart0_mode_flag = 0;//暂存Uart0_mode_contrl的值
static bool Uart0_TCTrigger( unsigned int Task_ID )
{//发送触发函数
	if( Uart0_mode_contrl != 0 )
	{
		Uart0_mode_flag = Uart0_mode_contrl;
		Uart0_mode_contrl = 0;//执行一次发送任务即关闭发送
		return true;
	}
	return false;
}


static void Uart0_Server_Send( unsigned int Task_ID )
{//发送服务函数
	static uint8_t tc_buf[7] = {0xAA,0x55,0,0,0,0,0};
	
	extern float Primitive_Height;//激光测距原始高度，结合角度计算
	
	uint16_t fly_Height = (uint16_t)Primitive_Height;
	
	if(Uart0_mode_flag > 0 && Uart0_mode_flag < 9)
	{
		tc_buf[2] = 0x1f;//模式》》》31模式 用于控制外扩板动作执行，
		tc_buf[3] = 1;//数据长度位
		tc_buf[4] = Uart0_mode_flag;
		tc_buf[5] = tc_buf[0] + tc_buf[1] + tc_buf[2] + tc_buf[3] + tc_buf[4];
		Uart0_Send(tc_buf,6);//发送
		Uart0_mode_flag = 0;
	}
	else if(Uart0_mode_flag == 33)
	{
		tc_buf[2] = 0x21;//模式》》》33 模式 用于将目前无人机的高度信息发送给外扩板
		tc_buf[3] = 2;//数据长度位
		tc_buf[4] = fly_Height >> 8;
		tc_buf[5] = fly_Height;
		tc_buf[6] = tc_buf[0] + tc_buf[1] + tc_buf[2] + tc_buf[3] + tc_buf[4] + tc_buf[5];
		Uart0_Send(tc_buf,7);//发送
		Uart0_mode_flag = 0;
	}
}


uint16_t UWB_data = 0;//UWB 的距离
uint16_t uart0_msg_pack[31] = {0};
uint8_t uart0_num = 0;
static void Uart0_TX_Server(uint8_t mode)//用于查询变量专用
{
	uint8_t tc_buf[8] = {0};
	uint8_t tc_cnt = 0;
	uint8_t sum = 0;
	uint16_t tc_data_buf = 0;
	
	tc_buf[0] = 0xaa;//包头1
	tc_buf[1] = 0x55;//包头2
	
	tc_buf[2] = mode;//模式位
	
	switch(mode)
	{
		case 1:
		{//返巡线模式相关参数
			tc_buf[3] = 3;//数据长度
			for(tc_cnt = 0;tc_cnt < 10;tc_cnt ++)
			{
				tc_buf[4] = tc_cnt + 1;//参数序号
				
				tc_data_buf = uart0_msg_pack[1+tc_cnt];//读取参数值	从uart0_msg_pack[1]开始
				
				tc_buf[5] = tc_data_buf >> 8;//发送高八位
				tc_buf[6] = tc_data_buf;//发送低八位
				
				sum = tc_buf[0] + tc_buf[1] + tc_buf[2] + tc_buf[3] + tc_buf[4] + tc_buf[5] + tc_buf[6];
				tc_buf[7] = sum;
				
				Uart0_Send(tc_buf,8);//发送
			}
		}break;
		
		case 2:
		{//返回寻色块模式相关参数
			tc_buf[3] = 3;//数据长度
			for(tc_cnt = 0;tc_cnt < 10;tc_cnt ++)
			{
				tc_buf[3] = 3;//数据长度
				for(tc_cnt = 0;tc_cnt < 10;tc_cnt ++)
				{
					tc_buf[4] = tc_cnt + 1;//参数序号
					
					tc_data_buf = uart0_msg_pack[11+tc_cnt];//读取参数值	从uart0_msg_pack[11]开始
					
					tc_buf[5] = tc_data_buf >> 8;//发送高八位
					tc_buf[6] = tc_data_buf;//发送低八位
				
					sum = tc_buf[0] + tc_buf[1] + tc_buf[2] + tc_buf[3] + tc_buf[4] + tc_buf[5] + tc_buf[6];
					tc_buf[7] = sum;
				
					Uart0_Send(tc_buf,8);//发送
				}
			}
		}break;
		
		case 3:
		{//返回穿框模式相关参数
			for(tc_cnt = 0;tc_cnt < 10;tc_cnt ++)
			{
				tc_buf[3] = 3;//数据长度
				for(tc_cnt = 0;tc_cnt < 10;tc_cnt ++)
				{
					tc_buf[4] = tc_cnt + 1;//参数序号
					
					tc_data_buf = uart0_msg_pack[21+tc_cnt];//读取参数值	从uart0_msg_pack[21]开始
					
					tc_buf[5] = tc_data_buf >> 8;//发送高八位
					tc_buf[6] = tc_data_buf;//发送低八位
				
					sum = tc_buf[0] + tc_buf[1] + tc_buf[2] + tc_buf[3] + tc_buf[4] + tc_buf[5] + tc_buf[6];
					tc_buf[7] = sum;
				
					Uart0_Send(tc_buf,8);//发送
				}
			}
		}break;
		default:;
	}
}


static bool Uart0_RCTrigger( unsigned int Task_ID )
{//接收外扩板串口数据的触发函数
	if( Uart0_DataAvailable() )
		return true;
	return false;
}


static void Uart0_Server( unsigned int Task_ID )
{
	//远程调参协议解析
	
	/*状态机变量*/
		static uint8_t rc_step1 = 0;	//0：接收包头'0xAA' '0X55'
																	//1：接收1字节消息类别
																	//2：接收1字节消息长度
																	//3：接收数据包内容
																	//4：接收1字节校验
		static uint8_t rc_step2 = 0;
	
		#define MAX_SDI_PACKET_SIZE 30
		static uint8_t msg_type;//模式类型
		static uint8_t msg_nume;//参数序号
		static uint8_t msg_length;//数据长度
		static uint8_t msg_pack[MAX_SDI_PACKET_SIZE];//缓冲区
		static uint8_t sum;//校验和
		
		#define reset_SDI_RC ( rc_step1 = rc_step2 = 0 )//状态复位
	/*状态机变量*/
	
	
	uint8_t rc_buf[20];
	uint8_t length = read_Uart0( rc_buf , 20 );//获取串口数据
	for( uint8_t i = 0 ; i < length ; ++i )
	{
		uint8_t r_data = rc_buf[i];
		
		switch(rc_step1)
		{
			case 0:
			{
				reset_SDI_RC;//状态复位
				sum = 0;//校验和清零
				msg_type = 0;//模式类型清零
				msg_length = 0;//数据长度清零
				if(r_data == 0xAA)//校验包头1 》》0xaa
				{
					rc_step1 ++;
					sum += r_data;
				}
			}break;
			
			case 1:
			{
				if(r_data == 0x55)//校验包头2 》》 0x55
				{
					rc_step1 ++;
					sum += r_data;
				}
				else reset_SDI_RC;
			}break;
			
			case 2:
			{
				if(r_data <= 255 && r_data > 0)//只允许 1~255 写入模式，9 为查询模式
				{
					msg_type = r_data;//缓冲模式类型
					rc_step1 ++;
					sum += r_data;
				}
				else reset_SDI_RC;
			}break;
			
			case 3:
			{
				if(r_data < 12 && r_data > 0)//限定数据接收长度
				{
					msg_length = r_data;//保存数据长度位
					
					rc_step1 ++;
					rc_step2 = 0;//准备接收，从msg_pack[0]开始
					sum += r_data;
				}
				else reset_SDI_RC;
			}break;
			
			case 4:
			{
				if(rc_step2 < msg_length)
				{
					msg_pack[ rc_step2 ++] = r_data;//缓冲区第0位开始存数据
					sum += r_data;
					if(rc_step2 == msg_length)
					{
						if((msg_type == 1) || (msg_type == 2) || (msg_type == 3))
						{
							rc_step1 = 10;//接收完毕，跳转到相应模式状态;
						}
						else if(msg_type == 9)//查询模式
						{
							rc_step1 = 11;
						}
						else if(msg_type == 33)//UWB数据
						{
							rc_step1 = 12;
						}
						else reset_SDI_RC;
					}
				}
			}break;
			
			
			
			
			case 10:
			{//巡线模式参数调整模式
				if(sum == r_data && msg_pack[0] < 11 && msg_pack[0] > 0)//校验和正确，参数序号在允许范围内，进行数据处理
				{
						uart0_msg_pack[0] = msg_type;//输出模式类型
						uart0_msg_pack[msg_pack[0] + (msg_type - 1)*10] = ((msg_pack[1]<<8) | msg_pack[2]);//输出相应参数序号有效数据
						uart0_num = msg_pack[0];//存参数序号
				}
				
				reset_SDI_RC;//重新来过
			}break; 
			
			
			
			case 11:
			{//查询变量模式
				if(sum == r_data)
				{
					Uart0_TX_Server(msg_pack[0]);//返回参数模式
				}
				reset_SDI_RC;//重新来过
			}break;
			
			
			
			case 12:
			{//UWB数据
				if(sum == r_data)
				{
					UWB_data = (msg_pack[0] << 8) | msg_pack[1];
				}
				reset_SDI_RC;//重新来过
			}break;
			
			default:;
		}
	}
	
}





void Uart0_Send( const uint8_t* data , uint16_t length )
{
	IntDisable( INT_UART0 );
	
	//获取剩余的缓冲区空间
	int16_t buffer_space = RingBuf_uint8_t_get_Freesize( &Uart0_Tx_RingBuf );
	//获取DMA中待发送的字节数
	int16_t DMA_Remain = uDMAChannelSizeGet( UDMA_CH9_UART0TX );
	
	//计算要发送的字节数
	int16_t max_send_count = buffer_space - DMA_Remain;
	if( max_send_count < 0 )
		max_send_count = 0;
	uint16_t send_count = ( length < max_send_count ) ? length : max_send_count;
	
	//将待发送字节压入缓冲区
	RingBuf_uint8_t_push_length( &Uart0_Tx_RingBuf , data , send_count );
//	for( uint8_t i = 0 ; i < send_count ; ++i )
//		RingBuf_uint8_t_push( &Uart0_Tx_RingBuf , data[i] );
	
	//获取DMA发送是否完成
	if( uDMAChannelIsEnabled( UDMA_CH9_UART0TX ) == false )
	{
		//DMA已完成
		//可以继续发送
		uint16_t length;
		uint8_t* p = RingBuf_uint8_t_pop_DMABuf( &Uart0_Tx_RingBuf , &length );
		if( length )
		{
			uDMAChannelTransferSet( UDMA_PRI_SELECT | UDMA_CH9_UART0TX , \
				UDMA_MODE_BASIC , p , (void*)&UART0->DR , length );
			uDMAChannelEnable( UDMA_CH9_UART0TX );
		}
	}
	IntEnable( INT_UART0 );
}
static void UART0_Handler()
{
	UARTIntClear( UART0_BASE , UART_INT_OE );
	UARTRxErrorClear( UART0_BASE );
	while( ( UART0->FR & (1<<4) ) == false	)
	{
		//接收
		uint8_t rdata = UART0->DR;
		RingBuf_uint8_t_push( &Uart0_Rx_RingBuf , rdata );
	}
	
	if( uDMAChannelIsEnabled( UDMA_CH9_UART0TX ) == false )
	{
		uint16_t length;
		uint8_t* p = RingBuf_uint8_t_pop_DMABuf( &Uart0_Tx_RingBuf , &length );
		if( length )
		{
			uDMAChannelTransferSet( UDMA_PRI_SELECT | UDMA_CH9_UART0TX , \
				UDMA_MODE_BASIC , p , (void*)&UART0->DR , length );
			uDMAChannelEnable( UDMA_CH9_UART0TX );
		}
	}
}
uint16_t read_Uart0( uint8_t* data , uint16_t length )
{
	IntDisable( INT_UART0 );
	uint8_t read_bytes = RingBuf_uint8_t_pop_length( &Uart0_Rx_RingBuf , data , length );
	IntEnable( INT_UART0 );
	return read_bytes;
}
uint16_t Uart0_DataAvailable()
{
	IntDisable( INT_UART0 );
	uint16_t bytes2read = RingBuf_uint8_t_get_Bytes2read( &Uart0_Rx_RingBuf );
	IntEnable( INT_UART0 );
	return bytes2read;
}