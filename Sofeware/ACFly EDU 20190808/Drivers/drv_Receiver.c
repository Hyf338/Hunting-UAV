#include "Basic.h"
#include "drv_Receiver.h"
#include "Receiver_Backend.h"
#include "STS.h"
#include "Configurations.h"
#include "AC_Math.h"

#include "TM4C123GH6PM.h"
#include "uart.h"
#include "sysctl.h"
#include "gpio.h"
#include "pin_map.h"
#include "interrupt.h"
#include "hw_ints.h"
#include "hw_gpio.h"
#include "Timer.h"

static void Sbus_Receive_Handler();
static void PPM_Receive_Handler();
static void RC_Check_Server( unsigned int Task_ID );

void init_drv_Receiver()
{
	/*Sbus*/
		//开启Sbus串口Uart4时钟
		SysCtlPeripheralEnable( SYSCTL_PERIPH_UART4 );
		
		//开启Sbus串口Uart4引脚(PC4)
		SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOC );
		GPIOPinTypeUART( GPIOC_BASE , GPIO_PIN_4 );
		GPIOPinConfigure( GPIO_PC4_U4RX );
		
		//配置Sbus串口(Uart4)
		UARTConfigSetExpClk( UART4_BASE , SysCtlClockGet() , 100e3 , /*100khz波特率*/\
			UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_TWO | UART_CONFIG_PAR_EVEN );	//8位数据位 2位停止位 偶校验
		UARTFIFOLevelSet( UART4_BASE , UART_FIFO_TX1_8 , UART_FIFO_RX1_8 );
		
		//开启Sbus串口中断
		UARTFIFOEnable( UART4_BASE );
		UARTEnable( UART4_BASE );
		UARTIntRegister( UART4_BASE , Sbus_Receive_Handler );
		IntPrioritySet( INT_UART4 , INT_PRIO_6 );
		UARTIntEnable( UART4_BASE , UART_INT_RX | UART_INT_RT | UART_INT_OE );
	/*Sbus*/
	
	/*PPM*/
		//打开GPIOC电源（PC3 PPM捕获）
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

		//GPIOC3解锁
		GPIOC->LOCK = GPIO_LOCK_KEY;
		GPIOC->CR |= 0x8;
		GPIOC->LOCK = 0;
		
		//GPIOC配置为定时器捕获模式
		GPIOPinTypeTimer(GPIOC_BASE, GPIO_PIN_3);
		GPIOPinConfigure(GPIO_PC3_T5CCP1);
		
		//开启定时器5(T5CCP1)
		SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
		
		//配置定时器5B为捕获上升沿
		TimerConfigure( TIMER5_BASE ,TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_CAP_TIME_UP ); 
		TimerControlEvent(TIMER5_BASE,TIMER_B,TIMER_EVENT_POS_EDGE);	
		TimerLoadSet( TIMER5_BASE , TIMER_B , 0xffff );
		TimerPrescaleSet( TIMER5_BASE , TIMER_B , 0xff );

		//开启定时器中断
		TimerIntRegister(TIMER5_BASE,  TIMER_B , PPM_Receive_Handler);	
		IntPrioritySet( INT_TIMER5B , INT_PRIO_6);
		TimerIntEnable( TIMER5_BASE , TIMER_CAPB_EVENT);
		TimerEnable( TIMER5_BASE, TIMER_B );
		IntEnable( INT_TIMER5B );
	/*PPM*/
	
	//设置初始的接收机配置
	for( uint8_t i = 0 ; i < Receivers_Count ; ++i )
	{
		uint8_t buf[8];
		for( uint8_t k = 0 ; k < 8 ; ++k )
			buf[k] = k;
		Cfg_set_initial_Channel_Reflection( i , buf );
		
		float buf_f[8];
		for( uint8_t k = 0 ; k < 8 ; ++k )
			buf_f[k] = 0;
		Cfg_set_initial_Channel_Min( i , buf_f );
		
		for( uint8_t k = 0 ; k < 8 ; ++k )
			buf_f[k] = 1;
		Cfg_set_initial_Channel_Scale( i , buf_f );
	}
	
	//添加接收机检测任务
	//用于检测接收机是否断开
	STS_Add_Task( STS_Task_Trigger_Mode_RoughTime , 0.05f , 0 , RC_Check_Server );
}

static void RC_Check_Server( unsigned int Task_ID )
{
	for( uint8_t i = 0 ; i < Receivers_Count ; ++i )
	{
		Receiver* rc = get_Receiver_NC( i );
		if( rc->present )
		{
			float rc_no_data_time = get_pass_time( rc->last_update_time );
			if( rc_no_data_time > 0.1f )
			{
				//接收机断开
				rc->present = rc->connected = rc->available = false;
			}
		}
	}
}

static void Sbus_Receive_Handler()
{
	//清除中断	
	uint32_t sbus_err = UARTRxErrorGet( UART4_BASE );
	UARTIntClear( UART4_BASE , UART_INT_OE | UART_INT_RT );
	UARTRxErrorClear( UART4_BASE );
	
	//上一次接收时间
	static TIME last_Receive_Time;
	float receive_time_gap = get_pass_time_st( &last_Receive_Time );
	
	//状态机计数器
	//0：等待包头
	//1-22：接收数据data1-data22
	//23：flag
	//24：包尾
	static uint8_t SBUS_RC_State = 0;
	static float rc_buf[16];
	static unsigned char current_rc_channel = 0;
	static unsigned char rc_bit_count = 0;
	static unsigned short current_rc = 0;
	static bool failsafe;
	
	#define SBUS_RC_Reset (SBUS_RC_State = current_rc_channel = rc_bit_count = current_rc = 0)
	
	//如果距离上次接收时间很长
	//重置状态机
	if( receive_time_gap > 2e-3f || sbus_err )
		SBUS_RC_Reset;
	
	//状态机接收数据
	while( ( UART4->FR & (1<<4) ) == false	)
	{
		//接收
		uint8_t rdata = UART4->DR & 0xff;
		
		if( SBUS_RC_State == 0 )
		{
			//接收包头
			if( rdata == 0x0f )
				++SBUS_RC_State;
		}
		else if( SBUS_RC_State <= 22 )
		{
			//数据
			rc_bit_count += 8;
			if( rc_bit_count >= 11 )
			{
				rc_bit_count -= 11;
				unsigned char l_byte_count = 8 - rc_bit_count;
				rc_buf[ current_rc_channel++ ] = 0.04885197850512945774303859306302f *  (float)( current_rc | ( ((unsigned char)( rdata << rc_bit_count )) << 3 ) );
				current_rc = (unsigned char)( rdata >> l_byte_count );
			}
			else
				current_rc |= rdata << (rc_bit_count - 8);	

			++SBUS_RC_State;
		}
		else if( SBUS_RC_State == 23 )
		{
			//判断是否失控
			#define SBUS_FAILSAFE_BIT	3
			#define SBUS_FRAMELOST_BIT	2
			failsafe = rdata & (1<<SBUS_FAILSAFE_BIT);
			++SBUS_RC_State;
		}
		else
		{
			//判断包尾
			if( rdata == 0 )
			{			
				Receiver_Update( RC_Type_Sbus , !failsafe , rc_buf , 16 );			
				SBUS_RC_Reset;
			}
		}
	}
}

static void PPM_Receive_Handler()
{
	TimerIntClear( TIMER5_BASE , TIMER_CAPB_EVENT );
	
	static uint32_t last_value = 0;
	uint32_t trig_value = TimerValueGet( TIMER5_BASE , TIMER_B );
	
	float t;
	if( trig_value > last_value )
		t = ( trig_value - last_value ) * ( 1e-6f / 80 );
	else
		t = ( trig_value + 0xffffff - last_value ) * ( 1e-6f / 80 );
	
	static float rc_buf[8];
	/*状态机*/
		static uint8_t current_RC_channel = 255;
		#define RC_channel_valid ( current_RC_channel != 255 )
		#define set_RC_channel_invalid ( current_RC_channel = 255 )
		#define reset_RC_channel ( current_RC_channel = 0 )
		#define is_Synchro_Blank( channel ) ( in_range_float( channel , 1000.0f , 150.0f ) )	//是否同步块
	/*状态机*/
	
	float new_channel = t*100000 - 100;
		
	//判断通道是否有效
	if( is_Synchro_Blank( new_channel ) )
	{
		if( current_RC_channel >= 6 && current_RC_channel <= 8 )
		{
			//捕获完成
			Receiver_Update( RC_Type_PPM , true , rc_buf , current_RC_channel );
		}
		reset_RC_channel;
	}
	else if( new_channel < -10 || new_channel > 110 )
		set_RC_channel_invalid;
	else
	{
	
		if( current_RC_channel >= 0 && current_RC_channel < 8 )
		{
			rc_buf[ current_RC_channel ] = new_channel;
			++current_RC_channel;
		}
		
	}
	
	last_value = trig_value;
}