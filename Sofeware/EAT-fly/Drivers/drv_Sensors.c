#include "Basic.h"
#include "drv_Sensors.h"

#include "TM4C123GH6PM.h"
#include "I2C.h"
#include "sysctl.h"
#include "gpio.h"
#include "pin_map.h"
#include "interrupt.h"
#include "hw_ints.h"
#include "timer.h"

#include "AC_Math.h"
#include "Configurations.h"
#include "Sensors_Backend.h"
#include "MS_main.h"
#include "Ctrl_main.h"

/*i2c静态读写函数*/
	static void i2c0WriteData(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length);
	static void i2c0ReadData(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length) ;
/*i2c静态读写函数*/

/*i2c0 内部传感器状态机*/
	//启动i2c接收状态机进行一次接收
	static void i2c_read_start(bool read_spl06 , bool read_ist8310);
	//周期读取定时器中断函数
	static void Timer2A_IRQHandler();
	 
	//状态机变量
	static i2c_sensor i2c_current_read_sensor = i2c_sensor_null;
	static uint8_t i2c_read_counter = 0;
	static bool read_icm20602 = false;
	static bool read_spl06 = false;
	static bool read_ist8310 = false;
	static bool read_spl06_nexttime = false;
	static bool read_ist8310_nexttime = false;

	//状态机接收中断
	static void I2C0_Transfered_Handler();
	
	//解算控制任务中断
	static void MainMCHandler();
/*i2c0 内部传感器状态机*/

/*外置罗盘定义*/
	typedef struct
	{
		//iic地址
		unsigned char device_address;
		//数据寄存器地址
		unsigned char data_address;
		
		//是否高位在前
		bool MSB;
		//是否需要手动开始采样
		bool need_set_sample;
		//采样寄存器地址
		unsigned char sample_address;
		//采样寄存器值
		unsigned char sample_cfg;
		
		//ID寄存器地址
		unsigned char ID_address;
		//ID
		unsigned char ID;
		
		//配置寄存器地址
		unsigned char configurations_address;
		//配置数目
		unsigned char configuration_count;
		//配置
		unsigned char configurations[10];
		//灵敏度（数据->Gauss）
		float sensitivity;
		
		//采样率（多少ms一次数据）
		unsigned char sample_mseconds;
		
		//轴向顺序
		//从1开始，负数为反向
		//如：1,-3,2代表轴向为 x , -z , y
		signed char axis_index[3];
	}External_MagnetoMeter;
	
	
	static const External_MagnetoMeter External_MagnetoMeters[] = 
	{
	/*  iic地址, 数据寄存器地址 , 高位在前, 手动采样 , 采样寄存器地址, 采样设置, ID 地址 , ID   , 配置寄存器地址 , 配置数目,  配置                                                     ,  灵敏度      , 采样率毫秒  , 轴向                 ,  */
		{ 0x1e   ,      3         ,    true ,   false  ,            0  ,      0  ,      0  , 0x10 ,       0        , 3       , { (1<<7) | (0<<5) | (7<<2) | (0<<0) , (7<<5) , 0 }        , 0.00435f     , 5           , { -3 , -1 , -2 }   } , //HMC5983
		{ 0x1e   ,      3         ,    true ,   false  ,            0  ,      0  ,      0  , 0xf0 ,       0        , 3       , { (0<<7) | (0<<5) | (6<<2) | (0<<0) , (7<<5) , 0 }        , 0.00435f     , 20          , { -3 , -1 , -2 }   } , //HMC5883
		{ 0x0e   ,      1         ,    true ,   false  ,            0  ,      0  ,      7  , 0xc4 ,       0x10     , 1       , { (0<<5) | (0<<3) | (0<<2) | (0<<1) | (1<<0) }            , 0.001f       , 10          , { -2 , -1 , 3 }    } , //Mag3110
		{ 0x0e   ,      3         ,   false ,    true  ,          0xa  ,      1  ,      0  , 0x10 ,       0x41     , 2       , { 0x24 , 0xc0 }                                                , 0.003f       , 20          , { -1 , -2 , -3 }    } , //IST8310
	};
	static const uint8_t Supported_External_MagnetoMeter_Count = sizeof( External_MagnetoMeters ) / sizeof( External_MagnetoMeter );
/*外置罗盘定义*/
	
/*外置罗盘状态机*/
	static signed char Current_External_Magnetometer = -1;
	static signed char Current_Scan_External_Magnetometer = 0;
	typedef enum
	{
		External_Magnetometer_Operation_Scan ,
		External_Magnetometer_Operation_SetConfig ,
		External_Magnetometer_Operation_CheckConfig ,
		
		External_Magnetometer_Operation_ReadData ,
	}External_Magnetometer_Operation;
	static External_Magnetometer_Operation current_External_Magnetometer_Operation;
	static uint8_t External_Magnetometer_Operation_Counter = 0;
	
	//状态机接收中断
	static void I2C1_Transfered_Handler();
/*外置罗盘状态机*/
	
#pragma region Sensor_Inf
	#define icm20602_Addr 0x69
	
	#define IST8310_Addr 0x0E
	
	#define SPL06_Addr 0x77 //SDO LOW
	typedef struct
	{
		int16_t c0;	int16_t c1;
		int32_t c00;	int32_t c10;	int16_t c01;	int16_t c11;
		int16_t c20;	int16_t c21;	
		int16_t c30;
		float KP;	float KT;
	}SPL06_COEFFICIENTS;
	static SPL06_COEFFICIENTS SPL06_coefficients;
	static void spl06_pressure_rateset( uint8_t u8SmplRate, uint8_t u8OverSmpl)
	{
		uint8_t reg = 0;
		switch(u8SmplRate)
		{
		case 2:
			reg |= (1<<4);
			break;
		case 4:
			reg |= (2<<4);
			break;
		case 8:
			reg |= (3<<4);
			break;
		case 16:
			reg |= (4<<4);
			break;
		case 32:
			reg |= (5<<4);
			break;
		case 64:
			reg |= (6<<4);
			break;
		case 128:
			reg |= (7<<4);
			break;
		case 1:
		default:
			break;
		}
		switch(u8OverSmpl)
		{
			case 2:
				reg |= 1;
				SPL06_coefficients.KP = 1.0f / 1572864;
				break;
			case 4:
				reg |= 2;
				SPL06_coefficients.KP = 1.0f / 3670016;
				break;
			case 8:
				reg |= 3;
				SPL06_coefficients.KP = 1.0f / 7864320;
				break;
			case 16:
				SPL06_coefficients.KP = 1.0f / 253952;
				reg |= 4;
				break;
			case 32:
				SPL06_coefficients.KP = 1.0f / 516096;
				reg |= 5;
				break;
			case 64:
				SPL06_coefficients.KP = 1.0f / 1040384;
				reg |= 6;
				break;
			case 128:
				SPL06_coefficients.KP = 1.0f / 2088960;
				reg |= 7;
				break;
			case 1:
			default:
				SPL06_coefficients.KP = 1.0f / 524288;
				break;
		}
		uint8_t buf[2];
		buf[0] = reg;//pressure 64 samples per sec , 32 times over sampling
		i2c0WriteData( SPL06_Addr , 0x6 , buf , 1 );
		if(u8OverSmpl > 8)
		{
			i2c0ReadData( SPL06_Addr , 0x9 , buf , 1 );		
			buf[0] |= (1<<2);	
			i2c0WriteData(SPL06_Addr, 0x09, buf , 1 );
		}
	}
	static void spl06_temperature_rateset( uint8_t u8SmplRate, uint8_t u8OverSmpl)
	{
		uint8_t reg = 0;
		switch(u8SmplRate)
		{
			case 2:
				reg |= (1<<4);
				break;
			case 4:
				reg |= (2<<4);
				break;
			case 8:
				reg |= (3<<4);
				break;
			case 16:
				reg |= (4<<4);
				break;
			case 32:
				reg |= (5<<4);
				break;
			case 64:
				reg |= (6<<4);
				break;
			case 128:
				reg |= (7<<4);
				break;
			case 1:
			default:
				break;
		}
		switch(u8OverSmpl)
		{
			case 2:
				reg |= 1;
				SPL06_coefficients.KT = 1.0f / 1572864;
				break;
			case 4:
				reg |= 2;
				SPL06_coefficients.KT = 1.0f / 3670016;
				break;
			case 8:
				reg |= 3;
				SPL06_coefficients.KT = 1.0f / 7864320;
				break;
			case 16:
				SPL06_coefficients.KT = 1.0f / 253952;
				reg |= 4;
				break;
			case 32:
				SPL06_coefficients.KT = 1.0f / 516096;
				reg |= 5;
				break;
			case 64:
				SPL06_coefficients.KT = 1.0f / 1040384;
				reg |= 6;
				break;
			case 128:
				SPL06_coefficients.KT = 1.0f / 2088960;
				reg |= 7;
				break;
			case 1:
			default:
				SPL06_coefficients.KT = 1.0f / 524288;
				break;
		}
		uint8_t buf[2];
		buf[0] = (1<<7) | reg;//pressure 64 samples per sec , 32 times over sampling
		i2c0WriteData( SPL06_Addr , 0x7 , buf , 1 );
		if(u8OverSmpl > 8)
		{
			i2c0ReadData( SPL06_Addr , 0x9 , buf , 1 );		
			buf[0] |= (1<<2);	
			i2c0WriteData(SPL06_Addr, 0x09, buf , 1 );
		}
	}
#pragma endpragma

void init_drv_Sensors()
{
	/*配置i2c0拉取内置传感器数据*/
		SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
		delay( 1e-6f );
		SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C0);
		delay( 1e-6f );
		
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //开启GPIOB电源
		delay( 1e-6f );
		
		GPIOPinTypeI2CSCL(GPIOB_BASE, GPIO_PIN_2); // Use pin with I2C SCL peripheral
		GPIOPinTypeI2C(GPIOB_BASE, GPIO_PIN_3); // Use pin with I2C peripheral
		
		// Use alternate function
		GPIOPinConfigure(GPIO_PB2_I2C0SCL);
		GPIOPinConfigure(GPIO_PB3_I2C0SDA);
		delay( 1e-6f );
		
		SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0); //开启I2C0外设电源
		delay( 1e-6f );
		I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true); // Enable and set frequency to 400 kHz
		delay( 1e-6f );	
		
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_FIFO_BURST_SEND_FINISH);
		delay( 1e-3f);
	/*配置i2c0拉取内置传感器数据*/
	
	/*配置i2c1拉取外置罗盘数据*/
		SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
		delay( 1e-6f );
		SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C1);
		delay( 1e-6f );
		
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //开启GPIOA电源
		delay( 1e-6f );
		
		GPIOPinTypeI2CSCL(GPIOA_BASE, GPIO_PIN_6); // Use pin with I2C SCL peripheral
		GPIOPinTypeI2C(GPIOA_BASE, GPIO_PIN_7); // Use pin with I2C peripheral
		
		// Use alternate function
		GPIOPinConfigure(GPIO_PA6_I2C1SCL);
		GPIOPinConfigure(GPIO_PA7_I2C1SDA);
		delay( 1e-6f );
		
		SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1); //开启I2C1外设电源
		delay( 1e-6f );
		I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), true); // Enable and set frequency to 400 kHz
		delay( 1e-6f );	
		
		I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_FIFO_BURST_SEND_FINISH);
		delay( 1e-3f);
	/*配置i2c1拉取外置罗盘数据*/
	
	static unsigned char buf[ 36 ];
	
	/*ICM20602 configurations*/	
//		//关闭output limit
//		buf[0] = (1<<1);
//		i2c0WriteData( icm20602_Addr , 105 , buf , 1 );
	
		//复位
		buf[0] = (1<<7) | (1<<0);	//reset and select clock source
		i2c0WriteData( icm20602_Addr , 107 , buf , 1 );
		delay( 0.1f );
		
		//退出SLEEP模式并选择时钟源
		buf[0] = (0<<7) | (1<<0);
		i2c0WriteData( icm20602_Addr , 107 , buf , 1 );
		delay( 0.1f );	//wait for device reset		
			
		//复位Signal Path
		buf[0] = (1<<1) | (1<<0);
		i2c0WriteData( icm20602_Addr , 104 , buf , 1 );
		delay(0.1f);
		
		//复位FIFO及信道
		buf[0] = (1<<2) | (1<<0);
		i2c0WriteData( icm20602_Addr , 106 , buf , 1 );
		delay(0.1f);
		
		//打开加速度计陀螺仪
		buf[0] = 0;
		i2c0WriteData( icm20602_Addr , 108 , buf , 1 );
		delay( 0.1f );
		
//		//关闭output limit
//		buf[0] = (1<<1);
//		i2c0WriteData( icm20602_Addr , 105 , buf , 1 );
		
		//配置传感器
		buf[0] = 0;	//25:sample rate divider = 0
		buf[1] = (0 << 0);	//26:set DLPF
		buf[2] = (3 << 3) | (0b00 << 0);	//27:gyro full scale 2000dps
		buf[3] = (3 << 3);	//28:accelerometer full scale +/-16G
		buf[4] = (0 << 3) | (2<<0);	//29:accelerometer LPF
		buf[5] = 0;	//30:
		i2c0WriteData( icm20602_Addr , 25 , buf , 6 );
		
		//注册传感器
		IMUAccelerometerRegister( 0 , 0.47884033203125f );
		IMUGyroscopeRegister( 0 , 0.00106422515365507901031932363932 );
	/*ICM20602 configurations*/	
	
	/*IST8310 configurations*/
		//复位
		buf[0] = (1<<0);
		i2c0WriteData( IST8310_Addr , 0xB , buf , 1 );
		delay( 0.1f );
		
		//开启16x内部平均
		buf[0] = 0x24;
		i2c0WriteData( IST8310_Addr , 0x41 , buf , 1 );
		
		//Set/Reset内部平均
		buf[0] = 0xc0;
		i2c0WriteData( IST8310_Addr , 0x42 , buf , 1 );
		
		//注册传感器
		IMUMagnetometerRegister( 2 , 0.003f );
	/*IST8310 configurations*/
	
	/*SPL06 configurations*/
		//复位
		buf[0] = (1<<7) | (0b1001);
		i2c0WriteData( SPL06_Addr , 0xc , buf , 1 );
		delay( 0.1f );

		//配置传感器
		spl06_pressure_rateset( 64 , 32 );//pressure 64 samples per sec , 32 times over sampling		
		spl06_temperature_rateset( 128 , 2 );//temperature 128 samples per sec , 2 times over sampling
		buf[0] = (0b111);//continues measurement mode
		i2c0WriteData( SPL06_Addr , 0x8 , buf , 1 );
		delay( 0.1f );
		
		//读出厂校准系数
		i2c0ReadData( SPL06_Addr , 0x10 , buf , 18 );
		SPL06_coefficients.c0 = ( buf[0] << 4 ) | ( buf[1] >> 4 );
		SPL06_coefficients.c0 = ( SPL06_coefficients.c0 & 0x0800 ) ? (0xF000|SPL06_coefficients.c0) : SPL06_coefficients.c0;
		SPL06_coefficients.c1 = ( (buf[1] & 0xf) << 8 ) | ( buf[2] );
		SPL06_coefficients.c1 = ( SPL06_coefficients.c1 & 0x0800 ) ? (0xF000|SPL06_coefficients.c1) : SPL06_coefficients.c1;
		SPL06_coefficients.c00 = ( buf[3] << 12 ) | ( buf[4] << 4 ) | ( buf[5] >> 4 );
		SPL06_coefficients.c00 = ( SPL06_coefficients.c00 & 0x080000 ) ? (0xFFF00000|SPL06_coefficients.c00) : SPL06_coefficients.c00;
		SPL06_coefficients.c10 = ( (buf[5] & 0xf) << 16 ) | ( buf[6] << 8 ) | ( buf[7] >> 0 );
		SPL06_coefficients.c10 = ( SPL06_coefficients.c10 & 0x080000 ) ? (0xFFF00000|SPL06_coefficients.c10) : SPL06_coefficients.c10;
		SPL06_coefficients.c01 = ( buf[8] << 8 ) | ( buf[9] << 0 );
		SPL06_coefficients.c11 = ( buf[10] << 8 ) | ( buf[11] << 0 );
		SPL06_coefficients.c20 = ( buf[12] << 8 ) | ( buf[13] << 0 );
		SPL06_coefficients.c21 = ( buf[14] << 8 ) | ( buf[15] << 0 );
		SPL06_coefficients.c30 = ( buf[16] << 8 ) | ( buf[17] << 0 );
		
		//注册传感器
		PositionSensorRegister( internal_baro_sensor_index , \
														Position_Sensor_Type_RelativePositioning , \
														Position_Sensor_DataType_s_z , \
														Position_Sensor_frame_ENU , \
														0.1f , \
														false );
	/*SPL06 configurations*/
	
	//开启I2C0接收中断
	I2CIntRegister(I2C0_BASE,I2C0_Transfered_Handler);//中断函数注册
	I2CMasterIntEnable( I2C0_BASE );
	IntEnable(INT_I2C0);
	IntPrioritySet(INT_I2C0,INT_PRIO_1);
	
	//开启I2C1接收中断
	I2CIntRegister(I2C1_BASE,I2C1_Transfered_Handler);//中断函数注册
	I2CMasterIntEnable( I2C1_BASE );
	IntEnable(INT_I2C1);
	IntPrioritySet(INT_I2C1,INT_PRIO_1);
	
	//开启定时器2中断周期读取传感器
	SysCtlPeripheralEnable( SYSCTL_PERIPH_TIMER2 );
	TimerConfigure( TIMER2_BASE , TIMER_CFG_A_PERIODIC );
	TimerLoadSet( TIMER2_BASE , TIMER_A , SysCtlClockGet() / 1000 );
	TimerIntRegister( TIMER2_BASE , TIMER_A , Timer2A_IRQHandler );
	TimerIntEnable( TIMER2_BASE , TIMER_TIMA_TIMEOUT );
	IntEnable(INT_TIMER2A);
	IntPrioritySet(INT_TIMER2A,INT_PRIO_3);
	TimerEnable( TIMER2_BASE , TIMER_A );
	
	//开启一个不用的外设中断
	//用于在中断中运行解算及控制任务
	IntPrioritySet(INT_I2C3 , INT_PRIO_7);
	I2CIntRegister( I2C3_BASE , MainMCHandler );
	IntEnable(INT_I2C3);
	
	/*写入初始传感器校准数据*/
		for( uint8_t i = 0 ; i < IMU_Sensors_Count ; ++i )
		{
			vector3_float offset = { 0 , 0, 0 };
			vector3_float scale = { 1, 1 ,1 };
			Cfg_set_initial_AccelerometerOffset( i , offset );
			Cfg_set_initial_GyroscopeOffset( i , offset );
			Cfg_set_initial_MagnetometerOffset( i , offset );
			Cfg_set_initial_AccelerometerSensivitity( i , scale );
			Cfg_set_initial_GyroscopeSensivitity( i , scale );
			Cfg_set_initial_MagnetometerSensivitity( i , scale );
		}
	/*写入初始传感器校准数据*/
}


/*i2c0 内部传感器接收状态机*/
#pragma region 内部传感器接收状态机
	//启动i2c接收状态机进行一次接收
	static void i2c_read_start(bool spl06 , bool ist8310)
	{
		if( i2c_current_read_sensor == i2c_sensor_null )
		{	//当前无通信，启动新一次通信
			i2c_current_read_sensor = i2c_sensor_icm20602;
			i2c_read_counter = 0;
			
			read_spl06 = spl06;
			read_ist8310 = ist8310;
			
			I2CMasterSlaveAddrSet(I2C0_BASE, icm20602_Addr, false); // 设置寄存器地址（写模式）
			I2CMasterDataPut(I2C0_BASE, 59);
			I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START); // 发送起始位 写入寄存器编号				
		}
		else
		{	//当前正在通信
			//让状态机通信完成后读取
			read_icm20602 = true;
			read_spl06_nexttime = spl06;
			read_ist8310_nexttime = ist8310;
		}
	}
	
	static void Timer2A_IRQHandler()
	{
		//清空中断
		TimerIntClear( TIMER2_BASE , TIMER_TIMA_TIMEOUT );
		
		static uint16_t ist8310_counter = 0;
		static uint16_t ispl06_counter = 0;
		
		//按频率读取SPL06
		bool spl06 = false;
		if( ++ispl06_counter >= 33 )
		{	//30hz
			ispl06_counter = 0;
			spl06 = true;
		}
		
		//按频率读取IST8310
		bool ist8310 = false;
		if( ++ist8310_counter >= 20 )
		{	//50hz
			ist8310_counter = 0;
			ist8310 = true;
		}
		
		//开始读取
		i2c_read_start( spl06 , ist8310 );
		
		/*读取外置罗盘*/
			static uint8_t ExtMag_Counter = 0;
			if( ++ExtMag_Counter == 10 )
			{	//10ms
				ExtMag_Counter = 0;
				if( Current_External_Magnetometer < 0 )
				{
					//外置罗盘不存在
					//检测是否有外置罗盘连接
					current_External_Magnetometer_Operation = External_Magnetometer_Operation_Scan;
					External_Magnetometer_Operation_Counter = 0;
					
					I2CMasterSlaveAddrSet(I2C1_BASE, External_MagnetoMeters[ Current_Scan_External_Magnetometer ].device_address, false); // 设置寄存器地址（写模式）
					I2CMasterDataPut( I2C1_BASE , External_MagnetoMeters[ Current_Scan_External_Magnetometer ].ID_address ); // Place address into data register
					I2CMasterControl( I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START); // Send data
				}
				else
				{
					//外置罗盘可用
					//根据频率读取外置罗盘数据
					static uint8_t ExtMag_Sample_Counter = 0;
					ExtMag_Sample_Counter += 10;
					if( ExtMag_Sample_Counter >= External_MagnetoMeters[ Current_External_Magnetometer ].sample_mseconds )
					{
						ExtMag_Sample_Counter = 0;
						
						current_External_Magnetometer_Operation = External_Magnetometer_Operation_ReadData;
						External_Magnetometer_Operation_Counter = 0;
						
						I2CMasterSlaveAddrSet(I2C1_BASE, External_MagnetoMeters[ Current_External_Magnetometer ].device_address, false); // 设置寄存器地址（写模式）
						I2CMasterDataPut( I2C1_BASE , External_MagnetoMeters[ Current_External_Magnetometer ].data_address ); // Place address into data register
						I2CMasterControl( I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START); // Send data
					}
				}
			}
		/*读取外置罗盘*/
	}

	static uint8_t data_buffer[32];
	static void I2C0_Transfered_Handler()
	{
		I2CMasterIntClear( I2C0_BASE );
		
		if( I2CMasterErr( I2C0_BASE ) )
		{
			//通信错误
			i2c_current_read_sensor = i2c_sensor_null;
			return;
		}
		
		switch( i2c_current_read_sensor )
		{
			case i2c_sensor_null:
				//错误，不应该发生
				break;
			
			case i2c_sensor_icm20602:
			{
				if( i2c_read_counter == 0 )
				{				
					//已发送start 并写入了寄存器地址
					I2CMasterSlaveAddrSet(I2C0_BASE, icm20602_Addr, true); // 再次设置寄存器地址（读模式）
					I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START); // 发送重复起始位 开始读数据				
				}
				else if( i2c_read_counter < 13 )
				{
					//读入所有数据
					data_buffer[ i2c_read_counter - 1 ] = I2CMasterDataGet(I2C0_BASE);
					I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT); // 继续发送ACK拉取数据
				}
				else if( i2c_read_counter == 13 )
				{
					//拉取倒数第二个数据
					//用NAK回应 并 发送stop
					data_buffer[ i2c_read_counter - 1 ] = I2CMasterDataGet(I2C0_BASE);
					I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
				}
				else
				{
					//传输完成 拉取最后一个数据
					data_buffer[ i2c_read_counter - 1 ] = I2CMasterDataGet(I2C0_BASE);
					vector3_int data;
					data.y =  -(int32_t)( (int16_t)__REV16( *(uint16_t*)&data_buffer[0] ) );
					data.x =  -(int32_t)( (int16_t)__REV16( *(uint16_t*)&data_buffer[2] ) );
					data.z =  -(int32_t)( (int16_t)__REV16( *(uint16_t*)&data_buffer[4] ) );
					IMUAccelerometerUpdate( 0 , data );
					data.y =  -(int32_t)( (int16_t)__REV16( *(uint16_t*)&data_buffer[8] ) );
					data.x =  -(int32_t)( (int16_t)__REV16( *(uint16_t*)&data_buffer[10] ) );
					data.z =  -(int32_t)( (int16_t)__REV16( *(uint16_t*)&data_buffer[12] ) );
					IMUGyroscopeUpdate( 0 , data );
					
					if( read_spl06 )
					{
						i2c_read_counter = 0;
						i2c_current_read_sensor = i2c_sensor_spl06;
						
						I2CMasterSlaveAddrSet(I2C0_BASE, SPL06_Addr, false); // 设置寄存器地址（写模式）
						I2CMasterDataPut(I2C0_BASE, 0);
						I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START); // 发送起始位 写入寄存器编号		
						break;
					}
					else if( read_ist8310 )
					{
						i2c_read_counter = 0;
						i2c_current_read_sensor = i2c_sensor_ist8310;
						
						I2CMasterSlaveAddrSet(I2C0_BASE, IST8310_Addr, false); // 设置寄存器地址（写模式）
						I2CMasterDataPut(I2C0_BASE, 3);
						I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START); // 发送起始位 写入寄存器编号		
					}
					else
					{
						if( read_icm20602 )
						{
							read_icm20602 = false;
							i2c_current_read_sensor = i2c_sensor_icm20602;
							i2c_read_counter = 0;
							
							I2CMasterSlaveAddrSet(I2C0_BASE, icm20602_Addr, false); // 设置寄存器地址（写模式）
							I2CMasterDataPut(I2C0_BASE, 59);
							I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START); // 发送起始位 写入寄存器编号		
							
							read_spl06 = read_spl06_nexttime;
							read_ist8310 = read_ist8310_nexttime;
						}
						else
						{									
							i2c_read_counter = 0;
							i2c_current_read_sensor = i2c_sensor_null;
						}
						//挂起解算控制任务中断
						NVIC_SetPendingIRQ( I2C3_IRQn );
					}
					break;
				}
				++i2c_read_counter;
				
				break;
			}
			
			case i2c_sensor_spl06:
			{
				if( i2c_read_counter == 0 )
				{				
					//已发送start 并写入了寄存器地址
					I2CMasterSlaveAddrSet(I2C0_BASE, SPL06_Addr, true); // 再次设置寄存器地址（读模式）
					I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START); // 发送重复起始位 开始读数据				
				}
				else if( i2c_read_counter < 5 )
				{
					//读入所有数据
					data_buffer[ i2c_read_counter - 1 ] = I2CMasterDataGet(I2C0_BASE);
					I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT); // 继续发送ACK拉取数据
				}
				else if( i2c_read_counter == 5 )
				{
					//拉取倒数第二个数据
					//用NAK回应 并 发送stop
					data_buffer[ i2c_read_counter - 1 ] = I2CMasterDataGet(I2C0_BASE);
					I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
				}
				else
				{
					//传输完成 拉取最后一个数据
					data_buffer[ i2c_read_counter - 1 ] = I2CMasterDataGet(I2C0_BASE);
					
					/*计算气压高度*/
						typedef struct
						{
							unsigned int pressure:24 ;
							unsigned int temperature:24 ;
						}__PACKED SPL06Data;
						SPL06Data* SPL06Datas = (SPL06Data*)data_buffer;
						int32_t buf32[2];
						buf32[0] = __REV( SPL06Datas->pressure ) >> 8;
						buf32[0] = ( buf32[0] & 0x800000 ) ? (0xFF000000|buf32[0]) : buf32[0];
						buf32[1] = __REV( SPL06Datas->temperature ) >> 8;
						buf32[1] = ( buf32[1] & 0x800000 ) ? (0xFF000000|buf32[1]) : buf32[1];
						
						float fPsc = buf32[0] * SPL06_coefficients.KP;
						float fTsc = buf32[1] * SPL06_coefficients.KT;
						float qua2 = SPL06_coefficients.c10 + fPsc * (SPL06_coefficients.c20 + fPsc* SPL06_coefficients.c30);
						float qua3 = fTsc * fPsc * (SPL06_coefficients.c11 + fPsc * SPL06_coefficients.c21);
						
						float pressure = SPL06_coefficients.c00 + fPsc * qua2 + fTsc * SPL06_coefficients.c01 + qua3;
						float temperature = SPL06_coefficients.c0*0.5f + SPL06_coefficients.c1*fTsc;
						
						if( pressure > 0 )
						{
							vector3_float position;
							position.z = 4430000 * ( 1.0f - powf( pressure / 101325.0f , 1.0f / 5.256f ) );
							PositionSensorUpdatePosition( internal_baro_sensor_index , position , true , -1 );
						}
					/*计算气压高度*/
						
					read_spl06 = false;
					if( read_ist8310 )
					{
						i2c_read_counter = 0;
						i2c_current_read_sensor = i2c_sensor_ist8310;
						
						I2CMasterSlaveAddrSet(I2C0_BASE, IST8310_Addr, false); // 设置寄存器地址（写模式）
						I2CMasterDataPut(I2C0_BASE, 3);
						I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START); // 发送起始位 写入寄存器编号		
					}
					else
					{					
						if( read_icm20602 )
						{
							read_icm20602 = false;
							i2c_current_read_sensor = i2c_sensor_icm20602;
							i2c_read_counter = 0;
							
							I2CMasterSlaveAddrSet(I2C0_BASE, icm20602_Addr, false); // 设置寄存器地址（写模式）
							I2CMasterDataPut(I2C0_BASE, 59);
							I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START); // 发送起始位 写入寄存器编号		
							
							read_spl06 = read_spl06_nexttime;
							read_ist8310 = read_ist8310_nexttime;
						}
						else
						{									
							i2c_read_counter = 0;
							i2c_current_read_sensor = i2c_sensor_null;
						}
						//挂起解算控制任务中断
						NVIC_SetPendingIRQ( I2C3_IRQn );
					}
					break;
				}
				++i2c_read_counter;
				
				break;
			}
			
			case i2c_sensor_ist8310:
			{
				if( i2c_read_counter == 0 )
				{				
					//已发送start 并写入了寄存器地址
					I2CMasterSlaveAddrSet(I2C0_BASE, IST8310_Addr, true); // 再次设置寄存器地址（读模式）
					I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START); // 发送重复起始位 开始读数据				
				}
				else if( i2c_read_counter < 5 )
				{
					//读入所有数据
					data_buffer[ i2c_read_counter - 1 ] = I2CMasterDataGet(I2C0_BASE);
					I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT); // 继续发送ACK拉取数据
				}
				else if( i2c_read_counter == 5 )
				{
					//拉取倒数第二个数据
					//用NAK回应 并 发送stop
					data_buffer[ i2c_read_counter - 1 ] = I2CMasterDataGet(I2C0_BASE);
					I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
				}
				else if( i2c_read_counter == 6 )
				{
					//传输完成 拉取最后一个数据
					data_buffer[ i2c_read_counter - 1 ] = I2CMasterDataGet(I2C0_BASE);
					
					vector3_int data;
					data.y = -(int32_t)( (int16_t)( *(uint16_t*)&data_buffer[0] ) );
					data.x = (int32_t)( (int16_t)( *(uint16_t*)&data_buffer[2] ) );
					data.z = -(int32_t)( (int16_t)( *(uint16_t*)&data_buffer[4] ) );
					IMUMagnetometerUpdate( 2 , data );
				
					//配置IST8310继续采样
					I2CMasterSlaveAddrSet(I2C0_BASE, IST8310_Addr, false); // 设置寄存器地址（写模式）
					I2CMasterDataPut(I2C0_BASE, 0xa);
					I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START); // 发送起始位 写入寄存器编号							
				}
				else if( i2c_read_counter == 7 )
				{				
					//已发送start 并写入了寄存器地址
					I2CMasterDataPut(I2C0_BASE, 0x01);
					I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH); // 配置8310采样 发送stop结束传输
				}
				else
				{
					//ist8310 配置完成
					read_ist8310 = false;
					if( read_icm20602 )
					{
						read_icm20602 = false;
						i2c_current_read_sensor = i2c_sensor_icm20602;
						i2c_read_counter = 0;
						
						I2CMasterSlaveAddrSet(I2C0_BASE, icm20602_Addr, false); // 设置寄存器地址（写模式）
						I2CMasterDataPut(I2C0_BASE, 59);
						I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START); // 发送起始位 写入寄存器编号		
						
						read_spl06 = read_spl06_nexttime;
						read_ist8310 = read_ist8310_nexttime;
					}
					else
					{									
						i2c_read_counter = 0;
						i2c_current_read_sensor = i2c_sensor_null;
					}
					//挂起解算控制任务中断
					NVIC_SetPendingIRQ( I2C3_IRQn );
					break;
				}
				++i2c_read_counter;
				
				break;
			}
		}
	}
	
	//主解算控制任务中断
	static void MainMCHandler()
	{
		NVIC_ClearPendingIRQ( I2C3_IRQn );
		
		MS_main();
		static uint16_t Ctrl_Counter = 0;
		if( ++Ctrl_Counter >= 5 )
		{		
			Ctrl_Counter = 0;
			//200hz运行控制
			ctrl_main();
		}
	}
#pragma endpragma 
/*i2c0 内部传感器接收状态机*/

/*i2c0静态读写函数*/
#pragma region i2c静态读写函数
	static void i2c0WriteData(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length) 
	{
			I2CMasterSlaveAddrSet(I2C0_BASE, addr, false); // Set to write mode

			I2CMasterDataPut(I2C0_BASE, regAddr); // Place address into data register
			I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START); // Send start condition
			while (I2CMasterBusy(I2C0_BASE)); // Wait until transfer is done

			for (uint8_t i = 0; i < length - 1; i++) {
					I2CMasterDataPut(I2C0_BASE, data[i]); // Place data into data register
					I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT); // Send continues condition
					while (I2CMasterBusy(I2C0_BASE)); // Wait until transfer is done
			}

			I2CMasterDataPut(I2C0_BASE, data[length - 1]); // Place data into data register
			I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH); // Send finish condition
			while (I2CMasterBusy(I2C0_BASE)); // Wait until transfer is done
	}
	static void i2c0ReadData(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length) 
	{
			I2CMasterSlaveAddrSet(I2C0_BASE, addr, false); // Set to write mode

			I2CMasterDataPut(I2C0_BASE, regAddr); // Place address into data register
			I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START); // Send data
			while (I2CMasterBusy(I2C0_BASE)); // Wait until transfer is done

			I2CMasterSlaveAddrSet(I2C0_BASE, addr, true); // Set to read mode

			if( length > 1 )
			{
				I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START); // Send start condition
				while (I2CMasterBusy(I2C0_BASE)); // Wait until transfer is done
				data[0] = I2CMasterDataGet(I2C0_BASE); // Place data into data register

				for (uint8_t i = 1; i < length - 1; i++) {
						I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT); // Send continues condition
						while (I2CMasterBusy(I2C0_BASE)); // Wait until transfer is done
						data[i] = I2CMasterDataGet(I2C0_BASE); // Place data into data register
				}

				I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH); // Send finish condition
				while (I2CMasterBusy(I2C0_BASE)); // Wait until transfer is done
				data[length - 1] = I2CMasterDataGet(I2C0_BASE); // Place data into data register
			}
			else
			{
				I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE); // Send start condition
				while (I2CMasterBusy(I2C0_BASE)); // Wait until transfer is done
				data[0] = I2CMasterDataGet(I2C0_BASE); // Place data into data register
			}
	}
#pragma endpragma
/*i2c0静态读写函数*/

	
	
	
/*i2c1外置罗盘接收状态机*/
	static void I2C1_Transfered_Handler()
	{
		bool i2c_error = I2CMasterErr( I2C1_BASE );
		I2CMasterIntClear( I2C1_BASE );
		
		switch( current_External_Magnetometer_Operation )
		{
			
			case External_Magnetometer_Operation_ReadData:
			{
				if( i2c_error )
				{
					//传感器读取时发生错误
					IMUMagnetometerUnRegister( External_Magnetometer_Index );
					Current_External_Magnetometer = -1;
				}
				else
				{
					static uint8_t data_buffer[8];
					if( External_Magnetometer_Operation_Counter == 0 )
					{
						//已发送start 并写入了寄存器地址
						++External_Magnetometer_Operation_Counter;
						I2CMasterSlaveAddrSet(I2C1_BASE, External_MagnetoMeters[ Current_External_Magnetometer ].device_address, true); // 再次设置寄存器地址（读模式）
						I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START); // 发送重复起始位 开始读数据
					}
					else if( External_Magnetometer_Operation_Counter < 5 )
					{
						//读入所有数据						
						data_buffer[ External_Magnetometer_Operation_Counter - 1 ] = I2CMasterDataGet(I2C1_BASE);
						++External_Magnetometer_Operation_Counter;
						I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT); // 继续发送ACK拉取数据						
					}
					else if( External_Magnetometer_Operation_Counter == 5 )
					{
						//拉取倒数第二个数据
						//用NAK回应 并 发送stop				
						data_buffer[ External_Magnetometer_Operation_Counter - 1 ] = I2CMasterDataGet(I2C1_BASE);
						++External_Magnetometer_Operation_Counter;
						I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
					}
					else if( External_Magnetometer_Operation_Counter == 6 )
					{
						//传输完成 拉取最后一个数据
						data_buffer[ External_Magnetometer_Operation_Counter - 1 ] = I2CMasterDataGet(I2C1_BASE);
						
						const External_MagnetoMeter* ext_mag = &External_MagnetoMeters[ Current_External_Magnetometer ];
						
						/*更新传感器数据*/						
							//大端转小端
							if( ext_mag->MSB )
							{
								((uint16_t*)&data_buffer)[0] = __REV16( ((uint16_t*)&data_buffer)[0] );
								((uint16_t*)&data_buffer)[1] = __REV16( ((uint16_t*)&data_buffer)[1] );
								((uint16_t*)&data_buffer)[2] = __REV16( ((uint16_t*)&data_buffer)[2] );
							}			
							
							//轴向转换
							vector3_int data;
							if( ext_mag->axis_index[0] > 0 )
								data.x = (signed short)((uint16_t*)&data_buffer)[ ext_mag->axis_index[0] - 1 ];
							else
								data.x = -(signed short)((uint16_t*)&data_buffer)[ (-ext_mag->axis_index[0]) - 1 ];
							if( ext_mag->axis_index[1] > 0 )
									data.y = (signed short)((uint16_t*)&data_buffer)[ ext_mag->axis_index[1] - 1 ];
								else
									data.y = -(signed short)((uint16_t*)&data_buffer)[ (-ext_mag->axis_index[1]) - 1 ];
								if( ext_mag->axis_index[2] > 0 )
									data.z = (signed short)((uint16_t*)&data_buffer)[ ext_mag->axis_index[2] - 1 ];
								else
									data.z = -(signed short)((uint16_t*)&data_buffer)[ (-ext_mag->axis_index[2]) - 1 ];
								
							//更新数据
							IMUMagnetometerUpdate( External_Magnetometer_Index , data );
						/*更新传感器数据*/
								
						if( ext_mag->need_set_sample )
						{
							//手动采样
							//配置继续采样
							++External_Magnetometer_Operation_Counter;
							I2CMasterSlaveAddrSet(I2C1_BASE, ext_mag->device_address, false); // 设置寄存器地址（写模式）
							I2CMasterDataPut(I2C1_BASE, ext_mag->sample_address);
							I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START); // 发送起始位 写入寄存器编号			
						}
						else
						{
							//无需手动采样
							External_Magnetometer_Operation_Counter = 0;
						}
					}
					else if( External_Magnetometer_Operation_Counter == 7 )
					{				
						//已发送start 并写入了寄存器地址
						//发送手动采样命令
						++External_Magnetometer_Operation_Counter;
						I2CMasterDataPut(I2C1_BASE, External_MagnetoMeters[ Current_External_Magnetometer ].sample_cfg);
						I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH); // 配置8310采样 发送stop结束传输
					}
					else if( External_Magnetometer_Operation_Counter == 8 )
					{
						//手动采样完成
						External_Magnetometer_Operation_Counter = 0;
					}
				}
				break;
			}	//External_Magnetometer_Operation_ReadData
			
			case External_Magnetometer_Operation_Scan:
			{
				if( i2c_error )
				{
					//读取不到当前检测的传感器
					//切换到下一个传感器继续检测
					++Current_Scan_External_Magnetometer;
					if( (Current_Scan_External_Magnetometer >= Supported_External_MagnetoMeter_Count) \
						||(External_MagnetoMeters[ Current_Scan_External_Magnetometer ].device_address == 0) )
						Current_Scan_External_Magnetometer = 0;
				}
				else
				{
					switch( External_Magnetometer_Operation_Counter )
					{
						case 0:
							//已发送id寄存器地址
							//开始接收id寄存器的值
							++External_Magnetometer_Operation_Counter;
							I2CMasterSlaveAddrSet(I2C1_BASE, External_MagnetoMeters[ Current_Scan_External_Magnetometer ].device_address, true); // Set to read mode
							I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE); // Send start condition
							break;
						
						case 1:
							//已接收id寄存器的值
							//判断id寄存器的值是否正确
							if( I2CMasterDataGet(I2C1_BASE) == External_MagnetoMeters[ Current_Scan_External_Magnetometer ].ID )
							{
								//ID正确
								//发送配置
								External_Magnetometer_Operation_Counter = 0;
								current_External_Magnetometer_Operation = External_Magnetometer_Operation_SetConfig;
								
								I2CMasterSlaveAddrSet(I2C1_BASE, External_MagnetoMeters[ Current_Scan_External_Magnetometer ].device_address, false); // Set to write mode
								I2CMasterDataPut(I2C1_BASE, External_MagnetoMeters[ Current_Scan_External_Magnetometer ].configurations_address); // Place address into data register
								I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START); // Send start condition
							}
							else
							{
								//ID不正确
								//继续检测下一个传感器
								++Current_Scan_External_Magnetometer;
								if( (Current_Scan_External_Magnetometer >= Supported_External_MagnetoMeter_Count) \
									||(External_MagnetoMeters[ Current_Scan_External_Magnetometer ].device_address == 0) )
									Current_Scan_External_Magnetometer = 0;
							}
							break;
					}
				}
				
				break;
			}	//External_Magnetometer_Operation_Scan
			
			case External_Magnetometer_Operation_SetConfig:
			{
				if( i2c_error )
				{
					//读取不到当前检测的传感器
					//切换到下一个传感器继续检测
					++Current_Scan_External_Magnetometer;
					if( (Current_Scan_External_Magnetometer >= Supported_External_MagnetoMeter_Count) \
						||(External_MagnetoMeters[ Current_Scan_External_Magnetometer ].device_address == 0) )
						Current_Scan_External_Magnetometer = 0;
				}
				else
				{
					//发送配置信息
					if( External_Magnetometer_Operation_Counter < External_MagnetoMeters[ Current_Scan_External_Magnetometer ].configuration_count - 1 )
					{
						I2CMasterDataPut( I2C1_BASE, External_MagnetoMeters[ Current_Scan_External_Magnetometer ].configurations[ External_Magnetometer_Operation_Counter ] ); // Place data into data register
						I2CMasterControl( I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_CONT ); // Send continues condition
						++External_Magnetometer_Operation_Counter;
					}
					else if( External_Magnetometer_Operation_Counter == External_MagnetoMeters[ Current_Scan_External_Magnetometer ].configuration_count - 1 )
					{
						//发送最后一个
						I2CMasterDataPut( I2C1_BASE, External_MagnetoMeters[ Current_Scan_External_Magnetometer ].configurations[ External_Magnetometer_Operation_Counter ] ); // Place data into data register
						I2CMasterControl( I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH ); // Send finish condition
						++External_Magnetometer_Operation_Counter;
					}
					else
					{
						//配置发送完成
						//检测配置是否正确
						current_External_Magnetometer_Operation = External_Magnetometer_Operation_CheckConfig;
						External_Magnetometer_Operation_Counter = 0;
						
						I2CMasterSlaveAddrSet(I2C1_BASE, External_MagnetoMeters[ Current_Scan_External_Magnetometer ].device_address, false); // 设置寄存器地址（写模式）
						I2CMasterDataPut( I2C1_BASE , External_MagnetoMeters[ Current_Scan_External_Magnetometer ].configurations_address ); // Place address into data register
						I2CMasterControl( I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START); // Send data
					}
				}
				
				break;
			}	//External_Magnetometer_Operation_SetConfig
			
			case External_Magnetometer_Operation_CheckConfig:
			{
				if( i2c_error )
				{
					//读取不到当前检测的传感器
					//切换到下一个传感器继续检测
					++Current_Scan_External_Magnetometer;
					if( (Current_Scan_External_Magnetometer >= Supported_External_MagnetoMeter_Count) \
						||(External_MagnetoMeters[ Current_Scan_External_Magnetometer ].device_address == 0) )
						Current_Scan_External_Magnetometer = 0;
				}
				else
				{
					if( External_Magnetometer_Operation_Counter == 0 )
					{
						//已发送start 并写入了配置寄存器地址
						++External_Magnetometer_Operation_Counter;
						I2CMasterSlaveAddrSet(I2C1_BASE, External_MagnetoMeters[ Current_Scan_External_Magnetometer ].device_address, true); // 再次设置寄存器地址（读模式）
						if( External_MagnetoMeters[ Current_Scan_External_Magnetometer ].configuration_count > 1 )
							I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START); // 发送重复起始位 开始读数据
						else
							I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE); // 发送重复起始位 开始读数据
					}
					else if( External_Magnetometer_Operation_Counter < External_MagnetoMeters[ Current_Scan_External_Magnetometer ].configuration_count - 1 )
					{
						//读入所有配置
						uint8_t cfg = I2CMasterDataGet(I2C1_BASE);
						if( cfg == External_MagnetoMeters[ Current_Scan_External_Magnetometer ].configurations[ External_Magnetometer_Operation_Counter - 1 ] )
						{
							//配置正确
							++External_Magnetometer_Operation_Counter;
							I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT); // 继续发送ACK拉取数据
						}
						else
						{
							//配置不正确
							External_Magnetometer_Operation_Counter = 100;
							I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
						}
					}
					else if( External_Magnetometer_Operation_Counter == External_MagnetoMeters[ Current_Scan_External_Magnetometer ].configuration_count - 1 )
					{
						//拉取倒数第二个配置
						//用NAK回应 并 发送stop
						uint8_t cfg = I2CMasterDataGet(I2C1_BASE);
						if( cfg == External_MagnetoMeters[ Current_Scan_External_Magnetometer ].configurations[ External_Magnetometer_Operation_Counter - 1 ] )
						{
							//配置正确
							++External_Magnetometer_Operation_Counter;
							I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH); // 继续发送ACK拉取数据
						}
						else
						{
							//配置不正确
							External_Magnetometer_Operation_Counter = 100;
							I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
						}
					}
					else if( External_Magnetometer_Operation_Counter == External_MagnetoMeters[ Current_Scan_External_Magnetometer ].configuration_count )
					{
						//传输完成 拉取最后一个配置
						uint8_t cfg = I2CMasterDataGet(I2C1_BASE);
						if( cfg == External_MagnetoMeters[ Current_Scan_External_Magnetometer ].configurations[ External_Magnetometer_Operation_Counter - 1 ] )
						{
							//全部配置正确
							//判断外置罗盘存在且配置成功
							//注册外置罗盘
							Current_External_Magnetometer = Current_Scan_External_Magnetometer;
							IMUMagnetometerRegister( External_Magnetometer_Index , External_MagnetoMeters[ Current_Scan_External_Magnetometer ].sensitivity );
						}
						else
						{
							//配置不正确
							//切换到下一个传感器继续检测
							++Current_Scan_External_Magnetometer;
							if( (Current_Scan_External_Magnetometer >= Supported_External_MagnetoMeter_Count) \
								||(External_MagnetoMeters[ Current_Scan_External_Magnetometer ].device_address == 0) )
								Current_Scan_External_Magnetometer = 0;
							External_Magnetometer_Operation_Counter = 0;
						}
					}
					else
					{
						//配置不正确
						//切换到下一个传感器继续检测
						++Current_Scan_External_Magnetometer;
						if( (Current_Scan_External_Magnetometer >= Supported_External_MagnetoMeter_Count) \
							||(External_MagnetoMeters[ Current_Scan_External_Magnetometer ].device_address == 0) )
							Current_Scan_External_Magnetometer = 0;
						External_Magnetometer_Operation_Counter = 0;
					}
				}
				
				break;
			}	//External_Magnetometer_Operation_CheckConfig
			
		}
	}
/*i2c1外置罗盘接收状态机*/
