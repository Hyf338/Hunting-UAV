#include "Basic.h"
#include "M00_init.h"

#include "InteractiveInterface.h"
#include "MeasurementSystem.h"
#include "AC_Math.h"

typedef struct
{
	unsigned short exit_counter;
}MODE_INF;
static MODE_INF* Mode_Inf;

static void M00_init_MainFunc();
static void M00_init_enter();
static void M00_init_exit();
const Mode M00_init = 
{
	10 , //mode frequency
	M00_init_enter , //enter
	M00_init_exit ,	//exit
	M00_init_MainFunc ,	//mode main func
};

static void M00_init_enter()
{
	Led_setStatus( LED_status_running1 );
	OLED_Draw_Str8x6("Att Rdy:",5,64);
	OLED_Draw_Str8x6("Pos Rdy:",7,64);
	OLED_Draw_TickCross8x6( false , 5 , 118 );
	OLED_Draw_TickCross8x6( false , 7 , 118 );
	OLED_Update();
	
	//初始化模式变量
	Mode_Inf = malloc( sizeof( MODE_INF ) );
	Mode_Inf->exit_counter = 0;
}

static void M00_init_exit()
{
	free( Mode_Inf );
	OLED_Clear();
	OLED_Update();
}

static void M00_init_MainFunc()
{
	OLED_Draw_TickCross8x6( get_Attitude_Measurement_System_Status() == Measurement_System_Status_Ready , 5 , 118 );
	OLED_Draw_TickCross8x6( get_Altitude_Measurement_System_Status() == Measurement_System_Status_Ready , 7 , 118 );
	OLED_Update();
	if( get_Altitude_Measurement_System_Status() == Measurement_System_Status_Ready )
	{	
		if( ++Mode_Inf->exit_counter >= 15 )
		{
			Led_setSignal( LED_signal_success );
			change_Mode( 1 );
		}
	}
}