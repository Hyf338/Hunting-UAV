#include "Modes.h"

#include "STS.h"

//0-9为非飞行和校准模式
#include "M00_init.h"
#include "M01_Ground.h"

//10-19为校准模式
#include "M10_RCCalib.h"
#include "M12_AccCalib.h"
#include "M13_MagCalib.h"
#include "M15_HorizontalCalib.h"

//30-39为飞行模式
#include "M30_Att.h"
#include "M32_PosCtrl.h"
#include "M35_Auto1.h"

static unsigned int Mode_Task_ID;
static uint8_t current_mode = 0;
static Mode Modes[50] = {0};

static void Modes_Server( unsigned int Task_ID )
{
	Modes[ current_mode ].mode_main_func();
}

bool change_Mode( unsigned char mode )
{
	if( Modes[ mode ].mode_main_func == 0 )
		return false;
	Modes[ current_mode ].mode_exit();
	current_mode = mode;
	STS_Change_Task_Mode( Mode_Task_ID , STS_Task_Trigger_Mode_RoughTime , 1.0f / Modes[current_mode].mode_frequency , 0 );
	Modes[ current_mode ].mode_enter();
	return true;
}
unsigned char get_current_Mode()
{
	return current_mode;
}

void init_Modes()
{
	/*初始化模式*/
		//0-9为非飞行和校准模式
		Modes[0] = M00_init;
		Modes[1] = M01_Ground;
	
		//10-19为校准模式
		Modes[10] = M10_RCCalib;
		Modes[12] = M12_AccCalib;
		Modes[13] = M13_MagCalib;
		Modes[15] = M15_HorizontalCalib;

		//30-39为飞行模式
		Modes[30] = M30_Att;
		Modes[32] = M32_PosCtrl;
		Modes[35] = M35_Auto1;
	/*初始化模式*/
	
	current_mode = 0;
	Mode_Task_ID = STS_Add_Task( STS_Task_Trigger_Mode_RoughTime , 1.0f / Modes[current_mode].mode_frequency , 0 , Modes_Server );
	Modes[ current_mode ].mode_enter();
}