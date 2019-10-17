#include "Ctrl_main.h"

#include "Ctrl_Attitude.h"
#include "Ctrl_Position.h"

void init_ControlSystem()
{
	init_ctrl_Attitude();
	init_ctrl_Position();
}

void ctrl_main()
{
	ctrl_Position();
	ctrl_Attitude();
}