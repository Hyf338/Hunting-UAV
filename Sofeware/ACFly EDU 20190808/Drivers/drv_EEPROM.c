#include "Basic.h"
#include "drv_EEPROM.h"

#include "eeprom.h"
#include "sysctl.h"

void init_drv_EEPROM()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0); // EEPROM activate
	EEPROMInit(); // EEPROM start
}
