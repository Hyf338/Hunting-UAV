#include "Basic.h"
#include "drv_uDMA.h"

#include "sysctl.h"
#include "udma.h"

//uDMA Control Table内存
//注意：如需用alternate DMA功能需将空间扩大一倍
#if defined(ewarm)
	#pragma data_alignment=1024
	uint8_t uDMA_buf[1024];
#elif defined(ccs)
	#pragma DATA_ALIGN(ui8ControlTable, 1024)
	uint8_t uDMA_buf[1024];
#else
	uint8_t uDMA_buf[1024] __attribute__ ((aligned(1024)));
#endif

void init_drv_uDMA()
{
	//开启uDMA时钟 masterenable
	SysCtlPeripheralEnable( SYSCTL_PERIPH_UDMA );
	uDMAEnable();
	
	//设定uDMA Control Table
	uDMAControlBaseSet( uDMA_buf );
}