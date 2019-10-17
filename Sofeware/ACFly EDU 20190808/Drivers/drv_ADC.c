#include "drv_adc.h"
#include "Sensors.h"
#include "Configurations.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "sysctl.h"
#include "adc.h"
#include "STS.h"
#include <gpio.h>
#include "TM4C123GH6PM.h"
#include "hw_ints.h"
#include "interrupt.h"

static void ADC_Server(unsigned int Task_ID);
void init_drv_ADC(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	GPIOPinTypeADC(ADC0_BASE,GPIO_PIN_2);
	ADCSequenceConfigure(ADC0_BASE,3,ADC_TRIGGER_PROCESSOR,0);
	ADCHardwareOversampleConfigure(ADC0_BASE,64);
  ADCSequenceStepConfigure(ADC0_BASE,3,0,ADC_CTL_CH5|ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE,3);

	STS_Add_Task( STS_Task_Trigger_Mode_RoughTime , 1.0f/50.0f , 0 , ADC_Server );
}

static float MainBatteryVoltage = 0;
static void ADC_Server(unsigned int Task_ID)
{
	if( ( ADC0->SSFSTAT3 & (1<<8) ) == 0 )
	{
		//ADC FIFOÒÑÂú
		uint32_t ADCvalue;
		while( ( ADC0->SSFSTAT3 & (1<<8) ) == 0 )
			ADCvalue = ADC0->SSFIFO3;
		MainBatteryVoltage += 1.0f*1.0f/50.0f * ( (float)ADCvalue/4095.0f*3.3f*Cfg_get_BatVoltageADCMag() - MainBatteryVoltage );
	}
	
  ADCProcessorTrigger(ADC0_BASE , 3);
}

float getBatteryVoltage()
{
	return MainBatteryVoltage;
}

