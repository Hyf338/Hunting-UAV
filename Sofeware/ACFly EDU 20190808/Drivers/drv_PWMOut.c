#include "drv_PWMOut.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "sysctl.h"
#include "pin_map.h"
#include "hw_memmap.h"
#include "pwm.h"
#include "gpio.h"

void init_drv_PWMOut()
{
	SysCtlPWMClockSet(SYSCTL_PWMDIV_8); // Set divider to 80M/8=10M=0.1us
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); // Enable PWM peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // Enable GPIOB peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//Í¬Ê±ÊÇuart5	¡¶BY¡·
	SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated


	GPIOPinConfigure(GPIO_PB6_M0PWM0);
	GPIOPinConfigure(GPIO_PB7_M0PWM1);	
	GPIOPinConfigure(GPIO_PB4_M0PWM2);
	GPIOPinConfigure(GPIO_PB5_M0PWM3);
	GPIOPinConfigure(GPIO_PE4_M0PWM4);
	GPIOPinConfigure(GPIO_PE5_M0PWM5);
	GPIOPinConfigure(GPIO_PD0_M0PWM6);
	GPIOPinConfigure(GPIO_PD1_M0PWM7);

	
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);//M0PWM0
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);//M0PWM1	
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);//M0PWM2
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);//M0PWM3	
	GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);//M0PWM4		//U5RX		¡¶BY¡·
	GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);//M0PWM5		//U5TX		¡¶BY¡·
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);//M0PWM6
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);//M0PWM7	
	

	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 50000); // Set the period
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 50000);	
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 50000);	
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 50000);   

	
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);
	PWMGenEnable(PWM0_BASE, PWM_GEN_1);	
	PWMGenEnable(PWM0_BASE, PWM_GEN_2);
	PWMGenEnable(PWM0_BASE, PWM_GEN_3);		
		
	
	PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT| PWM_OUT_2_BIT|PWM_OUT_3_BIT|PWM_OUT_4_BIT|PWM_OUT_5_BIT|PWM_OUT_6_BIT|PWM_OUT_7_BIT, true);	
	
	PWM_PullDownAll();
}

void PWM_Out(float out[8])
{
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,out[0]*100+10000);//PB7 
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,out[1]*100+10000);//PB6 
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_7,out[2]*100+10000);//PD1 
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_6,out[3]*100+10000);//PD0 
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_5,out[4]*100+10000);//PE5  
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_4,out[5]*100+10000);//PE4  
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,out[6]*100+10000);//PB4  
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,out[7]*100+10000);//PB5 	
}
void PWM_PullDownAll()
{
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,10000);//PB7 
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,10000);//PB6 
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_7,10000);//PD1 
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_6,10000);//PD0 
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_5,10000);//PE5  
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_4,10000);//PE4  
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,10000);//PB4  
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,10000);//PB5 	
}
void PWM_PullUpAll()
{
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,20000);//PB7 
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,20000);//PB6 
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_7,20000);//PD1 
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_6,20000);//PD0 
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_5,20000);//PE5  
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_4,20000);//PE4  
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,20000);//PB4  
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,20000);//PB5
}