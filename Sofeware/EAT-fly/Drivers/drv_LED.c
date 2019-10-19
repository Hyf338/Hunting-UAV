#include "Basic.h"
#include "drv_LED.h"

#include "sysctl.h"
#include "pin_map.h"
#include "hw_memmap.h"
#include "gpio.h"
#include "pwm.h"

#include "STS.h"
#include "InteractiveInterface.h"

static inline void Buzzer( bool on )
{
	if( on )
		PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);
	else
		PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, false);
}

static inline void LED_Red( float on )
{
	if( on > 100 )
		on = 100;
	else if( on < 0 )
		on = 0;
	PWMPulseWidthSet( PWM1_BASE , PWM_OUT_7 , (5000 - on*49.99f) );
}
static inline void LED_Green( bool on )
{
	if( on )
		GPIOPinWrite( GPIO_PORTA_BASE, GPIO_PIN_4 , 0 );
	else
		GPIOPinWrite( GPIO_PORTA_BASE, GPIO_PIN_4 , GPIO_PIN_4 );
}
static inline void LED_Blue( bool on )
{
	if( on )
		GPIOPinWrite( GPIO_PORTA_BASE, GPIO_PIN_3 , 0 );
	else
		GPIOPinWrite( GPIO_PORTA_BASE, GPIO_PIN_3 , GPIO_PIN_3 );
}

#define LED_Task_Time 0.02f
static void LED_server( unsigned int Task_ID );
void init_drv_LED()
{
	//LED Red(PF3)
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
//  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);	
	
	SysCtlPeripheralEnable( SYSCTL_PERIPH_PWM1 );
	SysCtlPWMClockSet(SYSCTL_PWMDIV_8);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlDelay(2); 	
	GPIOPinConfigure(GPIO_PF3_M1PWM7);	
	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);//M0PWM0	
	PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);	
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 5000);
  PWMGenEnable(PWM1_BASE, PWM_GEN_3);	
	PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);
	
	//LED Green(PA4)
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4);	
	
	//LED Blue(PA3)
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);	
	
	//蜂鸣器初始化(PF2 M1PWM6)
	SysCtlPeripheralEnable( SYSCTL_PERIPH_PWM1 );
	SysCtlPWMClockSet(SYSCTL_PWMDIV_8);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlDelay(2); 	
	GPIOPinConfigure(GPIO_PF2_M1PWM6);	
	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);//M0PWM0	
	PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, false);
	PWMPulseWidthSet( PWM1_BASE , PWM_OUT_6 , 2500 );
	
	//打开LED
	LED_Red(100);
	LED_Green(true);
	LED_Blue(true);
	
	STS_Add_Task( STS_Task_Trigger_Mode_RoughTime , 0.02f , 0 , LED_server );
}

static float led_progress;
static LED_status led_status = LED_status_ready1;
static float led_counter = 0;
static LED_signal led_signal = LED_signal_null;
static float led_signal_counter = 0;

void Led_setProgress( float progress )
{
	led_status = LED_status_progress;
	led_counter = 0;
	led_progress = progress;
}
void Led_setStatus( LED_status status )
{
	if( led_status != status )
		led_counter = 0;
	led_status = status;	
}
void Led_setSignal( LED_signal signal )
{
	if( led_signal != signal )
		led_signal_counter = 0;
	led_signal = signal;	
}

static void LED_server( unsigned int Task_ID )
{
	if( led_signal != LED_signal_null )
	{
		bool control_light = true;
		switch( led_signal )
		{
			case LED_signal_continue:
			{
				if( led_signal_counter < 0.2f )			
				{
					LED_Red(0);
					LED_Green(false);
					LED_Blue(true);
					Buzzer( true );
				}
				else if( led_signal_counter < 0.4f )			
				{
					LED_Red(0);
					LED_Green(false);
					LED_Blue(false);
					Buzzer( false );
				}
				else if( led_signal_counter < 0.6f )			
				{
					LED_Red(0);
					LED_Green(false);
					LED_Blue(true);
				}
				else if( led_signal_counter < 0.8f )
				{
					LED_Red(0);
					LED_Green(false);
					LED_Blue(false);
				}
				else
					led_signal = LED_signal_null;
				
				break;
			}
			
			case LED_signal_success:
			{
				if( led_signal_counter < 0.2f )			
				{
					LED_Red(0);
					LED_Green(true);
					LED_Blue(false);
					Buzzer( true );
				}
				else if( led_signal_counter < 0.4f )			
				{
					LED_Red(0);
					LED_Green(false);
					LED_Blue(false);
					Buzzer( false );
				}
				else if( led_signal_counter < 0.6f )			
				{
					LED_Red(0);
					LED_Green(true);
					LED_Blue(false);
					Buzzer( true );
				}
				else if( led_signal_counter < 0.8f )
				{
					LED_Red(0);
					LED_Green(false);
					LED_Blue(false);
					Buzzer( false );
				}
				else
					led_signal = LED_signal_null;
				
				break;
			}
			
			case LED_signal_error:
			{
				if( led_signal_counter < 0.2f )			
				{
					LED_Red(100);
					LED_Green(false);
					LED_Blue(false);
					Buzzer( true );
				}
				else if( led_signal_counter < 0.4f )			
				{
					LED_Red(0);
					LED_Green(false);
					LED_Blue(false);
				}
				else if( led_signal_counter < 0.6f )			
				{
					LED_Red(100);
					LED_Green(false);
					LED_Blue(false);
				}
				else if( led_signal_counter < 0.8f )
				{
					LED_Red(0);
					LED_Green(false);
					LED_Blue(false);
					Buzzer( false );
				}
				else
					led_signal = LED_signal_null;
				
				break;
			}
			
			case LED_signal_1:
			{
				control_light = false;
				if( led_signal_counter < 0.05f )			
					Buzzer( true );
				else if( led_signal_counter < 0.1f )
					Buzzer( false );
				else
					led_signal = LED_signal_null;
				
				break;
			}
			
			case LED_signal_2:
			{
				control_light = false;
				if( led_signal_counter < 0.05f )			
					Buzzer( true );
				else if( led_signal_counter < 0.1f )
					Buzzer( false );
				else if( led_signal_counter < 0.15f )			
					Buzzer( true );
				else if( led_signal_counter < 0.2f )
					Buzzer( false );
				else
					led_signal = LED_signal_null;
				
				break;
			}
			
			case LED_signal_3:
			{
				control_light = false;
				if( led_signal_counter < 0.05f )			
					Buzzer( true );
				else if( led_signal_counter < 0.1f )
					Buzzer( false );
				else if( led_signal_counter < 0.15f )			
					Buzzer( true );
				else if( led_signal_counter < 0.2f )
					Buzzer( false );
				else if( led_signal_counter < 0.25f )			
					Buzzer( true );
				else if( led_signal_counter < 0.3f )
					Buzzer( false );
				else
					led_signal = LED_signal_null;
				
				break;
			}
			
			case LED_signal_null:
				break;
		}
		led_signal_counter += LED_Task_Time;
		if( control_light )
			return;
	}
	
	switch( led_status )
	{
		case LED_status_ready1:
		{
			led_counter += LED_Task_Time;
			if( led_counter >= 1.5f )
				led_counter = 0;
			
			if( led_counter < 0.5f )			
			{
				LED_Red(0);
				LED_Green(false);
				LED_Blue(true);
			}
			else if( led_counter < 1.0f )			
			{
				LED_Red(0);
				LED_Green(true);
				LED_Blue(false);
			}
			else	
			{
				LED_Red(100);
				LED_Green(false);
				LED_Blue(false);
			}
			
			break;
		}
		
		case LED_status_running1:
		{
			led_counter += LED_Task_Time;
			if( led_counter >= 0.5f )
				led_counter = 0;
			
			if( led_counter < 0.1f )			
			{
				LED_Red(100);
				LED_Green(false);
				LED_Blue(false);
			}
			else if( led_counter < 0.2f )			
			{
				LED_Red(0);
				LED_Green(true);
				LED_Blue(false);
			}
			else	
			{
				LED_Red(0);
				LED_Green(false);
				LED_Blue(true);
			}
			
			break;
		}
		
		case LED_status_running2:
		{
			led_counter += LED_Task_Time;
			if( led_counter >= 0.3f )
				led_counter = 0;
			
			if( led_counter < 0.1f )			
			{
				LED_Red(100);
				LED_Green(false);
				LED_Blue(false);
			}
			else if( led_counter < 0.2f )			
			{
				LED_Red(0);
				LED_Green(true);
				LED_Blue(false);
			}
			else	
			{
				LED_Red(0);
				LED_Green(false);
				LED_Blue(true);
			}
			
			break;
		}
		
		case LED_status_waiting:
		{
			LED_Red(0);
			LED_Green(false);
			LED_Blue(true);
			break;
		}
		
		case LED_status_error:
		{
			LED_Red(100);
			LED_Green(false);
			LED_Blue(false);
			break;
		}
		
		case LED_status_progress:
		{
			if( led_progress < 0.1f )
			{
				LED_Red(100);
				LED_Green(false);
				LED_Blue(false);
			}
			else
			{
				LED_Red(led_progress);
				LED_Green(false);
				LED_Blue(true);
			}
			break;
		}
	}
}