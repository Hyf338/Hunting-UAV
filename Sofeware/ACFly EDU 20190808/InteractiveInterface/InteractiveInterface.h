#pragma once

#include <stdbool.h>

/*LED接口*/
	typedef enum
	{
		LED_status_ready1 ,
		
		LED_status_running1 ,
		LED_status_running2 ,
		
		LED_status_waiting ,
		LED_status_error ,
		
		LED_status_progress ,
	}LED_status;
	typedef enum
	{
		LED_signal_null ,
		
		LED_signal_continue ,
		LED_signal_success ,
		LED_signal_error ,
		
		LED_signal_1 ,
		LED_signal_2 ,
		LED_signal_3 ,
	}LED_signal;

	//0-100渐变
	void Led_setProgress( float progress );
	void Led_setStatus( LED_status status );
	void Led_setSignal( LED_signal signal );
/*LED接口*/
	
/*OLED接口*/
	//刷新屏幕
	void OLED_Update();
	
	//清屏
	void OLED_Clear();
	void OLED_Clear_Lines( unsigned char start , unsigned char count );
	
	//写8x6文字
	bool OLED_Draw_Str8x6( const char* str , unsigned char StRow , unsigned char StColumn );	
	//写16x8文字
	bool OLED_Draw_Str16x8( const char* str , unsigned char StRow , unsigned char StColumn );
	
	//画勾叉
	bool OLED_Draw_TickCross8x6( bool Tick, unsigned char StRow , unsigned char StColumn );
	//画点
	bool OLED_Draw_Point8x6( unsigned char StRow , unsigned char StColumn );
	
	//画竖直进度条
	bool OLED_Draw_VerticalProgressBar24x14( float progress , unsigned char StRow , unsigned char StColumn );
/*OLED接口*/