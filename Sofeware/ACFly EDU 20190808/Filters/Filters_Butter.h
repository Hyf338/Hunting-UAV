#pragma once

#include <stdbool.h>

/*二阶ButterWorth低通*/
	typedef struct
	{
		bool available;	
		float k_1[3];
	
		float in_1[2];
		float out_1[2];
	}Filter_Butter2_LP_float;
	typedef struct
	{
		bool available;	
		double k_1[3];

		double in_1[2];
		double out_1[2];
	}Filter_Butter2_LP_double;
	
	bool Filter_Butter2_LP_float_setCutoffFrequency( Filter_Butter2_LP_float* filter , float sample_freq , float cutoff_freq );
	bool Filter_Butter2_LP_double_setCutoffFrequency( Filter_Butter2_LP_double* filter , double sample_freq , double cutoff_freq );
	
	//复位滤波器
	static inline void Filter_Butter2_LP_float_reset( Filter_Butter2_LP_float* filter , float initial_value )
	{
		filter->in_1[0] = filter->in_1[1] = initial_value;
		filter->out_1[0] = filter->out_1[1] = initial_value;
	}
	static inline void Filter_Butter2_LP_double_reset( Filter_Butter2_LP_double* filter , double initial_value )
	{
		filter->in_1[0] = filter->in_1[1] = initial_value;
		filter->out_1[0] = filter->out_1[1] = initial_value;
	}
	
	//初始化滤波器
	static inline void Filter_Butter2_LP_float_init( Filter_Butter2_LP_float* filter , float sample_freq , float cutoff_freq )
	{
		Filter_Butter2_LP_float_reset( filter , 0 );
		Filter_Butter2_LP_float_setCutoffFrequency( filter , sample_freq , cutoff_freq );
	}
	static inline void Filter_Butter2_LP_double_init( Filter_Butter2_LP_double* filter , double sample_freq , double cutoff_freq )
	{
		Filter_Butter2_LP_double_reset( filter , 0 );
		Filter_Butter2_LP_double_setCutoffFrequency( filter , sample_freq , cutoff_freq );
	}
	
	//获取滤波结果
	static inline float Filter_Butter2_LP_float_getResult( const Filter_Butter2_LP_float* filter )
	{
		return filter->out_1[0];
	}
	static inline double Filter_Butter2_LP_double_getResult( const Filter_Butter2_LP_double* filter )
	{
		return filter->out_1[0];
	}
	
	//复制滤波器
	static inline void Filter_Butter2_LP_float_setCutoffFrequency_from( Filter_Butter2_LP_float* filter , const Filter_Butter2_LP_float* filter_cp )
	{
		filter->available = filter_cp->available;
		filter->k_1[0] = filter_cp->k_1[0];	filter->k_1[1] = filter_cp->k_1[1];	filter->k_1[2] = filter_cp->k_1[2];
	}
	static inline void Filter_Butter2_LP_double_setCutoffFrequency_from( Filter_Butter2_LP_double* filter , const Filter_Butter2_LP_double* filter_cp )
	{
		filter->available = filter_cp->available;
		filter->k_1[0] = filter_cp->k_1[0];	filter->k_1[1] = filter_cp->k_1[1];	filter->k_1[2] = filter_cp->k_1[2];
	}
	
	//滤波
	static inline float Filter_Butter2_LP_float_run( Filter_Butter2_LP_float* filter , float newdata )
	{
		if( filter->available )
		{		
			float out_1_2 = filter->out_1[1];	filter->out_1[1] = filter->out_1[0];
			filter->out_1[0] = filter->k_1[0]*( newdata + 2*filter->in_1[0] + filter->in_1[1] ) - filter->k_1[1]*filter->out_1[1] - filter->k_1[2]*out_1_2;			
			filter->in_1[1] = filter->in_1[0];	filter->in_1[0] = newdata;
		}
		else
			Filter_Butter2_LP_float_reset( filter , newdata );
		return filter->out_1[0];
	}
	static inline double Filter_Butter2_LP_double_run( Filter_Butter2_LP_double* filter , double newdata )
	{
		if( filter->available )
		{
			double out_1_2 = filter->out_1[1];	filter->out_1[1] = filter->out_1[0];
			filter->out_1[0] = filter->k_1[0]*( newdata + 2*filter->in_1[0] + filter->in_1[1] ) - filter->k_1[1]*filter->out_1[1] - filter->k_1[2]*out_1_2;			
			filter->in_1[1] = filter->in_1[0];	filter->in_1[0] = newdata;
		}
		else
			Filter_Butter2_LP_double_reset( filter , newdata );
		return filter->out_1[0];
	}
/*二阶ButterWorth低通*/

/*四阶ButterWorth低通*/
	typedef struct
	{
		bool available;	
		float k_1[3];
		float k_2[3];

		float in_1[2];
		float out_1[2];
		float in_2[2];
		float out_2[2];
	}Filter_Butter4_LP_float;
	typedef struct
	{
		bool available;	
		double k_1[3];
		double k_2[3];

		double in_1[2];
		double out_1[2];
		double in_2[2];
		double out_2[2];
	}Filter_Butter4_LP_double;
	bool Filter_Butter4_LP_float_setCutoffFrequency( Filter_Butter4_LP_float* filter , float sample_freq , float cutoff_freq );
	bool Filter_Butter4_LP_double_setCutoffFrequency( Filter_Butter4_LP_double* filter , double sample_freq , double cutoff_freq );
	
	static inline void Filter_Butter4_LP_float_reset( Filter_Butter4_LP_float* filter , float initial_value )
	{
		filter->in_1[0] = filter->in_1[1] = initial_value;
		filter->out_1[0] = filter->out_1[1] = initial_value;
		
		filter->in_2[0] = filter->in_2[1] = initial_value;
		filter->out_2[0] = filter->out_2[1] = initial_value;
	}
	static inline void Filter_Butter4_LP_double_reset( Filter_Butter4_LP_double* filter , double initial_value )
	{
		filter->in_1[0] = filter->in_1[1] = initial_value;
		filter->out_1[0] = filter->out_1[1] = initial_value;
		
		filter->in_2[0] = filter->in_2[1] = initial_value;
		filter->out_2[0] = filter->out_2[1] = initial_value;
	}
	
	//初始化滤波器
	static inline void Filter_Butter4_LP_float_init( Filter_Butter4_LP_float* filter , float sample_freq , float cutoff_freq )
	{
		Filter_Butter4_LP_float_reset( filter , 0 );
		Filter_Butter4_LP_float_setCutoffFrequency( filter , sample_freq , cutoff_freq );
	}
	static inline void Filter_Butter4_LP_double_init( Filter_Butter4_LP_double* filter , double sample_freq , double cutoff_freq )
	{
		Filter_Butter4_LP_double_reset( filter , 0 );
		Filter_Butter4_LP_double_setCutoffFrequency( filter , sample_freq , cutoff_freq );
	}
	
	//获取滤波结果
	static inline float Filter_Butter4_LP_float_getResult( const Filter_Butter4_LP_float* filter )
	{
		return filter->out_2[0];
	}
	static inline double Filter_Butter4_LP_double_getResult( const Filter_Butter4_LP_double* filter )
	{
		return filter->out_2[0];
	}
	
	//复制滤波器
	static inline void Filter_Butter4_LP_float_setCutoffFrequency_from( Filter_Butter4_LP_float* filter , const Filter_Butter4_LP_float* filter_cp )
	{
		filter->available = filter_cp->available;
		filter->k_1[0] = filter_cp->k_1[0];	filter->k_1[1] = filter_cp->k_1[1];	filter->k_1[2] = filter_cp->k_1[2];
		filter->k_2[0] = filter_cp->k_2[0];	filter->k_2[1] = filter_cp->k_2[1];	filter->k_2[2] = filter_cp->k_2[2];
	}
	static inline void Filter_Butter4_LP_double_setCutoffFrequency_from( Filter_Butter4_LP_double* filter , const Filter_Butter4_LP_double* filter_cp )
	{
		filter->available = filter_cp->available;
		filter->k_1[0] = filter_cp->k_1[0];	filter->k_1[1] = filter_cp->k_1[1];	filter->k_1[2] = filter_cp->k_1[2];
		filter->k_2[0] = filter_cp->k_2[0];	filter->k_2[1] = filter_cp->k_2[1];	filter->k_2[2] = filter_cp->k_2[2];
	}
	
	//滤波
	static inline float Filter_Butter4_LP_float_run( Filter_Butter4_LP_float* filter , float newdata )
	{
		if( filter->available )
		{
			float out_1_2 = filter->out_1[1];	filter->out_1[1] = filter->out_1[0];
			filter->out_1[0] = filter->k_1[0]*( newdata + 2*filter->in_1[0] + filter->in_1[1] ) - filter->k_1[1]*filter->out_1[1] - filter->k_1[2]*out_1_2;			
			filter->in_1[1] = filter->in_1[0];	filter->in_1[0] = newdata;
			
			newdata = filter->out_1[0];
			float out_2_2 = filter->out_2[1];	filter->out_2[1] = filter->out_2[0];
			filter->out_2[0] = filter->k_2[0]*( newdata + 2*filter->in_2[0] + filter->in_2[1] ) - filter->k_2[1]*filter->out_2[1] - filter->k_2[2]*out_2_2;			
			filter->in_2[1] = filter->in_2[0];	filter->in_2[0] = newdata;
		}
		else
			Filter_Butter4_LP_float_reset( filter , newdata );
		return filter->out_2[0];
	}
	static inline double Filter_Butter4_LP_double_run( Filter_Butter4_LP_double* filter , double newdata )
	{
		if( filter->available )
		{
			double out_1_2 = filter->out_1[1];	filter->out_1[1] = filter->out_1[0];
			filter->out_1[0] = filter->k_1[0]*( newdata + 2*filter->in_1[0] + filter->in_1[1] ) - filter->k_1[1]*filter->out_1[1] - filter->k_1[2]*out_1_2;			
			filter->in_1[1] = filter->in_1[0];	filter->in_1[0] = newdata;
			
			newdata = filter->out_1[0];
			double out_2_2 = filter->out_2[1];	filter->out_2[1] = filter->out_2[0];
			filter->out_2[0] = filter->k_2[0]*( newdata + 2*filter->in_2[0] + filter->in_2[1] ) - filter->k_2[1]*filter->out_2[1] - filter->k_2[2]*out_2_2;			
			filter->in_2[1] = filter->in_2[0];	filter->in_2[0] = newdata;
		}
		else
			Filter_Butter4_LP_double_reset( filter , newdata );
		return filter->out_2[0];
	}
/*四阶ButterWorth低通*/

/*八阶ButterWorth低通*/
	typedef struct
	{	
		float k_1[3];
		float k_2[3];
		float k_3[3];
		float k_4[3];

		float in_1[2];
		float out_1[2];
		float in_2[2];
		float out_2[2];
		float in_3[2];
		float out_3[2];
		float in_4[2];
		float out_4[2];
		
		bool available;	
	}Filter_Butter8_LP_float;
	typedef struct
	{		
		double k_1[3];
		double k_2[3];
		double k_3[3];
		double k_4[3];

		double in_1[2];
		double out_1[2];
		double in_2[2];
		double out_2[2];
		double in_3[2];
		double out_3[2];
		double in_4[2];
		double out_4[2];
		
		bool available;	
	}Filter_Butter8_LP_double;
	bool Filter_Butter8_LP_float_setCutoffFrequency( Filter_Butter8_LP_float* filter , float sample_freq , float cutoff_freq );
	bool Filter_Butter8_LP_double_setCutoffFrequency( Filter_Butter8_LP_double* filter , double sample_freq , double cutoff_freq );
	
	//复位滤波器
	static inline void Filter_Butter8_LP_float_reset( Filter_Butter8_LP_float* filter , float initial_value )
	{
		filter->in_1[0] = filter->in_1[1] = initial_value;
		filter->out_1[0] = filter->out_1[1] = initial_value;
		
		filter->in_2[0] = filter->in_2[1] = initial_value;
		filter->out_2[0] = filter->out_2[1] = initial_value;
		
		filter->in_3[0] = filter->in_3[1] = initial_value;
		filter->out_3[0] = filter->out_3[1] = initial_value;
		
		filter->in_4[0] = filter->in_4[1] = initial_value;
		filter->out_4[0] = filter->out_4[1] = initial_value;
	}
	static inline void Filter_Butter8_LP_double_reset( Filter_Butter8_LP_double* filter , double initial_value )
	{
		filter->in_1[0] = filter->in_1[1] = initial_value;
		filter->out_1[0] = filter->out_1[1] = initial_value;
		
		filter->in_2[0] = filter->in_2[1] = initial_value;
		filter->out_2[0] = filter->out_2[1] = initial_value;
		
		filter->in_3[0] = filter->in_3[1] = initial_value;
		filter->out_3[0] = filter->out_3[1] = initial_value;
		
		filter->in_4[0] = filter->in_4[1] = initial_value;
		filter->out_4[0] = filter->out_4[1] = initial_value;
	}
	
	//初始化滤波器
	static inline void Filter_Butter8_LP_float_init( Filter_Butter8_LP_float* filter , float sample_freq , float cutoff_freq )
	{
		Filter_Butter8_LP_float_reset( filter , 0 );
		Filter_Butter8_LP_float_setCutoffFrequency( filter , sample_freq , cutoff_freq );
	}
	static inline void Filter_Butter8_LP_double_init( Filter_Butter8_LP_double* filter , double sample_freq , double cutoff_freq )
	{
		Filter_Butter8_LP_double_reset( filter , 0 );
		Filter_Butter8_LP_double_setCutoffFrequency( filter , sample_freq , cutoff_freq );
	}
	
	//获取滤波结果
	static inline float Filter_Butter8_LP_float_getResult( const Filter_Butter8_LP_float* filter )
	{
		return filter->out_4[0];
	}
	static inline double Filter_Butter8_LP_double_getResult( const Filter_Butter8_LP_double* filter )
	{
		return filter->out_4[0];
	}
	
	//复制滤波器
	static inline void Filter_Butter8_LP_float_setCutoffFrequency_from( Filter_Butter8_LP_float* filter , const Filter_Butter8_LP_float* filter_cp )
	{
		filter->available = filter_cp->available;
		filter->k_1[0] = filter_cp->k_1[0];	filter->k_1[1] = filter_cp->k_1[1];	filter->k_1[2] = filter_cp->k_1[2];
		filter->k_2[0] = filter_cp->k_2[0];	filter->k_2[1] = filter_cp->k_2[1];	filter->k_2[2] = filter_cp->k_2[2];
		filter->k_3[0] = filter_cp->k_3[0];	filter->k_3[1] = filter_cp->k_3[1];	filter->k_3[2] = filter_cp->k_3[2];
		filter->k_4[0] = filter_cp->k_4[0];	filter->k_4[1] = filter_cp->k_4[1];	filter->k_4[2] = filter_cp->k_4[2];
	}
	static inline void Filter_Butter8_LP_double_setCutoffFrequency_from( Filter_Butter8_LP_double* filter , const Filter_Butter8_LP_double* filter_cp )
	{
		filter->available = filter_cp->available;
		filter->k_1[0] = filter_cp->k_1[0];	filter->k_1[1] = filter_cp->k_1[1];	filter->k_1[2] = filter_cp->k_1[2];
		filter->k_2[0] = filter_cp->k_2[0];	filter->k_2[1] = filter_cp->k_2[1];	filter->k_2[2] = filter_cp->k_2[2];
		filter->k_3[0] = filter_cp->k_3[0];	filter->k_3[1] = filter_cp->k_3[1];	filter->k_3[2] = filter_cp->k_3[2];
		filter->k_4[0] = filter_cp->k_4[0];	filter->k_4[1] = filter_cp->k_4[1];	filter->k_4[2] = filter_cp->k_4[2];
	}
	
	//滤波
	static inline float Filter_Butter8_LP_float_run( Filter_Butter8_LP_float* filter , float newdata )
	{
		if( filter->available )
		{
			float out_1_2 = filter->out_1[1];	filter->out_1[1] = filter->out_1[0];
			filter->out_1[0] = filter->k_1[0]*( newdata + 2*filter->in_1[0] + filter->in_1[1] ) - filter->k_1[1]*filter->out_1[1] - filter->k_1[2]*out_1_2;			
			filter->in_1[1] = filter->in_1[0];	filter->in_1[0] = newdata;
			
			newdata = filter->out_1[0];
			float out_2_2 = filter->out_2[1];	filter->out_2[1] = filter->out_2[0];
			filter->out_2[0] = filter->k_2[0]*( newdata + 2*filter->in_2[0] + filter->in_2[1] ) - filter->k_2[1]*filter->out_2[1] - filter->k_2[2]*out_2_2;			
			filter->in_2[1] = filter->in_2[0];	filter->in_2[0] = newdata;
			
			newdata = filter->out_2[0];
			float out_3_2 = filter->out_3[1];	filter->out_3[1] = filter->out_3[0];
			filter->out_3[0] = filter->k_3[0]*( newdata + 2*filter->in_3[0] + filter->in_3[1] ) - filter->k_3[1]*filter->out_3[1] - filter->k_3[2]*out_3_2;			
			filter->in_3[1] = filter->in_3[0];	filter->in_3[0] = newdata;
			
			newdata = filter->out_3[0];
			float out_4_2 = filter->out_4[1];	filter->out_4[1] = filter->out_4[0];
			filter->out_4[0] = filter->k_4[0]*( newdata + 2*filter->in_4[0] + filter->in_4[1] ) - filter->k_4[1]*filter->out_4[1] - filter->k_4[2]*out_4_2;			
			filter->in_4[1] = filter->in_4[0];	filter->in_4[0] = newdata;
		}
		else
			Filter_Butter8_LP_float_reset( filter , newdata );
		return filter->out_4[0];
	}
	static inline double Filter_Butter8_LP_double_run( Filter_Butter8_LP_double* filter , double newdata )
	{
		if( filter->available )
		{
			float out_1_2 = filter->out_1[1];	filter->out_1[1] = filter->out_1[0];
			filter->out_1[0] = filter->k_1[0]*( newdata + 2*filter->in_1[0] + filter->in_1[1] ) - filter->k_1[1]*filter->out_1[1] - filter->k_1[2]*out_1_2;			
			filter->in_1[1] = filter->in_1[0];	filter->in_1[0] = newdata;
			
			newdata = filter->out_1[0];
			float out_2_2 = filter->out_2[1];	filter->out_2[1] = filter->out_2[0];
			filter->out_2[0] = filter->k_2[0]*( newdata + 2*filter->in_2[0] + filter->in_2[1] ) - filter->k_2[1]*filter->out_2[1] - filter->k_2[2]*out_2_2;			
			filter->in_2[1] = filter->in_2[0];	filter->in_2[0] = newdata;
			
			newdata = filter->out_2[0];
			float out_3_2 = filter->out_3[1];	filter->out_3[1] = filter->out_3[0];
			filter->out_3[0] = filter->k_3[0]*( newdata + 2*filter->in_3[0] + filter->in_3[1] ) - filter->k_3[1]*filter->out_3[1] - filter->k_3[2]*out_3_2;			
			filter->in_3[1] = filter->in_3[0];	filter->in_3[0] = newdata;
			
			newdata = filter->out_3[0];
			float out_4_2 = filter->out_4[1];	filter->out_4[1] = filter->out_4[0];
			filter->out_4[0] = filter->k_4[0]*( newdata + 2*filter->in_4[0] + filter->in_4[1] ) - filter->k_4[1]*filter->out_4[1] - filter->k_4[2]*out_4_2;			
			filter->in_4[1] = filter->in_4[0];	filter->in_4[0] = newdata;
		}
		else
			Filter_Butter8_LP_double_reset( filter , newdata );
		return filter->out_4[0];
	}
/*八阶ButterWorth低通*/