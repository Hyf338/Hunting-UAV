#include "Filters_Butter.h"

#include "AC_Math.h"

static inline bool BUT_IIR_calc_freq_f( float* k , const float sample_freq, const float cutoff_freq , const float cp )
{	
	const float Pi = 3.1415926535897932384626433832795f;
	if ( (cutoff_freq <= 0.0001) || (sample_freq <= 2.1 * cutoff_freq) ) {
			// no filtering
		return false;
	}
	float cos_PI_cp = cosf( Pi * cp );
	
	float fr = sample_freq / cutoff_freq;
	float ohm = tanf(Pi/fr);
	float ohm2 = ohm*ohm;
	float c = 1.0f+2.0f*cos_PI_cp*ohm + ohm2;
	float inv_c = 1.0f / c;
	k[0] = ohm2 * inv_c;
	k[1] = 2.0f*(ohm2-1.0f) * inv_c;
	k[2] = (1.0f-2.0f*cos_PI_cp*ohm+ohm2) * inv_c;
	
	return true;
}
static inline bool BUT_IIR_calc_freq_d( double* k , const double sample_freq, const double cutoff_freq , const double cp )
{	
	const double Pi = 3.1415926535897932384626433832795;
	if ( (cutoff_freq <= 0.0001) || (sample_freq <= 1.99 * cutoff_freq) ) {
			// no filtering
		return false;
	}
	double cos_PI_cp = cos( Pi * cp );
	
	double fr = sample_freq / cutoff_freq;
	double ohm = tan(Pi/fr);
	double ohm2 = ohm*ohm;
	double c = 1.0+2.0*cos_PI_cp*ohm + ohm2;
	double inv_c = 1.0 / c;
	k[0] = ohm2 * inv_c;
	k[1] = 2.0*(ohm2-1.0) * inv_c;
	k[2] = (1.0-2.0*cos_PI_cp*ohm+ohm2) * inv_c;
	
	return true;
}

/*二阶ButterWorth低通*/
	bool Filter_Butter2_LP_float_setCutoffFrequency( Filter_Butter2_LP_float* filter , float sample_freq , float cutoff_freq )
	{
		filter->available = BUT_IIR_calc_freq_f( filter->k_1 , sample_freq , cutoff_freq , 1.0f / 4 );
		return filter->available;
	}
	bool Filter_Butter2_LP_double_setCutoffFrequency( Filter_Butter2_LP_double* filter , double sample_freq , double cutoff_freq )
	{
		filter->available = BUT_IIR_calc_freq_d( filter->k_1 , sample_freq , cutoff_freq , 1.0 / 4 );
		return filter->available;
	}
/*二阶ButterWorth低通*/
	
/*四阶ButterWorth低通*/
	bool Filter_Butter4_LP_float_setCutoffFrequency( Filter_Butter4_LP_float* filter , float sample_freq , float cutoff_freq )
	{
		filter->available = BUT_IIR_calc_freq_f( filter->k_1 , sample_freq , cutoff_freq , 1.0f / 8 );
		BUT_IIR_calc_freq_f( filter->k_2 , sample_freq , cutoff_freq , 3.0f / 8 );
		return filter->available;
	}
	bool Filter_Butter4_LP_double_setCutoffFrequency( Filter_Butter4_LP_double* filter , double sample_freq , double cutoff_freq )
	{
		filter->available = BUT_IIR_calc_freq_d( filter->k_1 , sample_freq , cutoff_freq , 1.0 / 8 );
		BUT_IIR_calc_freq_d( filter->k_2 , sample_freq , cutoff_freq , 3.0 / 8 );
		return filter->available;
	}
/*四阶ButterWorth低通*/
	
/*八阶ButterWorth低通*/
	bool Filter_Butter8_LP_float_setCutoffFrequency( Filter_Butter8_LP_float* filter , float sample_freq , float cutoff_freq )
	{
		filter->available = BUT_IIR_calc_freq_f( filter->k_1 , sample_freq , cutoff_freq , 1.0f / 16 );
		BUT_IIR_calc_freq_f( filter->k_2 , sample_freq , cutoff_freq , 3.0f / 16 );
		BUT_IIR_calc_freq_f( filter->k_3 , sample_freq , cutoff_freq , 5.0f / 16 );
		BUT_IIR_calc_freq_f( filter->k_4 , sample_freq , cutoff_freq , 7.0f / 16 );
		return filter->available;
	}
	bool Filter_Butter8_LP_double_setCutoffFrequency( Filter_Butter8_LP_double* filter , double sample_freq , double cutoff_freq )
	{
		filter->available = BUT_IIR_calc_freq_d( filter->k_1 , sample_freq , cutoff_freq , 1.0 / 16 );
		BUT_IIR_calc_freq_d( filter->k_2 , sample_freq , cutoff_freq , 3.0 / 16 );
		BUT_IIR_calc_freq_d( filter->k_3 , sample_freq , cutoff_freq , 5.0 / 16 );
		BUT_IIR_calc_freq_d( filter->k_4 , sample_freq , cutoff_freq , 7.0 / 16 );
		return filter->available;
	}
/*八阶ButterWorth低通*/