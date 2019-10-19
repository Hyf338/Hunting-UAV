#pragma once

/*
	位置 速度 加速度融合卡尔曼滤波器
	朱文杰 20181225
*/
typedef struct
{
	//修正权重
	float kg11 , kg12 , kg21 , kg22 , kg31 , kg32;
	//协方差矩阵
	float P[3][3];

	//加速度预测过程噪声 , 加速度偏移过程噪声
	float q1 , q3;
	//位置观测噪声 , 速度观测噪声
	float r1 , r2;	
	
	//位置修正量
	float s_correction;
	//速度修正量
	float v_correction;
	//加速度偏移修正量
	float b_correction;
}PositionKalmanFilter;

//初始化滤波器
static inline void PositionKalmanFilter_init( PositionKalmanFilter* filter , float r1 , float r2 , float q1 , float q3 )
{
	filter->r1 = r1;	filter->r2 = r2;
	filter->q1 = q1;	filter->q3 = q3;
	filter->kg11 = filter->kg12 = filter->kg21 = filter->kg22 = filter->kg31 = filter->kg32 = 0;
	for( unsigned char i = 0 ; i < 3 ; ++i )
		for( unsigned char j = 0 ; j < 3 ; ++j )
			filter->P[i][j] = 0;
	filter->s_correction = filter->v_correction = filter->b_correction = 0;
}

//使协方差矩阵对称
static inline void PositionKalmanFilter_force_symmetry( PositionKalmanFilter* filter )
{
	filter->P[1][0] = filter->P[0][1] = 0.5f * ( filter->P[1][0] + filter->P[0][1] );
	filter->P[2][0] = filter->P[0][2] = 0.5f * ( filter->P[2][0] + filter->P[0][2] );
	filter->P[2][1] = filter->P[1][2] = 0.5f * ( filter->P[2][1] + filter->P[1][2] );
	
	if( filter->P[0][0] < 0.0f ) filter->P[0][0] = 0.0f;
	if( filter->P[1][1] < 0.0f ) filter->P[1][1] = 0.0f;
	if( filter->P[2][2] < 0.0f ) filter->P[2][2] = 0.0f;
}

/*运行滤波器*/
	//s：位置	v：速度	b：加速度偏移
	//PositionKalmanFilter_run_xxx_yyy中：
	//xxx代表观测量，yyy代表修正的估计量
	//如s_sv代表输入位置观测量，修正估计位置及速度
	
	static inline void PositionKalmanFilter_run_v_v( PositionKalmanFilter* filter , float update_time , float vel_err , float* v )
	{
		/*time*/
			float t = update_time;
			if( t > 0.6f ) t = 0.6f;
		/*time*/
		
		float pmid[3][3];
		
		pmid[1][1] = filter->P[1][1] + filter->q1*t*t;
		
		filter->kg21 = pmid[1][1] / (pmid[1][1] + filter->r2);
		
		filter->s_correction = 0;
		filter->v_correction = filter->kg21*vel_err;
		filter->b_correction = 0;
		*v += filter->v_correction;
		
		filter->P[1][1] = -pmid[1][1]*(filter->kg21 - 1);
		
		PositionKalmanFilter_force_symmetry( filter );
	}

	static inline void PositionKalmanFilter_run_v_vb( PositionKalmanFilter* filter , const float update_time , const float vel_err , float* v , float* b )
	{
		/*time*/
			float t = update_time;
			if( t > 0.6f ) t = 0.6f;
			float t2 = t*t;
			float t3 = t2*t;
			float t4 = t2*t2;
		/*time*/

		float pmid[3][3];
		float temp;		
		
		pmid[0][0] = (filter->P[2][2]*t4)*0.25f - filter->P[1][2]*t3 + (filter->P[1][1] - filter->P[0][2])*t2 + 2*filter->P[0][1]*t + filter->P[0][0] + 0.25f*filter->q1*t4 ;
		pmid[0][1] = (filter->P[2][2]*t3)*0.5f - 1.5f*filter->P[1][2]*t2 + (filter->P[1][1] - filter->P[0][2])*t + filter->P[0][1] + 0.5f*filter->q1*t3 ;	
		pmid[0][2] = - (filter->P[2][2]*t2)*0.5f + filter->P[1][2]*t + filter->P[0][2] ;	
		
		//pmid[1][0] = pmid[0][1] ;
		pmid[1][1] = filter->P[2][2]*t2 - 2*filter->P[1][2]*t + filter->P[1][1] + filter->q1*t2 ;	
		pmid[1][2] = -filter->P[2][2]*t + filter->P[1][2] ;
		
		//pmid[2][0] = pmid[0][2] ;
		//pmid[2][1] = pmid[1][2] ;
		pmid[2][2] = filter->P[2][2] + filter->q3*t2 ;
		
		temp = 1.0f / (pmid[1][1] + filter->r2);
		filter->kg21 = pmid[1][1] * temp;
		filter->kg31 = pmid[1][2] * temp;
		
		filter->s_correction = 0;
		filter->v_correction = filter->kg21*vel_err;
		filter->b_correction = filter->kg31*vel_err;
		*v += filter->v_correction;
		*b += filter->b_correction;
		
		filter->P[0][0] = pmid[0][0] ;
		filter->P[0][1] = pmid[0][1];
		filter->P[0][2] = pmid[0][2];
		filter->P[1][0] = pmid[0][1] - filter->kg21*pmid[0][0] ;
		filter->P[1][1] = pmid[1][1] - filter->kg21*pmid[0][1] ;
		filter->P[1][2] = pmid[1][2] - filter->kg21*pmid[0][2] ;
		filter->P[2][0] = pmid[0][2] - filter->kg31*pmid[0][0] ;
		filter->P[2][1] = pmid[1][2] - filter->kg31*pmid[0][1] ;
		filter->P[2][2] = pmid[2][2] - filter->kg31*pmid[0][2] ;
		
		PositionKalmanFilter_force_symmetry( filter );
	}

	static inline void PositionKalmanFilter_run_s_sv( PositionKalmanFilter* filter , const float update_time , const float pos_err , float* s , float* v)
	{
		/*time*/
			float t = update_time;
			if( t > 0.6f ) t = 0.6f;
			float t2 = t*t;
			float t3 = t2*t;
			float t4 = t2*t2;
		/*time*/

		float pmid[3][3];
		float temp;		
		
		pmid[0][0] = (filter->P[2][2]*t4)*0.25f - filter->P[1][2]*t3 + (filter->P[1][1] - filter->P[0][2])*t2 + 2*filter->P[0][1]*t + filter->P[0][0] + 0.25f*filter->q1*t4 ;
		pmid[0][1] = (filter->P[2][2]*t3)*0.5f - 1.5f*filter->P[1][2]*t2 + (filter->P[1][1] - filter->P[0][2])*t + filter->P[0][1] + 0.5f*filter->q1*t3 ;	
		pmid[0][2] = - (filter->P[2][2]*t2)*0.5f + filter->P[1][2]*t + filter->P[0][2] ;	
		
		//pmid[1][0] = pmid[0][1] ;
		pmid[1][1] = filter->P[2][2]*t2 - 2*filter->P[1][2]*t + filter->P[1][1] + filter->q1*t2 ;	
		pmid[1][2] = -filter->P[2][2]*t + filter->P[1][2] ;
		
		//pmid[2][0] = pmid[0][2] ;
		//pmid[2][1] = pmid[1][2] ;
		pmid[2][2] = filter->P[2][2] + filter->q3*t2 ;
		
		temp = 1.0f / ( pmid[0][0] + filter->r1 );
		filter->kg11 = pmid[0][0] * temp ;
		filter->kg21 = pmid[0][1] * temp ;
		
		filter->s_correction = filter->kg11*pos_err;
		filter->v_correction = filter->kg21*pos_err;
		filter->b_correction = 0;
		*s += filter->s_correction ;
		*v += filter->v_correction ;
		
		temp = 1.0f - filter->kg11 ;
		filter->P[0][0] = pmid[0][0]*temp ;
		filter->P[0][1] = pmid[0][1]*temp ;
		filter->P[0][2] = pmid[0][2]*temp ;
		filter->P[1][0] = pmid[0][1] - filter->kg21*pmid[0][0] ;
		filter->P[1][1] = pmid[1][1] - filter->kg21*pmid[0][1] ;
		filter->P[1][2] = pmid[1][2] - filter->kg21*pmid[0][2] ;
		filter->P[2][0] = pmid[0][2];
		filter->P[2][1] = pmid[1][2];
		filter->P[2][2] = pmid[2][2];
		
		PositionKalmanFilter_force_symmetry( filter );
	}

	static inline void PositionKalmanFilter_run_s_svb( PositionKalmanFilter* filter , const float update_time , const float pos_err , float* s , float* v , float* b )
	{
		/*time*/
			float t = update_time;
			if( t > 0.6f ) t = 0.6f;
			float t2 = t*t;
			float t3 = t2*t;
			float t4 = t2*t2;
		/*time*/

		float pmid[3][3];
		float temp;		
		
		pmid[0][0] = (filter->P[2][2]*t4)*0.25f - filter->P[1][2]*t3 + (filter->P[1][1] - filter->P[0][2])*t2 + 2*filter->P[0][1]*t + filter->P[0][0] + 0.25f*filter->q1*t4 ;
		pmid[0][1] = (filter->P[2][2]*t3)*0.5f - 1.5f*filter->P[1][2]*t2 + (filter->P[1][1] - filter->P[0][2])*t + filter->P[0][1] + 0.5f*filter->q1*t3 ;	
		pmid[0][2] = - (filter->P[2][2]*t2)*0.5f + filter->P[1][2]*t + filter->P[0][2] ;	
		
		//pmid[1][0] = pmid[0][1] ;
		pmid[1][1] = filter->P[2][2]*t2 - 2*filter->P[1][2]*t + filter->P[1][1] + filter->q1*t2 ;	
		pmid[1][2] = -filter->P[2][2]*t + filter->P[1][2] ;
		
		//pmid[2][0] = pmid[0][2] ;
		//pmid[2][1] = pmid[1][2] ;
		pmid[2][2] = filter->P[2][2] + filter->q3*t2 ;
		
		temp = 1.0f/(pmid[0][0] + filter->r1) ;
		filter->kg11 = pmid[0][0] * temp ;
		filter->kg21 = pmid[0][1] * temp ;
		filter->kg31 = pmid[0][2] * temp ;
		
		filter->s_correction = filter->kg11*pos_err;
		filter->v_correction = filter->kg21*pos_err;
		filter->b_correction = filter->kg31*pos_err;
		*s += filter->s_correction ;
		*v += filter->v_correction ;
		*b += filter->b_correction ;
		
		temp = 1.0f - filter->kg11 ;
		filter->P[0][0] = pmid[0][0]*temp ;
		filter->P[0][1] = pmid[0][1]*temp ;
		filter->P[0][2] = pmid[0][2]*temp ;
		filter->P[1][0] = pmid[0][1] - filter->kg21*pmid[0][0] ;
		filter->P[1][1] = pmid[1][1] - filter->kg21*pmid[0][1] ;
		filter->P[1][2] = pmid[1][2] - filter->kg21*pmid[0][2] ;
		filter->P[2][0] = pmid[0][2] - filter->kg31*pmid[0][0] ;
		filter->P[2][1] = pmid[1][2] - filter->kg31*pmid[0][1] ;
		filter->P[2][2] = pmid[2][2] - filter->kg31*pmid[0][2] ;
		
		PositionKalmanFilter_force_symmetry( filter );
	}
	
	static inline void PositionKalmanFilter_run_sv_sv( PositionKalmanFilter* filter , const float update_time , const float pos_err , const float vel_err , float* s , float* v )
	{
		/*time*/
			float t = update_time;
			if( t > 0.6f ) t = 0.6f;
			float t2 = t*t;
		/*time*/
		
		float temp;
					
		float pmid[2][2];
		pmid[0][0] = filter->P[1][1]*t2 + (filter->P[0][1] + filter->P[1][0])*t + filter->P[0][0] + 0.25f*filter->q1*t2*t2 ;
		pmid[0][1] = filter->P[1][1]*t + filter->P[0][1] + 0.5f*filter->q1*t2*t ;	
		
		//pmid[1][0] = filter->P[1][1]*t + filter->P[1][0] ;	
		pmid[1][0] = pmid[0][1] ;	
		pmid[1][1] = filter->P[1][1] + filter->q1*t2 ;	
		
		temp = 1.0f / (pmid[0][0]*pmid[1][1] - pmid[0][1]*pmid[1][0] + pmid[0][0]*filter->r2 + pmid[1][1]*filter->r1 + filter->r1*filter->r2) ;
		filter->kg11 = (pmid[0][0]*pmid[1][1] - pmid[0][1]*pmid[1][0] + pmid[0][0]*filter->r2)*temp;
		filter->kg12 = (pmid[0][1]*filter->r1)*temp;
		filter->kg21 = (pmid[1][0]*filter->r2)*temp;
		filter->kg22 = (pmid[0][0]*pmid[1][1] - pmid[0][1]*pmid[1][0] + pmid[1][1]*filter->r1)*temp;
		
		filter->s_correction = pos_err*filter->kg11 + filter->kg12*vel_err;
		filter->v_correction = pos_err*filter->kg21 + filter->kg22*vel_err;
		filter->b_correction = 0;
		*s += filter->s_correction ;
		*v += filter->v_correction ;
		
		temp = filter->kg11 - 1.0f ;
		filter->P[0][0] = - filter->kg12*pmid[1][0] - pmid[0][0]*temp ;
		filter->P[0][1] = - filter->kg12*pmid[1][1] - pmid[0][1]*temp ;
		temp = filter->kg22 - 1.0f ;
		filter->P[1][0] = - filter->kg21*pmid[0][0] - pmid[1][0]*temp ;
		filter->P[1][1] = - filter->kg21*pmid[0][1] - pmid[1][1]*temp ;
		PositionKalmanFilter_force_symmetry( filter );
	}
	
	static inline void PositionKalmanFilter_run_sv_svb( PositionKalmanFilter* filter , const float update_time , const float pos_err , const float vel_err , float* s , float* v , float* b )
	{
		/*time*/
			float t = update_time;
			if( t > 0.6f ) t = 0.6f;
			float t2 = t*t;
			float t3 = t2*t;
			float t4 = t2*t2;
		/*time*/
		
		float pmid[3][3];
		float temp;
		
		
		pmid[0][0] = (filter->P[2][2]*t4)*0.25f + (- filter->P[1][2] - filter->P[2][1])*0.5f*t3 + (filter->P[1][1]*2.0f - filter->P[0][2] - filter->P[2][0])*0.5f*t2 + (filter->P[0][1] + filter->P[1][0])*t + filter->P[0][0] + 0.25f*filter->q1*t4 ;
		pmid[0][1] = (filter->P[2][2]*t3)*0.5f + (- filter->P[1][2] - filter->P[2][1]*0.5f)*t2 + (filter->P[1][1] - filter->P[0][2])*t + filter->P[0][1] + 0.5f*filter->q1*t3 ;	
		pmid[0][2] = - (filter->P[2][2]*t2)*0.5f + filter->P[1][2]*t + filter->P[0][2] ;	
		
		//pmid[1][0] = (filter->P[2][2]*t3)*0.5f + (- filter->P[1][2]*0.5f - filter->P[2][1])*t2 + (filter->P[1][1] - filter->P[2][0])*t + filter->P[1][0] ;	
		pmid[1][0] = pmid[0][1] ;
		pmid[1][1] = filter->P[2][2]*t2 + (- filter->P[1][2] - filter->P[2][1])*t + filter->P[1][1] + filter->q1*t2 ;	
		pmid[1][2] = -filter->P[2][2]*t + filter->P[1][2] ;
		
		//pmid[2][0] = - (filter->P[2][2]*t2)*0.5f + filter->P[2][1]*t + filter->P[2][0] ;
		pmid[2][0] = pmid[0][2] ;
		//pmid[2][1] = -filter->P[2][2]*t + filter->P[2][1] ;
		pmid[2][1] = pmid[1][2] ;
		pmid[2][2] = filter->P[2][2] + filter->q3*t2 ;
		
		temp = 1.0f / (pmid[0][0]*pmid[1][1] - pmid[0][1]*pmid[1][0] + pmid[0][0]*filter->r2 + pmid[1][1]*filter->r1 + filter->r1*filter->r2) ;
		filter->kg11 = (pmid[0][0]*pmid[1][1] - pmid[0][1]*pmid[1][0] + pmid[0][0]*filter->r2)*temp;
		filter->kg12 = (pmid[0][1]*filter->r1)*temp;
		filter->kg21 = (pmid[1][0]*filter->r2)*temp;
		filter->kg22 = (pmid[0][0]*pmid[1][1] - pmid[0][1]*pmid[1][0] + pmid[1][1]*filter->r1)*temp;
		filter->kg31 = (pmid[1][1]*pmid[2][0] - pmid[1][0]*pmid[2][1] + pmid[2][0]*filter->r2)*temp;
		filter->kg32 = (pmid[0][0]*pmid[2][1] - pmid[0][1]*pmid[2][0] + pmid[2][1]*filter->r1)*temp;
		
		filter->s_correction = pos_err*filter->kg11 + filter->kg12*vel_err;
		filter->v_correction = pos_err*filter->kg21 + filter->kg22*vel_err;
		filter->b_correction = pos_err*filter->kg31 + filter->kg32*vel_err;
		*s += filter->s_correction ;
		*v += filter->v_correction ;
		*b += filter->b_correction ;
		
		temp = filter->kg11 - 1.0f ;
		filter->P[0][0] = - filter->kg12*pmid[1][0] - pmid[0][0]*temp ;
		filter->P[0][1] = - filter->kg12*pmid[1][1] - pmid[0][1]*temp ;
		filter->P[0][2] = - filter->kg12*pmid[1][2] - pmid[0][2]*temp ;
		temp = filter->kg22 - 1.0f ;
		filter->P[1][0] = - filter->kg21*pmid[0][0] - pmid[1][0]*temp ;
		filter->P[1][1] = - filter->kg21*pmid[0][1] - pmid[1][1]*temp ;
		filter->P[1][2] = - filter->kg21*pmid[0][2] - pmid[1][2]*temp ;
		filter->P[2][0] = pmid[2][0] - filter->kg31*pmid[0][0] - filter->kg32*pmid[1][0] ;
		filter->P[2][1] = pmid[2][1] - filter->kg31*pmid[0][1] - filter->kg32*pmid[1][1] ;
		filter->P[2][2] = pmid[2][2] - filter->kg31*pmid[0][2] - filter->kg32*pmid[1][2] ;
		PositionKalmanFilter_force_symmetry( filter );
	}
/*运行滤波器*/