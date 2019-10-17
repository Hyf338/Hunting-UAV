#pragma once

#define __FPU_PRESENT 1 
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include "arm_math.h"
#if defined  (__GNUC__)
	#ifndef ALIGN4
    #define ALIGN4 __attribute__((aligned(4)))
	#endif
#endif

#include "vector_3.h"

//ENU坐标转换为BodyHeading（x为机头朝向（与地面平行），y为朝向机头左方（与地面平行），z为上方）
#define map_ENU2BodyHeading_x( enu_x , enu_y , Yaw_sin , Yaw_cos ) ( enu_x*Yaw_cos + enu_y*Yaw_sin )
#define map_ENU2BodyHeading_y( enu_x , enu_y , Yaw_sin , Yaw_cos ) ( -enu_x*Yaw_sin + enu_y*Yaw_cos )
//BoduHeading转换为ENU坐标
#define map_BodyHeading2ENU_x( body_x , body_y , Yaw_sin , Yaw_cos ) ( body_x*Yaw_cos - body_y*Yaw_sin )
#define map_BodyHeading2ENU_y( body_x , body_y , Yaw_sin , Yaw_cos ) ( body_x*Yaw_sin + body_y*Yaw_cos )

#define M_PI 3.14159265358979323846 
#define PI_f 3.14159265358979323846f
#define rad2degree(x) (x*57.295779513f)
#define degree2rad(x) (x*0.0174532925f)
#define degree2rad_double(x) (x*0.0174532925)

#define constG 980.665f

static inline bool is_zero_f(float fVal1) 
{
	return fabsf(fVal1) < 1e-6f ? true : false;
}

static inline float safe_sqrt_f( float x )
{
	if( x >= 0 )
	{
		arm_sqrt_f32( x , &x );
		return x;
	}
	else
		return 0;
}

//nxn矩阵求逆
static inline bool Matrix_Inverse(float a[], const int n)  
{  
		int *is,*js,i,j,k,l,u,v;  
		float d,p;  
		is = malloc( n*sizeof(int) ); 
		js = malloc( n*sizeof(int) ); 
		for (k=0; k<=n-1; k++)  
		{  
				d=0.0f;  
				for (i=k; i<=n-1; ++i)  
				for (j=k; j<=n-1; ++j)  
				{  
						l=i*n+j; p=fabsf(a[l]);  
						if (p>d) { d=p; is[k]=i; js[k]=j;}  
				}  
				if (is_zero_f(d))  
				{  
					free( is );
					free( js ); 
						return(false);  
				}  
				if (is[k]!=k)  
						for (j=0; j<=n-1; ++j)  
						{  
								u=k*n+j; v=is[k]*n+j;  
								p=a[u]; a[u]=a[v]; a[v]=p;  
						}  
				if (js[k]!=k)  
						for (i=0; i<=n-1; ++i)  
						{  
								u=i*n+k; v=i*n+js[k];  
								p=a[u]; a[u]=a[v]; a[v]=p;  
						}  
				l=k*n+k;  
				a[l]=1.0f/a[l];  
				for (j=0; j<=n-1; ++j)  
						if (j!=k)  
						{ u=k*n+j; a[u]=a[u]*a[l];}  
				for (i=0; i<=n-1; ++i)  
						if (i!=k)  
								for (j=0; j<=n-1; ++j)  
				if (j!=k)  
				{  
						u=i*n+j;  
						a[u] -= a[i*n+k]*a[k*n+j];  
				}  
				for (i=0; i<=n-1; ++i)  
						if (i!=k)  
						{  
								u=i*n+k;  
								a[u] = -a[u]*a[l];  
						}  
		}  
		for (k=n-1; k>=0; --k)  
		{  
				if (js[k]!=k)  
				for (j=0; j<=n-1; ++j)  
				{  
						u=k*n+j; v=js[k]*n+j;  
				p=a[u]; a[u]=a[v]; a[v]=p;  
				}  
				if (is[k]!=k)  
				for (i=0; i<=n-1; ++i)  
				{   
						u=i*n+k; v=i*n+is[k];  
						p=a[u]; a[u]=a[v]; a[v]=p;  
				}  
		}  
		free( is );
		free( js );   
		return(true);  
}

//限幅
static inline float constrain_float( float x , float bound )
{
	if( x > bound )
		return bound;
	else if( x < -bound )
		return -bound;
	else return x;
}
static inline float constrain_range_float( float x , float max , float min )
{
	if( x > max )
		return max;
	else if( x < min )
		return min;
	else return x;
}

//sign函数
static inline float sign_f( float x )
{
	if( x > 0 )
		return 1;
	else if( x < 0 )
		return -1;
	return 0;
}

//求两单位向量夹角
static inline vector3_float get_included_angle_from_unit_vector( vector3_float vec_a , vector3_float vec_b , vector3_float last_angle )
{
	vector3_float angle_vec = vector3_float_cross_product( vec_a , vec_b );
	float angle_sin = vector3_float_square( angle_vec );
	arm_sqrt_f32( angle_sin , &angle_sin );
	float angle_cosin = vector3_float_dot_product( vec_a , vec_b );
	if( is_zero_f( angle_sin ) )
	{
		if( angle_cosin > 0.0f )	//angle is zero
		{
			vector3_float vec = { 0 , 0 , 0 };
			return vec;
		}
		else
		{	//angle is 180 degree
			float sq = vector3_float_square( last_angle );
			arm_sqrt_f32( sq , &sq );
			if( is_zero_f( sq ) )
			{
				vector3_float vec = { PI_f , 0 , 0 };
				return vec;
			}
			else
			{
				return vector3_float_mult( last_angle , (1.0f / sq) * PI_f ) ;
			}
		}
	}
	else
	{
		float angle;
		if( angle_sin > 1.0f )
			angle = 0.5f * PI_f;
		else
			angle = asinf( angle_sin );
		if( angle_cosin < 0.0f )
			angle = PI_f - angle;
		return vector3_float_mult( angle_vec , angle / angle_sin );
	}
}

/*判断是否在范围内*/
	static inline bool in_symmetry_range_float( const float input , const float range )
	{
		if( (input >= -range) && (input <= range) )
			return true;
		else 
			return false;
	}	
	static inline bool in_symmetry_range_offset_float( const float input , const float range , const float offset )
	{
		if( (input >= offset-range) && (input<= offset+range) )
			return true;
		else 
			return false;
	}
	static inline bool in_range_float( float input , float max , float min )
	{
		if( input > max )
			return false;
		else if( input < min )
			return false;
		else
			return true;
	}
/*判断是否在范围内*/

/*去除死区*/
	static inline float apply_deadband_float( const float input , const float range )
	{
		if( input > range )
			return input - range;
		else if( input < -range )
			return input + range;
		else 
			return 0;
	}
/*去除死区*/
	
/*二维向量限幅*/
	static inline void constrain_vector2_float( float* x , float* y , float length )
	{
		float xy_length = safe_sqrt_f( (*x)*(*x) + (*y)*(*y) );
		if( xy_length > length )
		{
			float scale = length / xy_length;
			(*x) *= scale;
			(*y) *= scale;
		}
	}
/*二维向量限幅*/