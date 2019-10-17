#pragma once

//三维向量运算
//20181220 朱文杰

typedef unsigned int uint;
typedef unsigned short ushort;

#define vector3( TYPE ) \
typedef struct \
{ \
	TYPE x; \
	TYPE y; \
	TYPE z; \
}vector3_##TYPE

vector3( float );
vector3( double );
vector3( int );
vector3( uint );
vector3( short );
vector3( ushort );

/*向量乘标量*/
	#define vector3_mult( TYPE ) \
	vector3_##TYPE vector3_##TYPE##_mult( vector3_##TYPE a , TYPE b ) \
	{ \
		vector3_##TYPE result; \
		result.x = a.x * b; \
		result.y = a.y * b; \
		result.z = a.z * b; \
		return result; \
	}
	static inline vector3_mult( double )
	static inline vector3_mult( float )
	static inline vector3_mult( int )
	static inline vector3_mult( uint )
	static inline vector3_mult( short )
	static inline vector3_mult( ushort )
/*向量乘标量*/

/*向量点乘*/
	#define vector3_dot_product( TYPE ) \
	TYPE vector3_##TYPE##_dot_product( vector3_##TYPE a , vector3_##TYPE b ) \
	{ \
		return a.x*b.x + a.y*b.y + a.z*b.z; \
	}	
	static inline vector3_dot_product( double )
	static inline vector3_dot_product( float )
	static inline vector3_dot_product( int )
	static inline vector3_dot_product( uint )
	static inline vector3_dot_product( short )
	static inline vector3_dot_product( ushort )
/*向量点乘*/

/*向量叉乘*/
	#define vector3_cross_product( TYPE ) \
	vector3_##TYPE vector3_##TYPE##_cross_product( vector3_##TYPE a , vector3_##TYPE b ) \
	{ \
		vector3_##TYPE result; \
		result.x = a.y*b.z - a.z*b.y; \
		result.y = a.z*b.x - a.x*b.z; \
		result.z = a.x*b.y - a.y*b.x; \
		return result; \
	}
	static inline vector3_cross_product( double )
	static inline vector3_cross_product( float )
	static inline vector3_cross_product( int )
	static inline vector3_cross_product( uint )
	static inline vector3_cross_product( short )
	static inline vector3_cross_product( ushort )
/*向量叉乘*/
	
/*向量元素相乘*/
	#define vector3_elementwise_product( TYPE ) \
	vector3_##TYPE vector3_##TYPE##_elementwise_product( vector3_##TYPE a , vector3_##TYPE b ) \
	{ \
		vector3_##TYPE result; \
		result.x = a.x * b.x; \
		result.y = a.y * b.y; \
		result.z = a.z * b.z; \
		return result; \
	}
	static inline vector3_elementwise_product( double )
	static inline vector3_elementwise_product( float )
	static inline vector3_elementwise_product( int )
	static inline vector3_elementwise_product( uint )
	static inline vector3_elementwise_product( short )
	static inline vector3_elementwise_product( ushort )
/*向量元素相乘*/
	
/*向量加*/
	#define vector3_plus( TYPE ) \
	vector3_##TYPE vector3_##TYPE##_plus( vector3_##TYPE a , vector3_##TYPE b ) \
	{ \
		vector3_##TYPE result; \
		result.x = a.x + b.x; \
		result.y = a.y + b.y; \
		result.z = a.z + b.z; \
		return result; \
	}
	static inline vector3_plus( double )
	static inline vector3_plus( float )
	static inline vector3_plus( int )
	static inline vector3_plus( uint )
	static inline vector3_plus( short )
	static inline vector3_plus( ushort )
/*向量加*/
	
/*向量减*/
	#define vector3_subtract( TYPE ) \
	vector3_##TYPE vector3_##TYPE##_subtract( vector3_##TYPE a , vector3_##TYPE b ) \
	{ \
		vector3_##TYPE result; \
		result.x = a.x - b.x; \
		result.y = a.y - b.y; \
		result.z = a.z - b.z; \
		return result; \
	}
	static inline vector3_subtract( double )
	static inline vector3_subtract( float )
	static inline vector3_subtract( int )
	static inline vector3_subtract( uint )
	static inline vector3_subtract( short )
	static inline vector3_subtract( ushort )
/*向量减*/
	
/*向量平方和*/
	#define vector3_square( TYPE ) \
	TYPE vector3_##TYPE##_square( vector3_##TYPE vec ) \
	{ \
		return vec.x*vec.x + vec.y*vec.y + vec.z*vec.z; \
	}
	static inline vector3_square( double )
	static inline vector3_square( float )
	static inline vector3_square( int )
	static inline vector3_square( uint )
	static inline vector3_square( short )
	static inline vector3_square( ushort )
/*向量平方和*/
	
/*向量限幅*/
	vector3_float vector3_float_constrain( vector3_float vec , float max_length );
/*向量限幅*/