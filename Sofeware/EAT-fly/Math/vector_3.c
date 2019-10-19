#include "vector_3.h"

#include "AC_Math.h"

/*向量限幅*/
	vector3_float vector3_float_constrain( vector3_float vec , float max_length )
	{
		float length = vector3_float_square( vec );
		float sq_max_length = max_length * max_length;
		if( length > sq_max_length )
		{
			float scale = sq_max_length / length;
			arm_sqrt_f32( scale , &scale );
			return vector3_float_mult( vec , scale );
		}	
		return vec;
	}
/*向量限幅*/