#include "AC_Math.h"
#include "Quaternion.h"

/*归一化四元数*/
	Quaternion Quaternion_normalize( Quaternion quat )
	{
		float inv_length = quat.qw*quat.qw + quat.qx*quat.qx + quat.qy*quat.qy + quat.qz*quat.qz;
		arm_sqrt_f32 ( inv_length , &inv_length );
		inv_length = 1.0f / inv_length;
		quat.qw *= inv_length;
		quat.qx *= inv_length;
		quat.qy *= inv_length;
		quat.qz *= inv_length;
		return quat;
	}
	QuaternionEf QuaternionEf_normalize( QuaternionEf quat )
	{
		Quaternion squat = Quaternion_normalize( *(Quaternion*)&quat );
		return get_QuaternionEf( squat );
	}
/*归一化四元数*/
	
/*由四元数获得旋转向量*/
	vector3_float Quaternion_get_Rotation_vec( Quaternion quat )
	{
		float theta = 2.0f * acosf( quat.qw );
		float sin_half_theta = safe_sqrt_f( 1.0f - quat.qw*quat.qw );
		float scale;
		if( is_zero_f( sin_half_theta ) )
			scale = 0.5f;
		else
			scale = theta / sin_half_theta;
		vector3_float result = 
		{
			quat.qx * scale , quat.qy * scale , quat.qz * scale
		};
		return result;
	}
	vector3_float QuaternionEf_get_Rotation_vec( Quaternion quat )
	{
		float theta = 2.0f * acosf( quat.qw );
		float sin_half_theta = safe_sqrt_f( 1.0f - quat.qw*quat.qw );
		float scale;
		if( is_zero_f( sin_half_theta ) )
			scale = 0.5f;
		else
			scale = theta / sin_half_theta;
		vector3_float result = 
		{
			quat.qx * scale , quat.qy * scale , quat.qz * scale
		};
		return result;
	}
/*由四元数获得旋转向量*/
	
/*用四元数旋转向量*/
	vector3_float Quaternion_rotate( Quaternion quat , vector3_float vec )
	{
		float qw2 = quat.qw * quat.qw;
		float qx2 = quat.qx * quat.qx;
		float qy2 = quat.qy * quat.qy;
		float qz2 = quat.qz * quat.qz;
		float qwx = quat.qw * quat.qx;
		float qwy = quat.qw * quat.qy;
		float qwz = quat.qw * quat.qz;
		float qxy = quat.qx * quat.qy;
		float qxz = quat.qx * quat.qz;
		float qyz = quat.qy * quat.qz;
		vector3_float result={
			(qw2 + qx2 - qy2 - qz2)*vec.x + (2*qxy - 2*qwz)*vec.y + (2*qwy + 2*qxz)*vec.z ,\
			(2*qwz + 2*qxy)*vec.x + (qw2 - qx2 + qy2 - qz2)*vec.y + (2*qyz - 2*qwx)*vec.z ,\
			(2*qxz - 2*qwy)*vec.x + (2*qwx + 2*qyz)*vec.y + (qw2 - qx2 - qy2 + qz2)*vec.z 
		};
		return result;
	}
	vector3_float Quaternion_rotate_axis_x( Quaternion quat )
	{
		float qw2 = quat.qw * quat.qw;
		float qx2 = quat.qx * quat.qx;
		float qy2 = quat.qy * quat.qy;
		float qz2 = quat.qz * quat.qz;
		float qwx = quat.qw * quat.qx;
		float qwy = quat.qw * quat.qy;
		float qwz = quat.qw * quat.qz;
		float qxy = quat.qx * quat.qy;
		float qxz = quat.qx * quat.qz;
		float qyz = quat.qy * quat.qz;
		vector3_float result={
			qw2 + qx2 - qy2 - qz2 ,\
			2*qwz + 2*qxy ,\
			2*qxz - 2*qwy
		};
		return result;
	}
	vector3_float Quaternion_rotate_axis_y( Quaternion quat )
	{
		float qw2 = quat.qw * quat.qw;
		float qx2 = quat.qx * quat.qx;
		float qy2 = quat.qy * quat.qy;
		float qz2 = quat.qz * quat.qz;
		float qwx = quat.qw * quat.qx;
		float qwy = quat.qw * quat.qy;
		float qwz = quat.qw * quat.qz;
		float qxy = quat.qx * quat.qy;
		float qxz = quat.qx * quat.qz;
		float qyz = quat.qy * quat.qz;
		vector3_float result={
			2*qxy - 2*qwz ,\
			qw2 - qx2 + qy2 - qz2 ,\
			2*qwx + 2*qyz 
		};
		return result;
	}
	vector3_float Quaternion_rotate_axis_z( Quaternion quat )
	{
		float qw2 = quat.qw * quat.qw;
		float qx2 = quat.qx * quat.qx;
		float qy2 = quat.qy * quat.qy;
		float qz2 = quat.qz * quat.qz;
		float qwx = quat.qw * quat.qx;
		float qwy = quat.qw * quat.qy;
		float qwz = quat.qw * quat.qz;
		float qxy = quat.qx * quat.qy;
		float qxz = quat.qx * quat.qz;
		float qyz = quat.qy * quat.qz;
		vector3_float result={
			2*qwy + 2*qxz ,\
			2*qyz - 2*qwx ,\
			qw2 - qx2 - qy2 + qz2
		};
		return result;
	}
	vector3_float Quaternion_reverse_rotate( Quaternion quat , vector3_float vec )
	{
		float qw2 = quat.qw * quat.qw;
		float qx2 = quat.qx * quat.qx;
		float qy2 = quat.qy * quat.qy;
		float qz2 = quat.qz * quat.qz;
		float qwx = quat.qw * quat.qx;
		float qwy = quat.qw * quat.qy;
		float qwz = quat.qw * quat.qz;
		float qxy = quat.qx * quat.qy;
		float qxz = quat.qx * quat.qz;
		float qyz = quat.qy * quat.qz;
		vector3_float result={
			(qw2 + qx2 - qy2 - qz2)*vec.x + 2*(qwz + qxy)*vec.y + 2*(qxz - qwy)*vec.z ,\
			2*(qxy - qwz)*vec.x + (qw2 - qx2 + qy2 - qz2)*vec.y + 2*(qwx + qyz)*vec.z ,\
			2*(qwy + qxz)*vec.x + 2*(qyz - qwx)*vec.y + (qw2 - qx2 - qy2 + qz2)*vec.z
		};
		return result;
	}
	vector3_float Quaternion_reverse_rotate_axis_x( Quaternion quat )
	{
		float qw2 = quat.qw * quat.qw;
		float qx2 = quat.qx * quat.qx;
		float qy2 = quat.qy * quat.qy;
		float qz2 = quat.qz * quat.qz;
		float qwx = quat.qw * quat.qx;
		float qwy = quat.qw * quat.qy;
		float qwz = quat.qw * quat.qz;
		float qxy = quat.qx * quat.qy;
		float qxz = quat.qx * quat.qz;
		float qyz = quat.qy * quat.qz;
		vector3_float result={
			qw2 + qx2 - qy2 - qz2 ,\
			2*qxy - 2*qwz ,\
			2*qwy + 2*qxz
		};
		return result;
	}
	vector3_float Quaternion_reverse_rotate_axis_y( Quaternion quat )
	{
		float qw2 = quat.qw * quat.qw;
		float qx2 = quat.qx * quat.qx;
		float qy2 = quat.qy * quat.qy;
		float qz2 = quat.qz * quat.qz;
		float qwx = quat.qw * quat.qx;
		float qwy = quat.qw * quat.qy;
		float qwz = quat.qw * quat.qz;
		float qxy = quat.qx * quat.qy;
		float qxz = quat.qx * quat.qz;
		float qyz = quat.qy * quat.qz;
		vector3_float result={
			2*qwz + 2*qxy ,\
			qw2 - qx2 + qy2 - qz2 ,\
			2*qyz - 2*qwx
		};
		return result;
	}
	vector3_float Quaternion_reverse_rotate_axis_z( Quaternion quat )
	{
		float qw2 = quat.qw * quat.qw;
		float qx2 = quat.qx * quat.qx;
		float qy2 = quat.qy * quat.qy;
		float qz2 = quat.qz * quat.qz;
		float qwx = quat.qw * quat.qx;
		float qwy = quat.qw * quat.qy;
		float qwz = quat.qw * quat.qz;
		float qxy = quat.qx * quat.qy;
		float qxz = quat.qx * quat.qz;
		float qyz = quat.qy * quat.qz;
		vector3_float result={
			2*qxz - 2*qwy ,\
			2*qwx + 2*qyz ,\
			qw2 - qx2 - qy2 + qz2
		};
		return result;
	}

	vector3_float QuaternionEf_rotate( QuaternionEf quat , vector3_float vec )
	{
		float qw2 = quat.qw2;
		float qx2 = quat.qx2;
		float qy2 = quat.qy2;
		float qz2 = quat.qz2;
		float qwx = quat.qwx;
		float qwy = quat.qwy;
		float qwz = quat.qwz;
		float qxy = quat.qx;
		float qxz = quat.qxz;
		float qyz = quat.qyz;
		vector3_float result={
			(qw2 + qx2 - qy2 - qz2)*vec.x + (2*qxy - 2*qwz)*vec.y + (2*qwy + 2*qxz)*vec.z ,\
			(2*qwz + 2*qxy)*vec.x + (qw2 - qx2 + qy2 - qz2)*vec.y + (2*qyz - 2*qwx)*vec.z ,\
			(2*qxz - 2*qwy)*vec.x + (2*qwx + 2*qyz)*vec.y + (qw2 - qx2 - qy2 + qz2)*vec.z 
		};
		return result;
	}
	vector3_float QuaternionEf_rotate_axis_x( QuaternionEf quat )
	{
		float qw2 = quat.qw2;
		float qx2 = quat.qx2;
		float qy2 = quat.qy2;
		float qz2 = quat.qz2;
		float qwx = quat.qwx;
		float qwy = quat.qwy;
		float qwz = quat.qwz;
		float qxy = quat.qx;
		float qxz = quat.qxz;
		float qyz = quat.qyz;
		vector3_float result={
			qw2 + qx2 - qy2 - qz2 ,\
			2*qwz + 2*qxy ,\
			2*qxz - 2*qwy
		};
		return result;
	}
	vector3_float QuaternionEf_rotate_axis_y( QuaternionEf quat )
	{
		float qw2 = quat.qw2;
		float qx2 = quat.qx2;
		float qy2 = quat.qy2;
		float qz2 = quat.qz2;
		float qwx = quat.qwx;
		float qwy = quat.qwy;
		float qwz = quat.qwz;
		float qxy = quat.qx;
		float qxz = quat.qxz;
		float qyz = quat.qyz;
		vector3_float result={
			2*qxy - 2*qwz ,\
			qw2 - qx2 + qy2 - qz2 ,\
			2*qwx + 2*qyz 
		};
		return result;
	}
	vector3_float QuaternionEf_rotate_axis_z( QuaternionEf quat )
	{
		float qw2 = quat.qw2;
		float qx2 = quat.qx2;
		float qy2 = quat.qy2;
		float qz2 = quat.qz2;
		float qwx = quat.qwx;
		float qwy = quat.qwy;
		float qwz = quat.qwz;
		float qxy = quat.qx;
		float qxz = quat.qxz;
		float qyz = quat.qyz;
		vector3_float result={
			2*qwy + 2*qxz ,\
			2*qyz - 2*qwx ,\
			qw2 - qx2 - qy2 + qz2
		};
		return result;
	}
	vector3_float QuaternionEf_reverse_rotate( QuaternionEf quat , vector3_float vec )
	{
		float qw2 = quat.qw2;
		float qx2 = quat.qx2;
		float qy2 = quat.qy2;
		float qz2 = quat.qz2;
		float qwx = quat.qwx;
		float qwy = quat.qwy;
		float qwz = quat.qwz;
		float qxy = quat.qx;
		float qxz = quat.qxz;
		float qyz = quat.qyz;
		vector3_float result={
			(qw2 + qx2 - qy2 - qz2)*vec.x + 2*(qwz + qxy)*vec.y + 2*(qxz - qwy)*vec.z ,\
			2*(qxy - qwz)*vec.x + (qw2 - qx2 + qy2 - qz2)*vec.y + 2*(qwx + qyz)*vec.z ,\
			2*(qwy + qxz)*vec.x + 2*(qyz - qwx)*vec.y + (qw2 - qx2 - qy2 + qz2)*vec.z
		};
		return result;
	}
	vector3_float QuaternionEf_reverse_rotate_axis_x( QuaternionEf quat )
	{
		float qw2 = quat.qw2;
		float qx2 = quat.qx2;
		float qy2 = quat.qy2;
		float qz2 = quat.qz2;
		float qwx = quat.qwx;
		float qwy = quat.qwy;
		float qwz = quat.qwz;
		float qxy = quat.qx;
		float qxz = quat.qxz;
		float qyz = quat.qyz;
		vector3_float result={
			qw2 + qx2 - qy2 - qz2 ,\
			2*qxy - 2*qwz ,\
			2*qwy + 2*qxz
		};
		return result;
	}
	vector3_float QuaternionEf_reverse_rotate_axis_y( QuaternionEf quat )
	{
		float qw2 = quat.qw2;
		float qx2 = quat.qx2;
		float qy2 = quat.qy2;
		float qz2 = quat.qz2;
		float qwx = quat.qwx;
		float qwy = quat.qwy;
		float qwz = quat.qwz;
		float qxy = quat.qx;
		float qxz = quat.qxz;
		float qyz = quat.qyz;
		vector3_float result={
			2*qwz + 2*qxy ,\
			qw2 - qx2 + qy2 - qz2 ,\
			2*qyz - 2*qwx
		};
		return result;
	}
	vector3_float QuaternionEf_reverse_rotate_axis_z( QuaternionEf quat )
	{
		float qw2 = quat.qw2;
		float qx2 = quat.qx2;
		float qy2 = quat.qy2;
		float qz2 = quat.qz2;
		float qwx = quat.qwx;
		float qwy = quat.qwy;
		float qwz = quat.qwz;
		float qxy = quat.qx;
		float qxz = quat.qxz;
		float qyz = quat.qyz;
		vector3_float result={
			2*qxz - 2*qwy ,\
			2*qwx + 2*qyz ,\
			qw2 - qx2 - qy2 + qz2
		};
		return result;
	}
/*用四元数旋转向量*/
	
/*四元数积分*/
	Quaternion Quaternion_Integral_Runge1( Quaternion quat , vector3_float delta_angle0 )
	{
		float tqw=quat.qw;	float tqx=quat.qx;	float tqy=quat.qy;	float tqz=quat.qz;
		quat.qw += 0.5f * ( -tqx*delta_angle0.x - tqy*delta_angle0.y - tqz*delta_angle0.z );
		quat.qx += 0.5f * ( tqw*delta_angle0.x + tqy*delta_angle0.z - tqz*delta_angle0.y );
		quat.qy += 0.5f * ( tqw*delta_angle0.y - tqx*delta_angle0.z + tqz*delta_angle0.x );
		quat.qz += 0.5f * ( tqw*delta_angle0.z + tqx*delta_angle0.y - tqy*delta_angle0.x );
		quat = Quaternion_normalize( quat );
		return quat;
	}
	Quaternion Quaternion_Integral_Runge2( Quaternion quat , vector3_float delta_angle0 , vector3_float delta_angle1 )
	{
		float qwdx = quat.qw*delta_angle1.x , qwdy = quat.qw*delta_angle1.y , qwdz = quat.qw*delta_angle1.z;
		float qxdx = quat.qx*delta_angle1.x , qxdy = quat.qx*delta_angle1.y , qxdz = quat.qx*delta_angle1.z;
		float qydx = quat.qy*delta_angle1.x , qydy = quat.qy*delta_angle1.y , qydz = quat.qy*delta_angle1.z;
		float qzdx = quat.qz*delta_angle1.x , qzdy = quat.qz*delta_angle1.y , qzdz = quat.qz*delta_angle1.z;
		float qw2 = quat.qw*2.0f , qx2 = quat.qx*2.0f , qy2 = quat.qy*2.0f ,  qz2 = quat.qz*2.0f;
		
		quat.qw += 0.125f*( (qydz - qwdx - qzdy - qx2)*delta_angle0.x + (qzdx - qwdy - qy2 - qxdz)*delta_angle0.y + (qxdy - qydx - qz2 - qwdz)*delta_angle0.z ) - 0.25f*( qxdx + qydy + qzdz );
		quat.qx += 0.125f*( (qw2 - qxdx + qydy + qzdz)*delta_angle0.x + (qwdz - qydx - qxdy - qz2)*delta_angle0.y + (qy2 - qwdy - qzdx - qxdz)*delta_angle0.z ) + 0.25f*( qwdx - qzdy + qydz );
		quat.qy += 0.125f*( (qz2 - qydx - qxdy - qwdz)*delta_angle0.x + (qw2 + qxdx - qydy + qzdz)*delta_angle0.y + (qwdx - qx2 - qzdy - qydz)*delta_angle0.z ) + 0.25f*( qzdx + qwdy - qxdz );
		quat.qz += 0.125f*( (qwdy - qy2 - qzdx - qxdz)*delta_angle0.x + (qx2 - qwdx - qzdy - qydz)*delta_angle0.y + (qw2 + qxdx + qydy - qzdz)*delta_angle0.z ) - 0.25f*( qydx - qxdy - qwdz );
		quat = Quaternion_normalize( quat );
		return quat;
	}
	
	QuaternionEf QuaternionEf_Integral_Runge1( QuaternionEf quat , vector3_float delta_angle0 )
	{
		float tqw=quat.qw;	float tqx=quat.qx;	float tqy=quat.qy;	float tqz=quat.qz;
		quat.qw += 0.5f * ( -tqx*delta_angle0.x - tqy*delta_angle0.y - tqz*delta_angle0.z );
		quat.qx += 0.5f * ( tqw*delta_angle0.x + tqy*delta_angle0.z - tqz*delta_angle0.y );
		quat.qy += 0.5f * ( tqw*delta_angle0.y - tqx*delta_angle0.z + tqz*delta_angle0.x );
		quat.qz += 0.5f * ( tqw*delta_angle0.z + tqx*delta_angle0.y - tqy*delta_angle0.x );
		quat = QuaternionEf_normalize( quat );
		return quat;
	}
	QuaternionEf QuaternionEf_Integral_Runge2( QuaternionEf quat , vector3_float delta_angle0, vector3_float delta_angle1  )
	{
		float qwdx = quat.qw*delta_angle1.x , qwdy = quat.qw*delta_angle1.y , qwdz = quat.qw*delta_angle1.z;
		float qxdx = quat.qx*delta_angle1.x , qxdy = quat.qx*delta_angle1.y , qxdz = quat.qx*delta_angle1.z;
		float qydx = quat.qy*delta_angle1.x , qydy = quat.qy*delta_angle1.y , qydz = quat.qy*delta_angle1.z;
		float qzdx = quat.qz*delta_angle1.x , qzdy = quat.qz*delta_angle1.y , qzdz = quat.qz*delta_angle1.z;
		float qw2 = quat.qw*2.0f , qx2 = quat.qx*2.0f , qy2 = quat.qy*2.0f ,  qz2 = quat.qz*2.0f;
		
		quat.qw += 0.125f*( (qydz - qwdx - qzdy - qx2)*delta_angle0.x + (qzdx - qwdy - qy2 - qxdz)*delta_angle0.y + (qxdy - qydx - qz2 - qwdz)*delta_angle0.z ) - 0.25f*( qxdx + qydy + qzdz );
		quat.qx += 0.125f*( (qw2 - qxdx + qydy + qzdz)*delta_angle0.x + (qwdz - qydx - qxdy - qz2)*delta_angle0.y + (qy2 - qwdy - qzdx - qxdz)*delta_angle0.z ) + 0.25f*( qwdx - qzdy + qydz );
		quat.qy += 0.125f*( (qz2 - qydx - qxdy - qwdz)*delta_angle0.x + (qw2 + qxdx - qydy + qzdz)*delta_angle0.y + (qwdx - qx2 - qzdy - qydz)*delta_angle0.z ) + 0.25f*( qzdx + qwdy - qxdz );
		quat.qz += 0.125f*( (qwdy - qy2 - qzdx - qxdz)*delta_angle0.x + (qx2 - qwdx - qzdy - qydz)*delta_angle0.y + (qw2 + qxdx + qydy - qzdz)*delta_angle0.z ) - 0.25f*( qydx - qxdy - qwdz );
		quat = QuaternionEf_normalize( quat );
		return quat;
	}
/*四元数积分*/
	
/*四元数乘法*/
	Quaternion Quaternion_Mult( Quaternion quat_a, Quaternion quat_b )
	{
		Quaternion result = 
		{
			quat_a.qw*quat_b.qw - quat_a.qx*quat_b.qx - quat_a.qy*quat_b.qy - quat_a.qz*quat_b.qz,
			quat_a.qw*quat_b.qx + quat_a.qx*quat_b.qw + quat_a.qy*quat_b.qz - quat_a.qz*quat_b.qy, 
			quat_a.qw*quat_b.qy + quat_a.qy*quat_b.qw - quat_a.qx*quat_b.qz + quat_a.qz*quat_b.qx, 
			quat_a.qw*quat_b.qz + quat_a.qx*quat_b.qy - quat_a.qy*quat_b.qx + quat_a.qz*quat_b.qw 
		};
		result = Quaternion_normalize( result );
		return result;
	}
	QuaternionEf QuaternionEf_Mult( QuaternionEf quat_a , QuaternionEf quat_b )
	{
		Quaternion result = 
		{
			quat_a.qw*quat_b.qw - quat_a.qx*quat_b.qx - quat_a.qy*quat_b.qy - quat_a.qz*quat_b.qz,
			quat_a.qw*quat_b.qx + quat_a.qx*quat_b.qw + quat_a.qy*quat_b.qz - quat_a.qz*quat_b.qy, 
			quat_a.qw*quat_b.qy + quat_a.qy*quat_b.qw - quat_a.qx*quat_b.qz + quat_a.qz*quat_b.qx, 
			quat_a.qw*quat_b.qz + quat_a.qx*quat_b.qy - quat_a.qy*quat_b.qx + quat_a.qz*quat_b.qw 
		};
		return get_QuaternionEf( Quaternion_normalize( result ) );
	}
/*四元数乘法*/
	
/*用旋转向量旋转四元数*/
	Quaternion Quaternion_rotate_delta_angle( Quaternion quat , vector3_float delta_angle )
	{
		float angle;
		float angle_sin , angle_cosin;
		arm_sqrt_f32( vector3_float_square( delta_angle ) , &angle );
		float half_angle = angle * 0.5f;
		if( is_zero_f( half_angle ) )
			return quat;
		arm_sin_cos_f32( rad2degree( half_angle ) , &angle_sin , &angle_cosin );
		
		float qxyz_scale = angle_sin / angle;	
		float Gw = angle_cosin;	float Gx = delta_angle.x * qxyz_scale;	float Gy = delta_angle.y * qxyz_scale;	float Gz = delta_angle.z * qxyz_scale;
		float tqw=quat.qw;	float tqx=quat.qx;	float tqy=quat.qy;	float tqz=quat.qz;
		Quaternion result;
		result.qw = Gw*tqw - Gx*tqx - Gy*tqy - Gz*tqz;
		result.qx = Gw*tqx + Gx*tqw + Gy*tqz - Gz*tqy;
		result.qy = Gw*tqy + Gy*tqw - Gx*tqz + Gz*tqx;
		result.qz = Gw*tqz + Gx*tqy - Gy*tqx + Gz*tqw;
		
		return Quaternion_normalize( result );
	}
	QuaternionEf QuaternionEf_rotate_delta_angle( QuaternionEf quat , vector3_float delta_angle )
	{
		float angle;
		float angle_sin , angle_cosin;
		arm_sqrt_f32( vector3_float_square( delta_angle ) , &angle );
		float half_angle = angle * 0.5f;
		if( is_zero_f( half_angle ) )
			return quat;
		arm_sin_cos_f32( rad2degree( half_angle ) , &angle_sin , &angle_cosin );
		
		float qxyz_scale = angle_sin / angle;	
		float Gw = angle_cosin;	float Gx = delta_angle.x * qxyz_scale;	float Gy = delta_angle.y * qxyz_scale;	float Gz = delta_angle.z * qxyz_scale;
		float tqw=quat.qw;	float tqx=quat.qx;	float tqy=quat.qy;	float tqz=quat.qz;
		Quaternion result;
		result.qw = Gw*tqw - Gx*tqx - Gy*tqy - Gz*tqz;
		result.qx = Gw*tqx + Gx*tqw + Gy*tqz - Gz*tqy;
		result.qy = Gw*tqy + Gy*tqw - Gx*tqz + Gz*tqx;
		result.qz = Gw*tqz + Gx*tqy - Gy*tqx + Gz*tqw;
		
		return get_QuaternionEf( Quaternion_normalize( result ) );
	}
/*用旋转向量旋转四元数*/
	
/*四元数转欧拉角*/
	float Quaternion_getPitch( Quaternion quat )
	{
		return asinf( 2.0f*(quat.qw*quat.qy-quat.qx*quat.qz) );
	}
	float Quaternion_getRoll( Quaternion quat )
	{
		return atan2f( 2.0f*(quat.qw*quat.qx+quat.qy*quat.qz) ,\
			1.0f-2.0f*(quat.qx*quat.qx+quat.qy*quat.qy) );
	}	
	float Quaternion_getYaw( Quaternion quat )
	{
		return atan2f( 2.0f*(quat.qw*quat.qz+quat.qx*quat.qy) ,\
			1.0f-2.0f*(quat.qy*quat.qy+quat.qz*quat.qz) );
	}
/*四元数转欧拉角*/
	
/*从四元数中提取Pitch Roll旋转四元数（去除Yaw）*/
	Quaternion Quaternion_get_PRQuat( Quaternion quat )
	{
		float tqx;
		float tqy;
		float tqw;
		
		float qw2 = quat.qw * quat.qw;
		float qx2 = quat.qx * quat.qx;
		float qy2 = quat.qy * quat.qy;
		float qz2 = quat.qz * quat.qz;
		float qwx = quat.qw * quat.qx;
		float qwy = quat.qw * quat.qy;
		float qwz = quat.qw * quat.qz;
		float qxy = quat.qx * quat.qy;
		float qxz = quat.qx * quat.qz;
		float qyz = quat.qy * quat.qz;
		
		float qw2Pqz2 = ( qw2 + qz2 );
		if( !is_zero_f( qw2Pqz2 ) )
		{		
			tqw = safe_sqrt_f( qw2Pqz2 );
			float inv_tqw = 1.0f / tqw;
			tqx = ( qwx + qyz ) * inv_tqw;
			tqy = ( qwy - qxz ) * inv_tqw;					
		}
		else
		{
			tqw = 0.0f;
			tqx = quat.qx;	tqy = quat.qy;
		}
		Quaternion result = { tqw , tqx , tqy , 0 };
		return result;
	}
/*从四元数中提取Pitch Roll旋转四元数（去除Yaw）*/