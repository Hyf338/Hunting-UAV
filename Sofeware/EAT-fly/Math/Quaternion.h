#pragma once

#include "vector_3.h"

//四元数
typedef struct
{
	float qw;
	float qx;
	float qy;
	float qz;
}Quaternion;
//高效四元数
//带运算中间变量
//用于提升运算效率
typedef struct
{
	float qw;
	float qx;
	float qy;
	float qz;
	
	float qw2 , qx2 , qy2, qz2 , qwx , qwy , qwz , qxy , qxz , qyz;
}QuaternionEf;

//由标准四元数获取高效四元数
static inline QuaternionEf get_QuaternionEf( Quaternion quat )
{
	QuaternionEf result;
	result.qw = quat.qw;
	result.qx = quat.qx;
	result.qy = quat.qy;
	result.qz = quat.qz;
	result.qw2 = quat.qw * quat.qw;
	result.qx2 = quat.qx * quat.qx;
	result.qy2 = quat.qy * quat.qy;
	result.qz2 = quat.qz * quat.qz;
	result.qwx = quat.qw * quat.qx;
	result.qwy = quat.qw * quat.qy;
	result.qwz = quat.qw * quat.qz;
	result.qxy = quat.qx * quat.qy;
	result.qxz = quat.qx * quat.qz;
	result.qyz = quat.qy * quat.qz;
	return result;
}
static inline Quaternion get_Quaternion( QuaternionEf quat )
{
	Quaternion result;
	result.qw = quat.qw;
	result.qx = quat.qx;
	result.qy = quat.qy;
	result.qz = quat.qz;
	return result;
}

/*获得归一化四元数*/
	Quaternion Quaternion_normalize( Quaternion quat );
	QuaternionEf QuaternionEf_normalize( QuaternionEf quat );
/*获得归一化四元数*/

/*初始化四元数*/
	static inline Quaternion Quaternion_init_qs( float qw , float qx , float qy , float qz )
	{
		Quaternion quat;
		quat.qw = qw;
		quat.qx = qx;
		quat.qy = qy;
		quat.qz = qz;
		return quat;
	}
/*初始化四元数*/

/*由四元数获得旋转向量*/
	vector3_float Quaternion_get_Rotation_vec( Quaternion quat );
	vector3_float QuaternionEf_get_Rotation_vec( Quaternion quat );
/*由四元数获得旋转向量*/

/*获得用四元数旋转后的向量*/
	vector3_float Quaternion_rotate( Quaternion quat , vector3_float vec );
	vector3_float Quaternion_rotate_axis_x( Quaternion quat );
	vector3_float Quaternion_rotate_axis_y( Quaternion quat );
	vector3_float Quaternion_rotate_axis_z( Quaternion quat );
	vector3_float Quaternion_reverse_rotate( Quaternion quat , vector3_float vec );
	vector3_float Quaternion_reverse_rotate_axis_x( Quaternion quat );
	vector3_float Quaternion_reverse_rotate_axis_y( Quaternion quat );
	vector3_float Quaternion_reverse_rotate_axis_z( Quaternion quat );

	vector3_float QuaternionEf_rotate( QuaternionEf quat , vector3_float vec );
	vector3_float QuaternionEf_rotate_axis_x( QuaternionEf quat );
	vector3_float QuaternionEf_rotate_axis_y( QuaternionEf quat );
	vector3_float QuaternionEf_rotate_axis_z( QuaternionEf quat );
	vector3_float QuaternionEf_reverse_rotate( QuaternionEf quat , vector3_float vec );
	vector3_float QuaternionEf_reverse_rotate_axis_x( QuaternionEf quat );
	vector3_float QuaternionEf_reverse_rotate_axis_y( QuaternionEf quat );
	vector3_float QuaternionEf_reverse_rotate_axis_z( QuaternionEf quat );
/*获得用四元数旋转后的向量*/

/*四元数积分*/
	Quaternion Quaternion_Integral_Runge1( Quaternion quat , vector3_float delta_angle0 );
	Quaternion Quaternion_Integral_Runge2( Quaternion quat , vector3_float delta_angle0 , vector3_float delta_angle1 );
	
	QuaternionEf QuaternionEf_Integral_Runge1( QuaternionEf quat , vector3_float delta_angle0 );
	QuaternionEf QuaternionEf_Integral_Runge2( QuaternionEf quat , vector3_float delta_angle0, vector3_float delta_angle1  );
/*四元数积分*/

/*四元数乘法*/
	Quaternion Quaternion_Mult( Quaternion quat_a , Quaternion quat_b );
	QuaternionEf QuaternionEf_Mult( QuaternionEf quat_a , QuaternionEf quat_b );
/*四元数乘法*/

/*用旋转向量旋转四元数*/
	Quaternion Quaternion_rotate_delta_angle( Quaternion quat , vector3_float delta_angle );
	QuaternionEf QuaternionEf_rotate_delta_angle( QuaternionEf quat , vector3_float delta_angle );
/*用旋转向量旋转四元数*/

/*四元数转欧拉角*/
	float Quaternion_getPitch( Quaternion quat );
	float Quaternion_getRoll( Quaternion quat );
	float Quaternion_getYaw( Quaternion quat );
/*四元数转欧拉角*/

/*从四元数中提取Pitch Roll旋转四元数（去除Yaw）*/
	Quaternion Quaternion_get_PRQuat( Quaternion quat );
/*从四元数中提取Pitch Roll旋转四元数（去除Yaw）*/

/*四元数取逆（共轭）*/
	static inline Quaternion Quaternion_conjugate( Quaternion quat )
	{
		Quaternion result;
		result.qw = quat.qw;
		result.qx = -quat.qx;
		result.qy = -quat.qy;
		result.qz = -quat.qz;
		return result;
	}
	static inline QuaternionEf QuaternionEf_conjugate( QuaternionEf quat )
	{
		Quaternion result;
		result.qw = quat.qw;
		result.qx = -quat.qx;
		result.qy = -quat.qy;
		result.qz = -quat.qz;
		return get_QuaternionEf( result );
	}
/*四元数取逆（共轭）*/