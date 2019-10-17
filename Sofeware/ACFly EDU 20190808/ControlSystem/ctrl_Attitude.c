//朱文杰 20190101
//带模型ESO+反步姿态控制器
//悬停油门无参数观测器
//风力扰动观测器
//多旋翼优先级动力分配
//电机推力非线性修正
//论文正在发表
//！！请勿用于商业用途！！
//！！抄袭必究！！



#include "Basic.h"
#include "ctrl_Attitude.h"

#include "ControlSystem.h"
#include "MeasurementSystem.h"
#include "Configurations.h"
#include "drv_PWMOut.h"
#include "Receiver.h"
#include "Sensors.h"

#include "AC_Math.h"
#include "Quaternion.h"
#include "TD4.h"
#include "Filters_Butter.h"
#include "ESO_AngularRate.h"
#include "ESO_h.h"

/*状态观测器*/
	//高度ESO
	static ESO_h ESO_height;
	static float hover_throttle = 0;
	static float WindDisturbance_x = 0;
	static float WindDisturbance_y = 0;
	static bool inFlight = false;
	static inline void update_output_throttle( float throttle , float h )
	{
		vector3_float AccENU = get_AccelerationCtrl();
		float output_minimum_throttle = Cfg_get_MotorStartingThrottle();
		float lean_cosin = get_lean_angle_cosin();
		
		//观测悬停油门
		float r_throttle = throttle - output_minimum_throttle;
		if( r_throttle < 0 )
			r_throttle = 0;
		r_throttle *= lean_cosin;
		ESO_h_update_u( &ESO_height , r_throttle );
		hover_throttle = ESO_h_run( &ESO_height , AccENU.z , h ) + output_minimum_throttle;
		
		//更新飞行状态
		static uint16_t onGround_counter = 0;
		if( inFlight == false )
		{
			onGround_counter = 0;
			if( AccENU.z > 20 && throttle > output_minimum_throttle + 5 )
				inFlight = true;
		}
		else
		{
			if( hover_throttle < output_minimum_throttle + 2 && \
					in_symmetry_range_float( vector3_float_square( AccENU ) , 20*20 ) && \
					in_symmetry_range_float( vector3_float_square( get_VelocityENU() ) , 25*25 ) \
				)
			{
				if( ++onGround_counter >= 300 )
					inFlight = false;
			}
			else
				onGround_counter = 0;
		}
		
		//观测水平分力
		if( inFlight )
		{
			vector3_float active_force_xy_vec = Quaternion_rotate_axis_z( get_history_Airframe_attitude( 0.42f / 10 ) );
			if( lean_cosin < 0.3f )
				lean_cosin = 0.3f;
			active_force_xy_vec = vector3_float_mult( active_force_xy_vec , ( AccENU.z + constG ) / lean_cosin );
			vector3_float WindDisturbance_xy;
			WindDisturbance_xy.x = AccENU.x - active_force_xy_vec.x;
			WindDisturbance_xy.y = AccENU.y - active_force_xy_vec.y;
			
			float lp_factor = 2 * PI_f * (1.0f/200) * 0.3f;
			WindDisturbance_x += lp_factor * ( WindDisturbance_xy.x - WindDisturbance_x );
			WindDisturbance_y += lp_factor * ( WindDisturbance_xy.y - WindDisturbance_y );
		}
		else
			WindDisturbance_x = WindDisturbance_y = 0;
	}
/*状态观测器*/
	
/*观测器*/
	float get_hover_throttle()
	{
		return hover_throttle;
	}
	bool get_is_inFlight()
	{
		return inFlight;
	}
	float get_WindDisturbance_x()
	{
		return WindDisturbance_x;
	}
	float get_WindDisturbance_y()
	{
		return WindDisturbance_y;
	}
/*观测器*/

/*模式及期望*/
	//期望TD4滤波器
	static TD4 Target_tracker[3];
	//姿态ESO
	static ESO_AngularRate ESO[3];
	//扰动滤波器
	static Filter_Butter2_LP_float disturbance_filter[3];

	//姿态控制模式
	static bool Attitude_Control_Enabled = false;
	static Attitude_ControlMode RollPitch_ControlMode = Attitude_ControlMode_Angle;
	static Attitude_ControlMode Yaw_ControlMode = Attitude_ControlMode_Angle;

	static float throttle = 0;
	static float target_Roll;
	static float target_Pitch;
	static float target_Yaw;
	static vector3_float target_AngularRate;
	
	float get_Target_Throttle()
	{
		return throttle;
	}
	
	bool Attitude_Control_Enable()
	{
		if( Attitude_Control_Enabled == true ||\
			get_Attitude_Measurement_System_Status() != Measurement_System_Status_Ready )
			return false;
		
		Attitude_Control_Enabled = true;
		target_Yaw = Quaternion_getYaw( get_Airframe_attitude() );
		RollPitch_ControlMode = Attitude_ControlMode_Angle;
		Yaw_ControlMode = Attitude_ControlMode_Angle;
		
		/*初始化*/
			//初始化期望TD4滤波器
			TD4_init( &Target_tracker[0] , Cfg_get_RPYCtrl_TD4Pn( 0 , 1 ) , Cfg_get_RPYCtrl_TD4Pn( 0 , 2 ) , Cfg_get_RPYCtrl_TD4Pn( 0 , 3 ) , Cfg_get_RPYCtrl_TD4Pn( 0 , 4 ) );
			TD4_init( &Target_tracker[1] , Cfg_get_RPYCtrl_TD4Pn( 1 , 1 ) , Cfg_get_RPYCtrl_TD4Pn( 1 , 2 ) , Cfg_get_RPYCtrl_TD4Pn( 1 , 3 ) , Cfg_get_RPYCtrl_TD4Pn( 1 , 4 ) );
			TD4_init( &Target_tracker[2] , Cfg_get_RPYCtrl_TD4Pn( 2 , 1 ) , Cfg_get_RPYCtrl_TD4Pn( 2 , 2 ) , Cfg_get_RPYCtrl_TD4Pn( 2 , 3 ) , Cfg_get_RPYCtrl_TD4Pn( 2 , 4 ) );
			
			//初始化姿态ESO
			init_ESO_AngularRate( &ESO[0] , Cfg_get_MotorT(0) , Cfg_get_RPYCtrl_b(0) , 0.7f , 10 );
			init_ESO_AngularRate( &ESO[1] , Cfg_get_MotorT(1) , Cfg_get_RPYCtrl_b(1) , 0.7f , 10 );
			init_ESO_AngularRate( &ESO[2] , 1.0f/200 , Cfg_get_RPYCtrl_b(2) , 0.7f , 10 );
		/*初始化*/
		
		return true;
	}
	bool Attitude_Control_Disable()
	{
		if( Attitude_Control_Enabled == false )
			return false;		
		Attitude_Control_Enabled = false;
		Altitude_Control_Disable();
		Position_Control_Disable();
		return true;
	}
	
	bool Attitude_Control_set_Throttle( float thr )
	{
		if( Attitude_Control_Enabled == false )
			return false;
		throttle = thr;
		return true;
	}
	float Attitude_Control_get_Target_Roll() { return target_Roll; }
	float Attitude_Control_get_Target_Pitch() { return target_Pitch; }
	float Attitude_Control_get_Target_Yaw() { return target_Yaw; }
	bool Attitude_Control_set_Target_RollPitch( float Roll , float Pitch )
	{
		float angle = safe_sqrt_f( Roll*Roll + Pitch*Pitch );
		if( angle > 0.8f )
		{
			float scale = 0.8f / angle;
			Roll *= scale;
			Pitch *= scale;
		}
		
		if( Attitude_Control_Enabled == false )
			return false;
		target_Roll = Roll;
		target_Pitch = Pitch;
		RollPitch_ControlMode = Attitude_ControlMode_Angle;
		return true;
	}
	bool Attitude_Control_set_Target_Yaw( float Yaw )
	{
		if( Attitude_Control_Enabled == false )
			return false;
		target_Yaw = Yaw;
		if( Yaw_ControlMode != Attitude_ControlMode_Angle )
		{
			Target_tracker[2].x1 = Quaternion_getYaw( get_Airframe_attitude() );
			Yaw_ControlMode = Attitude_ControlMode_Angle;
		}
		return true;
	}
	bool Attitude_Control_set_Target_YawRelative( float Yaw )
	{
		if( Attitude_Control_Enabled == false )
			return false;
		float currentYaw = Quaternion_getYaw( get_Airframe_attitude() );
		if( Yaw_ControlMode != Attitude_ControlMode_Angle )
		{
			Target_tracker[2].x1 = currentYaw;
			Yaw_ControlMode = Attitude_ControlMode_Angle;
		}
		target_Yaw = Target_tracker[2].x1 + Yaw;
		return true;
	}
	bool Attitude_Control_set_Target_YawRate( float YawRate )
	{
		if( Attitude_Control_Enabled == false )
			return false;
		target_AngularRate.z = YawRate;
		Yaw_ControlMode = Attitude_ControlMode_AngularRate;
		return true;
	}
	bool Attitude_Control_set_YawLock()
	{
		if( Attitude_Control_Enabled == false )
			return false;
		if( Yaw_ControlMode == Attitude_ControlMode_AngularRate )
			Yaw_ControlMode = Attitude_ControlMode_Locking;
		return true;
	}
/*模式及期望*/

/*机型*/
	//四旋翼X模式
	static void ctrl_Attitude_MultiRotor_X4_PWM( float outRoll , float outPitch , float outYaw );
	//六旋翼X模式
	static void ctrl_Attitude_MultiRotor_X6_PWM( float outRoll , float outPitch , float outYaw );
		
	//四旋翼十字型
	static void ctrl_Attitude_MultiRotor_C4_PWM( float outRoll , float outPitch , float outYaw );
		
	//双层四旋翼十字型
	static void ctrl_Attitude_MultiRotor_C42_PWM( float outRoll , float outPitch , float outYaw );
/*机型*/

void ctrl_Attitude()
{
	if( Attitude_Control_Enabled == false )
	{
		update_output_throttle( 0 , 1.0f / 200 );
		PWM_PullDownAll();
		return;
	}
	
	//Debug保护
	//油门低于起转油门拉低所有输出
	if( throttle < Cfg_get_MotorStartingThrottle() - 0.1f )
	{
		update_output_throttle( 0 , 1.0f / 200 );
		PWM_PullDownAll();
		return;
	}
	
	//根据电池电压调整控制对象增益
	float BatV = getBatteryVoltage();
	float VST = Cfg_get_BatSTVoltage();
	if( BatV > 7 && VST > 7 )
	{
		float scale = BatV / VST;
		ESO[0].b = Cfg_get_RPYCtrl_b(0) * scale;
		ESO[1].b = Cfg_get_RPYCtrl_b(1) * scale;
		ESO[2].b = Cfg_get_RPYCtrl_b(2) * scale;
	}
	else
	{
		ESO[0].b = Cfg_get_RPYCtrl_b(0);
		ESO[1].b = Cfg_get_RPYCtrl_b(1);
		ESO[2].b = Cfg_get_RPYCtrl_b(2);
	}
	
	//获取控制参数
	float Ps = Cfg_get_RPYCtrl_Pn(0,1);
	vector3_float P2 = { Cfg_get_RPYCtrl_Pn(0,2) , Cfg_get_RPYCtrl_Pn(1,2) , Cfg_get_RPYCtrl_Pn(2,2) };
	vector3_float P3 = { Cfg_get_RPYCtrl_Pn(0,3) , Cfg_get_RPYCtrl_Pn(1,3) , Cfg_get_RPYCtrl_Pn(2,3) };
	vector3_float P4 = { Cfg_get_RPYCtrl_Pn(0,4) , Cfg_get_RPYCtrl_Pn(1,4) , Cfg_get_RPYCtrl_Pn(2,4) };
	
	//目标Roll Pitch四元数
	Quaternion target_quat_PR;			
	//目标角速度
	vector3_float target_angular_velocity;

	//获取当前四元数的Pitch Roll分量四元数
	QuaternionEf current_quat_PR = get_QuaternionEf( Quaternion_get_PRQuat( get_Airframe_attitude() ) );
	//获取Yaw角
	float Yaw = Quaternion_getYaw( get_Airframe_attitude() );
	
	//计算旋转矩阵
	float tqw2 = current_quat_PR.qw2;	float tqx2 = current_quat_PR.qx2;	float tqy2 = current_quat_PR.qy2;
	float tqwx = current_quat_PR.qwx;	float tqwy = current_quat_PR.qwy;	float tqxy = current_quat_PR.qxy;					
	float Rotation_Matrix[3][3] = //反向旋转
	{
		{ (tqw2 + tqx2 - tqy2) , (2*tqxy) , (-2*tqwy) } ,
		{ (2*tqxy) , (tqw2 - tqx2 + tqy2) , (2*tqwx) } ,
		{ (2*tqwy) ,(- 2*tqwx) , (tqw2 - tqx2 - tqy2) } ,
	};
	float Rotation_Matrix_P[3][3] = //正向旋转
	{
		{ (tqw2 + tqx2 - tqy2) , (2*tqxy) , (2*tqwy) } ,
		{ (2*tqxy) , (tqw2 - tqx2 + tqy2) , (-2*tqwx) } ,
		{ (-2*tqwy) ,(2*tqwx) , (tqw2 - tqx2 - tqy2) } ,
	};
	
	//运行扩张状态观测器得到估计角速度、角加速度
	vector3_float AngularRateCtrl = get_AngularRateCtrl();
	ESO_AngularRate_run( &ESO[0] , AngularRateCtrl.x , 1.0f / 200 );
	ESO_AngularRate_run( &ESO[1] , AngularRateCtrl.y , 1.0f / 200 );
	ESO_AngularRate_run( &ESO[2] , AngularRateCtrl.z , 1.0f / 200 );
	vector3_float angular_rate_ESO = 
	{ 
		ESO_AngularRate_get_EsAngularRate( &ESO[0] ) ,
		ESO_AngularRate_get_EsAngularRate( &ESO[1] ) ,
		ESO_AngularRate_get_EsAngularRate( &ESO[2] ) , 
	};
	vector3_float angular_acceleration_ESO = 
	{ 
		ESO_AngularRate_get_EsAngularAcceleration( &ESO[0] ) ,
		ESO_AngularRate_get_EsAngularAcceleration( &ESO[1] ) ,
		ESO_AngularRate_get_EsAngularAcceleration( &ESO[2] ) , 
	};
	
	//计算ENU坐标系下的角速度、角加速度
	vector3_float angular_rate_ENU;
	angular_rate_ENU.x = Rotation_Matrix_P[0][0]*angular_rate_ESO.x + Rotation_Matrix_P[0][1]*angular_rate_ESO.y + Rotation_Matrix_P[0][2]*angular_rate_ESO.z;
	angular_rate_ENU.y = Rotation_Matrix_P[1][0]*angular_rate_ESO.x + Rotation_Matrix_P[1][1]*angular_rate_ESO.y + Rotation_Matrix_P[1][2]*angular_rate_ESO.z;
	angular_rate_ENU.z = Rotation_Matrix_P[2][0]*angular_rate_ESO.x + Rotation_Matrix_P[2][1]*angular_rate_ESO.y + Rotation_Matrix_P[2][2]*angular_rate_ESO.z;
	vector3_float angular_acceleration_ENU;
	angular_acceleration_ENU.x = Rotation_Matrix_P[0][0]*angular_acceleration_ESO.x + Rotation_Matrix_P[0][1]*angular_acceleration_ESO.y + Rotation_Matrix_P[0][2]*angular_acceleration_ESO.z;
	angular_acceleration_ENU.y = Rotation_Matrix_P[1][0]*angular_acceleration_ESO.x + Rotation_Matrix_P[1][1]*angular_acceleration_ESO.y + Rotation_Matrix_P[1][2]*angular_acceleration_ESO.z;
	angular_acceleration_ENU.z = Rotation_Matrix_P[2][0]*angular_acceleration_ESO.x + Rotation_Matrix_P[2][1]*angular_acceleration_ESO.y + Rotation_Matrix_P[2][2]*angular_acceleration_ESO.z;
	
	//由Roll Pitch控制模式
	//计算Roll Pitch目标角速度（ENU系）
	vector3_float target_angular_rate_RP;
	switch( RollPitch_ControlMode )
	{
		case Attitude_ControlMode_Angle:
		{
			//TD4滤目标角度
			TD4_track4( &Target_tracker[0] , target_Roll , 1.0f / 200 );
			TD4_track4( &Target_tracker[1] , target_Pitch , 1.0f / 200 );
			
			//使用目标角度构造目标四元数
			//calculate target quat Q1
			//      front
			//       x
			//       ^
			//       |
			// y < --O
			float sin_R = arm_sin_f32( Target_tracker[0].x1 );
			float sin_P = arm_sin_f32( Target_tracker[1].x1 );		
			float C = 1 - sin_P*sin_P - sin_R*sin_R;
			if( C >= 0 )
			{
				C = safe_sqrt_f( C );
				float w2 = 0.5f * ( 1 + C );
				float w = safe_sqrt_f( w2 );
				if( w > 0.0001f )
				{
					float inv_w = 0.5f / w;
					target_quat_PR.qw = w;
					target_quat_PR.qx = sin_R * inv_w;
					target_quat_PR.qy = sin_P * inv_w;
					target_quat_PR.qz = 0;
				}
				else
				{
					target_quat_PR.qw = 1;
					target_quat_PR.qx = 0;
					target_quat_PR.qy = 0;
					target_quat_PR.qz = 0;
				}
			}
			
			//计算误差四元数Q
			//Q*Q1=Qt  Q1为当前机体四元数，Qt为目标四元数
			//Q=Qt*inv(Q1)
			Quaternion current_quat_conj = Quaternion_conjugate( get_Quaternion( current_quat_PR ) );
			vector3_float PR_rotation = Quaternion_get_Rotation_vec( Quaternion_Mult( target_quat_PR , current_quat_conj ) );
			vector3_float feed_foward_ratePR = { Target_tracker[0].x2 , Target_tracker[1].x2 , 0 };
			target_angular_rate_RP = vector3_float_plus( vector3_float_mult( PR_rotation , Ps ) , feed_foward_ratePR );
			break;
		}
	}
	
	float target_angular_rate_Y;
	switch(Yaw_ControlMode)
	{
		case Attitude_ControlMode_Angle:
		{
			//TD4滤目标角度
			Target_tracker[2].r2n = Target_tracker[2].r2p = 1.5f;
			TD4_track4( &Target_tracker[2] , target_Yaw , 1.0f / 200 );
			
			//角度误差化为-180 - +180
			float angle_error = Target_tracker[2].x1 - Yaw;
			while( angle_error < -PI_f )
				angle_error+=2*PI_f;
			while( angle_error > PI_f )
				angle_error-=2*PI_f;

			//求目标角速度
			target_angular_rate_Y = angle_error * Ps + Target_tracker[2].x2;
			target_angular_rate_Y = constrain_float( target_angular_rate_Y , 2.5f );
			break;
		}
		case Attitude_ControlMode_AngularRate:
		{
			TD4_track3( &Target_tracker[2] , target_AngularRate.z , 1.0f / 200 );
			target_angular_rate_Y = Target_tracker[2].x2;
			break;
		}
		case Attitude_ControlMode_Locking:
		{
			TD4_track3( &Target_tracker[2] , 0 , 1.0f / 200 );
			target_angular_rate_Y = Target_tracker[2].x2;
			if( in_symmetry_range_float( target_angular_rate_Y , 0.001f ) && in_symmetry_range_float( angular_rate_ENU.z , 0.05f ) )
			{							
				Target_tracker[2].x1 = target_Yaw = Yaw;
				Yaw_ControlMode = Attitude_ControlMode_Angle;
			}
			
			break;
		}
	}
	
	//计算前馈量
		float YawAngleP =  ( Target_tracker[2].tracking_mode == 4 ) ? ( Ps ) : 0;
		vector3_float Tv1_ENU = { Ps*( Target_tracker[0].x2 - angular_rate_ENU.x ) + Target_tracker[0].x3 ,
															Ps*( Target_tracker[1].x2 - angular_rate_ENU.y ) + Target_tracker[1].x3 ,
															YawAngleP*( Target_tracker[2].x2 - angular_rate_ENU.z ) + Target_tracker[2].x3 };
		vector3_float Tv2_ENU = { Ps*( Target_tracker[0].x3 - angular_acceleration_ENU.x ) + Target_tracker[0].x4 ,
															Ps*( Target_tracker[1].x3 - angular_acceleration_ENU.y ) + Target_tracker[1].x4 ,
															YawAngleP*( Target_tracker[2].x3 - angular_acceleration_ENU.z ) + Target_tracker[2].x4 };
		
		vector3_float Tv1;
		Tv1.x = Rotation_Matrix[0][0]*Tv1_ENU.x + Rotation_Matrix[0][1]*Tv1_ENU.y + Rotation_Matrix[0][2]*Tv1_ENU.z;
		Tv1.y = Rotation_Matrix[1][0]*Tv1_ENU.x + Rotation_Matrix[1][1]*Tv1_ENU.y + Rotation_Matrix[1][2]*Tv1_ENU.z;
		Tv1.z = Rotation_Matrix[2][0]*Tv1_ENU.x + Rotation_Matrix[2][1]*Tv1_ENU.y + Rotation_Matrix[2][2]*Tv1_ENU.z;
		vector3_float Tv2;
		Tv2.x = Rotation_Matrix[0][0]*Tv2_ENU.x + Rotation_Matrix[0][1]*Tv2_ENU.y + Rotation_Matrix[0][2]*Tv2_ENU.z;
		Tv2.y = Rotation_Matrix[1][0]*Tv2_ENU.x + Rotation_Matrix[1][1]*Tv2_ENU.y + Rotation_Matrix[1][2]*Tv2_ENU.z;
		Tv2.z = Rotation_Matrix[2][0]*Tv2_ENU.x + Rotation_Matrix[2][1]*Tv2_ENU.y + Rotation_Matrix[2][2]*Tv2_ENU.z;
		vector3_float Ta1 = { P2.x*( Tv1.x - angular_acceleration_ESO.x ) + Tv2.x ,
													P2.y*( Tv1.y - angular_acceleration_ESO.y ) + Tv2.y ,
													P2.z*( Tv1.z - angular_acceleration_ESO.z ) + Tv2.z };
	//计算前馈量
													
	//把目标速度从Bodyheading旋转到机体
		vector3_float target_angular_rate_ENU;
		target_angular_rate_ENU.x = target_angular_rate_RP.x;
		target_angular_rate_ENU.y = target_angular_rate_RP.y;
		target_angular_rate_ENU.z = target_angular_rate_RP.z + target_angular_rate_Y;

		vector3_float target_angular_rate_body;
		target_angular_rate_body.x = Rotation_Matrix[0][0]*target_angular_rate_ENU.x + Rotation_Matrix[0][1]*target_angular_rate_ENU.y + Rotation_Matrix[0][2]*target_angular_rate_ENU.z;
		target_angular_rate_body.y = Rotation_Matrix[1][0]*target_angular_rate_ENU.x + Rotation_Matrix[1][1]*target_angular_rate_ENU.y + Rotation_Matrix[1][2]*target_angular_rate_ENU.z;
		target_angular_rate_body.z = Rotation_Matrix[2][0]*target_angular_rate_ENU.x + Rotation_Matrix[2][1]*target_angular_rate_ENU.y + Rotation_Matrix[2][2]*target_angular_rate_ENU.z;
	//把目标速度从Bodyheading旋转到机体
													
	//计算目标角加速度
		vector3_float target_angular_acceleration = vector3_float_subtract( target_angular_rate_body , angular_rate_ESO );
		target_angular_acceleration.x *= P2.x;
		target_angular_acceleration.y *= P2.y;
		target_angular_acceleration.z *= P2.z;
		target_angular_acceleration = vector3_float_plus( target_angular_acceleration , Tv1 );
	//计算目标角加速度
													
	//计算角加速度误差
	vector3_float angular_acceleration_error = vector3_float_subtract( target_angular_acceleration , angular_acceleration_ESO );
	
	vector3_float disturbance = { 
		ESO_AngularRate_get_EsDisturbance( &ESO[0] ) ,
		ESO_AngularRate_get_EsDisturbance( &ESO[1] ) ,
		ESO_AngularRate_get_EsDisturbance( &ESO[2] ) ,
	};
	static vector3_float last_disturbance = { 0 , 0 , 0 };		
	vector3_float disturbance_Derivative = vector3_float_mult( vector3_float_subtract( disturbance , last_disturbance ) , 200 );
	last_disturbance = disturbance;
	
	float disturbance_x = Filter_Butter2_LP_float_run( &disturbance_filter[0] , disturbance_Derivative.x );
	float disturbance_y = Filter_Butter2_LP_float_run( &disturbance_filter[1] , disturbance_Derivative.y );
	float disturbance_z = Filter_Butter2_LP_float_run( &disturbance_filter[2] , disturbance_Derivative.z );
	
	float outRoll;float outPitch;float outYaw;
	if( get_is_inFlight() )
	{
		outRoll = 	( ESO_AngularRate_get_EsMainPower( &ESO[0] ) + ESO[0].T * ( angular_acceleration_error.x * P3.x + Ta1.x /*- disturbance_x*/ ) )/ESO[0].b;
		outPitch =	( ESO_AngularRate_get_EsMainPower( &ESO[1] ) + ESO[1].T * ( angular_acceleration_error.y * P3.y + Ta1.y /*- disturbance_y*/ ) )/ESO[1].b;
//		outYaw =		( ESO_AngularRate_get_EsMainPower( &ESO[2] ) + ESO[2].T * ( angular_acceleration_error.z * P.z + Ta1.z /*- disturbance_z*/ ) )/ESO[2].b;
		outYaw = ( target_angular_acceleration.z - disturbance.z ) / ESO[2].b;
	}
	else
	{
		outRoll = 	ESO[0].T * ( angular_acceleration_error.x * P3.x )/ESO[0].b;
		outPitch =	ESO[1].T * ( angular_acceleration_error.y * P3.y )/ESO[1].b;
		//outYaw =		ESO[2].T * ( angular_acceleration_error.z * P.z )/ESO[2].b;
		outYaw = ( target_angular_acceleration.z ) / ESO[2].b;
	}
	
	switch( Cfg_get_UAVType() )
	{
		case UAVType_Rotor4_X:
			ctrl_Attitude_MultiRotor_X4_PWM( outRoll , outPitch , outYaw );
			break;		
		case UAVType_Rotor6_X:
			ctrl_Attitude_MultiRotor_X6_PWM( outRoll , outPitch , outYaw );
			break;
		
		case UAVType_Rotor4_C:
			ctrl_Attitude_MultiRotor_C4_PWM( outRoll , outPitch , outYaw );
			break;
		
		case UAVType_Rotor42_C:
			ctrl_Attitude_MultiRotor_C42_PWM( outRoll , outPitch , outYaw );
			break;
		
		default:
			PWM_PullDownAll();
			break;
	}
}

//电机非线性输出 线性修正
static inline void throttle_nonlinear_compensation( float out[8] )
{
	float output_minimum_throttle = Cfg_get_MotorStartingThrottle();
	float output_range = 100.0f - output_minimum_throttle;
	float inv_output_range = 1.0f / output_range;
	
	//a：非线性因子(0-1)
	//m：最大油门比例(0.6-1)
	
	//设油门-力曲线方程为：
	//F = kx^2 + (1-a)x ( 0<=x<=m F最大值为1 )
	//x = m时：km^2 + (1-a)m = 1
	//得k = ( 1 - (1-a)m ) / m^2
	//a_1 = a - 1
	//Hk  = 1 / 2k
	//K4  = 4* k
	//解方程组：kx^2 + (1-a)x = out
	//得到的x即为线性化后的输出
	float _lift_max = Cfg_get_MotorFullThrottleRatio();
	float a_1 = Cfg_get_MotorNonlineFactor() - 1;
	float k = ( 1 + a_1*_lift_max ) / (_lift_max*_lift_max);
	float Hk = 1.0f / (2*k);
	float K4 = 4 * k;
		
	for( uint8_t i = 0 ; i < 8 ; ++i )
	{
		out[i] -= output_minimum_throttle;
		out[i] *= inv_output_range;
		out[i] = Hk*( a_1 + safe_sqrt_f( a_1*a_1 + K4*out[i] ) );
		out[i] *= output_range;
		out[i] += output_minimum_throttle;
	}
}

//四旋翼X模式
static void ctrl_Attitude_MultiRotor_X4_PWM( float outRoll , float outPitch , float outYaw )
{
	float rotor_output[8] = {0,0,0,0,0,0,0,0};

	float output_minimum_throttle = Cfg_get_MotorStartingThrottle();
	if( get_current_Receiver()->data[0] < 5 )
	{
		PWM_PullDownAll();
		update_output_throttle( 0 , 1.0f/200 );
		return;
	}		
	
	if( throttle < output_minimum_throttle - 0.1f )
	{
		PWM_PullDownAll();
		update_output_throttle( 0 , 1.0f/200 );
		return;
	}		
			
	float output_throttle = throttle;
	float output_midpoint = ( 100.0f - output_minimum_throttle ) / 2;
//		float cos_angle = lean_cos;
//		if( cos_angle < 0.5f )cos_angle = 0.5f;
//		output_throttle = throttle / cos_angle;
	
	/*pitch roll output limit*/
		//if output for balance is out of range
		//at this throttle, lower throttle
		//to insure balance control 			
		rotor_output[0] = -outPitch+outRoll;
		rotor_output[1] = +outPitch+outRoll;		
		rotor_output[2] = +outPitch-outRoll;
		rotor_output[3] = -outPitch-outRoll;
		float output_max;
		float output_min ;
		output_max = rotor_output[0];
		for( int i = 1 ; i < 4 ; ++i )
		{
			if( rotor_output[i] > output_max ) output_max = rotor_output[i];
		}
		
		float max_allow_output = 100.0f - output_throttle;
		float min_allow_output = output_minimum_throttle - output_throttle;			
		//ensure Pitch Roll output
		//lower throttle if necessary
		//float allow_ouput_range = ( max_allow_output < -min_allow_output ) ? max_allow_output : -min_allow_output;
		float allow_ouput_range;
		if( max_allow_output < -min_allow_output )
		{
			allow_ouput_range = max_allow_output;
			if( output_max > allow_ouput_range )
			{
				if( output_max > output_midpoint )
				{
					output_throttle = output_midpoint + output_minimum_throttle;
					allow_ouput_range = output_midpoint;
				}
				else
				{
					output_throttle = 100.0f - output_max;
					allow_ouput_range = output_max;
				}
			}
		}
		else
		{
			allow_ouput_range = -min_allow_output;
			if( output_max > allow_ouput_range )
			{
				float hover_throttle_force = hover_throttle - output_minimum_throttle;
				float max_allowed_output_range = ( hover_throttle_force > output_midpoint ? output_midpoint : hover_throttle_force ) * 0.8f;
				if( max_allowed_output_range < output_throttle - output_minimum_throttle )
					max_allowed_output_range = output_throttle - output_minimum_throttle;
				float max_allowed_throttle = max_allowed_output_range + output_minimum_throttle;
				if( output_max > max_allowed_output_range )
				{
					output_throttle = max_allowed_throttle;
					allow_ouput_range = max_allowed_output_range;
				}
				else
				{
					output_throttle = output_minimum_throttle + output_max;
					allow_ouput_range = output_max;
				}
			}
		}
		
		if( output_max > allow_ouput_range )
		{
			float scale  = allow_ouput_range / output_max;
			rotor_output[0] *= scale;
			rotor_output[1] *= scale;
			rotor_output[2] *= scale;
			rotor_output[3] *= scale;
			ESO_AngularRate_update_u( &ESO[0] , outRoll * scale );
			ESO_AngularRate_update_u( &ESO[1] , outPitch * scale );
		}		
		else
		{
			ESO_AngularRate_update_u( &ESO[0] , outRoll );
			ESO_AngularRate_update_u( &ESO[1] , outPitch );
		}
	/*pitch roll output limit*/
	
	/*yaw output limit*/
		//for Yaw control, it has the lowest priority
		//lower output to ensure attitude control and alt control 
		output_max = 0.0f;
		output_min = 100.0f;

		/*find min yaw_scale*/
			float yaw_scale = 1.0f;
		
			for( int i = 0 ; i < 4 ; ++i )
			{
				float current_out_yaw = ( (i&1) == 1 ) ? outYaw : -outYaw;
				
				float current_rotor_output = output_throttle + rotor_output[i];
				max_allow_output = 100.0f - current_rotor_output;
				min_allow_output = output_minimum_throttle - current_rotor_output;
				if( current_out_yaw > max_allow_output + 0.01f )
				{
					float new_yaw_scale = max_allow_output / current_out_yaw;
					if( new_yaw_scale < yaw_scale ) yaw_scale = new_yaw_scale;
				}
				else if( current_out_yaw < min_allow_output - 0.01f )
				{
					float new_yaw_scale = min_allow_output / current_out_yaw;
					if( new_yaw_scale < yaw_scale ) yaw_scale = new_yaw_scale;
				}
			}
						
		/*find min yaw_scale*/
		
		//lower yaw output to ensure attitude control and alt control
		if( yaw_scale < 0 )
			yaw_scale = 0;
		outYaw *= yaw_scale;
		ESO_AngularRate_update_u( &ESO[2] , outYaw );
	/*yaw output limit*/
		
	update_output_throttle( output_throttle , 1.0f/200 );
	rotor_output[0] += output_throttle-outYaw;
	rotor_output[1] += output_throttle+outYaw;
	rotor_output[2] += output_throttle-outYaw;
	rotor_output[3] += output_throttle+outYaw;
	
	throttle_nonlinear_compensation( rotor_output );
	PWM_Out( rotor_output );
}

//四旋翼十字型
static void ctrl_Attitude_MultiRotor_C4_PWM( float outRoll , float outPitch , float outYaw )
{
	float rotor_output[8] = {0,0,0,0,0,0,0,0};

	float output_minimum_throttle = Cfg_get_MotorStartingThrottle();
	if( get_current_Receiver()->data[0] < 5 )
	{
		PWM_PullDownAll();
		update_output_throttle( 0 , 1.0f/200 );
		return;
	}		
	
	if( throttle < output_minimum_throttle - 0.1f )
	{
		PWM_PullDownAll();
		update_output_throttle( 0 , 1.0f/200 );
		return;
	}		
			
	float output_throttle = throttle;
	float output_midpoint = ( 100.0f - output_minimum_throttle ) / 2;
//		float cos_angle = lean_cos;
//		if( cos_angle < 0.5f )cos_angle = 0.5f;
//		output_throttle = throttle / cos_angle;
	
	/*pitch roll output limit*/
		//if output for balance is out of range
		//at this throttle, lower throttle
		//to insure balance control 			
		rotor_output[0] = -outPitch;
		rotor_output[1] = +outRoll;
		rotor_output[2] = +outPitch;
		rotor_output[3] = -outRoll;
		float output_max;
		float output_min ;
		output_max = rotor_output[0];
		for( int i = 1 ; i < 4 ; ++i )
		{
			if( rotor_output[i] > output_max ) output_max = rotor_output[i];
		}
		
		float max_allow_output = 100.0f - output_throttle;
		float min_allow_output = output_minimum_throttle - output_throttle;			
		//ensure Pitch Roll output
		//lower throttle if necessary
		//float allow_ouput_range = ( max_allow_output < -min_allow_output ) ? max_allow_output : -min_allow_output;
		float allow_ouput_range;
		if( max_allow_output < -min_allow_output )
		{
			allow_ouput_range = max_allow_output;
			if( output_max > allow_ouput_range )
			{
				if( output_max > output_midpoint )
				{
					output_throttle = output_midpoint + output_minimum_throttle;
					allow_ouput_range = output_midpoint;
				}
				else
				{
					output_throttle = 100.0f - output_max;
					allow_ouput_range = output_max;
				}
			}
		}
		else
		{
			allow_ouput_range = -min_allow_output;
			if( output_max > allow_ouput_range )
			{
				float hover_throttle_force = hover_throttle - output_minimum_throttle;
				float max_allowed_output_range = ( hover_throttle_force > output_midpoint ? output_midpoint : hover_throttle_force ) * 0.8f;
				if( max_allowed_output_range < output_throttle - output_minimum_throttle )
					max_allowed_output_range = output_throttle - output_minimum_throttle;
				float max_allowed_throttle = max_allowed_output_range + output_minimum_throttle;
				if( output_max > max_allowed_output_range )
				{
					output_throttle = max_allowed_throttle;
					allow_ouput_range = max_allowed_output_range;
				}
				else
				{
					output_throttle = output_minimum_throttle + output_max;
					allow_ouput_range = output_max;
				}
			}
		}
		
		if( output_max > allow_ouput_range )
		{
			float scale  = allow_ouput_range / output_max;
			rotor_output[0] *= scale;
			rotor_output[1] *= scale;
			rotor_output[2] *= scale;
			rotor_output[3] *= scale;
			ESO_AngularRate_update_u( &ESO[0] , outRoll * scale );
			ESO_AngularRate_update_u( &ESO[1] , outPitch * scale );
		}		
		else
		{
			ESO_AngularRate_update_u( &ESO[0] , outRoll );
			ESO_AngularRate_update_u( &ESO[1] , outPitch );
		}
	/*pitch roll output limit*/
	
	/*yaw output limit*/
		//for Yaw control, it has the lowest priority
		//lower output to ensure attitude control and alt control 
		output_max = 0.0f;
		output_min = 100.0f;

		/*find min yaw_scale*/
			float yaw_scale = 1.0f;
		
			for( int i = 0 ; i < 4 ; ++i )
			{
				float current_out_yaw = ( (i&1) == 1 ) ? outYaw : -outYaw;
				
				float current_rotor_output = output_throttle + rotor_output[i];
				max_allow_output = 100.0f - current_rotor_output;
				min_allow_output = output_minimum_throttle - current_rotor_output;
				if( current_out_yaw > max_allow_output + 0.01f )
				{
					float new_yaw_scale = max_allow_output / current_out_yaw;
					if( new_yaw_scale < yaw_scale ) yaw_scale = new_yaw_scale;
				}
				else if( current_out_yaw < min_allow_output - 0.01f )
				{
					float new_yaw_scale = min_allow_output / current_out_yaw;
					if( new_yaw_scale < yaw_scale ) yaw_scale = new_yaw_scale;
				}
			}
						
		/*find min yaw_scale*/
		
		//lower yaw output to ensure attitude control and alt control
		if( yaw_scale < 0 )
			yaw_scale = 0;
		outYaw *= yaw_scale;
		ESO_AngularRate_update_u( &ESO[2] , outYaw );
	/*yaw output limit*/
		
	update_output_throttle( output_throttle , 1.0f/200 );
	rotor_output[0] += output_throttle-outYaw;
	rotor_output[1] += output_throttle+outYaw;
	rotor_output[2] += output_throttle-outYaw;
	rotor_output[3] += output_throttle+outYaw;
	
	throttle_nonlinear_compensation( rotor_output );
	PWM_Out( rotor_output );
}

//六旋翼X模式
static void ctrl_Attitude_MultiRotor_X6_PWM( float outRoll , float outPitch , float outYaw )
{
	float rotor_output[8] = {0,0,0,0,0,0,0,0};

	float output_minimum_throttle = Cfg_get_MotorStartingThrottle();
	if( get_current_Receiver()->data[0] < 5 )
	{
		PWM_PullDownAll();
		update_output_throttle( 0 , 1.0f/200 );
		return;
	}		
	
	if( throttle < output_minimum_throttle - 0.1f )
	{
		PWM_PullDownAll();
		update_output_throttle( 0 , 1.0f/200 );
		return;
	}		
			
	float output_throttle = throttle;
	float output_midpoint = ( 100.0f - output_minimum_throttle ) / 2;
//		float cos_angle = lean_cos;
//		if( cos_angle < 0.5f )cos_angle = 0.5f;
//		output_throttle = throttle / cos_angle;
	
	/*pitch roll output limit*/
		//if output for balance is out of range
		//at this throttle, lower throttle
		//to insure balance control 			
		float RollS = outRoll * 1.1547005383792515290182975610039f;
		float half_outRoll = 0.5f * RollS;
		rotor_output[0] = -outPitch+half_outRoll;
		rotor_output[1] = RollS;
		rotor_output[2] = +outPitch+half_outRoll;
		rotor_output[3] = +outPitch-half_outRoll;
		rotor_output[4] = -RollS;
		rotor_output[5] = -outPitch-half_outRoll;
	
		float output_max;
		float output_min ;
		output_max = rotor_output[0];
		for( int i = 1 ; i < 6 ; ++i )
		{
			if( rotor_output[i] > output_max ) output_max = rotor_output[i];
		}
		
		float max_allow_output = 100.0f - output_throttle;
		float min_allow_output = output_minimum_throttle - output_throttle;			
		//ensure Pitch Roll output
		//lower throttle if necessary
		//float allow_ouput_range = ( max_allow_output < -min_allow_output ) ? max_allow_output : -min_allow_output;
		float allow_ouput_range;
		if( max_allow_output < -min_allow_output )
		{
			allow_ouput_range = max_allow_output;
			if( output_max > allow_ouput_range )
			{
				if( output_max > output_midpoint )
				{
					output_throttle = output_midpoint + output_minimum_throttle;
					allow_ouput_range = output_midpoint;
				}
				else
				{
					output_throttle = 100.0f - output_max;
					allow_ouput_range = output_max;
				}
			}
		}
		else
		{
			allow_ouput_range = -min_allow_output;
			if( output_max > allow_ouput_range )
			{
				float hover_throttle_force = hover_throttle - output_minimum_throttle;
				float max_allowed_output_range = ( hover_throttle_force > output_midpoint ? output_midpoint : hover_throttle_force ) * 0.8f;
				if( max_allowed_output_range < output_throttle - output_minimum_throttle )
					max_allowed_output_range = output_throttle - output_minimum_throttle;
				float max_allowed_throttle = max_allowed_output_range + output_minimum_throttle;
				if( output_max > max_allowed_output_range )
				{
					output_throttle = max_allowed_throttle;
					allow_ouput_range = max_allowed_output_range;
				}
				else
				{
					output_throttle = output_minimum_throttle + output_max;
					allow_ouput_range = output_max;
				}
			}
		}
		
		if( output_max > allow_ouput_range )
		{
			float scale  = allow_ouput_range / output_max;
			rotor_output[0] *= scale;
			rotor_output[1] *= scale;
			rotor_output[2] *= scale;
			rotor_output[3] *= scale;
			rotor_output[4] *= scale;
			rotor_output[5] *= scale;
			ESO_AngularRate_update_u( &ESO[0] , outRoll * scale );
			ESO_AngularRate_update_u( &ESO[1] , outPitch * scale );
		}		
		else
		{
			ESO_AngularRate_update_u( &ESO[0] , outRoll );
			ESO_AngularRate_update_u( &ESO[1] , outPitch );
		}
	/*pitch roll output limit*/
	
	/*yaw output limit*/
		//for Yaw control, it has the lowest priority
		//lower output to ensure attitude control and alt control 
		output_max = 0.0f;
		output_min = 100.0f;

		/*find min yaw_scale*/
			float yaw_scale = 1.0f;
		
			for( int i = 0 ; i < 6 ; ++i )
			{
				float current_out_yaw = ( (i&1) == 1 ) ? outYaw : -outYaw;
				
				float current_rotor_output = output_throttle + rotor_output[i];
				max_allow_output = 100.0f - current_rotor_output;
				min_allow_output = output_minimum_throttle - current_rotor_output;
				if( current_out_yaw > max_allow_output + 0.01f )
				{
					float new_yaw_scale = max_allow_output / current_out_yaw;
					if( new_yaw_scale < yaw_scale ) yaw_scale = new_yaw_scale;
				}
				else if( current_out_yaw < min_allow_output - 0.01f )
				{
					float new_yaw_scale = min_allow_output / current_out_yaw;
					if( new_yaw_scale < yaw_scale ) yaw_scale = new_yaw_scale;
				}
			}
						
		/*find min yaw_scale*/
		
		//lower yaw output to ensure attitude control and alt control
		if( yaw_scale < 0 )
			yaw_scale = 0;
		outYaw *= yaw_scale;
		ESO_AngularRate_update_u( &ESO[2] , outYaw );
	/*yaw output limit*/
		
	update_output_throttle( output_throttle , 1.0f/200 );
	rotor_output[0] += output_throttle-outYaw;
	rotor_output[1] += output_throttle+outYaw;
	rotor_output[2] += output_throttle-outYaw;
	rotor_output[3] += output_throttle+outYaw;
	rotor_output[4] += output_throttle-outYaw;
	rotor_output[5] += output_throttle+outYaw;
	
	throttle_nonlinear_compensation( rotor_output );
	PWM_Out( rotor_output );
}

//双层四旋翼十字型
static void ctrl_Attitude_MultiRotor_C42_PWM( float outRoll , float outPitch , float outYaw )
{
	float rotor_output[8] = {0,0,0,0,0,0,0,0};

	float output_minimum_throttle = Cfg_get_MotorStartingThrottle();
	if( get_current_Receiver()->data[0] < 5 )
	{
		PWM_PullDownAll();
		update_output_throttle( 0 , 1.0f/200 );
		return;
	}		
	
	if( throttle < output_minimum_throttle - 0.1f )
	{
		PWM_PullDownAll();
		update_output_throttle( 0 , 1.0f/200 );
		return;
	}		
			
	float output_throttle = throttle;
	float output_midpoint = ( 100.0f - output_minimum_throttle ) / 2;
//		float cos_angle = lean_cos;
//		if( cos_angle < 0.5f )cos_angle = 0.5f;
//		output_throttle = throttle / cos_angle;
	
	/*pitch roll output limit*/
		//if output for balance is out of range
		//at this throttle, lower throttle
		//to insure balance control 			
		rotor_output[0] = -outPitch;
		rotor_output[1] = +outRoll;
		rotor_output[2] = +outPitch;
		rotor_output[3] = -outRoll;
		float output_max;
		float output_min ;
		output_max = rotor_output[0];
		for( int i = 1 ; i < 4 ; ++i )
		{
			if( rotor_output[i] > output_max ) output_max = rotor_output[i];
		}
		
		float max_allow_output = 100.0f - output_throttle;
		float min_allow_output = output_minimum_throttle - output_throttle;			
		//ensure Pitch Roll output
		//lower throttle if necessary
		//float allow_ouput_range = ( max_allow_output < -min_allow_output ) ? max_allow_output : -min_allow_output;
		float allow_ouput_range;
		if( max_allow_output < -min_allow_output )
		{
			allow_ouput_range = max_allow_output;
			if( output_max > allow_ouput_range )
			{
				if( output_max > output_midpoint )
				{
					output_throttle = output_midpoint + output_minimum_throttle;
					allow_ouput_range = output_midpoint;
				}
				else
				{
					output_throttle = 100.0f - output_max;
					allow_ouput_range = output_max;
				}
			}
		}
		else
		{
			allow_ouput_range = -min_allow_output;
			if( output_max > allow_ouput_range )
			{
				float hover_throttle_force = hover_throttle - output_minimum_throttle;
				float max_allowed_output_range = ( hover_throttle_force > output_midpoint ? output_midpoint : hover_throttle_force ) * 0.8f;
				if( max_allowed_output_range < output_throttle - output_minimum_throttle )
					max_allowed_output_range = output_throttle - output_minimum_throttle;
				float max_allowed_throttle = max_allowed_output_range + output_minimum_throttle;
				if( output_max > max_allowed_output_range )
				{
					output_throttle = max_allowed_throttle;
					allow_ouput_range = max_allowed_output_range;
				}
				else
				{
					output_throttle = output_minimum_throttle + output_max;
					allow_ouput_range = output_max;
				}
			}
		}
		
		if( output_max > allow_ouput_range )
		{
			float scale  = allow_ouput_range / output_max;
			rotor_output[0] *= scale;
			rotor_output[1] *= scale;
			rotor_output[2] *= scale;
			rotor_output[3] *= scale;
			ESO_AngularRate_update_u( &ESO[0] , outRoll * scale );
			ESO_AngularRate_update_u( &ESO[1] , outPitch * scale );
		}		
		else
		{
			ESO_AngularRate_update_u( &ESO[0] , outRoll );
			ESO_AngularRate_update_u( &ESO[1] , outPitch );
		}
	/*pitch roll output limit*/
	
	/*yaw output limit*/
		//for Yaw control, it has the lowest priority
		//lower output to ensure attitude control and alt control 
		output_max = 0.0f;
		output_min = 100.0f;

		/*find min yaw_scale*/
			float yaw_scale1 = 1.0f;
			float yaw_scale2 = 1.0f;
			for( int i = 0 ; i < 4 ; ++i )
			{
				float current_out_yaw = ( (i&1) == 1 ) ? outYaw : -outYaw;
				
				float current_rotor_output = output_throttle + rotor_output[i];
				max_allow_output = 100.0f - current_rotor_output;
				min_allow_output = output_minimum_throttle - current_rotor_output;
				if( current_out_yaw > max_allow_output + 0.01f )
				{
					float new_yaw_scale = max_allow_output / current_out_yaw;
					if( new_yaw_scale < yaw_scale1 ) yaw_scale1 = new_yaw_scale;
				}
				else if( current_out_yaw < min_allow_output - 0.01f )
				{
					float new_yaw_scale = min_allow_output / current_out_yaw;
					if( new_yaw_scale < yaw_scale1 ) yaw_scale1 = new_yaw_scale;
				}
			}
			for( int i = 0 ; i < 4 ; ++i )
			{
				float current_out_yaw = ( (i&1) == 0 ) ? outYaw : -outYaw;
				
				float current_rotor_output = output_throttle + rotor_output[i];
				max_allow_output = 100.0f - current_rotor_output;
				min_allow_output = output_minimum_throttle - current_rotor_output;
				if( current_out_yaw > max_allow_output + 0.01f )
				{
					float new_yaw_scale = max_allow_output / current_out_yaw;
					if( new_yaw_scale < yaw_scale2 ) yaw_scale2 = new_yaw_scale;
				}
				else if( current_out_yaw < min_allow_output - 0.01f )
				{
					float new_yaw_scale = min_allow_output / current_out_yaw;
					if( new_yaw_scale < yaw_scale2 ) yaw_scale2 = new_yaw_scale;
				}
			}
						
		/*find min yaw_scale*/
		
		//lower yaw output to ensure attitude control and alt control
		if( yaw_scale1 < 0 )
			yaw_scale1 = 0;
		if( yaw_scale2 < 0 )
			yaw_scale2 = 0;
		float outYaw1 = outYaw * yaw_scale1;
		float outYaw2 = outYaw * yaw_scale2;
		ESO_AngularRate_update_u( &ESO[2] , outYaw * 0.5f*(yaw_scale1+yaw_scale2) );
	/*yaw output limit*/
		
	update_output_throttle( output_throttle , 1.0f/200 );
	rotor_output[4] = rotor_output[0];
	rotor_output[5] = rotor_output[1];
	rotor_output[6] = rotor_output[2];
	rotor_output[7] = rotor_output[3];
	rotor_output[0] += output_throttle-outYaw1;
	rotor_output[1] += output_throttle+outYaw1;
	rotor_output[2] += output_throttle-outYaw1;
	rotor_output[3] += output_throttle+outYaw1;
	rotor_output[4] += output_throttle+outYaw2;
	rotor_output[5] += output_throttle-outYaw2;
	rotor_output[6] += output_throttle+outYaw2;
	rotor_output[7] += output_throttle-outYaw2;
	
	throttle_nonlinear_compensation( rotor_output );
	PWM_Out( rotor_output );
}


void init_ctrl_Attitude()
{
	//飞行器类型初始设置
	Cfg_set_initial_UAVType( UAVType_Rotor4_X );
	
	//电机起转油门初始设置
	Cfg_set_initial_MotorStartingThrottle( 10 );
	
	//电机非线性修正因子初始设置
	Cfg_set_initial_MotorNonlineFactor( 0.65f );
	
	//电机满油门比例初始设置
	Cfg_set_initial_MotorFullThrottleRatio( 0.95f );
	
	//电机惯性时间初始设置
	Cfg_set_initial_MotorT( 0.1f );
	
	//电池基准电压初始设置
	Cfg_set_initial_BatSTVoltage( 11.6f );
	
	//电池电压ADC放大倍数初始设置
	Cfg_set_initial_BatVoltageADCMag( 23 );
	
	//Roll初始设置
	Cfg_set_initial_RPYCtrl_b( 0 , 5.5f );
	Cfg_set_initial_RPYCtrl_Pn( 0 , 15.0f , 1 );
	Cfg_set_initial_RPYCtrl_Pn( 0 , 15.0f , 2 );
	Cfg_set_initial_RPYCtrl_Pn( 0 , 15.0f , 3 );
	Cfg_set_initial_RPYCtrl_Pn( 0 , 15.0f , 4 );
	Cfg_set_initial_RPYCtrl_TD4Pn( 0 , 25.0f , 1 );
	Cfg_set_initial_RPYCtrl_TD4Pn( 0 , 25.0f , 2 );
	Cfg_set_initial_RPYCtrl_TD4Pn( 0 , 25.0f , 3 );
	Cfg_set_initial_RPYCtrl_TD4Pn( 0 , 25.0f , 4 );
	
	//Pitch初始设置
	Cfg_set_initial_RPYCtrl_b( 1 , 5.5f );
	Cfg_set_initial_RPYCtrl_Pn( 1 , 15.0f , 1 );
	Cfg_set_initial_RPYCtrl_Pn( 1 , 15.0f , 2 );
	Cfg_set_initial_RPYCtrl_Pn( 1 , 15.0f , 3 );
	Cfg_set_initial_RPYCtrl_Pn( 1 , 15.0f , 4 );
	Cfg_set_initial_RPYCtrl_TD4Pn( 1 , 25.0f , 1 );
	Cfg_set_initial_RPYCtrl_TD4Pn( 1 , 25.0f , 2 );
	Cfg_set_initial_RPYCtrl_TD4Pn( 1 , 25.0f , 3 );
	Cfg_set_initial_RPYCtrl_TD4Pn( 1 , 25.0f , 4 );
	
	//Yaw初始设置
	Cfg_set_initial_RPYCtrl_b( 2 , 1.0f );
	Cfg_set_initial_RPYCtrl_Pn( 2 , 15.0f , 1 );
	Cfg_set_initial_RPYCtrl_Pn( 2 , 15.0f , 2 );
	Cfg_set_initial_RPYCtrl_Pn( 2 , 15.0f , 3 );
	Cfg_set_initial_RPYCtrl_Pn( 2 , 15.0f , 4 );
	Cfg_set_initial_RPYCtrl_TD4Pn( 2 , 25.0f , 1 );
	Cfg_set_initial_RPYCtrl_TD4Pn( 2 , 10.0f , 2 );
	Cfg_set_initial_RPYCtrl_TD4Pn( 2 , 25.0f , 3 );
	Cfg_set_initial_RPYCtrl_TD4Pn( 2 , 25.0f , 4 );

	//初始化扰动滤波器
	Filter_Butter2_LP_float_init( &disturbance_filter[0] , 200 , 10 );
	Filter_Butter2_LP_float_init( &disturbance_filter[1] , 200 , 10 );
	Filter_Butter2_LP_float_init( &disturbance_filter[2] , 200 , 10 );
	
	//初始化高度ESO
	ESO_h_init( &ESO_height , 0.1f , 0.05f );
}