#pragma once

#include <stdbool.h>

/*观测器*/
	//获取悬停油门
	float get_hover_throttle();
	//获取是否在飞行
	bool get_is_inFlight();
	//获取ENU x风力（造成的加速度cm/s^2）
	float get_WindDisturbance_x();
	//获取ENU y风力（造成的加速度cm/s^2）
	float get_WindDisturbance_y();
/*观测器*/

/*姿态控制*/
	typedef enum
	{
		Attitude_ControlMode_Angle ,
		Attitude_ControlMode_AngularRate ,
		Attitude_ControlMode_Locking ,
	}Attitude_ControlMode;
	
	//打开关闭姿态控制器
	bool Attitude_Control_Enable();
	bool Attitude_Control_Disable();

	//获取当前油门
	float get_Target_Throttle();
	//设定油门
	bool Attitude_Control_set_Throttle( float thr );
	//获取目标Roll
	float Attitude_Control_get_Target_Roll();
	//获取目标Pitch
	float Attitude_Control_get_Target_Pitch();
	//获取目标Yaw
	float Attitude_Control_get_Target_Yaw();
	//设定目标Roll Pitch
	bool Attitude_Control_set_Target_RollPitch( float Roll , float Pitch );

	//设定目标Yaw
	bool Attitude_Control_set_Target_Yaw( float Yaw );
	bool Attitude_Control_set_Target_YawRelative( float Yaw );
	//设定目标Yaw速度
	//正值为逆时钟旋转  《BY》
	bool Attitude_Control_set_Target_YawRate( float YawRate );
	//锁定Yaw（刹车后锁角度）
	bool Attitude_Control_set_YawLock();
/*姿态控制*/
	
/*位置控制*/
	typedef enum
	{				
		Position_ControlMode_Velocity = 11 ,	//速度控制模式
		Position_ControlMode_Locking = 10 ,	//刹车后锁位置
		
		Position_ControlMode_RouteLine = 20 ,	//巡线模式
		Position_ControlMode_Position = 21 ,	//位置锁定模式
		Position_ControlMode_Takeoff = 22 ,	//起飞模式
	}Position_ControlMode;
	#define Is_PositionControlMode(x) (x >=20 && x<=29)
	
	//打开关闭高度控制器
	bool Altitude_Control_Enable();
	bool Altitude_Control_Disable();
	
	/*高度*/
		//设定目标高度
	
	//以下两个直接设定飞行高度的函数不要用于起飞操作		《BY》
	//到指定高度，比如100，那就是悬停至100厘米，该值不要出现负数或0（因为会直接砸地板），为安全考虑，该值不可低于50		《BY》
		bool Position_Control_set_TargetPositionZ( float posz );
	
	//上升高度，比如100，那就是在原有位置上升100厘米，-100，那就是在原有位置下降100厘米		《BY》
		bool Position_Control_set_TargetPositionZRelative( float posz );//相对高度
	
		//设定目标垂直速度
	//可以为负数		《BY》
		bool Position_Control_set_TargetVelocityZ( float velz );
	
		//刹车后锁高度
	//可以在进行刹车锁高度操作，结合修该程序引入的三维位置反馈的 Z 值，可进行判断相应高度刹车		《BY》
		bool Position_Control_set_ZLock();
	
		//获取当前高度控制模式
	//主要用于判断动作完成，在每个状态开始后，都要加上该函数进行判断该状态是否已经完成，完成后才可以进入下一个状态		《BY》
		Position_ControlMode get_Altitude_ControlMode();
	
		//起飞到当前高度上方的height高度
	//主要用于起飞		《BY》
		bool Position_Control_Takeoff_HeightRelative( float height );
	/*高度*/
	
	/*水平位置*/
		//打开关闭水平位置控制
		bool get_Position_Control_Enabled();
		bool Position_Control_Enable();
		bool Position_Control_Disable();
		
		//获取当前水平位置控制模式
		Position_ControlMode get_Position_ControlMode();
	
		//设定目标水平位置
		//以起飞坐标为原点，罗盘为参考系，如（100，0），代表起飞坐标向前1米，（0，0）代表回起飞坐标处		《BY》
		bool Position_Control_set_TargetPositionXY( float posx , float posy );
		
		//设定目标水平位置（相对当前坐标）
		//以当前坐标为原点，罗盘为参考系，如（100，0），代表当前坐标向前1米，（0，0）代表不动		《BY》
		bool Position_Control_set_TargetPositionXYRelative( float posx , float posy );
		
		//设定目标水平位置（相对当前坐标且偏移在Bodyheading系下）
		//以当前实时位置为参考系进行移动，两个参数可为负数，如（100，100），代表往机头正左前方前进 100*根号2 的距离		《BY》
		bool Position_Control_set_TargetPositionXYRelativeBodyHeading( float posx , float posy );
		
		//根据经纬度设定目标水平位置
		//这个函数不需要用到，需要结合gps的		《BY》
		bool Position_Control_set_TargetPositionXY_LatLon( double Lat , double Lon );
		
		//设定目标水平速度（Bodyheading朝向）
		//以机头为参考系，结合位置反馈的x和y，可进行定位刹车，注意这里的x和y是以罗盘为参考系		《BY》
		//x正向前，y正向左		《BY》
		bool Position_Control_set_TargetVelocityBodyHeadingXY( float velx , float vely );
		//设定目标水平速度（Bodyheading朝向）并限制最大角度（补偿风力后的角度）
		bool Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( float velx , float vely , float maxRoll , float maxPitch );
		//刹车后锁定水平位置 
		bool Position_Control_set_XYLock();
	/*水平位置*/
/*位置控制*/