#include "Modes.h"
#include "Basic.h"
#include "M13_MagCalib.h"
#include <stdlib.h>

#include "MeasurementSystem.h"
#include "InteractiveInterface.h"
#include "Sensors.h"
#include "AC_Math.h"
#include "Configurations.h"

//磁力计校准
//最小二乘椭球拟合
//朱文杰 20181226
//请勿用于商业用途
//！！抄袭必究！！
/*
	Ellipsoid equation :
		A( x - a )^2 + B( y - b )^2 + ( c - z )^2 - r = 0;

	es = [ a ]
			 [ b ]
			 [ c ]
			 [ A ]
			 [ B ]
			 [ r ]
*/

static void M13_MagCalib_MainFunc();
static void M13_MagCalib_enter();
static void M13_MagCalib_exit();
const Mode M13_MagCalib = 
{
	100 , //mode frequency
	M13_MagCalib_enter , //enter
	M13_MagCalib_exit ,	//exit
	M13_MagCalib_MainFunc ,	//mode main func
};



typedef struct
{
	//牛顿迭代法 雅可比矩阵
	float dF_matrix[ 6 * 6 ];
	float F_matrix[ 6 * 1 ];
	float dx_matrix[ 6 * 1 ];

	//校准结果
	float es[ 6 * 1 ];
	
	//最小二乘的累加和
	float sum_x[ IMU_Sensors_Count ];
	float sum_x2[ IMU_Sensors_Count ];
	float sum_x3[ IMU_Sensors_Count ];
	float sum_x4[ IMU_Sensors_Count ];
	float sum_y[ IMU_Sensors_Count ];
	float sum_y2[ IMU_Sensors_Count ];
	float sum_y3[ IMU_Sensors_Count ];
	float sum_y4[ IMU_Sensors_Count ];
	float sum_z[ IMU_Sensors_Count ];
	float sum_z2[ IMU_Sensors_Count ];
	float sum_z3[ IMU_Sensors_Count ];
	float sum_xy[ IMU_Sensors_Count ];
	float sum_x2y[ IMU_Sensors_Count ];
	float sum_xy2[ IMU_Sensors_Count ];
	float sum_x2y2[ IMU_Sensors_Count ];
	float sum_xz[ IMU_Sensors_Count ];
	float sum_x2z[ IMU_Sensors_Count ];
	float sum_xz2[ IMU_Sensors_Count ];
	float sum_x2z2[ IMU_Sensors_Count ];
	float sum_yz[ IMU_Sensors_Count ];
	float sum_y2z[ IMU_Sensors_Count ];
	float sum_yz2[ IMU_Sensors_Count ];
	float sum_y2z2[ IMU_Sensors_Count ];
	float n;
	
	//校准状态机
	unsigned char Calibration_Step;
	unsigned int Calibration_Step2;
	unsigned char current_Calibrate_Sensor;
	TIME rotate_start_time;
	
	//记录最大最小磁力用于判断校准是否完成
	signed int max_x , min_x , max_y , min_y , max_z , min_z;
}MODE_INF;
static MODE_INF* Mode_Inf;

static inline void reset_sums()
{
	for( unsigned char i = 0 ; i < IMU_Sensors_Count ; ++i )
	{
		Mode_Inf->sum_x[i] = Mode_Inf->sum_x2[i] = Mode_Inf->sum_x3[i] = Mode_Inf->sum_x4[i] = 0;
		Mode_Inf->sum_y[i] = Mode_Inf->sum_y2[i] = Mode_Inf->sum_y3[i] = Mode_Inf->sum_y4[i] = 0;
		Mode_Inf->sum_z[i] = Mode_Inf->sum_z2[i] = Mode_Inf->sum_z3[i] = 0;
		Mode_Inf->sum_xy[i] = Mode_Inf->sum_x2y[i] = Mode_Inf->sum_xy2[i] = Mode_Inf->sum_x2y2[i] = 0;
		Mode_Inf->sum_xz[i] = Mode_Inf->sum_x2z[i] = Mode_Inf->sum_xz2[i] = Mode_Inf->sum_x2z2[i] = 0;
		Mode_Inf->sum_yz[i] = Mode_Inf->sum_y2z[i] = Mode_Inf->sum_yz2[i] = Mode_Inf->sum_y2z2[i] = 0;
		Mode_Inf->n = 0;
	}
}

static void M13_MagCalib_enter()
{
	//设置状态灯
	Led_setProgress(0);
	
	//初始化模式变量
	Mode_Inf = malloc( sizeof( MODE_INF ) );
	Mode_Inf->Calibration_Step = 0;	
	Mode_Inf->max_x = Mode_Inf->min_x = Mode_Inf->max_y = Mode_Inf->min_y = Mode_Inf->max_z = Mode_Inf->min_z = 0;
	Time_set_inValid( &Mode_Inf->rotate_start_time );
	reset_sums();
}

static void M13_MagCalib_exit()
{
	free( Mode_Inf );
}

static inline void get_F_dF( const unsigned char index );
static void M13_MagCalib_MainFunc()
{
	switch( Mode_Inf->Calibration_Step )
	{
		
		case 0:	//计算累加和（旋转机体）
		{
			float rotate_speed = vector3_float_square( get_AngularRateCtrl() );
			arm_sqrt_f32( rotate_speed , &rotate_speed );
			if( rotate_speed > 0.6f )
			{
				if( Time_isValid( Mode_Inf->rotate_start_time ) == false )
					Mode_Inf->rotate_start_time = get_TIME_now();
			}
			else
				Time_set_inValid( &Mode_Inf->rotate_start_time );
			if( Time_isValid( Mode_Inf->rotate_start_time ) && get_pass_time( Mode_Inf->rotate_start_time ) > 0.2f )
			{
				//update min and max mag to check if calibration data is ready
				const IMU_Sensor* internal_magnetometer = GetMagnetometer(Internal_Magnetometer_Index);
				if( internal_magnetometer->data_raw.x > Mode_Inf->max_x )
					Mode_Inf->max_x = internal_magnetometer->data_raw.x;
				else if( internal_magnetometer->data_raw.x < Mode_Inf->min_x )
					Mode_Inf->min_x = internal_magnetometer->data_raw.x;
				if( internal_magnetometer->data_raw.y > Mode_Inf->max_y )
					Mode_Inf->max_y = internal_magnetometer->data_raw.y;
				else if( internal_magnetometer->data_raw.y < Mode_Inf->min_y )
					Mode_Inf->min_y = internal_magnetometer->data_raw.y;
				if( internal_magnetometer->data_raw.z > Mode_Inf->max_z )
					Mode_Inf->max_z = internal_magnetometer->data_raw.z;
				else if( internal_magnetometer->data_raw.z < Mode_Inf->min_z )
					Mode_Inf->min_z = internal_magnetometer->data_raw.z;
				
				/*update sums*/
					for( unsigned char i = 0 ; i < IMU_Sensors_Count ; ++i )
					{
						const IMU_Sensor* sensor = GetMagnetometer(i);
						if( sensor->present )
						{
							float x = sensor->data_raw.x;	float y = sensor->data_raw.y;	float z = sensor->data_raw.z;
							float x2 = x * x;	float x3 = x2 * x; float x4 = x3 * x;
							float y2 = y * y;	float y3 = y2 * y; float y4 = y3 * y;
							float z2 = z * z;	float z3 = z2 * z;
							float xy = x * y;	float x2y = x * xy;	float xy2 = xy * y;	float x2y2 = x * xy2;	
							float xz = x * z;	float x2z = x * xz;	float xz2 = xz * z;	float x2z2 = x * xz2;	
							float yz = y * z;	float y2z = y * yz;	float yz2 = yz * z;	float y2z2 = y * yz2;	
						
							Mode_Inf->sum_x[i] += x;	Mode_Inf->sum_x2[i] += x2;	Mode_Inf->sum_x3[i] += x3;	Mode_Inf->sum_x4[i] += x4;
							Mode_Inf->sum_y[i] += y;	Mode_Inf->sum_y2[i] += y2;	Mode_Inf->sum_y3[i] += y3;	Mode_Inf->sum_y4[i] += y4;
							Mode_Inf->sum_z[i] += z;	Mode_Inf->sum_z2[i] += z2;	Mode_Inf->sum_z3[i] += z3;
							Mode_Inf->sum_xy[i] += xy;	Mode_Inf->sum_x2y[i] += x2y;	Mode_Inf->sum_xy2[i] += xy2;	Mode_Inf->sum_x2y2[i] += x2y2;
							Mode_Inf->sum_xz[i] += xz;	Mode_Inf->sum_x2z[i] += x2z;	Mode_Inf->sum_xz2[i] += xz2;	Mode_Inf->sum_x2z2[i] += x2z2;
							Mode_Inf->sum_yz[i] += yz;	Mode_Inf->sum_y2z[i] += y2z;	Mode_Inf->sum_yz2[i] += yz2;	Mode_Inf->sum_y2z2[i] += y2z2;						
						}
					}
					++Mode_Inf->n;
				/*update sums*/
					
				Led_setProgress( (Mode_Inf->n>2000) ? 100 : Mode_Inf->n / 20 );
				float calibrate_complete_condition = 0.4f / internal_magnetometer->sensitivity;
				if( 
					(Mode_Inf->max_x - Mode_Inf->min_x > calibrate_complete_condition) && \
					(Mode_Inf->max_y - Mode_Inf->min_y > calibrate_complete_condition) && \
					(Mode_Inf->max_z - Mode_Inf->min_z > calibrate_complete_condition) && \
					(Mode_Inf->n > 2000) 
				)
				{
					Mode_Inf->Calibration_Step = 1;
					Mode_Inf->Calibration_Step2 = 0;
					Mode_Inf->current_Calibrate_Sensor = 0;
					Led_setStatus( LED_status_running2 );
					Led_setSignal( LED_signal_continue );
				}
				else if( Mode_Inf->n > 5000 )
					reset_sums();
			}
			else
				Led_setProgress(0);
			
			break;
		}
		
		case 1:	//计算校准结果
		{
			const IMU_Sensor* sensor = GetMagnetometer(Mode_Inf->current_Calibrate_Sensor);
			if( Mode_Inf->Calibration_Step2 == 0 )
			{
				if( sensor->present == false )
				{
					++Mode_Inf->current_Calibrate_Sensor;
					if( Mode_Inf->current_Calibrate_Sensor >= IMU_Sensors_Count )
						change_Mode( 01 );
					return;
				}
				//初始化校准
				Mode_Inf->es[0] = Mode_Inf->es[1] = Mode_Inf->es[2] = 0;
				Mode_Inf->es[3] = Mode_Inf->es[4] = 1;	Mode_Inf->es[5] = 0.5f / sensor->sensitivity; Mode_Inf->es[5] *= Mode_Inf->es[5];
			}
			else
			{
				get_F_dF( Mode_Inf->current_Calibrate_Sensor );			
				
				bool status;
				status = Matrix_Inverse( Mode_Inf->dF_matrix , 6 );
				Mode_Inf->dx_matrix[0] = Mode_Inf->dF_matrix[0]*Mode_Inf->F_matrix[0] + Mode_Inf->dF_matrix[1]*Mode_Inf->F_matrix[1] + Mode_Inf->dF_matrix[2]*Mode_Inf->F_matrix[2] + Mode_Inf->dF_matrix[3]*Mode_Inf->F_matrix[3] + Mode_Inf->dF_matrix[4]*Mode_Inf->F_matrix[4] + Mode_Inf->dF_matrix[5]*Mode_Inf->F_matrix[5];
				Mode_Inf->dx_matrix[1] = Mode_Inf->dF_matrix[6]*Mode_Inf->F_matrix[0] + Mode_Inf->dF_matrix[7]*Mode_Inf->F_matrix[1] + Mode_Inf->dF_matrix[8]*Mode_Inf->F_matrix[2] + Mode_Inf->dF_matrix[9]*Mode_Inf->F_matrix[3] + Mode_Inf->dF_matrix[10]*Mode_Inf->F_matrix[4] + Mode_Inf->dF_matrix[11]*Mode_Inf->F_matrix[5];
				Mode_Inf->dx_matrix[2] = Mode_Inf->dF_matrix[12]*Mode_Inf->F_matrix[0] + Mode_Inf->dF_matrix[13]*Mode_Inf->F_matrix[1] + Mode_Inf->dF_matrix[14]*Mode_Inf->F_matrix[2] + Mode_Inf->dF_matrix[15]*Mode_Inf->F_matrix[3] + Mode_Inf->dF_matrix[16]*Mode_Inf->F_matrix[4] + Mode_Inf->dF_matrix[17]*Mode_Inf->F_matrix[5];
				Mode_Inf->dx_matrix[3] = Mode_Inf->dF_matrix[18]*Mode_Inf->F_matrix[0] + Mode_Inf->dF_matrix[19]*Mode_Inf->F_matrix[1] + Mode_Inf->dF_matrix[20]*Mode_Inf->F_matrix[2] + Mode_Inf->dF_matrix[21]*Mode_Inf->F_matrix[3] + Mode_Inf->dF_matrix[22]*Mode_Inf->F_matrix[4] + Mode_Inf->dF_matrix[23]*Mode_Inf->F_matrix[5];
				Mode_Inf->dx_matrix[4] = Mode_Inf->dF_matrix[24]*Mode_Inf->F_matrix[0] + Mode_Inf->dF_matrix[25]*Mode_Inf->F_matrix[1] + Mode_Inf->dF_matrix[26]*Mode_Inf->F_matrix[2] + Mode_Inf->dF_matrix[27]*Mode_Inf->F_matrix[3] + Mode_Inf->dF_matrix[28]*Mode_Inf->F_matrix[4] + Mode_Inf->dF_matrix[29]*Mode_Inf->F_matrix[5];
				Mode_Inf->dx_matrix[5] = Mode_Inf->dF_matrix[30]*Mode_Inf->F_matrix[0] + Mode_Inf->dF_matrix[31]*Mode_Inf->F_matrix[1] + Mode_Inf->dF_matrix[32]*Mode_Inf->F_matrix[2] + Mode_Inf->dF_matrix[33]*Mode_Inf->F_matrix[3] + Mode_Inf->dF_matrix[34]*Mode_Inf->F_matrix[4] + Mode_Inf->dF_matrix[35]*Mode_Inf->F_matrix[5];
				
				Mode_Inf->es[0] -= Mode_Inf->dx_matrix[0];
				Mode_Inf->es[1] -= Mode_Inf->dx_matrix[1];
				Mode_Inf->es[2] -= Mode_Inf->dx_matrix[2];
				Mode_Inf->es[3] -= Mode_Inf->dx_matrix[3];
				Mode_Inf->es[4] -= Mode_Inf->dx_matrix[4];
				Mode_Inf->es[5] -= Mode_Inf->dx_matrix[5];
				
				if( Mode_Inf->Calibration_Step2 > 200 && ( Mode_Inf->dx_matrix[0]*Mode_Inf->dx_matrix[0] + Mode_Inf->dx_matrix[1]*Mode_Inf->dx_matrix[1] + Mode_Inf->dx_matrix[2]*Mode_Inf->dx_matrix[2] + Mode_Inf->dx_matrix[3]*Mode_Inf->dx_matrix[3] + Mode_Inf->dx_matrix[4]*Mode_Inf->dx_matrix[4] ) < 0.1f )
				{
					const IMU_Sensor* sensor = GetMagnetometer(Mode_Inf->current_Calibrate_Sensor);
					
					arm_sqrt_f32( Mode_Inf->es[3] , &Mode_Inf->es[3] );
					arm_sqrt_f32( Mode_Inf->es[4] , &Mode_Inf->es[4] );
					arm_sqrt_f32( Mode_Inf->es[5] , &Mode_Inf->es[5] );
					vector3_float magnetic_force = { Mode_Inf->es[5] / Mode_Inf->es[3] , Mode_Inf->es[5] / Mode_Inf->es[4] , Mode_Inf->es[5] };
					vector3_float offset = { Mode_Inf->es[0] , Mode_Inf->es[1] , Mode_Inf->es[2] };
					Cfg_update_MagnetometerOffset( Mode_Inf->current_Calibrate_Sensor , offset );
					float standard_magnetic_force = 0.6f / sensor->sensitivity;
					vector3_float scale = { standard_magnetic_force / magnetic_force.x , \
																	standard_magnetic_force / magnetic_force.y , \
																	standard_magnetic_force / magnetic_force.z };
					Cfg_update_MagnetometerSensivitity( Mode_Inf->current_Calibrate_Sensor , scale );					
					Mode_Inf->Calibration_Step2 = 0;
					++Mode_Inf->current_Calibrate_Sensor;
					if( Mode_Inf->current_Calibrate_Sensor >= IMU_Sensors_Count )
					{
						Led_setSignal( LED_signal_success );
						change_Mode( 01 );
					}
					return;
				}
				
			}
			++Mode_Inf->Calibration_Step2;
			
			break;
		}
		
	}
}

static inline void get_F_dF( const unsigned char index )
{
	float Aa = Mode_Inf->es[3] * Mode_Inf->es[0];	float Bb = Mode_Inf->es[4] * Mode_Inf->es[1];
	float Aa2 = Aa*Mode_Inf->es[0];	float Bb2 = Bb*Mode_Inf->es[1];
	float Aa_2 = Aa * 2;	float Bb_2 = Bb * 2;
	float c2 = Mode_Inf->es[2]*Mode_Inf->es[2];	float c_2 = Mode_Inf->es[2] * 2;
	float A_2 = Mode_Inf->es[3] * 2;	float B_2 = Mode_Inf->es[4] * 2;
	float a2 = Mode_Inf->es[0]*Mode_Inf->es[0]; float a_2 = 2 * Mode_Inf->es[0];
	float b2 = Mode_Inf->es[1]*Mode_Inf->es[1];	float b_2 = 2 * Mode_Inf->es[1];
	 
	float temp1 = Aa2 + Bb2 + c2 - Mode_Inf->es[5]; 
	Mode_Inf->F_matrix[0] = Mode_Inf->es[3]*Mode_Inf->sum_x3[index]  - Aa_2*Mode_Inf->sum_x2[index] + Mode_Inf->es[4]*Mode_Inf->sum_xy2[index] - Bb_2*Mode_Inf->sum_xy[index] + Mode_Inf->sum_xz2[index] - c_2*Mode_Inf->sum_xz[index] + temp1*Mode_Inf->sum_x[index];
	Mode_Inf->F_matrix[1] = Mode_Inf->es[3]*Mode_Inf->sum_x2y[index] - Aa_2*Mode_Inf->sum_xy[index] + Mode_Inf->es[4]*Mode_Inf->sum_y3[index]  - Bb_2*Mode_Inf->sum_y2[index] + Mode_Inf->sum_yz2[index] - c_2*Mode_Inf->sum_yz[index] + temp1*Mode_Inf->sum_y[index];
	Mode_Inf->F_matrix[2] = Mode_Inf->es[3]*Mode_Inf->sum_x2z[index] - Aa_2*Mode_Inf->sum_xz[index] + Mode_Inf->es[4]*Mode_Inf->sum_y2z[index] - Bb_2*Mode_Inf->sum_yz[index] + Mode_Inf->sum_z3[index]  - c_2*Mode_Inf->sum_z2[index] + temp1*Mode_Inf->sum_z[index];
	Mode_Inf->F_matrix[3] = Mode_Inf->es[3]*Mode_Inf->sum_x4[index]   - Aa_2*Mode_Inf->sum_x3[index]  + Mode_Inf->es[4]*Mode_Inf->sum_x2y2[index] - Bb_2*Mode_Inf->sum_x2y[index] + Mode_Inf->sum_x2z2[index] - c_2*Mode_Inf->sum_x2z[index] + temp1*Mode_Inf->sum_x2[index];
	Mode_Inf->F_matrix[4] = Mode_Inf->es[3]*Mode_Inf->sum_x2y2[index] - Aa_2*Mode_Inf->sum_xy2[index] + Mode_Inf->es[4]*Mode_Inf->sum_y4[index]   - Bb_2*Mode_Inf->sum_y3[index]  + Mode_Inf->sum_y2z2[index] - c_2*Mode_Inf->sum_y2z[index] + temp1*Mode_Inf->sum_y2[index];
	Mode_Inf->F_matrix[5] = - Aa_2*Mode_Inf->sum_x[index] - Bb_2*Mode_Inf->sum_y[index] - c_2*Mode_Inf->sum_z[index] + Mode_Inf->es[3]*Mode_Inf->sum_x2[index] + Mode_Inf->es[4]*Mode_Inf->sum_y2[index] + Mode_Inf->sum_z2[index] + temp1*Mode_Inf->n;
	
	Mode_Inf->dF_matrix[0] = Aa_2*Mode_Inf->sum_x[index] - A_2*Mode_Inf->sum_x2[index];    Mode_Inf->dF_matrix[1] = Bb_2*Mode_Inf->sum_x[index] - B_2*Mode_Inf->sum_xy[index];    Mode_Inf->dF_matrix[2] = 2 * ( Mode_Inf->es[2]*Mode_Inf->sum_x[index] - Mode_Inf->sum_xz[index] );	  Mode_Inf->dF_matrix[3] = a2*Mode_Inf->sum_x[index] - a_2*Mode_Inf->sum_x2[index] + Mode_Inf->sum_x3[index];      Mode_Inf->dF_matrix[4] = b2*Mode_Inf->sum_x[index] - b_2*Mode_Inf->sum_xy[index] + Mode_Inf->sum_xy2[index];     Mode_Inf->dF_matrix[5]  = -Mode_Inf->sum_x[index];
	Mode_Inf->dF_matrix[6] = Aa_2*Mode_Inf->sum_y[index] - A_2*Mode_Inf->sum_xy[index];    Mode_Inf->dF_matrix[7] = Bb_2*Mode_Inf->sum_y[index] - B_2*Mode_Inf->sum_y2[index];    Mode_Inf->dF_matrix[8] = 2 * ( Mode_Inf->es[2]*Mode_Inf->sum_y[index] - Mode_Inf->sum_yz[index] );    Mode_Inf->dF_matrix[9] = a2*Mode_Inf->sum_y[index] - a_2*Mode_Inf->sum_xy[index] + Mode_Inf->sum_x2y[index];     Mode_Inf->dF_matrix[10] = b2*Mode_Inf->sum_y[index] - b_2*Mode_Inf->sum_y2[index] + Mode_Inf->sum_y3[index];     Mode_Inf->dF_matrix[11] = -Mode_Inf->sum_y[index];
	Mode_Inf->dF_matrix[12] = Aa_2*Mode_Inf->sum_z[index] - A_2*Mode_Inf->sum_xz[index];   Mode_Inf->dF_matrix[13] = Bb_2*Mode_Inf->sum_z[index] - B_2*Mode_Inf->sum_yz[index];   Mode_Inf->dF_matrix[14] = 2 * ( Mode_Inf->es[2]*Mode_Inf->sum_z[index] - Mode_Inf->sum_z2[index] );   Mode_Inf->dF_matrix[15] = a2*Mode_Inf->sum_z[index] - a_2*Mode_Inf->sum_xz[index] + Mode_Inf->sum_x2z[index];    Mode_Inf->dF_matrix[16] = b2*Mode_Inf->sum_z[index] - b_2*Mode_Inf->sum_yz[index] + Mode_Inf->sum_y2z[index];    Mode_Inf->dF_matrix[17] = -Mode_Inf->sum_z[index];
	Mode_Inf->dF_matrix[18] = Aa_2*Mode_Inf->sum_x2[index] - A_2*Mode_Inf->sum_x3[index];  Mode_Inf->dF_matrix[19] = Bb_2*Mode_Inf->sum_x2[index] - B_2*Mode_Inf->sum_x2y[index]; Mode_Inf->dF_matrix[20] = 2 * ( Mode_Inf->es[2]*Mode_Inf->sum_x2[index] - Mode_Inf->sum_x2z[index] ); Mode_Inf->dF_matrix[21] = a2*Mode_Inf->sum_x2[index] - a_2*Mode_Inf->sum_x3[index] + Mode_Inf->sum_x4[index];    Mode_Inf->dF_matrix[22] = b2*Mode_Inf->sum_x2[index] - b_2*Mode_Inf->sum_x2y[index] + Mode_Inf->sum_x2y2[index]; Mode_Inf->dF_matrix[23] = -Mode_Inf->sum_x2[index];
	Mode_Inf->dF_matrix[24] = Aa_2*Mode_Inf->sum_y2[index] - A_2*Mode_Inf->sum_xy2[index]; Mode_Inf->dF_matrix[25] = Bb_2*Mode_Inf->sum_y2[index] - B_2*Mode_Inf->sum_y3[index];  Mode_Inf->dF_matrix[26] = 2 * ( Mode_Inf->es[2]*Mode_Inf->sum_y2[index] - Mode_Inf->sum_y2z[index] ); Mode_Inf->dF_matrix[27] = a2*Mode_Inf->sum_y2[index] - a_2*Mode_Inf->sum_xy2[index] + Mode_Inf->sum_x2y2[index]; Mode_Inf->dF_matrix[28] = b2*Mode_Inf->sum_y2[index] - b_2*Mode_Inf->sum_y3[index] + Mode_Inf->sum_y4[index];    Mode_Inf->dF_matrix[29] = -Mode_Inf->sum_y2[index];
	Mode_Inf->dF_matrix[30] = Aa_2*Mode_Inf->n - A_2*Mode_Inf->sum_x[index];               Mode_Inf->dF_matrix[31] = Bb_2*Mode_Inf->n - B_2*Mode_Inf->sum_y[index];               Mode_Inf->dF_matrix[32] = 2 * ( Mode_Inf->es[2]*Mode_Inf->n - Mode_Inf->sum_z[index] );               Mode_Inf->dF_matrix[33] = a2*Mode_Inf->n - a_2*Mode_Inf->sum_x[index] + Mode_Inf->sum_x2[index];                 Mode_Inf->dF_matrix[34] = b2*Mode_Inf->n - b_2*Mode_Inf->sum_y[index] + Mode_Inf->sum_y2[index];                 Mode_Inf->dF_matrix[35] = -Mode_Inf->n;
}