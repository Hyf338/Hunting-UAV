#pragma once

void init_drv_Sensors();

typedef enum 
{
	i2c_sensor_null,
	
	i2c_sensor_icm20602,
	i2c_sensor_spl06,
	i2c_sensor_ist8310,
}i2c_sensor;

typedef struct
{
	signed short acc[3];
	signed short temp;
	signed short gyro[3];
}MPU_DATA;
MPU_DATA get_mpu_data(void);

typedef struct
{
	unsigned int temperature_raw;
	unsigned int pressure_raw;
}SPL06_RAW_DATA;
SPL06_RAW_DATA get_spl06_raw_data(void);

typedef struct
{
	signed short data[3];
}IST8310_DATA;
IST8310_DATA get_ist8310_data(void);