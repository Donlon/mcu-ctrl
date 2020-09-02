#ifndef __MPU6050_H
#define	__MPU6050_H
#include "main.h"

/* MPU6050 Register Address ------------------------------------------------------------*/
//Ӳ��ID 0x75 ֵ��113
#define	SMPLRT_DIV		 0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG				 0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		 0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	 0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
#define	ACCEL_CONFIG_2 0x1D 


#define	ACCEL_XOUT  	0x3B	//���ٶȼ��������
#define	ACCEL_XOUT_H	0x3B	
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT  	0x3D	
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT  	0x3F	
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	ACCEL_OFFS_XOUT  	0x77	
#define	ACCEL_OFFS_XOUT_H	0x77
#define	ACCEL_OFFS_XOUT_L	0x78
#define	ACCEL_OFFS_YOUT  	0x7A	
#define	ACCEL_OFFS_YOUT_H	0x7A
#define	ACCEL_OFFS_YOUT_L	0x7B
#define	ACCEL_OFFS_ZOUT  	0x7D	
#define	ACCEL_OFFS_ZOUT_H	0x7D
#define	ACCEL_OFFS_ZOUT_L	0x7E


#define	TEMP_OUT   		0x41	//�¶ȼƼ��������
#define	TEMP_OUT_H		0x41	//�¶ȼƼ��������
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT  	  0x43	
#define	GYRO_XOUT_H		0x43	//�������������
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT    	0x45
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT   	0x47	
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define	GYRO_OFFS_XOUT  	  0x13	
#define	GYRO_OFFS_XOUT_H		0x13	//�������������
#define	GYRO_OFFS_XOUT_L		0x14	
#define	GYRO_OFFS_YOUT    	0x15
#define	GYRO_OFFS_YOUT_H		0x15
#define	GYRO_OFFS_YOUT_L		0x16
#define	GYRO_OFFS_ZOUT   	  0x17	
#define	GYRO_OFFS_ZOUT_H		0x17
#define	GYRO_OFFS_ZOUT_L		0x18

#define	PWR_MGMT_1		0x6B	//��Դ��������ֵ��0x00(��������)
#define	PWR_MGMT_2		0x6C	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I			0x75	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)Ӳ��ID(�Ĵ���ֵ113)
#define	INT_PIN_CFG   0x37 


#define	MPU6050_ADDRESS	     0xD0	//�ӻ���ַ
#define I2C_MST_CTRL         0x24
#define USER_CTRL            0x6A
#define I2C_MST_DELAY_CTRL   0x67

//--------------------i2c slv0-------------------------------//
#define I2C_SLV0_ADDR                       0x25  
#define I2C_SLV0_REG                        0x26
#define I2C_SLV0_CTRL                       0x27 
#define I2C_SLV0_DO                         0x63 //output reg


#define ACCEL_FULL_RANGE_2G                  0x0
#define ACCEL_FULL_RANGE_4G                  0x1
#define ACCEL_FULL_RANGE_8G                  0x2
#define ACCEL_FULL_RANGE_16G                 0x3
#define ACCEL_FULL_RANGE                     ACCEL_FULL_RANGE_4G

#define GYRO_FULL_RANGE_250                  0x0
#define GYRO_FULL_RANGE_500                  0x1
#define GYRO_FULL_RANGE_1000                 0x2
#define GYRO_FULL_RANGE_2000                 0x3
#define GYRO_FULL_RANGE                      GYRO_FULL_RANGE_500

#define GYRO_CONFIG_DATA                     GYRO_FULL_RANGE<<3 | 0x00 
#define ACCEL_CONFIG_DATA                    ACCEL_FULL_RANGE<<3 | 0x00 
#define PI                                   3.1415926535

//#define DeltaTime                            0.01              // 10ms 
#define DeltaTime                            0.005              // 5ms 

typedef struct {
	int16_t accel_x ;
	int16_t accel_y ;
	int16_t accel_z ;
	int16_t gyro_x ;
	int16_t gyro_y ;
	int16_t gyro_z ;
	
//	int16_t gyro_offs_x ;
//	int16_t gyro_offs_y ;
//	int16_t gyro_offs_z ;
//	int16_t accel_offs_x ;
//	int16_t accel_offs_y ;
//	int16_t accel_offs_z ;

	int16_t   accel_factor ;
	float   gyro_factor ;
		
	int16_t temperature ;
	int8_t  dev_id ;

	int16_t accel_scale_x ;
	int16_t accel_scale_y ;
	int16_t accel_scale_z ;

	float gyro_scale_x ;
	float gyro_scale_y ;
	float gyro_scale_z ;

//	float accel_real_x ;
//	float accel_real_y ;
//	float accel_real_z ;

//  float accel_real_offs_x ;
//	float accel_real_offs_y ;
//	float accel_real_offs_z ;
//	float gyro_real_offs_x ;
//	float gyro_real_offs_y ;
//	float gyro_real_offs_z ;

//	float gyro_real_x ;
//	float gyro_real_y ;
//	float gyro_real_z ;

	float temperature_real ;

	float angle_x ;
	float angle_y ;
	float angle_z ;
	
	float Angle_Kalman ;
	float Gyro_Kalman ;
	float Angle_Complement_1st ;
	float Angle_Complement_2nd ;
} mpu6050_t ;

void MPU6050_Init(void);
HAL_StatusTypeDef MPU6050_Mem_Write( uint8_t addr , uint8_t data );
HAL_StatusTypeDef MPU6050_Mem_Read( uint8_t addr , uint8_t *data );
HAL_StatusTypeDef MPU6050_I2C_Write( uint8_t i2c_addr , uint8_t reg_addr , uint8_t data );
int16_t MPU6050_Data_Read(unsigned char reg_addr );
void MPU6050_Get_Accel_Gyro_Temp( void );
void MPU6050_Data_Process( void );


#endif 


