#include <math.h>
#include <string.h>

#include "main.h"
#include "i2c.h"
#include "common.h"
#include "mpu6050.h"

/*
 * 函数名：void InitMPU6050(void)
 * 描述  ：初始化Mpu6050
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */

I2C_HandleTypeDef *g_MPU6050_i2c = &hi2c2;
mpu6050_t g_mpu6050;

HAL_StatusTypeDef MPU6050_Mem_Write(uint8_t addr, uint8_t data) {
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Mem_Write(g_MPU6050_i2c, MPU6050_ADDRESS, addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *) &data, 1, 0xFFFF);
    return ret;
}

HAL_StatusTypeDef MPU6050_Mem_Read(uint8_t addr, uint8_t *data) {
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Mem_Read(g_MPU6050_i2c, MPU6050_ADDRESS, addr, I2C_MEMADD_SIZE_8BIT, data, 1, 0xFFFF);
    return ret;
}

int16_t MPU6050_Data_Read(uint8_t reg_addr) {
    uint8_t H, L;

    MPU6050_Mem_Read(reg_addr, (uint8_t *) &H);
    MPU6050_Mem_Read(reg_addr + 1, (uint8_t *) &L);

    return (H << 8u) + L;   //合成数据
}

void MPU6050_Init(void) {
    memset(&g_mpu6050, 0, sizeof(mpu6050_t));

    /***********************MPU6050_SET********************************************/
    MPU6050_Mem_Write(PWR_MGMT_1, 0x00);
    MPU6050_Mem_Write(CONFIG, 0x02);
    MPU6050_Mem_Write(USER_CTRL, 0x00);
    MPU6050_Mem_Write(SMPLRT_DIV, 0x07);
    MPU6050_Mem_Write(GYRO_CONFIG, GYRO_CONFIG_DATA);
    MPU6050_Mem_Write(ACCEL_CONFIG, ACCEL_CONFIG_DATA);

    /***********************MAG_SET********************************************/
    // MPU6050_I2C_Write(MPU6050_AK8963_ADDR, AK8963_CNTL2_REG, AK8963_CNTL2_SRST);
    // MPU6050_I2C_Write(MPU6050_AK8963_ADDR, AK8963_CNTL1_REG, 0x12);

    /***************************BMP280_SET*************************************/
    // MPU6050_I2C_Write(BMP280_ADDR, 0xF4, 0x25);

    MPU6050_Mem_Read(WHO_AM_I, (uint8_t *) &g_mpu6050.dev_id);

    int range = ACCEL_FULL_RANGE;
    switch (range) {
        case ACCEL_FULL_RANGE_2G :
            // g_mpu6050.accel_factor = 16384 ;
            g_mpu6050.accel_factor = 14 - 10;   // 2^14
            break;
        case ACCEL_FULL_RANGE_4G :
            // g_mpu6050.accel_factor = 8192 ;
            g_mpu6050.accel_factor = 13 - 10;   // 2^13
            break;
        case ACCEL_FULL_RANGE_8G :
            // g_mpu6050.accel_factor = 4096 ;
            g_mpu6050.accel_factor = 12 - 10;   // 2^12
            break;
        case ACCEL_FULL_RANGE_16G :
            // g_mpu6050.accel_factor = 2048 ;
            g_mpu6050.accel_factor = 11 - 10;   // 2^11
            break;
        default:
            break;
    }


    switch (range) {
        case GYRO_FULL_RANGE_250 :
            g_mpu6050.gyro_factor = 131.f;
            break;
        case GYRO_FULL_RANGE_500 :
            g_mpu6050.gyro_factor = 65.5f;
            break;
        case GYRO_FULL_RANGE_1000 :
            g_mpu6050.gyro_factor = 32.8f;
            break;
        case GYRO_FULL_RANGE_2000 :
            g_mpu6050.gyro_factor = 16.4f;
            break;
        default:
            break;
    }
}

void MPU6050_Get_Accel_Gyro_Temp(void) {
    g_mpu6050.accel_x = MPU6050_Data_Read(ACCEL_XOUT);
    g_mpu6050.accel_y = MPU6050_Data_Read(ACCEL_YOUT);
    g_mpu6050.accel_z = MPU6050_Data_Read(ACCEL_ZOUT);

    // g_mpu6050.accel_offs_x = MPU6050_Data_Read(ACCEL_OFFS_XOUT);
    // g_mpu6050.accel_offs_y = MPU6050_Data_Read(ACCEL_OFFS_YOUT);
    // g_mpu6050.accel_offs_z = MPU6050_Data_Read(ACCEL_OFFS_ZOUT);

    g_mpu6050.gyro_x = MPU6050_Data_Read(GYRO_XOUT);
    // g_mpu6050.gyro_y = MPU6050_Data_Read( GYRO_YOUT ) ;
    g_mpu6050.gyro_z = MPU6050_Data_Read(GYRO_ZOUT);

    // g_mpu6050.temperature = MPU6050_Data_Read( TEMP_OUT ) ;

    // g_mpu6050.gyro_offs_x = MPU6050_Data_Read(GYRO_OFFS_XOUT);
    // g_mpu6050.gyro_offs_y = MPU6050_Data_Read(GYRO_OFFS_YOUT);
    // g_mpu6050.gyro_offs_z = MPU6050_Data_Read(GYRO_OFFS_ZOUT);

    // g_mpu6050.accel_real_offs_x = g_mpu6050.accel_offs_x * 0.00098;
    // g_mpu6050.accel_real_offs_y = g_mpu6050.accel_offs_y * 0.00098;
    // g_mpu6050.accel_real_offs_z = g_mpu6050.accel_offs_z * 0.00098;
    // g_mpu6050.gyro_real_offs_x = g_mpu6050.gyro_offs_x * 4 / (2 ^ GYRO_FULL_RANGE) / 131.072;
    // g_mpu6050.gyro_real_offs_y = g_mpu6050.gyro_offs_y * 4 / (2 ^ GYRO_FULL_RANGE) / 131.072;
    // g_mpu6050.gyro_real_offs_z = g_mpu6050.gyro_offs_z * 4 / (2 ^ GYRO_FULL_RANGE) / 131.072;
}

void MPU6050_Data_Process(void) {
    // g_mpu6050.accel_scale_x = g_mpu6050.accel_x >> g_mpu6050.accel_factor;
    // g_mpu6050.accel_scale_y = g_mpu6050.accel_y >> g_mpu6050.accel_factor;
    // g_mpu6050.accel_scale_z = g_mpu6050.accel_z >> g_mpu6050.accel_factor;

    g_mpu6050.gyro_scale_x = g_mpu6050.gyro_x / g_mpu6050.gyro_factor;
    // g_mpu6050.gyro_scale_y =  -1 * g_mpu6050.gyro_y  / g_mpu6050.gyro_factor ;   // direction must be same as angle
    g_mpu6050.gyro_scale_z = g_mpu6050.gyro_z / g_mpu6050.gyro_factor;

    // g_mpu6050.temperature_real = g_mpu6050.temperature / 333.87 + 21;

    g_mpu6050.angle_y = atan2f(g_mpu6050.accel_y, g_mpu6050.accel_z) * 180.0f / (float) PI;
    // g_mpu6050.angle_x = g_mpu6050.accel_x * 180 / 16384 / 3.1415926535 ;
    // g_mpu6050.angle_y = atan2( g_mpu6050.accel_y , g_mpu6050.accel_z ) * 180.0 / PI;
    // g_mpu6050.angle_z = atan2( g_mpu6050.accel_z , g_mpu6050.accel_y ) * 180.0 / PI;

    Complementary_Filter_1st_Order(g_mpu6050.angle_y, g_mpu6050.gyro_scale_x);
    // Complementary_Filter_2nd_Order(g_mpu6050.angle_x, g_mpu6050.gyro_scale_y);
#ifdef KALMAN_FILTER_GYRO
    Kalman_Filter(g_mpu6050.angle_y, g_mpu6050.gyro_scale_x);
#endif
    // Complementary_Filter(g_mpu6050.angle_x, g_mpu6050.gyro_scale_y);
}



