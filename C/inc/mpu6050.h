#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

typedef struct
{
    int fd;
    uint8_t ADDRESS;
    uint8_t ACCEL_FS;
    uint8_t GYRO_FS;

    float ACCEL_LSB_GAIN;
    float GYRO_LSB_GAIN;

    uint8_t accel_raw[6];
    float accel[3];
    float accel_offset[3];

    uint8_t gyro_raw[6];
    float gyro[3];
    float gyro_offset[3];

    int16_t accel_raw_16[3];
    int16_t gyro_raw_16[3];

    float temperature;
    float angle[3];

} MPU6050_Type;

#define MPU6050_ADDRESS_AD0_LOW  0x68
#define MPU6050_ADDRESS_AD0_HIGH 0x69

#define MPU6050_RA_SMPLRT_DIV   0x19
#define MPU6050_RA_GYRO_CONFIG  0x1B
#define MPU6050_RA_ACCEL_CONFIG 0x1C
#define MPU6050_RA_PWR_MGMT_1   0x6B
#define MPU6050_RA_WHO_AM_I     0x75

#define MPU6050_ACCEL_FS_2  0x00
#define MPU6050_ACCEL_FS_4  0x01
#define MPU6050_ACCEL_FS_8  0x02
#define MPU6050_ACCEL_FS_16 0x03

#define MPU6050_GYRO_FS_250  0x00
#define MPU6050_GYRO_FS_500  0x01
#define MPU6050_GYRO_FS_1000 0x02
#define MPU6050_GYRO_FS_2000 0x03

#define MPU6050_RA_ACCEL_XOUT_H 0x3B
#define MPU6050_RA_GYRO_XOUT_H  0x43

int MPU6050_Init(MPU6050_Type *mpu, int fd_ext, const char *i2c_device, uint8_t address, uint8_t accel_fs, uint8_t gyro_fs);
void MPU6050_Read(MPU6050_Type *mpu);
void MPU6050_Calibration(MPU6050_Type *mpu);
void MPU6050_Close(MPU6050_Type *mpu);

#endif