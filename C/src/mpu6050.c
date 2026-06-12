#include "mpu6050.h"
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

static int MPU6050_WriteReg(MPU6050_Type *mpu, uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    return write(mpu->fd, buffer, 2) == 2 ? 0 : -1;
}

static int MPU6050_ReadRegs(MPU6050_Type *mpu, uint8_t reg, uint8_t *data, uint16_t len)
{
    if(write(mpu->fd, &reg, 1) != 1) return -1;
    if(read(mpu->fd, data, len) != len) return -1;
    return 0;
}

int MPU6050_Init(MPU6050_Type *mpu, const char *i2c_device, uint8_t address, uint8_t accel_fs, uint8_t gyro_fs)
{
    memset(mpu, 0, sizeof(MPU6050_Type));

    mpu->ADDRESS = address;
    mpu->ACCEL_FS = accel_fs;
    mpu->GYRO_FS = gyro_fs;

    mpu->fd = open(i2c_device, O_RDWR);
    if(mpu->fd < 0) return -1;

    if(ioctl(mpu->fd, I2C_SLAVE, address) < 0) return -1;


    switch(accel_fs)
    {
        case MPU6050_ACCEL_FS_2:  mpu->ACCEL_LSB_GAIN = 16384.0f; break;
        case MPU6050_ACCEL_FS_4:  mpu->ACCEL_LSB_GAIN = 8192.0f;  break;
        case MPU6050_ACCEL_FS_8:  mpu->ACCEL_LSB_GAIN = 4096.0f;  break;
        case MPU6050_ACCEL_FS_16: mpu->ACCEL_LSB_GAIN = 2048.0f;  break;
        default: return -1;
    }

    switch(gyro_fs)
    {
        case MPU6050_GYRO_FS_250:  mpu->GYRO_LSB_GAIN = 131.0f; break;
        case MPU6050_GYRO_FS_500:  mpu->GYRO_LSB_GAIN = 65.5f;  break;
        case MPU6050_GYRO_FS_1000: mpu->GYRO_LSB_GAIN = 32.8f;  break;
        case MPU6050_GYRO_FS_2000: mpu->GYRO_LSB_GAIN = 16.4f;  break;
        default: return -1;
    }

    uint8_t whoami;

    if(MPU6050_ReadRegs(mpu, MPU6050_RA_WHO_AM_I, &whoami, 1) < 0)
        return -1;

    if(whoami != 0x68)
        return -1;

    MPU6050_WriteReg(mpu, MPU6050_RA_PWR_MGMT_1, 0x00);
    MPU6050_WriteReg(mpu, MPU6050_RA_SMPLRT_DIV, 0x07);
    MPU6050_WriteReg(mpu, MPU6050_RA_ACCEL_CONFIG, accel_fs << 3);
    MPU6050_WriteReg(mpu, MPU6050_RA_GYRO_CONFIG, gyro_fs << 3);

    uint8_t pwr, accel_cfg, gyro_cfg;

    MPU6050_ReadRegs(mpu, MPU6050_RA_PWR_MGMT_1, &pwr, 1);
    MPU6050_ReadRegs(mpu, MPU6050_RA_ACCEL_CONFIG, &accel_cfg, 1);
    MPU6050_ReadRegs(mpu, MPU6050_RA_GYRO_CONFIG, &gyro_cfg, 1);

    printf("PWR_MGMT_1   = 0x%02X\n", pwr);    
    printf("ACCEL_CONFIG = 0x%02X\n", accel_cfg);
    printf("GYRO_CONFIG  = 0x%02X\n", gyro_cfg);

    return 0;
}

void MPU6050_Read(MPU6050_Type *mpu)
{
    MPU6050_ReadRegs(mpu, MPU6050_RA_ACCEL_XOUT_H, mpu->accel_raw, 6);
    MPU6050_ReadRegs(mpu, MPU6050_RA_GYRO_XOUT_H, mpu->gyro_raw, 6);

    mpu->accel_raw_16[0] = (int16_t)((mpu->accel_raw[0] << 8) | mpu->accel_raw[1]);
    mpu->accel_raw_16[1] = (int16_t)((mpu->accel_raw[2] << 8) | mpu->accel_raw[3]);
    mpu->accel_raw_16[2] = (int16_t)((mpu->accel_raw[4] << 8) | mpu->accel_raw[5]);

    mpu->gyro_raw_16[0] = (int16_t)((mpu->gyro_raw[0] << 8) | mpu->gyro_raw[1]);
    mpu->gyro_raw_16[1] = (int16_t)((mpu->gyro_raw[2] << 8) | mpu->gyro_raw[3]);
    mpu->gyro_raw_16[2] = (int16_t)((mpu->gyro_raw[4] << 8) | mpu->gyro_raw[5]);

    mpu->accel[0] = (float)mpu->accel_raw_16[0] / mpu->ACCEL_LSB_GAIN - mpu->accel_offset[0];
    mpu->accel[1] = (float)mpu->accel_raw_16[1] / mpu->ACCEL_LSB_GAIN - mpu->accel_offset[1];
    mpu->accel[2] = (float)mpu->accel_raw_16[2] / mpu->ACCEL_LSB_GAIN - mpu->accel_offset[2];

    mpu->gyro[0] = (float)mpu->gyro_raw_16[0] / mpu->GYRO_LSB_GAIN - mpu->gyro_offset[0];
    mpu->gyro[1] = (float)mpu->gyro_raw_16[1] / mpu->GYRO_LSB_GAIN - mpu->gyro_offset[1];
    mpu->gyro[2] = (float)mpu->gyro_raw_16[2] / mpu->GYRO_LSB_GAIN - mpu->gyro_offset[2];

    

    


}

void MPU6050_Calibration(MPU6050_Type *mpu)
{
    const int samples = 1000;
    float accel_sum[3] = {0};
    float gyro_sum[3] = {0};

    for(int i = 0; i < samples; i++)
    {
        MPU6050_Read(mpu);

        accel_sum[0] += mpu->accel[0];
        accel_sum[1] += mpu->accel[1];
        accel_sum[2] += mpu->accel[2];

        gyro_sum[0] += mpu->gyro[0];
        gyro_sum[1] += mpu->gyro[1];
        gyro_sum[2] += mpu->gyro[2];

        usleep(1000);
    }

    mpu->accel_offset[0] = accel_sum[0] / samples;
    mpu->accel_offset[1] = accel_sum[1] / samples;
    mpu->accel_offset[2] = accel_sum[2] / samples - 1.0f;

    mpu->gyro_offset[0] = gyro_sum[0] / samples;
    mpu->gyro_offset[1] = gyro_sum[1] / samples;
    mpu->gyro_offset[2] = gyro_sum[2] / samples;
}

void MPU6050_Close(MPU6050_Type *mpu)
{
    if(mpu->fd >= 0) close(mpu->fd);
}