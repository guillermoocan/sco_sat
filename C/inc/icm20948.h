#ifndef ICM20948_H
#define ICM20948_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <math.h>

typedef struct
{
    int fd;

    uint8_t ADDRESS;
    uint8_t BANK;

    float ACCEL_GAIN;
    float GYRO_GAIN;
    float MAG_GAIN;

    uint8_t accel_raw[6];
    uint8_t gyro_raw[6];
    uint8_t mag_raw[6];

    int16_t accel_raw_16[3];
    int16_t gyro_raw_16[3];
    int16_t mag_raw_16[3];

    float accel[3];
    float gyro[3];
    float mag[3];

    float accel_offset[3];
    float gyro_offset[3];

    float mag_bias[3];
    float mag_cov[3][3];

    float accel_norm;
    float gyro_norm;
    float mag_norm;

} ICM20948_Type;


/* =========================
 * ICM20948 ADDRESS
 * ========================= */

#define ICM20948_ADDRESS_AD0_LOW     0x68
#define ICM20948_ADDRESS_AD0_HIGH    0x69

#define ICM20948_CHIP_ID             0xEA


/* =========================
 * BANK 0
 * ========================= */

#define ICM20948_WHO_AM_I            0x00

#define ICM20948_USER_CTRL           0x03

#define ICM20948_PWR_MGMT_1          0x06
#define ICM20948_PWR_MGMT_2          0x07

#define ICM20948_ACCEL_XOUT_H        0x2D
#define ICM20948_GYRO_XOUT_H         0x33

#define ICM20948_EXT_SLV_DATA_00     0x3B
#define ICM20948_EXT_SLV_DATA_01     0x3C

#define ICM20948_BANK_SEL            0x7F


/* =========================
 * BANK 2
 * ========================= */

#define ICM20948_GYRO_SMPLRT_DIV     0x00
#define ICM20948_GYRO_CONFIG_1       0x01

#define ICM20948_ACCEL_SMPLRT_DIV_1  0x10
#define ICM20948_ACCEL_SMPLRT_DIV_2  0x11
#define ICM20948_ACCEL_CONFIG_1      0x14


/* =========================
 * BANK 3
 * ========================= */

#define ICM20948_I2C_MST_CTRL        0x01

#define ICM20948_I2C_SLV0_ADDR       0x03
#define ICM20948_I2C_SLV0_REG        0x04
#define ICM20948_I2C_SLV0_CTRL       0x05
#define ICM20948_I2C_SLV0_DO         0x06


/* =========================
 * CONTROL BITS
 * ========================= */

#define ICM20948_PWR_MGMT_1_RESET        0x80
#define ICM20948_PWR_MGMT_1_CLOCK_AUTO   0x01
#define ICM20948_PWR_MGMT_1_LP           0x20

#define ICM20948_USER_CTRL_I2C_MST_EN    0x20

#define ICM20948_I2C_MST_CTRL_NSR        0x10

#define ICM20948_I2C_SLV_ADDR_RNW        0x80

#define ICM20948_I2C_SLV_ENABLE          0x80
#define ICM20948_I2C_SLV_BYTE_SWAP       0x40
#define ICM20948_I2C_SLV_REG_DIS         0x20
#define ICM20948_I2C_SLV_REG_GROUP       0x10


/* =========================
 * AK09916 MAGNETOMETER
 * ========================= */

#define AK09916_ADDRESS              0x0C
#define AK09916_CHIP_ID              0x09

#define AK09916_WIA1                 0x00
#define AK09916_WIA2                 0x01

#define AK09916_ST1                  0x10

#define AK09916_HXL                  0x11
#define AK09916_HXH                  0x12
#define AK09916_HYL                  0x13
#define AK09916_HYH                  0x14
#define AK09916_HZL                  0x15
#define AK09916_HZH                  0x16

#define AK09916_ST2                  0x18

#define AK09916_CNTL2                0x31
#define AK09916_CNTL3                0x32

#define AK09916_ST1_DRDY             0x01
#define AK09916_ST1_DOR              0x02

#define AK09916_ST2_HOFL             0x08

#define AK09916_MODE_OFF             0x00
#define AK09916_MODE_SINGLE          0x01
#define AK09916_MODE_10HZ            0x02
#define AK09916_MODE_20HZ            0x04
#define AK09916_MODE_50HZ            0x06
#define AK09916_MODE_100HZ           0x08
#define AK09916_MODE_TEST            0x10

#define AK09916_RESET                0x01


/* =========================
 * PUBLIC API
 * ========================= */

int ICM20948_Init(ICM20948_Type *imu, const char *i2c_device, uint8_t address);

void ICM20948_ReadAccel(ICM20948_Type *imu);
void ICM20948_ReadGyro(ICM20948_Type *imu);
void ICM20948_ReadMag(ICM20948_Type *imu);
void ICM20948_Read(ICM20948_Type *imu);

void ICM20948_Calibrate(ICM20948_Type *imu);

void ICM20948_Close(ICM20948_Type *imu);


void ICM20948_Close(ICM20948_Type *imu)
{
    if(imu->fd >= 0)
    {
        close(imu->fd);
        imu->fd = -1;
    }
}



#endif