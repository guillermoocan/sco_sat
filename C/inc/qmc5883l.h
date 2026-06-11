#ifndef QMC5883L_H
#define QMC5883L_H

#include <stdint.h>

#define QMC5883L_ADDRESS 0x0D

#define QMC5883L_DATA_OUTPUT_X_LSB 0x00
#define QMC5883L_DATA_OUTPUT_X_MSB 0x01
#define QMC5883L_DATA_OUTPUT_Y_LSB 0x02
#define QMC5883L_DATA_OUTPUT_Y_MSB 0x03
#define QMC5883L_DATA_OUTPUT_Z_LSB 0x04
#define QMC5883L_DATA_OUTPUT_Z_MSB 0x05

#define QMC5883L_STATUS_REG 0x06
#define QMC5883L_TEMPERATURE_LSB 0x07
#define QMC5883L_TEMPERATURE_MSB 0x08

#define QMC5883L_CONFIG_REG_A 0x09
#define QMC5883L_CONFIG_REG_B 0x0A
#define QMC5883L_RESET_REG 0x0B

#define QMC5883L_CHIP_ID_REG 0x0D
#define QMC5883L_CHIP_ID 0xFF

#define QMC5883L_MODE_STANDBY 0x00
#define QMC5883L_MODE_CONTINUOUS 0x01

#define QMC5883L_ODR_10HZ 0x00
#define QMC5883L_ODR_50HZ 0x01
#define QMC5883L_ODR_100HZ 0x02
#define QMC5883L_ODR_200HZ 0x03

#define QMC5883L_FULL_SCALE_2G 0x00
#define QMC5883L_FULL_SCALE_8G 0x01

#define QMC5883L_OVERSAMPLE_512 0x00
#define QMC5883L_OVERSAMPLE_256 0x01
#define QMC5883L_OVERSAMPLE_128 0x02
#define QMC5883L_OVERSAMPLE_64 0x03

#define QMC5883L_CRB_SET_RESET_PERIOD 0x01

typedef struct
{
    int fd;

    uint8_t ADDRESS;
    uint8_t MODE;
    uint8_t DATA_OUTPUT_RATE;
    uint8_t FULL_SCALE;
    uint8_t OVER_SAMPLE;

    float GAIN_LSB;

    uint8_t field_raw[6];
    int16_t field_raw_16[3];

    float field[3];
    float field_mag;

} QMC5883L_Type;

int QMC5883L_Init(QMC5883L_Type *qmc, const char *i2c_device, uint8_t address, uint8_t mode, uint8_t data_output_rate, uint8_t full_scale, uint8_t over_sample);

void QMC5883L_Measure_Read(QMC5883L_Type *qmc);

void QMC5883L_Close(QMC5883L_Type *qmc);

#endif