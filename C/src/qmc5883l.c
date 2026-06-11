
#include "qmc5883l.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <math.h>

static float D[3][3] =
{
    {0.9854f, -0.0133f, -0.0172f},
    {-0.0133f, 1.0653f, 0.0009f},
    {-0.0172f, 0.0009f, 0.9531f}
};

static float b[3] =
{
    0.1738f,
   -0.0701f,
   -0.2817f
};

static int QMC5883L_WriteReg(QMC5883L_Type *qmc, uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    return write(qmc->fd, buffer, 2) == 2 ? 0 : -1;
}

static int QMC5883L_ReadRegs(QMC5883L_Type *qmc, uint8_t reg, uint8_t *data, uint16_t len)
{
    if(write(qmc->fd, &reg, 1) != 1) return -1;
    if(read(qmc->fd, data, len) != len) return -1;
    return 0;
}

int QMC5883L_Init(QMC5883L_Type *qmc, const char *i2c_device, uint8_t address, uint8_t mode, uint8_t data_output_rate, uint8_t full_scale, uint8_t over_sample)
{
    memset(qmc, 0, sizeof(QMC5883L_Type));

    qmc->ADDRESS = address;
    qmc->MODE = mode;
    qmc->DATA_OUTPUT_RATE = data_output_rate;
    qmc->FULL_SCALE = full_scale;
    qmc->OVER_SAMPLE = over_sample;

    qmc->fd = open(i2c_device, O_RDWR);

    if(qmc->fd < 0)
        return -1;

    if(ioctl(qmc->fd, I2C_SLAVE, address) < 0)
        return -1;

    uint8_t chip_id;

    if(QMC5883L_ReadRegs(qmc, QMC5883L_CHIP_ID_REG, &chip_id, 1) < 0)
        return -1;

    if(chip_id != QMC5883L_CHIP_ID)
    {
        printf("QMC5883L no detectado. ID = 0x%02X\n", chip_id);
        return -1;
    }

    uint8_t data = (over_sample << 6) |
                   (full_scale << 4) |
                   (data_output_rate << 2) |
                   mode;

    if(QMC5883L_WriteReg(qmc, QMC5883L_CONFIG_REG_A, data))
        return -1;

    if(QMC5883L_WriteReg(qmc, QMC5883L_CONFIG_REG_B, QMC5883L_CRB_SET_RESET_PERIOD))
        return -1;

    switch(full_scale)
    {
        case QMC5883L_FULL_SCALE_2G:
            qmc->GAIN_LSB = 12000.0f;
            break;

        case QMC5883L_FULL_SCALE_8G:
            qmc->GAIN_LSB = 3000.0f;
            break;

        default:
            return -1;
    }

    return 0;
}

void QMC5883L_Read(QMC5883L_Type *qmc)
{
    float f_aux[3];

    if(QMC5883L_ReadRegs(qmc, QMC5883L_DATA_OUTPUT_X_LSB, qmc->field_raw, 6))
        return;

    qmc->field_raw_16[0] = (int16_t)((qmc->field_raw[1] << 8) | qmc->field_raw[0]);
    qmc->field_raw_16[1] = (int16_t)((qmc->field_raw[3] << 8) | qmc->field_raw[2]);
    qmc->field_raw_16[2] = (int16_t)((qmc->field_raw[5] << 8) | qmc->field_raw[4]);

    f_aux[0] = (float)qmc->field_raw_16[0] / qmc->GAIN_LSB - b[0];
    f_aux[1] = (float)qmc->field_raw_16[1] / qmc->GAIN_LSB - b[1];
    f_aux[2] = (float)qmc->field_raw_16[2] / qmc->GAIN_LSB - b[2];

    qmc->field[0] = D[0][0]*f_aux[0] + D[0][1]*f_aux[1] + D[0][2]*f_aux[2];
    qmc->field[1] = D[1][0]*f_aux[0] + D[1][1]*f_aux[1] + D[1][2]*f_aux[2];
    qmc->field[2] = D[2][0]*f_aux[0] + D[2][1]*f_aux[1] + D[2][2]*f_aux[2];

    f_aux[0] = qmc->field[0];
    f_aux[1] = qmc->field[1];
    f_aux[2] = qmc->field[2];

    qmc->field[0] =  f_aux[1];
    qmc->field[1] = -f_aux[0];
    qmc->field[2] =  f_aux[2];

    qmc->field_mag = sqrtf(
        qmc->field[0]*qmc->field[0] +
        qmc->field[1]*qmc->field[1] +
        qmc->field[2]*qmc->field[2]
    );
}

void QMC5883L_Close(QMC5883L_Type *qmc)
{
    if(qmc->fd >= 0)
        close(qmc->fd);
}