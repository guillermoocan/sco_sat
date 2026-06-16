#include "icm20948.h"



static float b[3] = {
    46.143736f,
    37.240751f,
    -23.095304f
};

static float A[3][3] = {
    { 0.992534f,  0.008248f, -0.012997f},
    { 0.008248f,  1.019192f, -0.017242f},
    {-0.012997f, -0.017242f,  0.982020f}
};

static int ICM20948_WriteReg(ICM20948_Type *imu, uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    return write(imu->fd, buffer, 2) == 2 ? 0 : -1;
}

static int ICM20948_ReadRegs(ICM20948_Type *imu, uint8_t reg, uint8_t *data, uint16_t len)
{
    if(write(imu->fd, &reg, 1) != 1)
        return -1;

    if(read(imu->fd, data, len) != len)
        return -1;

    return 0;
}

static int ICM20948_SelectBank(ICM20948_Type *imu, uint8_t bank)
{
    if(imu->BANK == bank)
        return 0;

    uint8_t buffer[2] = {ICM20948_BANK_SEL, bank << 4};

    if(write(imu->fd, buffer, 2) != 2)
        return -1;

    imu->BANK = bank;

    return 0;
}

static int ICM20948_WriteBankReg(ICM20948_Type *imu, uint8_t bank, uint8_t reg, uint8_t value)
{
    if(ICM20948_SelectBank(imu, bank))
        return -1;

    return ICM20948_WriteReg(imu, reg, value);
}

static int ICM20948_ReadBankRegs(ICM20948_Type *imu, uint8_t bank, uint8_t reg, uint8_t *data, uint16_t len)
{
    if(ICM20948_SelectBank(imu, bank))
        return -1;

    return ICM20948_ReadRegs(imu, reg, data, len);
}

static int ICM20948_RegConfig(ICM20948_Type *imu, uint8_t bank, uint8_t reg, uint8_t mask, uint8_t enable)
{
    uint8_t value;

    if(ICM20948_ReadBankRegs(imu, bank, reg, &value, 1))
        return -1;

    if(enable)
        value |= mask;
    else
        value &= ~mask;

    return ICM20948_WriteBankReg(imu, bank, reg, value);
}

static int ICM20948_Slave0ConfigRead(ICM20948_Type *imu, uint8_t address, uint8_t reg, uint8_t len)
{
    if(ICM20948_WriteBankReg(imu, 3, ICM20948_I2C_SLV0_ADDR, ICM20948_I2C_SLV_ADDR_RNW | address))
        return -1;

    if(ICM20948_WriteBankReg(imu, 3, ICM20948_I2C_SLV0_REG, reg))
        return -1;

    if(ICM20948_WriteBankReg(imu, 3, ICM20948_I2C_SLV0_CTRL, ICM20948_I2C_SLV_ENABLE | len))
        return -1;

    usleep(10000);

    return 0;
}

static int ICM20948_Slave0ConfigWrite(ICM20948_Type *imu, uint8_t address, uint8_t reg, uint8_t len, uint8_t data)
{
    if(ICM20948_WriteBankReg(imu, 3, ICM20948_I2C_SLV0_ADDR, address))
        return -1;

    if(ICM20948_WriteBankReg(imu, 3, ICM20948_I2C_SLV0_REG, reg))
        return -1;

    if(ICM20948_WriteBankReg(imu, 3, ICM20948_I2C_SLV0_CTRL, ICM20948_I2C_SLV_ENABLE | len))
        return -1;

    if(ICM20948_WriteBankReg(imu, 3, ICM20948_I2C_SLV0_DO, data))
        return -1;

    usleep(10000);

    return 0;
}


int ICM20948_Init(ICM20948_Type *imu, const char *i2c_device, uint8_t address)
{
    memset(imu, 0, sizeof(ICM20948_Type));

    imu->ADDRESS = address;
    imu->BANK = 0;

    imu->ACCEL_GAIN = 1.0f / 16384.0f;
    imu->GYRO_GAIN  = 1.0f / 131.0f;
    imu->MAG_GAIN   = 0.15f;

    memcpy(imu->mag_bias, b, sizeof(b));
    memcpy(imu->mag_cov, A, sizeof(A));

    imu->fd = open(i2c_device, O_RDWR);

    if(imu->fd < 0)
        return -1;

    if(ioctl(imu->fd, I2C_SLAVE, address) < 0)
        return -1;

    write(imu->fd, &reg, 1);
    read(imu->fd, &whoami, 1);

    printf("WHOAMI = 0x%02X\n", whoami);

    uint8_t whoami;
    printf("ICM20948 found\n");
    if(ICM20948_ReadBankRegs(imu, 0, ICM20948_WHO_AM_I, &whoami, 1))
        return -1;

    printf("%x\n", whoami);
    if(whoami != ICM20948_CHIP_ID)
        return -1;

    printf("ICM20948 found\n");
    if(ICM20948_RegConfig(imu, 0, ICM20948_PWR_MGMT_1, ICM20948_PWR_MGMT_1_RESET, 1))
        return -1;

    usleep(10000);

    if(ICM20948_WriteBankReg(imu, 0, ICM20948_PWR_MGMT_1, ICM20948_PWR_MGMT_1_CLOCK_AUTO))
        return -1;

    if(ICM20948_WriteBankReg(imu, 0, ICM20948_PWR_MGMT_2, 0x00))
        return -1;

    if(ICM20948_WriteBankReg(imu, 3, ICM20948_I2C_MST_CTRL, ICM20948_I2C_MST_CTRL_NSR | 0x07))
        return -1;

    if(ICM20948_RegConfig(imu, 0, ICM20948_USER_CTRL, ICM20948_USER_CTRL_I2C_MST_EN, 1))
        return -1;

    if(ICM20948_Slave0ConfigRead(imu, AK09916_ADDRESS, AK09916_WIA2, 1))
        return -1;

    usleep(10000);

    if(ICM20948_ReadBankRegs(imu, 0, ICM20948_EXT_SLV_DATA_00, &whoami, 1))
        return -1;

    if(whoami != AK09916_CHIP_ID)
        return -1;

    if(ICM20948_Slave0ConfigWrite(imu, AK09916_ADDRESS, AK09916_CNTL3, 1, AK09916_RESET))
        return -1;

    if(ICM20948_Slave0ConfigRead(imu, AK09916_ADDRESS, AK09916_CNTL3, 1))
        return -1;

    do
    {
        usleep(10000);

        if(ICM20948_ReadBankRegs(imu, 0, ICM20948_EXT_SLV_DATA_00, &whoami, 1))
            return -1;

    } while(whoami == 0x01);

    if(ICM20948_Slave0ConfigWrite(imu, AK09916_ADDRESS, AK09916_CNTL2, 1, AK09916_MODE_100HZ))
        return -1;

    if(ICM20948_Slave0ConfigRead(imu, AK09916_ADDRESS, AK09916_ST1, 9))
        return -1;

    if(ICM20948_WriteBankReg(imu, 2, ICM20948_GYRO_SMPLRT_DIV, 0x0A))
        return -1;

    if(ICM20948_WriteBankReg(imu, 2, ICM20948_GYRO_CONFIG_1, 0x21))
        return -1;

    if(ICM20948_WriteBankReg(imu, 2, ICM20948_ACCEL_SMPLRT_DIV_1, 0x00))
        return -1;

    if(ICM20948_WriteBankReg(imu, 2, ICM20948_ACCEL_SMPLRT_DIV_2, 0x0A))
        return -1;

    if(ICM20948_WriteBankReg(imu, 2, ICM20948_ACCEL_CONFIG_1, 0x21))
        return -1;

    return 0;
}



void ICM20948_ReadAccel(ICM20948_Type *imu)
{
    if(ICM20948_ReadBankRegs(imu, 0, ICM20948_ACCEL_XOUT_H, imu->accel_raw, 6))
        return;

    imu->accel_raw_16[0] = (int16_t)((imu->accel_raw[0] << 8) | imu->accel_raw[1]);
    imu->accel_raw_16[1] = (int16_t)((imu->accel_raw[2] << 8) | imu->accel_raw[3]);
    imu->accel_raw_16[2] = (int16_t)((imu->accel_raw[4] << 8) | imu->accel_raw[5]);

    imu->accel[0] = (float)imu->accel_raw_16[0] * imu->ACCEL_GAIN - imu->accel_offset[0];
    imu->accel[1] = (float)imu->accel_raw_16[1] * imu->ACCEL_GAIN - imu->accel_offset[1];
    imu->accel[2] = (float)imu->accel_raw_16[2] * imu->ACCEL_GAIN - imu->accel_offset[2];

    imu->accel_norm = sqrtf(
        imu->accel[0] * imu->accel[0] +
        imu->accel[1] * imu->accel[1] +
        imu->accel[2] * imu->accel[2]
    );
}

void ICM20948_ReadGyro(ICM20948_Type *imu)
{
    if(ICM20948_ReadBankRegs(imu, 0, ICM20948_GYRO_XOUT_H, imu->gyro_raw, 6))
        return;

    imu->gyro_raw_16[0] = (int16_t)((imu->gyro_raw[0] << 8) | imu->gyro_raw[1]);
    imu->gyro_raw_16[1] = (int16_t)((imu->gyro_raw[2] << 8) | imu->gyro_raw[3]);
    imu->gyro_raw_16[2] = (int16_t)((imu->gyro_raw[4] << 8) | imu->gyro_raw[5]);

    imu->gyro[0] = (float)imu->gyro_raw_16[0] * imu->GYRO_GAIN - imu->gyro_offset[0];
    imu->gyro[1] = (float)imu->gyro_raw_16[1] * imu->GYRO_GAIN - imu->gyro_offset[1];
    imu->gyro[2] = (float)imu->gyro_raw_16[2] * imu->GYRO_GAIN - imu->gyro_offset[2];

    imu->gyro_norm = sqrtf(
        imu->gyro[0] * imu->gyro[0] +
        imu->gyro[1] * imu->gyro[1] +
        imu->gyro[2] * imu->gyro[2]
    );
}


void ICM20948_ReadMag(ICM20948_Type *imu)
{
    float mx, my, mz;
    float mx_d, my_d, mz_d;

    if(ICM20948_ReadBankRegs(imu, 0, ICM20948_EXT_SLV_DATA_01, imu->mag_raw, 6))
        return;

    imu->mag_raw_16[0] = (int16_t)((imu->mag_raw[1] << 8) | imu->mag_raw[0]);
    imu->mag_raw_16[1] = (int16_t)((imu->mag_raw[3] << 8) | imu->mag_raw[2]);
    imu->mag_raw_16[2] = (int16_t)((imu->mag_raw[5] << 8) | imu->mag_raw[4]);

    mx = (float)imu->mag_raw_16[0] * imu->MAG_GAIN;
    my = -(float)imu->mag_raw_16[1] * imu->MAG_GAIN;
    mz = -(float)imu->mag_raw_16[2] * imu->MAG_GAIN;

    mx -= imu->mag_bias[0];
    my -= imu->mag_bias[1];
    mz -= imu->mag_bias[2];

    mx_d = imu->mag_cov[0][0] * mx +
           imu->mag_cov[0][1] * my +
           imu->mag_cov[0][2] * mz;

    my_d = imu->mag_cov[1][0] * mx +
           imu->mag_cov[1][1] * my +
           imu->mag_cov[1][2] * mz;

    mz_d = imu->mag_cov[2][0] * mx +
           imu->mag_cov[2][1] * my +
           imu->mag_cov[2][2] * mz;

    imu->mag[0] = mx_d;
    imu->mag[1] = my_d;
    imu->mag[2] = mz_d;

    imu->mag_norm = sqrtf(
        imu->mag[0] * imu->mag[0] +
        imu->mag[1] * imu->mag[1] +
        imu->mag[2] * imu->mag[2]
    );
}

void ICM20948_Read(ICM20948_Type *imu)
{
    ICM20948_ReadAccel(imu);
    ICM20948_ReadGyro(imu);
    ICM20948_ReadMag(imu);
}

void ICM20948_Calibrate(ICM20948_Type *imu)
{
    const int samples = 300;

    float accel_sum[3] = {0};
    float gyro_sum[3] = {0};

    for(int i = 0; i < samples; i++)
    {
        ICM20948_ReadAccel(imu);
        ICM20948_ReadGyro(imu);

        accel_sum[0] += imu->accel[0];
        accel_sum[1] += imu->accel[1];
        accel_sum[2] += imu->accel[2];

        gyro_sum[0] += imu->gyro[0];
        gyro_sum[1] += imu->gyro[1];
        gyro_sum[2] += imu->gyro[2];

        usleep(1000);
    }

    imu->accel_offset[0] = accel_sum[0] / samples;
    imu->accel_offset[1] = accel_sum[1] / samples;
    imu->accel_offset[2] = accel_sum[2] / samples - 1.0f;

    imu->gyro_offset[0] = gyro_sum[0] / samples;
    imu->gyro_offset[1] = gyro_sum[1] / samples;
    imu->gyro_offset[2] = gyro_sum[2] / samples;
}

void ICM20948_Close(ICM20948_Type *imu)
{
    if(imu->fd >= 0)
    {
        close(imu->fd);
        imu->fd = -1;
    }
}



int ICM20948_CalibrateMag(ICM20948_Type *imu, const char *filename, uint32_t samples, uint32_t period_us)
{
    FILE *fp = fopen(filename, "w");

    if(!fp)
        return -1;

    fprintf(fp, "mx,my,mz\n");

    printf("\rCalculando calibracion...\n");

    for(uint32_t i = 0; i < samples; i++)
    {
        if(ICM20948_ReadBankRegs(imu, 0, ICM20948_EXT_SLV_DATA_01, imu->mag_raw, 6))
        {
            fclose(fp);
            return -1;
        }

        imu->mag_raw_16[0] = (int16_t)((imu->mag_raw[1] << 8) | imu->mag_raw[0]);
        imu->mag_raw_16[1] = (int16_t)((imu->mag_raw[3] << 8) | imu->mag_raw[2]);
        imu->mag_raw_16[2] = (int16_t)((imu->mag_raw[5] << 8) | imu->mag_raw[4]);

        imu->mag[0] =  (float)imu->mag_raw_16[0] * imu->MAG_GAIN;
        imu->mag[1] = -(float)imu->mag_raw_16[1] * imu->MAG_GAIN;
        imu->mag[2] = -(float)imu->mag_raw_16[2] * imu->MAG_GAIN;

        printf("%.3f,%.3f,%.3f\n",
               imu->mag[0],
               imu->mag[1],
               imu->mag[2]);

        fprintf(fp,
                "%.8f,%.8f,%.8f\n",
                imu->mag[0],
                imu->mag[1],
                imu->mag[2]);

        usleep(period_us);

        if(i % 100 == 0)
            printf("\rMuestra: %u/%u\n", i, samples);
    }

    printf("\rMuestras: %u/%u\n", samples, samples);

    fclose(fp);

    return 0;
}


