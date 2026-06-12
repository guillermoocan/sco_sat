#include "main.h"

int main(void)
{
    MPU6050_Type imu;
    QMC5883L_Type mag;

    if (MPU6050_Init(&imu, "/dev/i2c-1", MPU6050_ADDRESS_AD0_LOW, MPU6050_ACCEL_FS_4, MPU6050_GYRO_FS_250))
    {
        printf("MPU6050 no detectado\n");
        return -1;
    }

    printf("MPU6050 detectado\n");

    if (QMC5883L_Init(&mag, "/dev/i2c-1", QMC5883L_ADDRESS, QMC5883L_MODE_CONTINUOUS, QMC5883L_ODR_200HZ, QMC5883L_FULL_SCALE_8G, QMC5883L_OVERSAMPLE_512))
    {
        printf("QMC5883L no detectado\n");
        return -1;
    }

    printf("QMC5883L detectado\n");

    MPU6050_Calibration(&imu);

    while (1)
    {
        MPU6050_Read(&imu);
        QMC5883L_Read(&mag);

        printf("%6d %6d %6d | ",
            imu.accel_raw_16[0],
            imu.accel_raw_16[1],
            imu.accel_raw_16[2]);

        printf("ACC: %7.3f %7.3f %7.3f | ",
            imu.accel[0],
            imu.accel[1],
            imu.accel[2]);

        printf("GYRO: %7.3f %7.3f %7.3f | ",
            imu.gyro[0],
            imu.gyro[1],
            imu.gyro[2]);

        printf("MAG: %7.3f %7.3f %7.3f | ",
            mag.field[0],
            mag.field[1],
            mag.field[2]);

        printf("|B|: %7.3f\n", mag.field_mag);

        usleep(10000);
    }

    MPU6050_Close(&imu);
    QMC5883L_Close(&mag);

    return 0;
}