#include "main.h"

int main(void)
{
    MPU6050_Type imu;
    QMC5883L_Type mag;
    int fd;
    uint8_t reg;
    uint8_t id;

    fd = open("/dev/i2c-1", O_RDWR);

    if (ioctl(fd, I2C_SLAVE, QMC5883L_ADDRESS) < 0)
    {
        printf("Error seleccionando QMC5883L\n");
        return -1;
    }

    if (fd < 0)
    {
        printf("Error abriendo I2C\n");
        return -1;
    }

    if (MPU6050_Init(&imu, fd, MPU6050_ADDRESS_AD0_LOW, MPU6050_ACCEL_FS_2, MPU6050_GYRO_FS_250))
    {
        printf("MPU6050 no detectado\n");
        return -1;
    }

    printf("MPU6050 detectado\n");

    if (QMC5883L_Init(&mag, fd, QMC5883L_ADDRESS, QMC5883L_MODE_CONTINUOUS, QMC5883L_ODR_200HZ, QMC5883L_FULL_SCALE_8G, QMC5883L_OVERSAMPLE_512))
    {
        printf("QMC5883L no detectado :(\n");
        return -1;
    }

    printf("QMC5883L detectado\n");

    // QMC5883L_Calibrate(&mag, "../data/calibration.csv", 1000, 10000);

    // if(system("python3 ../data/calibration_python.py ../data/calibration.csv"))
    // {
    //     printf("Error ejecutando calibracion_python.py\n");
    //     return -1;
    // }


    while (1)
    {
        MPU6050_Read(&imu);
        QMC5883L_Read(&mag);

        printf(" ACC: %.3f %.3f %.3f | GYRO: %.3f %.3f %.3f | MAG: %.3f %.3f %.3f | |B|: %.3f\n",
            imu.accel[0], imu.accel[1], imu.accel[2],
            imu.gyro[0], imu.gyro[1], imu.gyro[2],
            mag.field[0], mag.field[1], mag.field[2],
            mag.field_mag);

        usleep(10000);
    }

    return 0;
}