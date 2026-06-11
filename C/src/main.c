#include "main.h"

int main()
{
    MPU6050_Type imu;

    if(MPU6050_Init(&imu, "/dev/i2c-1", 0x68, MPU6050_ACCEL_FS_2, MPU6050_GYRO_FS_250))
    {
        printf("MPU6050 no encontrado\n");
        return -1;
    }

    MPU6050_Calibration(&imu);
    
    while(1)
    {
        MPU6050_Read(&imu);

        

        printf("A: %.3f %.3f %.3f  G: %.3f %.3f %.3f\n",imu.accel[0], imu.accel[1], imu.accel[2],imu.gyro[0], imu.gyro[1], imu.gyro[2]);

        //usleep(1000);
    }

    MPU6050_Close(&imu);
    return 0;
}