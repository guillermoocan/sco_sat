#include "main.h"

int main(void)
{
    ICM20948_Type imu;
    SCO sco;

    float obs_0_r[3] = {0.0f, 0.0f, 1.0f};
    float obs_1_r[3] = {26.9282f,-1.8438f,-28.6434f};

    uint64_t t0, t1;
    double dt;

    if(ICM20948_Init(&imu, "/dev/i2c-1", ICM20948_ADDRESS_AD0_HIGH))
    {
        printf("ICM20948 no detectado\n");
        return -1;
    }

    printf("ICM20948 detectado\n");

    ICM20948_Calibrate(&imu);

    ICM20948_Read(&imu);

    SCO_Task_Initialization(
        &sco,
        obs_0_r,
        imu.accel,
        obs_1_r,
        imu.mag,
        imu.gyro
    );

    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    t0 = (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;

    while(1)
    {
        ICM20948_Read(&imu);

        clock_gettime(CLOCK_MONOTONIC, &ts);
        t1 = (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;

        dt = (double)(t1 - t0) * 1e-9;
        t0 = t1;

        SCO_Task_Estimation(
            &sco,
            obs_0_r,
            imu.accel,
            obs_1_r,
            imu.mag,
            imu.gyro,
            dt
        );

        printf(
            "%.6f %.6f %.6f %.6f "
            "%.6f %.6f %.6f %.6f "
            "%.6f %.6f %.6f "
            "%.6f %.6f %.6f\n",
            sco.q_quest.q[0],
            sco.q_quest.q[1],
            sco.q_quest.q[2],
            sco.q_quest.q[3],
            sco.q_est.q[0],
            sco.q_est.q[1],
            sco.q_est.q[2],
            sco.q_est.q[3],
            sco.rate.d[0],
            sco.rate.d[1],
            sco.rate.d[2],
            sco.w_est.d[0],
            sco.w_est.d[1],
            sco.w_est.d[2]
        );

        usleep(10000);
    }

    ICM20948_Close(&imu);

    return 0;
}







// #include "main.h"

// int main(void)
// {
//     MPU6050_Type mpu;
//     QMC5883L_Type magn;
//     SCO sco;

//     float obs_0_r[3] = {0.0f, 0.0f, 1.0f};
//     float obs_1_r[3] = {0.315f   ,0.013f  ,-0.255f};

//     uint64_t t0, t1;
//     double dt;

//     if (MPU6050_Init(&mpu, "/dev/i2c-1", MPU6050_ADDRESS_AD0_LOW, MPU6050_ACCEL_FS_2, MPU6050_GYRO_FS_250))
//     {
//         printf("MPU6050 no detectado\n");
//         return -1;
//     }

//     if (QMC5883L_Init(&magn, "/dev/i2c-1", QMC5883L_ADDRESS, QMC5883L_MODE_CONTINUOUS, QMC5883L_ODR_200HZ, QMC5883L_FULL_SCALE_2G, QMC5883L_OVERSAMPLE_512))
//     {
//         printf("QMC5883L no detectado\n");
//         return -1;
//     }

//     printf("Sensores detectados\n");

//     MPU6050_Calibration(&mpu);

//     MPU6050_Read(&mpu);
//     QMC5883L_Read(&magn);

//     SCO_Task_Initialization(&sco, obs_0_r, mpu.accel, obs_1_r, magn.field, mpu.gyro);

//     struct timespec ts;

//     clock_gettime(CLOCK_MONOTONIC, &ts);
//     t0 = (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;

//     while (1)
//     {
//         MPU6050_Read(&mpu);
//         QMC5883L_Read(&magn);

//         clock_gettime(CLOCK_MONOTONIC, &ts);
//         t1 = (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;

//         dt = (double)(t1 - t0) * 1e-9;

//         t0 = t1;

//         SCO_Task_Estimation(&sco, obs_0_r, mpu.accel, obs_1_r, magn.field, mpu.gyro, dt);

//         printf("%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
//         sco.q_quest.q[0],
//         sco.q_quest.q[1],
//         sco.q_quest.q[2],
//         sco.q_quest.q[3],
//         sco.q_est.q[0],
//         sco.q_est.q[1],
//         sco.q_est.q[2],
//         sco.q_est.q[3],
//         sco.rate.d[0],
//         sco.rate.d[1],
//         sco.rate.d[2],
//         sco.w_est.d[0],
//         sco.w_est.d[1],
//         sco.w_est.d[2]);

//         usleep(10000);
//     }

//     MPU6050_Close(&mpu);
//     QMC5883L_Close(&magn);

//     return 0;
// }











// #include "main.h"

// int main(void)
// {
//     MPU6050_Type imu;
//     QMC5883L_Type mag;

//     if (MPU6050_Init(&imu, "/dev/i2c-1", MPU6050_ADDRESS_AD0_LOW, MPU6050_ACCEL_FS_4, MPU6050_GYRO_FS_250))
//     {
//         printf("MPU6050 no detectado\n");
//         return -1;
//     }

//     printf("MPU6050 detectado\n");

//     if (QMC5883L_Init(&mag, "/dev/i2c-1", QMC5883L_ADDRESS, QMC5883L_MODE_CONTINUOUS, QMC5883L_ODR_200HZ, QMC5883L_FULL_SCALE_8G, QMC5883L_OVERSAMPLE_512))
//     {
//         printf("QMC5883L no detectado\n");
//         return -1;
//     }

//     printf("QMC5883L detectado\n");


//     QMC5883L_Calibrate(&mag, "../data/calibration.csv", 1000, 10000);
//     if(system("python3 ../data/calibration_python.py ../data/calibration.csv")) 
//     { 
//         printf("Error ejecutando calibracion_python.py\n"); 
//         return -1; 
//     }


//     MPU6050_Calibration(&imu);

//     while (1)
//     {
//         MPU6050_Read(&imu);
//         QMC5883L_Read(&mag);

//         printf("ACC: %7.3f %7.3f %7.3f | ",
//             imu.accel[0],
//             imu.accel[1],
//             imu.accel[2]);

//         printf("GYRO: %7.3f %7.3f %7.3f | ",
//             imu.gyro[0],
//             imu.gyro[1],
//             imu.gyro[2]);

//         printf("MAG: %7.3f %7.3f %7.3f | ",
//             mag.field[0],
//             mag.field[1],
//             mag.field[2]);

//         printf("|B|: %7.3f\n", mag.field_mag);

//         usleep(10000);
//     }

//     MPU6050_Close(&imu);
//     QMC5883L_Close(&mag);

//     return 0;
// }