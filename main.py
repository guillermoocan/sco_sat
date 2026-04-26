from machine import Pin, I2C
from utime import sleep_ms, ticks_us

from src.adcs import ADCS
from src.icm20948 import ICM20948
from src.utilities import r2rpy, q2r


I2C0_SDA = Pin(4)
I2C0_SCL = Pin(5)

w=[0.5,0.5]

bus = I2C(0, sda=I2C0_SDA, scl=I2C0_SCL, freq = 400_000)

imu = ICM20948(bus)
sat = ADCS()

imu.acc_cal()
sat.set_b_est(imu.gyr_cal_b())

sat.set_i_v1([0.0,0.0,1.0])
sat.set_i_v2([26.9282,-1.8438,-28.6434])

sat.set_b_v1(imu.acc())
sat.set_b_v2(imu.mag())
sat.set_rate(imu.gyr())

sat.set_w_est(sat.rate)

sat.quest(w)
sat.set_rpy_est(r2rpy(q2r(sat.q_est)))

t0 = ticks_us()

while True :

    sat.set_b_v1(imu.acc())
    sat.set_b_v2(imu.mag())
    sat.set_rate(imu.gyr())


    t1 = ticks_us()
    dt = (t1 - t0)*1e-6
    t0 = t1

    # #TRIAD

    sat.triad()
    sat.set_rpy(r2rpy(sat.R_est))

    # #QUEST

    # sat.quest(w)
    # sat.set_rpy(r2rpy(q2r(sat.q_est)))
    #sat.complementary_filter(dt,0.9)

    #SMEKF

    sat.smekf(dt)
    sat.set_rpy_est(r2rpy(q2r(sat.q_est)))
    sat.send_rpy()





