# from smbus2 import SMBus

# from src.adcs import ADCS
# from src.icm20948 import ICM20948
# from src.utilities import r2rpy, q2r

# import time

# w=[0.5,0.5]

# bus = SMBus(1)
# imu = ICM20948(bus)

# sat = ADCS()

# imu.acc_cal()
# sat.set_b_est(imu.gyr_cal_b())

# sat.set_i_v1([0.0,0.0,1.0])
# sat.set_i_v2([26.9282,-1.8438,-28.6434])

# sat.set_b_v1(imu.acc())
# sat.set_b_v2(imu.mag())
# sat.set_rate(imu.gyr())

# sat.set_w_est(sat.rate)

# sat.quest(w)
# sat.set_rpy_est(r2rpy(q2r(sat.q_est)))

# t0 = time.perf_counter()

# while True :

#     #imu.send_imu()

#     time.sleep(0.03) 


#     sat.set_b_v1(imu.acc())
#     sat.set_b_v2(imu.mag())
#     sat.set_rate(imu.gyr())


#     t1 = time.perf_counter()
#     dt = t1 - t0
#     t0 = t1

# #     # #TRIAD

# #     sat.triad()
# #     sat.set_rpy(r2rpy(sat.R_est))

# #     # #QUEST

# #     # sat.quest(w)
# #     # sat.set_rpy(r2rpy(q2r(sat.q_est)))
# #     #sat.complementary_filter(dt,0.9)

# #     #SMEKF

#     sat.smekf(dt)
#     sat.set_rpy_est(r2rpy(q2r(sat.q_est)))
#     sat.send_rpy()




import pigpio
import time

PWM_PIN = 18

pi = pigpio.pi()

if not pi.connected:
    raise RuntimeError("No conectado a pigpiod")

pi.set_PWM_frequency(PWM_PIN, 10000)
pi.set_PWM_range(PWM_PIN, 255)

print("25%")
pi.set_PWM_dutycycle(PWM_PIN, 220)
time.sleep(3)

print("50%")
pi.set_PWM_dutycycle(PWM_PIN, 200)
time.sleep(3)

print("75%")
pi.set_PWM_dutycycle(PWM_PIN, 240)
time.sleep(3)

print("STOP")
pi.set_PWM_dutycycle(PWM_PIN, 0)

pi.stop()
