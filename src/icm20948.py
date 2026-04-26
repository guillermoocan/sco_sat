from smbus2 import SMBus, i2c_msg
from struct import unpack_from
import time
import sys

LIBNAME = "ICM20948"

ICM_ADDRESS = 0x69
ICM_CHIP_ID = 0xEA
AK_I2C_ADDR = 0x0C
AK_CHIP_ID = 0x09


#Bank 0

ICM_WHO_AM_I = 0x00
ICM_BANK_SEL = 0x7F

ICM_PWR_MGMT_1 = 0x06
ICM_PWR_MGMT_1_RESET = 0x80
ICM_PWR_MGMT_1_CLOCK_AUTO = 0x01
ICM_PWR_MGMT_1_LP = 0x20

ICM_PWR_MGMT_2 = 0x07

ICM_USER_CTRL = 0x03
ICM_USER_CTRL_I2C_MST_EN = 0b00100000

ICM_EXT_SLV_SENS_DATA_00 = 0x3B # Linked to AK_ST1 (0x10) or AK_RSV2 (0x03)
ICM_EXT_SLV_SENS_DATA_01 = 0x3C # Linked to AK_HXL (0x11) or ??? (0x04)

ICM_DELAY_TIME_H = 0x28
ICM_ACCEL_XOUT_H = 0x2D
ICM_GYR_XOUT_H = 0x33


#Bank 2

ICM_GYR_SMPLRT_DIV = 0x00
ICM_GYR_CONFIG_1 = 0x01

ICM_ACCEL_SMPLRT_DIV_1 = 0x10
ICM_ACCEL_SMPLRT_DIV_2 = 0x11
ICM_ACCEL_CONFIG_1 = 0x14

#Bank 3

ICM_I2C_MST_CTRL = 0x01
ICM_I2C_MST_CTRL_NSR =   0b00010000 #Stop between reads

ICM_I2C_SLV0_ADDR = 0x03
ICM_I2C_SLV0_REG = 0x04
ICM_I2C_SLV0_CTRL = 0x05
ICM_I2C_SLV0_DO = 0x06


ICM_I2C_SLV_ADDR_RNW = 0x80
ICM_I2C_SLV_CTRL_SLV_ENABLE = 0x80
ICM_I2C_SLV_CTRL_BYTE_SWAP = 0x40
ICM_I2C_SLV_CTRL_REG_DIS = 0x20
ICM_I2C_SLV_CTRL_REG_GROUP = 0x10

#Magnetómetro AK09916

AK_WIA1 = 0x00
AK_WIA2 = 0x01
AK_RSV1 = 0x02
AK_RSV2 = 0x03 #Reserved register, used for DMP reading
AK_HXH_RSV = 0x04 # ? Hidden Magnetometer Data in Big Indian Format
AK_ST1 = 0x10
AK_ST1_DOR = 0b00000010   # Data overflow bit
AK_ST1_DRDY = 0b00000001  # Data self.ready bit
AK_HXL = 0x11
AK_ST2 = 0x18
AK_ST2_HOFL = 0b00001000   # Magnetic sensor overflow bit
AK_CNTL2 = 0x31
AK_CNTL2_MODE_OFF = 0
AK_CNTL2_MODE_SINGLE = 1
AK_CNTL2_MODE_10HZ  = 2
AK_CNTL2_MODE_20HZ  = 4
AK_CNTL2_MODE_50HZ  = 6
AK_CNTL2_MODE_100HZ = 8
AK_CNTL2_MODE_TEST = 16
AK_CNTL3 = 0x32
AK_CNTL3_RESET = 0x01

def sleep_ms(ms):
    time.sleep(ms / 1000)

def ticks_us():
    return time.time_ns() // 1000


class ICM20948:

    #Initialise the IMU
    def __init__(self, i2c):
    
        self._bus = i2c
        self._bank = 0
        self._addr = ICM_ADDRESS

        self._acc_bias = [0.0,0.0,0.0] # acc bias
        self._gyr_bias = [0.0,0.0,0.0] # gyr bias
        # self._mag_bias = [35.5750,   -5.6161,  -19.1681] # mag bias
        self._mag_bias = [40.9358,46.7353,-76.2205] # mag bias

        self._acc_gain = 1/16384
        self._gyr_gain = 1/131
        self._mag_gain = 0.15 

        # self._mag_cov = [[ 1.0186,    0.0105,    0.0416],[0.0105,    0.9981,   -0.0405],[0.0416,   -0.0405,    0.9871]]
        self._mag_cov = [[1,0,0],[0,1,0],[0,0,1]]
        self._acc_cov = [[1,0,0],[0,1,0],[0,0,1]]
        self._gyr_cov = [[1,0,0],[0,1,0],[0,0,1]]


        if self.read(0, ICM_WHO_AM_I) != ICM_CHIP_ID:
            raise RuntimeError("Unable to find ICM20948")
        else:
            print("Accelerometer and gyrscope found")
    
        #Primer paso: Reseteamos el chip 
        self.reg_config(0,ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_RESET, True) 
        sleep_ms(100)

        #Segundo paso: Asignamos reloj. En ese caso, vamos a asignar el auto clock para que elija el reloj interno. 

        self.write(0, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_CLOCK_AUTO)

        #Tercer paso: Encendemos acelerometro y giroscopio

        self.write(0, ICM_PWR_MGMT_2, 0x00)

        #Cuarto paso: Configuramos el I2C master interno del sensor. Lo necesitamos para leer el magnetómetro. Eno
        # NSR mete un stop entre las lecturas. Los bits menos significativos indican la velocidad del bus I2C interno. Según el datasheet, 0x07 (345.6 kHz / 46.67% duty cycle) es el valor recomendable. 

        self.write(3, ICM_I2C_MST_CTRL, ICM_I2C_MST_CTRL_NSR| 0x07 ) 

        #Quinto paso: Activamos el I2C interno

        self.reg_config(0, ICM_USER_CTRL, ICM_USER_CTRL_I2C_MST_EN, True)

        #Sexto paso: Configuramos el slave 0 del I2C interno para leer el magnetómetro
        
        self.slave0_config_read(AK_I2C_ADDR, AK_WIA2, 1)

    
        if self.read(0, ICM_EXT_SLV_SENS_DATA_00)  != AK_CHIP_ID:
            print( "Magnetometer not found")
        else :
            print("Magnetometer found ")
        sleep_ms(100)
        self.slave0_config_write(AK_I2C_ADDR, AK_CNTL3, 1, AK_CNTL3_RESET) # Mandamos el reset al magnetómetro
        self.slave0_config_read(AK_I2C_ADDR, AK_CNTL3, 1) # Mandamos el reset al magnetómetro
        while self.read(0, ICM_EXT_SLV_SENS_DATA_00) == 0x01: # Permanece hasta que se haya reseteado
            sleep_ms(10)            
    
        self._ready = True

        # Séptimo paso: Configuramos la taza de muestreo del magnetómetro

        self.slave0_config_write(AK_I2C_ADDR, AK_CNTL2, 1, AK_CNTL2_MODE_100HZ)
        self.slave0_config_read(AK_I2C_ADDR, AK_ST1, 9)

        
        # Octavo paso: Configuramos la taza de muestreo del giroscopio
        #Sample_rate = 1125 Hz / (1 + gyr_sample_rate_divider)

        self.write(2, ICM_GYR_SMPLRT_DIV, 0x0A) # 100 Hz

        #Noveno paso: Configuramos el rango y el ancho de banda del giroscopio

        self.write(2, ICM_GYR_CONFIG_1, 0x21) # 250 dps, 25 Hz bandwidth

        #Decimo paso: Configuramos la taza de muestreo del acelerómetro

        self.write(2, ICM_ACCEL_SMPLRT_DIV_1, 0x00)
        self.write(2, ICM_ACCEL_SMPLRT_DIV_2, 0X0A)

        #Onceavo paso: Configuramos el rango y el ancho de banda del acelerómetro

        self.write(2, ICM_ACCEL_CONFIG_1, 0x21) # 2 G, 25 Hz bandwidth
        
        
        #Doceavo paso: Mandamos el sensor a  baja potencia

        #self.reg_config(0,ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_LP, True)

        #print("ICM20948 initialized")


    def acc(self):

        data = self.read(0,ICM_ACCEL_XOUT_H, 6)

        ax, ay, az = unpack_from(">3h", data)
        ax *= self._acc_gain
        ay *= self._acc_gain
        az *= self._acc_gain 

        ax -= self._acc_bias[0]
        ay -= self._acc_bias[1]
        az -= self._acc_bias[2]

        self._acc = ax, ay, az
        return self._acc

    #Read gyroscope data
    def gyr(self):

        data = self.read(0, ICM_GYR_XOUT_H, 6)

        gx, gy, gz = unpack_from(">3h", data)

        gx *= self._gyr_gain
        gy *= self._gyr_gain      
        gz *= self._gyr_gain

        gx -= self._gyr_bias[0]
        gy -= self._gyr_bias[1]
        gz -= self._gyr_bias[2]

        self._gyr = gx, gy, gz

        return self._gyr
    

    #Read magnetometer data straight for slave DATA_01 (linked to AK_HXL)
    def mag(self):

        data = self.read(0, ICM_EXT_SLV_SENS_DATA_01, 6)
        mx_raw, my_raw, mz_raw = unpack_from("<3h", data)

        mx = mx_raw
        my = -my_raw
        mz = -mz_raw

        mx *= self._mag_gain
        my *= self._mag_gain
        mz *= self._mag_gain
    
        mx -= self._mag_bias[0]
        my -= self._mag_bias[1]
        mz -= self._mag_bias[2]
    
        self._mag = mx, my, mz
    
        mx_d=self._mag_cov[0][0]*mx + self._mag_cov[0][1]*my + self._mag_cov[0][2]*mz
        my_d=self._mag_cov[1][0]*mx + self._mag_cov[1][1]*my + self._mag_cov[1][2]*mz
        mz_d=self._mag_cov[2][0]*mx + self._mag_cov[2][1]*my + self._mag_cov[2][2]*mz

        self._mag = mx_d, my_d, mz_d
        return self._mag
    
    def acc_cal(self,timeout=2000):
        acc_bias = [0.0,0.0,0.0]
        it=0
        lasttime = ticks_us()
        while ((ticks_us() - lasttime) < timeout) :
            ax, ay, az = self.acc()
            acc_bias[0] += ax
            acc_bias[1] += ay
            acc_bias[2] += az
            it += 1
            sleep_ms(1)
        self._acc_bias[0] = (acc_bias[0] / it) 
        self._acc_bias[1] = (acc_bias[1] / it) 
        self._acc_bias[2] = (acc_bias[2] / it) - 1.0


    def gyr_cal(self,timeout=2000):

        gyr_bias = [0.0,0.0,0.0]
        ite=0
        lt = ticks_us()
        while ((ticks_us() - lt) < timeout) :
            gx, gy, gz = self.gyr()
            gyr_bias[0] += gx
            gyr_bias[1] += gy
            gyr_bias[2] += gz
            ite += 1
            sleep_ms(1)
        self._gyr_bias[0] = (gyr_bias[0] / ite) 
        self._gyr_bias[1] = (gyr_bias[1] / ite) 
        self._gyr_bias[2] = (gyr_bias[2] / ite)


    def gyr_cal_b(self,timeout=2000):

        gyr_bias = [0.0,0.0,0.0]
        ite=0
        lt = ticks_us()
        while ((ticks_us() - lt) < timeout) :
            gx, gy, gz = self.gyr()
            gyr_bias[0] += gx
            gyr_bias[1] += gy
            gyr_bias[2] += gz
            ite += 1
            sleep_ms(1)
        gyr_bias[0] = (gyr_bias[0] / ite) 
        gyr_bias[1] = (gyr_bias[1] / ite) 
        gyr_bias[2] = (gyr_bias[2] / ite)

        return gyr_bias



    def read(self, bank, reg, length=1):
        self.bank(bank)

        write_msg = i2c_msg.write(
            self._addr,
            [reg]
        )

        read_msg = i2c_msg.read(
            self._addr,
            length
        )

        self._bus.i2c_rdwr(write_msg, read_msg)

        data = bytearray(read_msg)

        if length == 1:
            return data[0]

        return data


    def write(self, bank, reg, value):
        self.bank(bank)

        write_msg = i2c_msg.write(
            self._addr,
            [reg, value]
        )

        self._bus.i2c_rdwr(write_msg)


    def bank(self, bank):
        if self._bank != bank:

            write_msg = i2c_msg.write(
                self._addr,
                [ICM_BANK_SEL, bank << 4]
            )

            self._bus.i2c_rdwr(write_msg)

            self._bank = bank
            
    def reg_config(self, bank, reg, ctrl, enable=True): # Permite escribir en una sección particular del registro sin afectar el resto de bytes del mismo. 
        self.bank(bank)
        value = self.read(bank, reg)
        if enable :
            value |= ctrl
        else :
            value &= ~ctrl
        self.write(bank, reg, value)
 
    def slave0_config_write(self, address, reg, length, data):
        self.write(3, ICM_I2C_SLV0_ADDR, address) # Asignamos la dirección del magnetómetro
        self.write(3, ICM_I2C_SLV0_REG, reg)
        self.write(3, ICM_I2C_SLV0_CTRL, ICM_I2C_SLV_CTRL_SLV_ENABLE | length)  # Activamos el slave y le indicamos que lea 1 byte
        self.write(3, ICM_I2C_SLV0_DO, data)
        sleep_ms(10)

    def slave0_config_read(self, address, reg, length):
        self.write(3, ICM_I2C_SLV0_ADDR, ICM_I2C_SLV_ADDR_RNW | address) # Asignamos la dirección del magnetómetro
        self.write(3, ICM_I2C_SLV0_REG, reg)
        self.write(3, ICM_I2C_SLV0_CTRL, ICM_I2C_SLV_CTRL_SLV_ENABLE | length)  # Activamos el slave y le indicamos que lea 1 byte
        sleep_ms(10)

    def send_imu(self):

        ax, ay, az = self.acc()
        gx, gy, gz = self.gyr()
        mx, my, mz = self.mag()

        print(f"{ax:.3f},{ay:.3f},{az:.3f},{gx:.3f},{gy:.3f},{gz:.3f},{mx:.3f},{my:.3f},{mz:.3f}")
    def print_imu(self):

        a = self.acc()
        g = self.gyr()
        m = self.mag()

        # Normas (L2)
        na = (a[0]**2 + a[1]**2 + a[2]**2)**0.5
        ng = (g[0]**2 + g[1]**2 + g[2]**2)**0.5
        nm = (m[0]**2 + m[1]**2 + m[2]**2)**0.5

        line = (
            f"Ax:{a[0]:7.4f} Ay:{a[1]:7.4f} Az:{a[2]:7.4f} | "
            f"|a|:{na:7.4f} || "
            f"Gx:{g[0]:7.4f} Gy:{g[1]:7.4f} Gz:{g[2]:7.4f} | "
            f"|g|:{ng:7.4f} || "
            f"Mx:{m[0]:7.4f} My:{m[1]:7.4f} Mz:{m[2]:7.4f} | "
            f"|m|:{nm:7.4f}\r"
        )

        sys.stdout.write(line)
        sys.stdout.flush()
