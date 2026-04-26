from smbus2 import SMBus
import time

I2C_BUS = 1
ADDR = 0x69

# Registros típicos ICM-20948
REG_BANK_SEL = 0x7F
WHO_AM_I = 0x00

def select_bank(bus, bank):
    bus.write_byte_data(ADDR, REG_BANK_SEL, bank << 4)

with SMBus(I2C_BUS) as bus:
    try:
        # Banco 0
        select_bank(bus, 0)

        who = bus.read_byte_data(ADDR, WHO_AM_I)

        print(f"WHO_AM_I: 0x{who:02X}")

        if who == 0xEA:
            print("Detectado: ICM-20948")
        else:
            print("Dispositivo detectado, pero no coincide con ICM-20948")

    except Exception as e:
        print("Error:", e)