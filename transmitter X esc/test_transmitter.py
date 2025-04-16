""" TRANSMITTER: BMP280, MPU6050, TSL2591, RFM69 & ESC AIKON """

import uasyncio as asyncio
import time
import math
from machine import SPI, I2C, Pin, ADC, SoftI2C, UART
from esc_control import set_speed, calibrate, arm
from bme280 import BME280, BMP280_I2CADDR
from tsl2591 import TSL2591
from MPU6050_lib import MPU6050
from rfm69 import RFM69

#DEBUG -----------------------------------------------------------------------------------------------------
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

#PARAMETRES -------------------------------------------------------------------------------------------------
NAME           = "OPS"
FREQ           = 433.1
ENCRYPTION_KEY = b"\x01\x02\x03\x04\x05\x06\x07\x08\x01\x02\x03\x04\x05\x06\x07\x08"
NODE_ID        = 120 # ID of this node
BASESTATION_ID = 100 # ID of the node (base station) to be contacted

#PARAMETRES RFM69 -------------------------------------------------------------------------------------------
# Buses & Pins
spi = SPI(0, sck=Pin(6), mosi=Pin(7), miso=Pin(4), baudrate=50000, polarity=0, phase=0, firstbit=SPI.MSB)
nss = Pin(5, Pin.OUT, value=True)
rst = Pin(3, Pin.OUT, value=False)

#I2C SETUP -------------------------------------------------------------------------------------------------
i2c_bmp = I2C(0, scl=Pin(9), sda=Pin(8))
i2c_mpu = SoftI2C(sda=Pin(0), scl=Pin(1), freq=400000)
i2c_tsl = SoftI2C(scl=Pin(15), sda=Pin(14), freq=100000)

#INITIALISATION RFM69 ----------------------------------------------------------------------------------------
try:
    rfm = RFM69(spi=spi, nss=nss, reset=rst)
    rfm.tx_power = 20
    rfm.bitrate = 4800
    rfm.frequency_deviation = 9500
    rfm.spi_write(0x1A, 0xF4)
    rfm.spi_write(0x19, 0xF4)
    rfm.frequency_mhz  = FREQ
    rfm.encryption_key = (ENCRYPTION_KEY)
    rfm.node           = NODE_ID # This instance is the node 120
    rfm.destination    = BASESTATION_ID # Send to specific node 100
    print("RFM69 initialisé")
except Exception as e:
    print(f"Erreur intialisation RFM69 : {e}")


#INITIALISATION CAPTEURS ------------------------------------------------------------------------------------
try:
    bmp = BME280(i2c=i2c_bmp, address=BMP280_I2CADDR)
except Exception as e:
    print(f"Erreur initialisation BMP280 : {e}")

try:
    imu = MPU6050(i2c_mpu)
except Exception as e:
    print(f"Erreur initialisation MPU6050 : {e}")

try:
    light_sensor = TSL2591(i2c_tsl)
except Exception as e:
    print(f"Erreur initialisation TSL2591 : {e}")
    
led = Pin(25, Pin.OUT)

#VARIABLES GLOBALES ----------------------------------------------------------------------------------------
state = {
    "yaw": 0.0,
    "full": 0,
    "ir": 0,
    "temp": None,
    "pressure": None,
    "humidity": None,
    "scan_data": [],
    "target_angle": None
}

#CONTROLEUR PID ---------------------------------------------------------------------------------------------
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.prev_error = 0

    def compute(self, target, current):
        error = target - current
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

pid = PID(3.5, 0.01, 1.2)

#CALIBRAGE GYROSCOPE ---------------------------------------------------------------------------------------
gyro_bias = [0, 0, 0]
def calibrate_gyro():
    print("Calibrage du gyroscope... Ne pas bouger le capteur.")
    for i in range(100):
        try:
            gyro_bias[0] += imu.gyro.x
            gyro_bias[1] += imu.gyro.y
            gyro_bias[2] += imu.gyro.z
        except Exception as e:
            print(f"Erreur de calibrage : {e}")
            gyro_bias = [0, 0, 0]
    gyro_bias[0] /= 100
    gyro_bias[1] /= 100
    gyro_bias[2] /= 100
    print("Calibrage terminé.")

#LECTURE DES CAPTEURS ---------------------------------------------------------------------------------------
async def read_sensors():
    filtre_complementaire = 0.9999999
    frequence = 0.5
    while True:
        try:
            ax = round(imu.accel.x, 2)
            ay = round(imu.accel.y, 2)
            az = round(imu.accel.z, 2)
            gx = round(imu.gyro.x - gyro_bias[0], 4)
            gy = round(imu.gyro.y - gyro_bias[1], 4)
            gz = round(imu.gyro.z - gyro_bias[2], 4)
        except Exception as e:
            print(f"Erreur extraction MPU6050 : {e}")
            ax, ay, az, gx, gy, gz = None, None, None, None, None, None

        if ay is not None and ax is not None:
            accel_yaw = math.atan2(ay, ax) * 180 / math.pi
        else:
            accel_yaw = 0
        if gz is not None:
            gyro_yaw = state["yaw"] + gz * frequence
        else:
            gyro_yaw = 0

        state["yaw"] = filtre_complementaire * gyro_yaw + (1 - filtre_complementaire) * accel_yaw

        try:
            lux, full, ir, visible = light_sensor.get_lux()
        except Exception as e:
            print(f"Erreur extraction TSL2591 : {e}")
            lux, full, ir, visible = None, None, None
            
        state["full"] = full
        state["ir"] = ir

        try:
            temp, pressure, humidity = bmp.raw_values
        except Exception as e:
            print(f"Erreur extraction BMP : {e}")
            temp, pressure, humidity = None, None, None
            
        state["temp"] = temp
        state["pressure"] = pressure
        state["humidity"] = humidity

        await asyncio.sleep(frequence)

#BALAYAGE LUMIERE ------------------------------------------------------------------------------------------------
async def scan_light():
    print("Début du balayage lumineux sur 360°...")
    state["scan_data"].clear()

    start_yaw = state["yaw"]
    target_yaw = (start_yaw + 360) % 360
    set_speed(30)  # Vitesse de rotation constante
    
    previous_yaw = start_yaw
    yaw_accumulated = 0
    last_sample_time = time.ticks_ms()

    while yaw_accumulated < 360:
        current_yaw = state["yaw"]
        delta_yaw = (current_yaw - previous_yaw + 540) % 360 - 180  # Pour gérer l'effet de passage 359 -> 0
        yaw_accumulated += abs(delta_yaw)
        previous_yaw = current_yaw

        # Échantillonnage toutes les 100ms
        if time.ticks_diff(time.ticks_ms(), last_sample_time) > 100:
            state["scan_data"].append((current_yaw, state["full"]))
            last_sample_time = time.ticks_ms()
        
        await asyncio.sleep(0.01)

    set_speed(0)
    print(f"Balayage terminé. {len(state['scan_data'])} points collectés.")

    if state["scan_data"]:
        state["target_angle"] = max(state["scan_data"], key=lambda x: x[1])[0]
        print("Angle avec la plus grande luminosité :", state["target_angle"])

#ALIGNEMENT VIA PID ---------------------------------------------------------------------------------------------
async def align_to_light():
    if state["target_angle"] is None:
        return
    print("Aligning to light source...")
    while True:
        current = state["yaw"]
        correction = pid.compute(state["target_angle"], current)
        correction = max(min(correction, 100), -100)
        set_speed(correction)
        if abs(current - state["target_angle"]) < 2.0:
            break
        await asyncio.sleep(0.05)
    set_speed(0)
    print("Aligned!")

#ENVOI DES DONNEES --------------------------------------------------------------------------------------------------
async def log_uart():
    counter = 0
    while True:
        msg = f"{counter},{state['yaw']},{state['full']},{state['temp']},{state['pressure']},{state['humidity']}"
        uart.write(msg + "\n")
        try:
            led.on()
            rfm.send(bytes(msg , "utf-8"))
            led.off()
        except Exception as e:
            print(f"Erreur lors de l'envoi des données : {e}")
            led.off()
        counter += 1
        await asyncio.sleep(0.5)

#BOUCLE PRINCIPALE -----------------------------------------------------------------------------------------------
async def main():
    calibrate()
    arm()
    calibrate_gyro()
    await scan_light()
    while True:
        await align_to_light()
        await asyncio.sleep(0.1)

#LANCEMENT DES TÂCHES --------------------------------------------------------------------------------------------
async def run_all():
    await asyncio.gather(
        read_sensors(),
        log_uart(),
        main()
    )

asyncio.run(run_all())
