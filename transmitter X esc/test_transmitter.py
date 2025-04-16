import uasyncio as asyncio
import time
import math
from machine import Pin, I2C, SoftI2C, UART
from esc_control import set_speed, calibrate, arm
from bme280 import BME280, BMP280_I2CADDR
from tsl2591 import TSL2591
from MPU6050_lib import MPU6050

#UART DEBUG -------------------------------------------------------------------------
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

# I2C Setup
i2c_bmp = I2C(0, scl=Pin(9), sda=Pin(8))
i2c_mpu = SoftI2C(sda=Pin(0), scl=Pin(1), freq=400000)
i2c_tsl = SoftI2C(scl=Pin(15), sda=Pin(14), freq=100000)

# Capteurs
bmp = BME280(i2c=i2c_bmp, address=BMP280_I2CADDR)
imu = MPU6050(i2c_mpu)
light_sensor = TSL2591(i2c_tsl)
led = Pin(25, Pin.OUT)

# Variables Globales
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

# PID Controller
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

# Gyro calibrage
gyro_bias = [0, 0, 0]
def calibrate_gyro():
    for _ in range(100):
        gyro_bias[0] += imu.gyro.x
        gyro_bias[1] += imu.gyro.y
        gyro_bias[2] += imu.gyro.z
        time.sleep(0.01)
    for i in range(3):
        gyro_bias[i] /= 100

# Lecture capteurs
async def read_sensors():
    filtre_complementaire = 0.9999
    frequence = 0.1
    while True:
        try:
            ax = imu.accel.x
            ay = imu.accel.y
            gz = imu.gyro.z - gyro_bias[2]
            accel_yaw = math.atan2(ay, ax) * 180 / math.pi
            gyro_yaw = state["yaw"] + gz * frequence
            state["yaw"] = filtre_complementaire * gyro_yaw + (1 - filtre_complementaire) * accel_yaw

            lux, full, ir, _ = light_sensor.get_lux()
            state["full"] = full
            state["ir"] = ir
            temp, pressure, humidity = bmp.raw_values
            state["temp"] = temp
            state["pressure"] = pressure
            state["humidity"] = humidity
        except Exception as e:
            print(f"Erreur sensors: {e}")
        await asyncio.sleep(frequence)

# Scan
async def scan_light():
    print("Scanning light...")
    state["scan_data"].clear()
    set_speed(30)
    t0 = time.time()
    while time.time() - t0 < 5:
        state["scan_data"].append((state["yaw"], state["full"]))
        await asyncio.sleep(0.1)
    set_speed(0)
    if state["scan_data"]:
        state["target_angle"] = max(state["scan_data"], key=lambda x: x[1])[0]
        print("Best light angle:", state["target_angle"])

# Alignement PID
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

# UART
async def log_uart():
    counter = 0
    while True:
        msg = f"{counter},{state['yaw']},{state['full']},{state['temp']},{state['pressure']},{state['humidity']}"
        uart.write(msg + "\n")
        counter += 1
        await asyncio.sleep(0.5)

# Main loop
async def main():
    calibrate()
    arm()
    calibrate_gyro()
    while True:
        await scan_light()
        await align_to_light()
        await asyncio.sleep(5)

# Launch tasks
async def run_all():
    await asyncio.gather(
        read_sensors(),
        log_uart(),
        main()
    )

asyncio.run(run_all())
