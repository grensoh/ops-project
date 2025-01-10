from machine import SPI, I2C, Pin, ADC
from bme280 import BME280, BMP280_I2CADDR
import time

i2c = I2C(0, scl=Pin(9), sda=Pin(8)) # initialize the i2c bus on GP9 and GP8

bmp = BME280(i2c=i2c, address=BMP280_I2CADDR) # create a bmp object

print("iteration_count, time_sec, pressure_hpa, bmp280_temp") # print header
counter = 1
ctime = time.time()

while True:
    temp, pressure, humidity =  bmp.raw_values # read BMP280: Temp, pressure (hPa), humidity
    msg = f"{counter}, {time.time()-ctime}, {pressure:.2f}, {temp:.2f}"
    print(msg)
    counter += 1
    time.sleep(0.5)
