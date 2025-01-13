from machine import SPI, I2C, Pin, ADC
from bme280 import BME280, BMP280_I2CADDR
import time

i2c = I2C(0, scl=Pin(9), sda=Pin(8)) # initialize the i2c bus on GP9 and GP8

bmp = BME280(i2c=i2c, address=BMP280_I2CADDR) # create a bmp object

#pb = pression au niveau de la mer (Pa) --> variable !!
#tb = température au niveau de la mer (K) --> variable !!
#lb = taux de chute de température standard (K/m) = -0,0065 K/m
#r = constante universelle des gaz = 8,31432 (N/m)/(mol.K)
#g0 = constante d'accélération gravitationnelle = 9,80665 m/s²
#m = masse molaire de l'air terrestre = 0,0289644 kg/mol

def calculate_altitude(pressure, pb=101325, tb=288, lb=-0,0065, r=8,31432, g0=9,80665, m=0,0289644):
    pressure = pressure * 100 #switching hPa to Pa
    cansat_height = (tb / lb) * ((pressure / pb) ** ((-r * lb) / (g0 * m)) - 1) #application of the formula
    return cansat_height

print("iteration_count, time_sec, pressure_hpa, bmp280_temp") # print header
counter = 1
ctime = time.time()

while True:
    temp, pressure, humidity =  bmp.raw_values # read BMP280: Temp, pressure (hPa), humidity
    msg = f"{counter}, {time.time()-ctime}, {pressure:.2f}, {temp:.2f}, {calculate_altitude(pressure)}"
    print(msg)
    counter += 1
    time.sleep(0.5)
