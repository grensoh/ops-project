""" Transmitter BMP280, MPU6050, TSL2591, RFM69 """

from machine import SPI, I2C, Pin, ADC, SoftI2C, UART
from rfm69 import RFM69
from bme280 import BME280, BMP280_I2CADDR
from tsl2591 import TSL2591
from MPU6050_lib import MPU6050
import time
import math

#DEBUG -----------------------------------------------------------------------------------------------------
#uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

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

#I2C --------------------------------------------------------------------------------------------------------
i2c_bmp = I2C(0, scl=Pin(9), sda=Pin(8)) # initialize the i2c bus on GP9 and GP8
i2c_mpu6050 = SoftI2C(sda=Pin(0), scl=Pin(1), freq=400000)
i2c_tsl2591 = SoftI2C(scl=Pin(15), sda=Pin(14), freq=100000)
led = Pin(25, Pin.OUT)

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
    print("RFM60 initialisé")
except Exception as e:
    print(f"Erreur intialisation RFM69 : {e}")

#INITIALISATION BMP280 & MPU6050 & TSL2591 --------------------------------------------------------------------
try:
    bmp = BME280(i2c=i2c_bmp, address=BMP280_I2CADDR)
except Exception as e:
    print("Erreur initialisation BMP280 : {e}")

try:
    imu = MPU6050(i2c_mpu6050)
except Exception as e:
    print(f"Erreur initialisation MPU6050 : {e}")
    
try:
    light_sensor = TSL2591(i2c_tsl2591)
except Exception as e:
    print(f"Erreur initialisation TSL2591 : {e}")

def safe_value(value, default="NA"):
    return value if value is not None else default

# ATTENTE AVANT CALIBRAGE ---------------------------------------------------------------------------------
print("Initialisation... Attente avant calibrage du gyroscope.")
for i in range(5, 0, -1):
    print(f"Calibrage dans {i} secondes...")
    led.on()
    time.sleep(1)
led.off()
print("Début du calibrage du gyroscope.")

#CALIBRAGE GYROSCOPE ------------------------------------------------------------------------------------------
gyro_bias = [0, 0, 0]
print("Calibrage du gyroscope... Ne pas bouger le capteur.")
for i in range(100):
    try:
        gyro_bias[0] += imu.gyro.x
        gyro_bias[1] += imu.gyro.y
        gyro_bias[2] += imu.gyro.z
    except Exception as e:
        print(f"Erreur de calibrage : {e}")
        gyro_bias = [0, 0, 0]
    time.sleep(0.01)
gyro_bias[0] /= 100
gyro_bias[1] /= 100
gyro_bias[2] /= 100
print("Calibrage terminé.")

#VARIABLES ----------------------------------------------------------------------------------------------------
frequence = 0.5 #fréquence de répétition de la boucle = fréquence de la mesure
yaw = 0.0 #angle
filtre_complementaire = 0.9999999 #coefficient du filtre
luminosite_par_angle = []
counter = 1 # set counter

#HEADER -------------------------------------------------------------------------------------------------------
print( 'Frequency     :', rfm.frequency_mhz )
print( 'encryption    :', rfm.encryption_key )
print( 'NODE_ID       :', NODE_ID )
print( 'BASESTATION_ID:', BASESTATION_ID )
print("Counter, timestamp, pressure, temp, humidity, ax, ay, az, gx, gy, gz, full, ir, yaw")

#BOUCLE PRINCIPALE ----------------------------------------------------------------------------------------------
while True:
    
    #EXTRACTION DU MPU6050 -------------------------------------------------------------------------------------
    try:
        ax = round(imu.accel.x, 2)
        ay = round(imu.accel.y, 2)
        az = round(imu.accel.z, 2)
        gx = round(imu.gyro.x - gyro_bias[0], 2)
        gy = round(imu.gyro.y - gyro_bias[1], 2)
        gz = round(imu.gyro.z - gyro_bias[2], 4)
    except Exception as e:
        print(f"Erreur dans la lecture du MPU6050 : {e}")
        ax, ay, az, gx, gy, gz = None, None, None, None, None, None
    
    #CALCUL DE YAW ---------------------------------------------------------------------------------------------
    if ay is not None and ax is not None:
        accel_yaw = math.atan2(ay, ax) * 180 / math.pi #calcul de l'angle de yaw à partir de l'accéléromètre
    else:
        print("accel_yaw = O")
        accel_yaw = 0
        
    if gz is not None:
        gyro_yaw = yaw + gz * frequence #intégration de la vitesse angulaire pour obtenir yaw
    else:
        gyro_yaw = 0
        
    yaw = filtre_complementaire * gyro_yaw + (1 - filtre_complementaire) * accel_yaw #filtre complémentaire
    
    #EXTRACTION DU TSL2591 -------------------------------------------------------------------------------------
    try:
        lux, full, ir, visible = light_sensor.get_lux() #request des mesures du tsl2591
    except Exception as e:
        print(f"Erreur dans la lecture du TSL2591 : {e}")
        lux, full, ir, visible = None, None, None, None
    
    #EXTRACTION DU BMP280 -------------------------------------------------------------------------------------
    try:
        temp, pressure, humidity = bmp.raw_values
    except Exception as e:
        print(f"Erreur dans la lecture du BMP280 : {e}")
        temp, pressure, humidity = None, None, None
    
    #MESSAGE BUILDING -----------------------------------------------------------------------------------------
    timestamp = time.time()
    msg = f"{counter},{safe_value(pressure)},{safe_value(temp)},{safe_value(humidity)},{safe_value(ax)},{safe_value(ay)},{safe_value(az)},{safe_value(gx)},{safe_value(gy)},{safe_value(gz)},{safe_value(full)},{safe_value(ir)},{safe_value(yaw)}"
    print(f"""
--- Données transmises ---
Counter     : {counter}
Timestamp   : {timestamp}
Pressure    : {safe_value(pressure)}
Température : {safe_value(temp)}
Humidité    : {safe_value(humidity)}
Acc X       : {safe_value(ax)}
Acc Y       : {safe_value(ay)}
Acc Z       : {safe_value(az)}
Gyro X      : {safe_value(gx)}
Gyro Y      : {safe_value(gy)}
Gyro Z      : {safe_value(gz)}
Lum. Full   : {safe_value(full)}
Lum. IR     : {safe_value(ir)}
Yaw         : {safe_value(yaw)}
""")
    
    #TRANSMISSION DU MESSAGE ----------------------------------------------------------------------------------
    #uart.write(f"{msg}\n")
    try:
        led.on()
        rfm.send(bytes(msg , "utf-8"))
        led.off()
    except Exception as e:
        print(f"Erreur lors de l'envoi des données : {e}")
        led.off()
    
    counter += 1
    time.sleep(frequence)
    #luminosite_par_angle.append({"timestamp": timestamp, "angle": yaw, "luminosite": full})
