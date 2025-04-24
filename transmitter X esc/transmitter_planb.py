""" TRANSMITTER: BMP280, MPU6050, TSL2591, RFM69 & ESC AIKON """

import uasyncio as asyncio
import time
import math
from machine import SPI, I2C, Pin, ADC, SoftI2C, UART
from esc import set_motor
from bme280 import BME280, BMP280_I2CADDR
from tsl2591 import TSL2591
from imu import MPU6050
from rfm69 import RFM69
import sdcard
import uos 

#DEBUG -----------------------------------------------------------------------------------------------------
uart = UART(0, baudrate=115200, tx=Pin(12), rx=Pin(13))

#PARAMETRES -------------------------------------------------------------------------------------------------
NAME           = "OPS"
FREQ           = 431
CSV_PATH       = "/sd/data.csv"
ENCRYPTION_KEY = b"\x01\x02\x03\x04\x05\x06\x07\x08\x01\x02\x03\x04\x05\x06\x07\x08"
NODE_ID        = 120 # ID of this node
BASESTATION_ID = 100 # ID of the node (base station) to be contacted

#PARAMETRES RFM69 -------------------------------------------------------------------------------------------
# Buses & Pins
spi = SPI(1, sck=Pin(10), mosi=Pin(11), miso=Pin(12), baudrate=50000, polarity=0, phase=0, firstbit=SPI.MSB)
nss = Pin(13, Pin.OUT, value=True)
rst = Pin(3, Pin.OUT, value=False)

#I2C SETUP -------------------------------------------------------------------------------------------------
i2c_bmp = I2C(0, scl=Pin(9), sda=Pin(8))
i2c_mpu = SoftI2C(sda=Pin(0), scl=Pin(1), freq=400000)
i2c_tsl = SoftI2C(scl=Pin(15), sda=Pin(14), freq=100000)

#FONCTION CARTE SD -----------------------------------------------------------------------------------------
def setup_sd():
    # Assign chip select (CS) pin (and start it high)
    cs = machine.Pin(17, machine.Pin.OUT)
    # Intialize SPI peripheral (start with 1 MHz)
    spi = machine.SPI(0,
                      baudrate=1000000,
                      polarity=0,
                      phase=0,
                      bits=8,
                      firstbit=machine.SPI.MSB,
                      sck=machine.Pin(18),
                      mosi=machine.Pin(19),
                      miso=machine.Pin(16))
    return sdcard.SDCard(spi, cs)

#INITIALISATION SDCARD ----------------------------------------------------------------------------------------
csv_file = None
try:
    # Initialize SD card
    sd = setup_sd()
    # Mount filesystem
    vfs = uos.VfsFat(sd)
    uos.mount(vfs, "/sd")
    # Create a file and write something to it
    #with open("/sd/test01.txt", "r") as file:
    #    print(f"READ: {file.read()}")
    csv_file = open(CSV_PATH, "w")
except Exception as e:
    print(f"Erreur initalisation SD or csv : {e}")
    
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
gyro_bias = [0,0,0]
state = {
    "yaw": 0.0,
    "full": 0,
    "ir": 0,
    "temp": None,
    "pressure": None,
    "humidity": None,
    "scan_data": [],
    "target_angle": None,
    "correction": 0.0,
    "ax": None,
    "ay": None,
    "az": None,
    "gx": None,
    "gy": None,
    "gz": None,
    "aligned": False,
}

#NETTOYAGE VALEURS ------------------------------------------------------------------------------------------
def safe_value(value, default="NA"):
    return value if value is not None else default

#CALIBRAGE GYROSCOPE ---------------------------------------------------------------------------------------
def calibrate_gyro():
    global gyro_bias
    print("Initialisation... Attente avant calibrage du gyroscope.")
    for i in range(5, 0, -1):
        print(f"Calibrage dans {i} secondes...")
        led.on()
        time.sleep(1)
        led.off()
    print("Calibrage du gyroscope... Ne pas bouger le capteur.")
    gyro_bias = [0, 0, 0]
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

#CALCUL ALTITUDE --------------------------------------------------------------------------------------
#pb = pression au niveau de la mer (Pa) --> variable !!
#tb = température au niveau de la mer (K) --> variable !!
#lb = taux de chute de température standard (K/m) = -0,0065 K/m
#r = constante universelle des gaz = 8,31432 (N/m)/(mol.K)
#g0 = constante d'accélération gravitationnelle = 9,80665 m/s²
#m = masse molaire de l'air terrestre = 0,0289644 kg/mol

def calculate_altitude(pressure):
    pb=101325
    tb=288
    lb=-0.0065
    r=8.31432
    g0=9.80665
    m=0.0289644
    try:
        pressure = pressure * 100 #switching hPa to Pa
        altitude = (tb / lb) * ((pressure / pb) ** ((-r * lb) / (g0 * m)) - 1) #application of the formula
        return altitude
    except Exception as e:
        print(f"Erreur dans le calcul de l'altitude : {e}")
        return "NA"
        
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
            state["ax"] = ax
            state["ay"] = ay
            state["az"] = az
            state["gx"] = gx
            state["gy"] = gy
            state["gz"] = gz
        except Exception as e:
            print(f"Erreur extraction MPU6050 : {e}")

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
            state["full"] = full
            state["ir"] = ir
        except Exception as e:
            print(f"Erreur extraction TSL2591 : {e}")
            
        try:
            temp, pressure, humidity = bmp.raw_values
            state["temp"] = temp
            state["pressure"] = pressure
            state["humidity"] = humidity
        except Exception as e:
            print(f"Erreur extraction BMP : {e}")

        await asyncio.sleep(frequence)

#ALTITUDE CHECKER ---------------------------------------------------------------------------------------------------
scan_triggered = False
initial_altitude = None
async def monitor_altitude_change():
    global initial_altitude, scan_triggered
    while scan_triggered is not True:
        if not scan_triggered and state["pressure"] is not None:
            try:
                current_altitude = calculate_altitude(state["pressure"])
            except Exception as e:
                print(f"Impossible de calculer l'altitude : {e}")
                current_altitude = "NA"
            if initial_altitude is None:
                initial_altitude = current_altitude
            elif current_altitude != "NA":
                delta = initial_altitude - current_altitude
                if delta >= 0.5: #debug -> 0.5 et vraie valeur -> 15
                    print(f"Perte d'altitude détectée : {delta:.2f} m")
                    scan_triggered = True  # Empêche les déclenchements suivants
                    asyncio.create_task(rotation_frequence())
        await asyncio.sleep(0.5)

#ENVOI DES DONNEES --------------------------------------------------------------------------------------------------
async def transmitting():
    counter = 0
    while True:
        timestamp = time.time()
        msg_rfm = f"{counter},{safe_value(state["pressure"])},{safe_value(state["temp"])},{safe_value(state["gx"])},{safe_value(state["gy"])},{safe_value(state["gz"])},{safe_value(state["full"])},{safe_value(state["ir"])},{safe_value(state["correction"])},{safe_value(state["yaw"])}"
        msg_sdcard = f"{counter},{timestamp},{safe_value(state["pressure"])},{safe_value(state["temp"])},{safe_value(state["humidity"])},{safe_value(state["ax"])},{safe_value(state["ay"])},{safe_value(state["az"])},{safe_value(state["gx"])},{safe_value(state["gy"])},{safe_value(state["gz"])},{safe_value(state["full"])},{safe_value(state["ir"])},{safe_value(state["correction"])},{safe_value(state["yaw"])}"
        #uart.write(f"{msg}\n")
        print(msg_sdcard)
        print(msg_rfm)
        try:
            led.on()
            rfm.send(bytes(msg_rfm , "utf-8"))
            led.off()
        except Exception as e:
            print(f"Erreur lors de l'envoi des données : {e}")
            led.off()

        if csv_file is not None:
            try:
                csv_file.write(f"{msg_sdcard}\n")
                if counter % 10 == 0:
                    csv_file.flush()
            except Exception as e:
                print(f"Erreur écriture sur la carte SD : {e}")
                continue
                  
            
        counter += 1
        await asyncio.sleep(0.5)

#BOUCLE PRINCIPALE -----------------------------------------------------------------------------------------------
async def launch_motor(direction):
  if direction == 1:
    print("Rotation dans le sens horaire...")
    sleep_time = 8
  elif direction == -1:
    print("Rotation dans le sens antihoraire...")
    sleep_time = 8
  elif direction == 0:
    print("Arrêt...")
    sleep_time = 4
  else:
    print("Direction invalide")

  try:
    set_motor(direction)
    state["correction"] = direction
    await asyncio.sleep(sleep_time)
  except Exception as e:
    print(f"Erreur moteur : {e}")

async def rotation_sequence():
    await asyncio.sleep(1)
    while True:
      launch_motor(0)
      launch_motor(1)
      launch_motor(0)
      launch_motor(-1)

#LANCEMENT DES TÂCHES --------------------------------------------------------------------------------------------
async def run_all():
    calibrate_gyro()
    
    asyncio.create_task(read_sensors())
    asyncio.create_task(transmitting())
    asyncio.create_task(monitor_altitude_change()) 

asyncio.run(run_all())
