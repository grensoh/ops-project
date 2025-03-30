from MPU6050_lib import MPU6050
import time
from machine import Pin, SoftI2C
from tsl2591 import TSL2591
import math

i2c_mpu6050 = SoftI2C(sda=Pin(0), scl=Pin(1), freq=400000)
imu = MPU6050(i2c_mpu6050)

i2c_tsl2591 = SoftI2C(scl=Pin(15), sda=Pin(14), freq=100000)
light_sensor = TSL2591(i2c_tsl2591)

# Calibrage du gyroscope
print("Calibrage du gyroscope... Ne pas bouger le capteur.")
time.sleep(2)
gyro_bias = [0, 0, 0]
for _ in range(100):
    gyro_bias[0] += imu.gyro.x
    gyro_bias[1] += imu.gyro.y
    gyro_bias[2] += imu.gyro.z
    time.sleep(0.01)
gyro_bias[0] /= 100
gyro_bias[1] /= 100
gyro_bias[2] /= 100
print("Calibrage terminé.")

frequence = 0.5 #fréquence de répétition de la boucle = fréquence de la mesure
yaw = 0.0
filtre_complementaire = 0.999999
luminosite_par_angle = []

while True:
    ax = round(imu.accel.x, 2)
    ay = round(imu.accel.y, 2)
    az = round(imu.accel.z, 2)
    gx = round(imu.gyro.x - gyro_bias[0])
    gy = round(imu.gyro.y - gyro_bias[1])
    gz = round(imu.gyro.z - gyro_bias[2], 4)
    
    lux, full, ir, visible = light_sensor.get_lux() #request des mesures du tsl2591
    
    accel_yaw = math.atan2(ay, ax) * 180 / math.pi #calcul de l'angle de yaw à partir de l'accéléromètre

    gyro_yaw = yaw + gz * frequence #intégration de la vitesse angulaire pour obtenir yaw

    yaw = filtre_complementaire * gyro_yaw + (1 - filtre_complementaire) * accel_yaw #filtre complémentaire
    
    timestamp = time.time()
    luminosite_par_angle.append({"timestamp": timestamp, "angle": yaw, "luminosite": full})
    print(f"Yaw: {yaw}, Full: {full:.2f}")
    time.sleep(frequence)
