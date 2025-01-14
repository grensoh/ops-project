from machine import SPI, I2C, Pin, ADC
from rfm69 import RFM69
from bme280 import BME280, BMP280_I2CADDR
import time

NAME           = "OPS"
FREQ           = 435

ENCRYPTION_KEY = b"\x01\x02\x03\x04\x05\x06\x07\x08\x01\x02\x03\x04\x05\x06\x07\x08"
NODE_ID        = 120 # ID of this node
BASESTATION_ID = 100 # ID of the node (base station) to be contacted

# Buses & Pins
spi = SPI(0, sck=Pin(6), mosi=Pin(7), miso=Pin(4), baudrate=50000, polarity=0, phase=0, firstbit=SPI.MSB)
nss = Pin(5, Pin.OUT, value=True)
rst = Pin(3, Pin.OUT, value=False)
i2c = I2C(0, scl=Pin(9), sda=Pin(8)) # initialize the i2c bus on GP9 and GP8

# RFM Module
rfm = RFM69(spi=spi, nss=nss, reset=rst)
rfm.frequency_mhz  = FREQ
rfm.encryption_key = (ENCRYPTION_KEY)
rfm.node           = NODE_ID # This instance is the node 120
rfm.destination    = BASESTATION_ID # Send to specific node 100

bmp = BME280(i2c=i2c, address=BMP280_I2CADDR) # create a bmp object
led = Pin(25, Pin.OUT) # Onboard LED

# Main Loop
print( 'Frequency     :', rfm.frequency_mhz )
print( 'encryption    :', rfm.encryption_key )
print( 'NODE_ID       :', NODE_ID )
print( 'BASESTATION_ID:', BASESTATION_ID )

print("iteration_count, time_sec, pressure_hpa, bmp280_temp") # print header
counter = 1 # set counter
ctime = time.time() # time now

while True:
    try:
        temp, pressure, humidity =  bmp.raw_values # read BMP280: Temp, pressure (hPa), humidity
    except OSError as e:
        print(f"BMP280 sensor reading error : {e}")
        temp, pressure, humidity = None, None, None

    if temp is not None and pressure is not None and humidity is not None:
        msg = f"{counter}, {time.time()-ctime}, {pressure:.2f}, {temp:.2f}, {humidity:.2f}, {NAME}"
        print(msg)
        led.on() # Led ON while sending data
        try:
            rfm.send(bytes(msg , "utf-8")) #trying to send data
        except Exception as e:
            print("Radio transmission error : {e}")
        led.off()
    else:
        print("Invalid data, sending aborted")
        
counter += 1 # increment counter
time.sleep(0.5) # wait before next reading
