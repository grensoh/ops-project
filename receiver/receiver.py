from machine import SPI, Pin
from rfm69 import RFM69
import time

FREQ           = 435
ENCRYPTION_KEY = b"\x01\x02\x03\x04\x05\x06\x07\x08\x01\x02\x03\x04\x05\x06\x07\x08"
NODE_ID        = 100 # ID of this node (BaseStation)

spi = SPI(0, sck=Pin(6), mosi=Pin(7), miso=Pin(4), baudrate=50000, polarity=0, phase=0, firstbit=SPI.MSB)
nss = Pin( 5, Pin.OUT, value=True )
rst = Pin( 3, Pin.OUT, value=False )

led = Pin( 25, Pin.OUT )

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

rfm = RFM69( spi=spi, nss=nss, reset=rst )

rfm.tx_power = 20
rfm.bitrate = 4800
rfm.frequency_deviation = 9500
rfm.spi_write(0x1A, 0xF4)
rfm.spi_write(0x19, 0xF4)

rfm.frequency_mhz = FREQ
rfm.encryption_key = ( ENCRYPTION_KEY )
rfm.node = NODE_ID

print( 'Freq            :', rfm.frequency_mhz )
print( 'NODE            :', rfm.node )

print("Waiting for packets...")
while True:
    packet = rfm.receive( timeout=0.5 ) # Without ACK
    if packet is None: # No packet received
        print( "." )
        pass
    else: # Received a packet!
        led.on()
        # And decode from ASCII text (to local utf-8)
        message = str(packet, "ascii") # this is our message
        rssi = str(rfm.last_rssi) # signal strength

        message_parts = message.split(",") #2, 1, 1000; 9.02, 0.2, OPS
        if len(message_parts) >= 4:
            # Extract pressure (third element in the message)
            try:
                pressure = float(message_parts[2])  # Pressure is the third element
                cansat_height = calculate_altitude(pressure) #application of the formula
            except ValueError:
                print("Invalid pressure data")
        else:
            print("Invalid message format")
            
        print(f"{message}, {cansat_height} m, RSSI: {rssi}") # print message with signal strength and cansat height
        led.off()
