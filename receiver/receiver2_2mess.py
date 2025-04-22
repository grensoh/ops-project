from machine import SPI, Pin, UART, SoftI2C
from rfm69 import RFM69
import time

#PARAMETRES -------------------------------------------------------------------------------------------
FREQ           = 433.1
ENCRYPTION_KEY = b"\x01\x02\x03\x04\x05\x06\x07\x08\x01\x02\x03\x04\x05\x06\x07\x08"
NODE_ID        = 100 # ID of this node (BaseStation)
led = Pin( 25, Pin.OUT )

#PARAMETRES RFM69 -------------------------------------------------------------------------------------
spi = SPI(0, sck=Pin(6), mosi=Pin(7), miso=Pin(4), baudrate=50000, polarity=0, phase=0, firstbit=SPI.MSB)
nss = Pin( 5, Pin.OUT, value=True )
rst = Pin( 3, Pin.OUT, value=False )

#CALCUL ALTITUDE --------------------------------------------------------------------------------------
#pb = pression au niveau de la mer (Pa) --> variable !!
#tb = température au niveau de la mer (K) --> variable !!
#lb = taux de chute de température standard (K/m) = -0,0065 K/m
#r = constante universelle des gaz = 8,31432 (N/m)/(mol.K)
#g0 = constante d'accélération gravitationnelle = 9,80665 m/s²
#m = masse molaire de l'air terrestre = 0,0289644 kg/mol

def calculate_altitude(pressure, pb=101325, tb=288, lb=-0.0065, r=8.31432, g0=9.80665, m=0.0289644):
    pressure = pressure * 100 #switching hPa to Pa
    cansat_height = (tb / lb) * ((pressure / pb) ** ((-r * lb) / (g0 * m)) - 1) #application of the formula
    return cansat_height

#INITIALISATION RFM69 --------------------------------------------------------------------------------
try:
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
except Exception as e:
    print(f"Erreur initialisation RFM69 : {e}")

#INITIALISATION UART --------------------------------------------------------------------------------
try:
    uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
except Exception as e:
    print(f"Erreur initialisation UART : {e}")


data = []

while True:
    try:
        packet = rfm.receive(timeout=0.5)
        if packet is None: # No packet received
            print( "." )
            pass
        else:
            led.on()
            #decode from ASCII text (to local utf-8)
            message = str(packet, "ascii")
            rssi = str(rfm.last_rssi) #signal strength
    
            message_parts = message.split(",")
            msgId = int(message_parts[1])
            if msgId == 1:
                if len(message_parts) == 9:
                    data = [message_parts[0]] + message_parts[2:9]
                else:
                    print("Invalid msg1 format")
            elif msgId == 2:
                if len(message_parts) == 9:
                    if len(data) > 0:
                        data += message_parts[2:9]
                    else:
                        print("Missing msg1 data")
                else:
                    print("Invalid msg2 format")

            # We receive all data (msg1 and msg2)
            if len(data) == 15:
                try:
                    pressure = float(data[2])  #pression = 2e élément
                    cansat_height = calculate_altitude(pressure)
                except ValueError:
                    print("Invalid pressure data")
                    cansat_height = "NA"
                print(f"{data}, {cansat_height} m, RSSI: {rssi}") # print message with signal strength and cansat height
                final_message = f"{data},{cansat_height}"

                try:
                    uart.write(f"{final_message}\n")
                except Exception as e:
                    print(f"Erreur lors de l'envoi UART : {e}")

                # Reception done - clear data to avoid mixing parts of different measurements
                data = []
                led.off()

    except Exception as e:
        print(f"Erreur dans la boucle de réception : {e}")
        led.off()
