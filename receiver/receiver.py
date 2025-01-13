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

rfm = RFM69( spi=spi, nss=nss, reset=rst )
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

        message_parts = message.split(",")
        if len(message_parts) >= 4:
            # Extract pressure (third element in the message)
            try:
                pressure = float(message_parts[2])  # Pressure is the third element
                pressure += 15  # Add 15 to the pressure value
                print(f"Pressure: {pressure:.2f} hPa (original + 15)")
            except ValueError:
                print("Invalid pressure data")
        else:
            print("Invalid message format")
            
        print(message + ", " + rssi) # print message with signal strength
        led.off()
