from machine import SPI, Pin, UART, SoftI2C
from rfm69 import RFM69
import time

#PARAMETRES -------------------------------------------------------------------------------------------
FREQ           = 431
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

#FONCTION POUR ASSEMBLER LE MSG ---------------------------------------------------------------------
fragment_buffer = {}

def assemble_message(received_fragments, total):
    elements = []
    for i in range(1, total + 1):
        part = received_fragments.get(i)
        if part:
            cleaned_part = part.strip().strip(",")
            elements.extend(cleaned_part.split(","))
        else:
            elements_per_fragment = 10 // total
            elements.extend(["NA"] * elements_per_fragment)
    return elements

#FONCTION POUR GERER LES FRAGMENTS ------------------------------------------------------------------
def handle_fragment(data, rssi):
    try:
        text = data.decode("utf-8")
        header, content = text.split(":", 1)
        index, total = map(int, header.split("/"))

        message_id = "default"

        # Initialisation du buffer de fragments pour ce message_id
        if message_id not in fragment_buffer:
            fragment_buffer[message_id] = {
                "received": {},
                "start_time": time.time()
            }

        # Ajout du fragment reçu
        fragment_buffer[message_id]["received"][index] = content.strip()

        received = fragment_buffer[message_id]["received"]
        
        # Vérifier si tous les fragments ont été reçus ou si timeout est atteint
        if len(received) == total:
            full_message = assemble_message(received, total)
            print(f"[MESSAGE COMPLET] {','.join(full_message)}")

            try:
                pressure = float(full_message[1])  # pression = 2e élément
                cansat_height = calculate_altitude(pressure)
                cansat_height = str(cansat_height)
            except ValueError:
                print("Données de pression invalides.")
                cansat_height = "NA"

            final_message = f"{','.join(full_message)},{cansat_height}"

            try:
                uart.write(f"{final_message}\n")
            except Exception as e:
                print(f"Erreur lors de l'envoi UART : {e}")

            del fragment_buffer[message_id]
            
        elif time.time() - fragment_buffer[message_id]["start_time"] > 0.5:
            # Si le timeout est atteint, on assemble le message avec des "NA" pour les fragments manquants
            full_message = assemble_message(received, total)
            print(f"[MESSAGE INCOMPLET - Timeout] {','.join(full_message)}")

            try:
                pressure = float(full_message[1])  # pression = 2e élément
                cansat_height = calculate_altitude(pressure)
                cansat_height = str(cansat_height)
            except ValueError:
                print("Données de pression invalides.")
                cansat_height = "NA"

            final_message = f"{','.join(full_message)},{cansat_height},{rssi}"

            # Envoi du message final incomplet par UART
            try:
                uart.write(f"{final_message}\n")
            except Exception as e:
                print(f"Erreur lors de l'envoi UART : {e}")

            del fragment_buffer[message_id]

    except Exception as e:
        print(f"Erreur lors du traitement du fragment : {e}")



while True:
    try:
        packet = rfm.receive(timeout=0.5)
        if packet is None: # No packet received
            print( "." )
            pass    
        else:
            led.on()
            message = packet  # Les données sont déjà en bytes
            rssi = str(rfm.last_rssi)  # signal strength
            handle_fragment(message, rssi)
            led.off()
    except Exception as e:
        print(f"Erreur dans la boucle de réception : {e}")
        led.off()
