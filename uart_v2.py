import serial
from influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS
from time import sleep

token = "WIWH9gC1xGvkm5KsjOHXrPknWQUrmgRCS9qv-M0wLWnhAu5vSaOWuLXT6d2dY5X47PakyixfTiL0MUFPGMV_zg=="
org = "ops-project"
bucket = "ops-project-bucket"

def safe_float(value):
    try:
        return float(value)
    except (ValueError, TypeError):
        return None

#CONNEXION AU PORT SERIE ----------------------------------------------------------------------------------------
print("Connexion au port série ...")
sleep(5)
try:
    ser = serial.Serial(port="COM4", baudrate=115200, timeout=1)
except Exception as e:
    print(f"Erreur lors de l'ouverture du port série : {e}\nSolutions : fermer Thonny et vérifier le port COM")
    exit(1)

print("Lecture des données du Pico...")


while True:
    try:
        line = ser.readline().decode("utf-8").strip() #Extraction des données reçues
        print(line)
        if line:
            message_parts = line.split(",") #Découpage de la chaine de données
            print(f"Message : {message_parts}")
            pressure = safe_float(message_parts[1].strip())
            temperature = safe_float(message_parts[2].strip())
            gx = safe_float(message_parts[3].strip())
            gy = safe_float(message_parts[4].strip())
            gz = safe_float(message_parts[5].strip())
            full = safe_float(message_parts[6].strip())
            ir = safe_float(message_parts[7].strip())
            correction = safe_float(message_parts[8]).strip())
            yaw = safe_float(message_parts[9].strip())
            altitude = safe_float(message_parts[10].strip())
            #altitude = 1200.0  # debugging
            print(pressure, temperature, gx, gy, gz, full, ir, correction, yaw, altitude)
            record = {
                "measurement": "cansat",
                "tags": {"flight": "1", "env": "dev"},
                "fields": {
                    "pressure": pressure,
                    "temperature": temperature,
                    "gx": gx,
                    "gy": gy,
                    "gz": gz,
                    "full": full,
                    "ir": ir,
                    "correction": correction,
                    "yaw": yaw,
                    "altitude": altitude,
                },
            }

            try:
                # Initialize SYNCHRONOUS instance of WriteApi
                with InfluxDBClient(url="http://localhost:8086", token=token, org=org) as client:
                    write_api = client.write_api(write_options=SYNCHRONOUS)
                    write_api.write(bucket=bucket, org=org, record=record)
            except Exception as e:
                print(f"Erreur lors de l'écriture InfluxDB : {e}")
                sleep(1)

    except serial.SerialException as e:
        print(f"Erreur série : {e}")
        sleep(1)
    except UnicodeDecodeError:
        print("Erreur de décodage — ligne ignorée")
        continue
    except Exception as e:
        print(f"Erreur : {e}")
