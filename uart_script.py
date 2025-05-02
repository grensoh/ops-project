import serial
from influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS
from flask import Flask, jsonify
from flask_cors import CORS
import threading
import time

token = "L7R-9LvFXqjaBENC6qWpZ9b0HyD7sSM53BurV4aTpEfJoJgjMfwzdy_QSoOzTRznlysPwkXw808fIHJnC4sG-g=="
bucket = "ops-project-bucket"
org = "ops-project"

latest_orientation = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}

app = Flask(__name__)
CORS(app)

@app.route("/")
def home():
    return "Visualisation CANSAT"

@app.route("/orientation")
def get_orientation():
    return jsonify(latest_orientation)

def run_flask():
    app.run(port=5000, debug=False, use_reloader=False)

def safe_float(value):
    try:
        return float(value)
    except (ValueError, TypeError):
        return None

def uart_reader():
    print("Connexion au port série ...")
    time.sleep(3)
    try:
        ser = serial.Serial(port="COM4", baudrate=115200, timeout=1)
    except Exception as e:
        print(f"Erreur lors de l'ouverture du port série : {e}\nSolutions : fermer Thonny et vérifier le port COM")
        return

    print("Lecture des données du Pico...")

    while True:
        try:
            line = ser.readline().decode("utf-8").strip()
            if line:
                message_parts = line.split(",")

                pressure = safe_float(message_parts[1])
                temperature = safe_float(message_parts[2])
                gx = safe_float(message_parts[3])
                gy = safe_float(message_parts[4])
                gz = safe_float(message_parts[5])
                full = safe_float(message_parts[6])
                ir = safe_float(message_parts[7])
                correction = safe_float(message_parts[8])
                pitch = safe_float(message_parts[9])
                roll = safe_float(message_parts[10])
                yaw = safe_float(message_parts[11])
                altitude = safe_float(message_parts[12])
                rssi = safe_float(message_parts[13])

                if None not in (roll, pitch, yaw):
                    latest_orientation["roll"] = roll
                    latest_orientation["pitch"] = pitch
                    latest_orientation["yaw"] = yaw

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
                        "pitch": pitch,
                        "roll": roll,
                        "yaw": yaw,
                        "altitude": altitude,
                        "rssi": rssi,
                    },
                }

                try:
                    with InfluxDBClient(url="http://localhost:8086", token=token, org=org) as client:
                        write_api = client.write_api(write_options=SYNCHRONOUS)
                        write_api.write(bucket=bucket, org=org, record=record)
                except Exception as e:
                    print(f"Erreur lors de l'écriture InfluxDB : {e}")
                    time.sleep(0.5)

        except serial.SerialException as e:
            print(f"Erreur série : {e}")
            time.sleep(0.5)
        except UnicodeDecodeError:
            print("Erreur de décodage — ligne ignorée")
            continue
        except Exception as e:
            print(f"Erreur : {e}")

if __name__ == "__main__":
    try:
        # Lancer Flask dans un thread
        flask_thread = threading.Thread(target=run_flask, daemon=True)
        flask_thread.start()

        print("Flask lancé sur http://localhost:5000/orientation")

        uart_reader()
    except KeyboardInterrupt:
        print("\nArrêt manuel.")
