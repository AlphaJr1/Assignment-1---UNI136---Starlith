import network
import time
import machine
import json
import dht
from umqtt.simple import MQTTClient
from machine import I2C, Pin
from machine_i2c_lcd import I2cLcd

WIFI_SSID = "Wokwi-GUEST"
WIFI_PASS = ""

MQTT_BROKER = "broker.emqx.io"
MQTT_PORT = 1883
MQTT_CLIENT_ID = "esp32_json_control"

TOPIC_SENSOR = b"/UNI136/AdrianAlfajri/data_sensor"
TOPIC_LED = b"/UNI136/AdrianAlfajri/aktuasi_led"

lampu_pin = machine.Pin(2, machine.Pin.OUT)
dht_sensor = dht.DHT22(machine.Pin(4))
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)

def scan_i2c():
    print("Scanning I2C devices...")
    devices = i2c.scan()
    if not devices:
        print("No I2C devices found! Periksa koneksi LCD.")
        return None
    print(f"I2C devices found: {devices}")
    return devices[0]  # Ambil alamat pertama

i2c_addr = scan_i2c()
if i2c_addr is None:
    raise RuntimeError("LCD tidak terdeteksi. Pastikan koneksi dan alamat I2C benar.")

lcd = I2cLcd(i2c, i2c_addr, 4, 20)  # Gunakan LCD 20x4
last_command = None

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        wlan.connect(WIFI_SSID, WIFI_PASS)
        while not wlan.isconnected():
            time.sleep(0.2)

def reconnect_mqtt(client):
    while True:
        try:
            client.connect()
            client.subscribe(TOPIC_LED)
            break
        except:
            time.sleep(2)

def sub_cb(topic, msg):
    global last_command
    print(f"[MQTT] Pesan diterima di {topic.decode()}: {msg.decode()}")
    try:
        data = json.loads(msg)
        command = data.get("msg", "")
    except:
        return

    if command == last_command:
        return

    last_command = command
    if command == "ON":
        lampu_pin.value(1)
        print("[ACTION] Lampu Dinyalakan!")
        update_lcd("ON")
    elif command == "OFF":
        lampu_pin.value(0)
        print("[ACTION] Lampu Dimatikan!")
        update_lcd("OFF")

def update_lcd(status):
    lcd.clear()
    time.sleep_ms(50)
    suhu, kelembaban = read_dht22()
    if suhu is not None and kelembaban is not None:
        lcd.move_to(0, 0)
        lcd.putstr(f"Temp: {suhu:.1f}C")
        time.sleep(5)
        lcd.clear()
        lcd.move_to(0, 0)
        lcd.putstr(f"Hum: {kelembaban:.1f}%")
        time.sleep(5)
        lcd.clear()
        if status == "ON":
            lcd.move_to(1, 0)
            lcd.putstr(f"Lampu: {status}")

def read_dht22():
    try:
        dht_sensor.measure()
        return dht_sensor.temperature(), dht_sensor.humidity()
    except:
        return None, None

def main():
    connect_wifi()
    client = MQTTClient(MQTT_CLIENT_ID, MQTT_BROKER, MQTT_PORT)
    client.set_callback(sub_cb)
    client.connect()
    client.subscribe(TOPIC_LED)

    try:
        while True:
            client.check_msg()
            suhu, kelembaban = read_dht22()
            if suhu is not None and kelembaban is not None:
                sensor_msg = json.dumps({"suhu": suhu, "kelembaban": kelembaban, "led_status": lampu_pin.value()})
                client.publish(TOPIC_SENSOR, sensor_msg)
                print(f"[PUBLISH] {sensor_msg}")
                update_lcd(f"{'ON' if lampu_pin.value() else 'OFF'}")
                if suhu > 30:
                    lampu_pin.value(1)
                    update_lcd("ON (HOT)")
            time.sleep(1)
    except KeyboardInterrupt:
        pass

main()
