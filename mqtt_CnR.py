import paho.mqtt.client as mqtt
import csv
import re
import time



# MQTTブローカー情報
# MQTT_BROKER = "192.168.1.238"
MQTT_BROKER = "127.0.0.1"
# MQTT_BROKER = "192.168.179.8"
MQTT_PORT = 1883
MQTT_KEEPALIVE = 60  # KeepAliveの秒数

# デバイス情報を保持する辞書
device_data = {}
try:
    with open("/Users/kannoso/Desktop/mqtt/mqtt/devices.csv", "r") as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            device_data[row["macAddress"]] = {
                "id": row["id"],
                "x": row["x"],
                "y": row["y"]
            }
    print(f"Loaded device data: {len(device_data)} devices.")
except FileNotFoundError:
    print("Error: devices.csv not found.")
    exit(1)

# MQTTクライアントの設定
client = mqtt.Client(protocol=mqtt.MQTTv311)

# トピック形式の判定用正規表現
ini_pattern = re.compile(r"^ini/([0-9A-F]{12})$")
pitch_yaw_pattern = re.compile(r"^(\d+)/(.+)$")  # e.g., id/pitch or id/yaw

# メッセージ受信時のコールバック
def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload.decode()

    print(f"Received message on topic: {topic} with payload: {payload}")

    # 初期化メッセージ処理
    match_ini = ini_pattern.match(topic)
    if match_ini:
        mac_address = match_ini.group(1)
        print(f"Received initialization from MAC address: {mac_address}")

        if mac_address in device_data:
            device_info = device_data[mac_address]
            # id, x, y をまとめて送信
            idxy_payload = f"{device_info['id']},{device_info['x']},{device_info['y']}"
            client.publish(f"{mac_address}/idxy", idxy_payload, qos=1)
            print(f"Published idxy to {mac_address}/idxy: {idxy_payload}")
        else:
            print(f"MAC address {mac_address} not found in device data")
        return

    # ps/id メッセージ処理
    if topic.startswith("ps/"):
        device_id = topic.split("/")[1]  # トピックからIDを抽出
        pitch, yaw = payload.split(",")  # カンマ区切りで分割
        print(f"Received pitch: {pitch}, yaw: {yaw} for device ID: {device_id}")
        # 必要に応じて追加の処理をここに記述
        return

    print(f"Unknown topic: {topic}, payload: {payload}")


# 接続時のコールバック
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT broker successfully!")
        # ini/+ トピックのサブスクライブ
        client.subscribe("ini/+")
        print("Subscribed to ini/+ for initialization messages")

        # ps/id トピックのサブスクライブ
        client.subscribe("ps/+")
        print("Subscribed to ps/+ for pitch and yaw messages")
    else:
        print(f"Failed to connect. Return code: {rc}")




# 切断時のコールバック
def on_disconnect(client, userdata, rc):
    if rc != 0:
        print(f"Unexpected disconnection. Return code: {rc}")
    else:
        print("Disconnected from MQTT broker.")


# 再接続時の処理（自動再接続を有効化）
def on_log(client, userdata, level, buf):
    if level == mqtt.MQTT_LOG_WARNING or level == mqtt.MQTT_LOG_ERR:
        print(f"MQTT Log: {buf}")

# コールバック関数を設定
client.on_message = on_message
client.on_disconnect = on_disconnect
client.on_connect = on_connect
client.on_log = on_log

# MQTTブローカーに接続
print("Connecting to MQTT broker...")
client.connect(MQTT_BROKER, MQTT_PORT, MQTT_KEEPALIVE)

# バックグラウンドでループを開始
client.loop_start()

# スクリプトを永続化
try:
    while True:
        time.sleep(1)  # 定期的にPingを送信
except KeyboardInterrupt:
    print("Exiting...")
    client.loop_stop()
    client.disconnect()
