# file: mqtt_setup.py
import paho.mqtt.client as mqtt

def create_mqtt_client(host: str = "127.0.0.1", port: int = 1883, keepalive: int = 60) -> mqtt.Client:
    """
    MQTT クライアントを作成し、指定ホストに接続し、ループを開始します。

    Parameters:
        host (str): MQTT ブローカーのホストアドレス（デフォルト: "127.0.0.1"）
        port (int): MQTT ブローカーのポート番号（デフォルト: 1883）
        keepalive (int): キープアライブ秒数（デフォルト: 60）

    Returns:
        mqtt.Client: 接続済みでループ開始済みの MQTT クライアント
    """
    client = mqtt.Client()
    client.connect(host, port, keepalive)
    client.loop_start()
    return client
