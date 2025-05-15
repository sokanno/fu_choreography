# 必要ライブラリ: python-osc
# pip install python-osc

from pythonosc import udp_client
import colorsys
import random
import time

# SuperCollider サーバーのホストとポート
SC_HOST = "127.0.0.1"
SC_PORT = 57120

client = udp_client.SimpleUDPClient(SC_HOST, SC_PORT)

def send_robot_osc(robot_id, color_rgb, angle_deg, height_norm):
    """
    robot_id: int
    color_rgb: (r, g, b) each 0.0–1.0
    angle_deg: 0–360
    height_norm: 0.0–1.0
    """
    r, g, b = color_rgb
    angle = (angle_deg / 360.0) * 2 * 3.14159265  # 0–2π に変換（必要なら）
    # SuperCollider側で角度を 0–360 のまま扱うならこの変換は不要
    client.send_message("/robot", [robot_id, r, g, b, angle_deg, height_norm])

# テスト送信
if __name__ == "__main__":
    for i in range(10):
        # ダミーデータ：ランダムな色・角度・高さ
        color = (random.random(), random.random(), random.random())
        angle = random.uniform(0, 360)
        height = random.random()
        send_robot_osc(i, color, angle, height)
        time.sleep(0.2)
