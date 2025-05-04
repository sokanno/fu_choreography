#!/usr/bin/env python3
import time
import threading
import re
import curses
import paho.mqtt.client as mqtt

# --- グローバル設定 ---
# デフォルトの設定値（必要に応じて変更してください）
expected_send_flag = 1          # 1: 送信有効
expected_framerate = 20         # 期待するフレームレート (fps)
expected_payload_size = 20      # 期待するペイロードのバイト数
# 評価に用いる時間窓（秒）―ここでは直近 2 秒間でフレームレートを計算
window_seconds = 2.0

# 各デバイスの受信状況を記録する辞書
# キー: device_id（文字列）
# 値: {"timestamps": [受信時刻, ...], "last_payload_length": int, "last_update": timestamp}
device_status = {}

# MQTT トピックのパターン
send_test_pattern = re.compile(r"^sendTest/(.+)$")

# --- MQTT コールバック ---
def on_message(client, userdata, msg):
    global expected_send_flag, expected_framerate, expected_payload_size
    topic = msg.topic
    payload = msg.payload.decode()
    # 受信メッセージの内容を確認（必要に応じてコメントアウト）
    # print(f"Received: {topic} -> {payload}")

    # sendTest/<device_id> トピックの場合
    m = send_test_pattern.match(topic)
    if m:
        device_id = m.group(1)
        now = time.time()
        if device_id not in device_status:
            device_status[device_id] = {"timestamps": [], "last_payload_length": 0, "last_update": now}
        rec = device_status[device_id]
        rec["timestamps"].append(now)
        # 時間窓外の古いタイムスタンプを削除
        rec["timestamps"] = [t for t in rec["timestamps"] if now - t <= window_seconds]
        rec["last_payload_length"] = len(payload)
        rec["last_update"] = now
        return

    # 各設定トピックの場合はグローバル変数を更新
    if topic == "sendFramerate":
        try:
            expected_framerate = int(payload)
        except Exception:
            pass
        return
    if topic == "sendPayloadSize":
        try:
            expected_payload_size = int(payload)
        except Exception:
            pass
        return
    if topic == "sendFlag":
        try:
            expected_send_flag = int(payload)
        except Exception:
            pass
        return

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        # 設定トピックと送信テスト用トピックの購読
        client.subscribe("sendTest/+")
        client.subscribe("sendFramerate")
        client.subscribe("sendPayloadSize")
        client.subscribe("sendFlag")
        # 一括で設定値を送信（必要に応じて）
        client.publish("sendFlag", str(expected_send_flag))
        client.publish("sendFramerate", str(expected_framerate))
        client.publish("sendPayloadSize", str(expected_payload_size))
    else:
        print("MQTT connect failed with rc", rc)

def on_disconnect(client, userdata, rc):
    if rc != 0:
        print("Unexpected MQTT disconnection.")

# --- MQTT クライアントをバックグラウンドで動作させる ---
def mqtt_thread():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    # 接続先ブローカー（必要に応じて変更してください）
    MQTT_BROKER = "192.168.1.238"
    MQTT_PORT = 1883
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_forever()

# --- curses を使った表示ループ ---
def display_loop(stdscr):
    global expected_framerate, expected_payload_size, expected_send_flag
    # カーソル非表示、入力ノンブロッキング
    curses.curs_set(0)
    stdscr.nodelay(True)
    curses.start_color()
    curses.use_default_colors()
    # カラー設定
    curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_GREEN)   # 両条件OK → 緑
    curses.init_pair(2, curses.COLOR_BLACK, curses.COLOR_YELLOW)  # どちらか一方NG → 黄
    curses.init_pair(3, curses.COLOR_BLACK, curses.COLOR_RED)     # 両条件NG → 赤
    curses.init_pair(4, curses.COLOR_BLACK, curses.COLOR_WHITE)   # 未受信 → 白

    # グリッドの設定（10×10 とする）
    grid_rows = 10
    grid_cols = 10
    cell_width = 12   # 各セルの幅（文字数）
    cell_height = 3   # 各セルの高さ（行数）

    while True:
        stdscr.clear()
        # タイトル行に設定値を表示（上部1行）
        title = (f"MQTT Monitor  |  Expected FPS: {expected_framerate}  |  "
                 f"Expected Payload Size: {expected_payload_size}  |  sendFlag: {expected_send_flag}  "
                 f"| f: framerate, p: payload, s: sendFlag, q: quit")
        stdscr.addstr(0, 0, title)
        # sorted() で device_id を昇順に取得
        devices = sorted(device_status.keys(), key=lambda x: int(x) if x.isdigit() else x)
        # グリッドに配置：左上から順に配置（表示可能なセル数を grid_rows * grid_cols とする）
        for idx, device_id in enumerate(devices):
            if idx >= grid_rows * grid_cols:
                break
            row = idx // grid_cols
            col = idx % grid_cols
            # セルの左上座標
            y0 = 2 + row * cell_height
            x0 = col * cell_width
            rec = device_status[device_id]
            now = time.time()
            # 直近 window_seconds 秒内の受信数から実効 FPS を算出
            fps = len([t for t in rec["timestamps"] if now - t <= window_seconds]) / window_seconds
            # payload サイズが期待値と合っているか
            payload_ok = (rec["last_payload_length"] == expected_payload_size)
            # フレームレートは、90% 以上で OK とみなす
            fr_ok = (fps >= 0.9 * expected_framerate)
            # 状態判定
            if fr_ok and payload_ok:
                color = curses.color_pair(1)
            elif (not fr_ok) and (not payload_ok):
                color = curses.color_pair(3)
            else:
                color = curses.color_pair(2)
            # セルの背景を色付けして表示
            for dy in range(cell_height):
                stdscr.addstr(y0 + dy, x0, " " * cell_width, color)
            # セル内に device_id, FPS, payload 長を表示
            stdscr.addstr(y0, x0, f"ID:{device_id}", color)
            stdscr.addstr(y0+1, x0, f"FPS:{fps:4.1f}", color)
            stdscr.addstr(y0+2, x0, f"PL:{rec['last_payload_length']:2d}", color)
        stdscr.refresh()
        time.sleep(0.1)
        # キー入力の確認（ノンブロッキング）
        try:
            key = stdscr.getch()
            if key == ord('q'):
                break
            elif key == ord('f'):
                # framerate の変更
                stdscr.nodelay(False)
                stdscr.addstr(0, 0, "Enter new framerate: " + " " * 20)
                stdscr.refresh()
                curses.echo()
                new_val = stdscr.getstr().decode().strip()
                curses.noecho()
                try:
                    expected_framerate = int(new_val)
                except Exception:
                    pass
                stdscr.nodelay(True)
            elif key == ord('p'):
                # payload size の変更
                stdscr.nodelay(False)
                stdscr.addstr(0, 0, "Enter new payload size: " + " " * 20)
                stdscr.refresh()
                curses.echo()
                new_val = stdscr.getstr().decode().strip()
                curses.noecho()
                try:
                    expected_payload_size = int(new_val)
                except Exception:
                    pass
                stdscr.nodelay(True)
            elif key == ord('s'):
                # sendFlag の変更
                stdscr.nodelay(False)
                stdscr.addstr(0, 0, "Enter new sendFlag (0 or 1): " + " " * 20)
                stdscr.refresh()
                curses.echo()
                new_val = stdscr.getstr().decode().strip()
                curses.noecho()
                try:
                    expected_send_flag = int(new_val)
                except Exception:
                    pass
                stdscr.nodelay(True)
        except Exception:
            pass

# --- メイン ---
def main():
    # MQTT クライアントを別スレッドで開始
    t = threading.Thread(target=mqtt_thread, daemon=True)
    t.start()
    # curses ラッパー内で表示ループを実行
    curses.wrapper(display_loop)

if __name__ == "__main__":
    main()
