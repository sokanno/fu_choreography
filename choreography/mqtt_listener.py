#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mqtt_listener.py
----------------
MQTT で送られてくる “ca” (audience) / “ss” (sound‑source) の 2 トピックを購読し、
ペイロードに含まれる int16×2 の座標値をメートル単位に変換して
スレッドセーフなキューに渡すユーティリティ。

・ペイロード形式 : 4 byte 固定
    >hh   # big‑endian signed short ×2
           (x_cm, y_cm)

・座標系  : cm → m へ変換してキューへ push
・利用例 :   from mqtt_listener import start, fetch_messages
              start()                # バックグラウンドで受信開始
              while True:
                  for msg in fetch_messages():
                      ...
"""

import struct
import time
import logging
from queue import Queue, Empty
from typing import List, Tuple, Dict, Any

import paho.mqtt.client as mqtt

# ---------------------------------------------------------------------------
# コンフィグ
# ---------------------------------------------------------------------------
# BROKER_HOST = "localhost"
BROKER_HOST = "192.168.1.2"
BROKER_PORT = 1883
KEEPALIVE    = 60            # sec

# 購読トピック (topic, qos)
TOPICS: List[Tuple[str, int]] = [
    ("ca", 0),   # coordinate of audience
    ("ss", 0),   # sound source
]

# ---------------------------------------------------------------------------
# グローバルキュー  : 受信スレッド → メインループ
# ---------------------------------------------------------------------------
msg_queue: Queue = Queue()

# ---------------------------------------------------------------------------
# MQTT コールバック
# ---------------------------------------------------------------------------
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        logging.info("[MQTT] Connected")
        client.subscribe(TOPICS)
        for t, _ in TOPICS:
            logging.debug(f"Subscribed to '{t}'")
    else:
        logging.error(f"[MQTT] Connection failed: rc={rc}")

def on_message(client, userdata, msg):
    """
    payload : 4 byte × N
              1人  -> >hh
              2人  -> >hhhh
              3人  -> >hhhhhh   …という可変長
    """
    logging.debug(
        f"[RAW] topic={msg.topic} len={len(msg.payload)}  "
        f"payload(hex)={msg.payload.hex(' ')}"
    )

    try:
        # ---- ここを追加 ----
        if len(msg.payload) == 0:
            # 空メッセージ → 「観測者なし」
            msg_queue.put({
                "topic"  : msg.topic,
                "coords" : [],          # 空リストで表現
                "time"   : time.time(),
            })
            logging.debug(f"[MQTT] {msg.topic} -> 0 person")
            return                    # これ以上の処理は不要
        # ---- ここまで ----
        # 4 の倍数でなければ不正
        if len(msg.payload) == 0 or len(msg.payload) % 4 != 0:
            raise ValueError(f"unexpected length {len(msg.payload)}")

        # (x_cm, y_cm) の列を取り出す
        coords_m: list[tuple[float, float]] = [
            (x_cm / 100.0, y_cm / 100.0)
            for x_cm, y_cm in struct.iter_unpack(">hh", msg.payload)
        ]

        msg_queue.put({
            "topic"  : msg.topic,
            "coords" : coords_m,     # ← リストで格納
            "time"   : time.time(),
        })

        logging.debug(
            f"[MQTT] {msg.topic} -> {len(coords_m)} person(s): "
            + ", ".join(f"({x:.2f}, {y:.2f})" for x, y in coords_m) + " m"
        )

    except Exception as e:
        logging.warning(f"[MQTT] Bad payload on '{msg.topic}': {e}")


# ---------------------------------------------------------------------------
# パブリック API
# ---------------------------------------------------------------------------
def start(broker_host: str = BROKER_HOST,
          broker_port: int = BROKER_PORT,
          keepalive  : int = KEEPALIVE) -> mqtt.Client:
    """
    MQTT クライアントを生成してバックグラウンドで受信を開始する。
    戻り値の client は stop() や追加 publish に使える。
    """
    client               = mqtt.Client()
    client.on_connect    = on_connect
    client.on_message    = on_message
    client.enable_logger()          # paho 内部ログを Python の logging 経由で出す

    client.connect(broker_host, broker_port, keepalive)
    client.loop_start()             # 別スレッドでネットワークループ開始
    return client

def fetch_messages(max_items: int | None = None) -> List[Dict[str, Any]]:
    """
    受信済みメッセージをすべて（または max_items 件）まとめて返す。
    メッセージが無ければ空リスト。
    """
    msgs: List[Dict[str, Any]] = []
    try:
        if max_items is None:
            while True:
                msgs.append(msg_queue.get_nowait())
        else:
            for _ in range(max_items):
                msgs.append(msg_queue.get_nowait())
    except Empty:
        pass
    return msgs

def stop(client: mqtt.Client):
    """start() で返した client を停止して clean exit する。"""
    client.loop_stop()
    client.disconnect()

# ---------------------------------------------------------------------------
# スクリプトとして実行した場合の簡易デモ
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="MQTT listener demo")
    parser.add_argument("--host", default=BROKER_HOST)
    parser.add_argument("--port", type=int, default=BROKER_PORT)
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s: %(message)s",
    )

    cli = start(args.host, args.port)
    logging.info("Listening...  (Ctrl‑C to stop)")
    try:
        while True:
            for m in fetch_messages():
                for i, (x, y) in enumerate(m["coords"], start=1):
                    print(f"{m['topic']}  P{i}: (x={x:.2f}, y={y:.2f}) "
                        f"[t={m['time']:.2f}]")
            time.sleep(0.02)

    except KeyboardInterrupt:
        logging.info("Stopping…")
    finally:
        stop(cli)
