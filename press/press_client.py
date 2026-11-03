"""OSC client for the FU press engine (SuperCollider, press_engine.scd).

縮む部屋 (パントマイム) シーンの音響プロトコルの薄いラッパ。
設定とロボット配置は territory/ の config.json / robots.json を共有する
(配置を変えたら choreography/gen_robots_json.py で territory 側を再生成すればよい)。

Protocol (see press_engine.scd header):
    /press/scene    int                1 = fade in, 0 = fade out
    /press/gap      gap_m, progress, top_z, bot_z
                    壁の中心間隔 [m], 閉じ具合 0..1, 上壁/下壁の高さ [m] (5 Hz)
                    top_z/bot_z はドローンのグリッサンド用 (壁の位置に同期)
    /press/bounce   idx, vel, gap, wall, freq_hz  バウンス1回 (idx 0-based, vel 0..1,
                    freq_hz = その筒の個体音程。鳴るたびにモデル側で上がる。
                                         gap = その筒の位置での壁間隔 [m], wall 1=上 0=下)
    /press/squeeze  dur, gap           潰し開始: 微振動ラトル dur 秒
    /press/release                     壁が消えた瞬間: 溜まったエコーを解放
    /press/reset                       次のサイクル開始: エコーを再び封じる
"""

import json
import os

from pythonosc import udp_client

_HERE = os.path.dirname(os.path.abspath(__file__))
_TERR = os.path.join(_HERE, "..", "territory")


def load_config(path=None):
    with open(path or os.path.join(_TERR, "config.json")) as f:
        return json.load(f)


def load_robots(path=None):
    """territory/robots.json を共有 (id, 正規化 x y, world_x world_y [m])。"""
    with open(path or os.path.join(_TERR, "robots.json")) as f:
        return json.load(f)["robots"]


class PressClient:
    def __init__(self, host=None, port=None, config_path=None):
        cfg = load_config(config_path)["osc"]
        self.client = udp_client.SimpleUDPClient(
            host or cfg["sc_host"], port or cfg["sc_port"]
        )

    def scene(self, on):
        self.client.send_message("/press/scene", [1 if on else 0])

    def gap(self, gap_m, progress, top_z=0.0, bot_z=0.0):
        self.client.send_message("/press/gap", [
            float(gap_m), float(progress), float(top_z), float(bot_z)])

    def bounce(self, idx, vel, gap_m, wall, freq_hz=0.0):
        self.client.send_message(
            "/press/bounce",
            [int(idx), float(vel), float(gap_m), int(wall), float(freq_hz)]
        )

    def squeeze(self, dur, gap_m):
        self.client.send_message("/press/squeeze", [float(dur), float(gap_m)])

    def release(self):
        self.client.send_message("/press/release", [1])

    def reset(self):
        self.client.send_message("/press/reset", [1])
