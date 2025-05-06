from pythonosc.dispatcher import Dispatcher
from pythonosc import osc_server
import threading

# グローバルに共有するパラメータ格納用辞書
params = {
    "separation":      0.14,
    "cohesion":        1.76,
    "noise_scale":     0.01,
    "wave_scale":      0.20,
    "wave_strength":   0.10,
    "noise_speed":     0.010,
    "led_amplitude":   1.00,
    "hue_offset":      0.20,
    "hue_span":        0.07,
    "sine_amplitude":  0.70,
    "sine_frequency":  0.10,
    "color_speed":     0.03,
    "tilt_angle":      10.0,
    "rotation_speed":  0.300,
    "plane_height":    2.50,
    "audience_count":  2,
    "audience_movement": 0.50,
    "detect_radius":   1.00,
    "empty_radius":    0.0,  # 初期値は調整してください
    "menu":              0,    # ドロップダウン用メニュー名
    "pause":             0.0,   # 0: 実行, 1: 一時停止
    "fish_align" : 0.5,
    "fish_sep"   : 0.35,
    "fish_coh"   : 1.2,
    "fin_amp"    : 4.0,
    "fin_freq"   : 1.5,

}

# 各パラメータ用ハンドラ

def _handle_param(addr, value):
    key = addr.strip("/")
    if key in params:
        print(f"[OSC] {addr} → {key} = {value}")
        try:
            params[key] = float(value)
        except ValueError:
            pass
    else:
        print(f"[OSC] Unknown param: {addr}")


def start_osc_listener(ip="0.0.0.0", port=8000):
    disp = Dispatcher()
    # OSCアドレスとハンドラを一括登録
    for key in params:
        disp.map(f"/{key}", _handle_param)

    server = osc_server.ThreadingOSCUDPServer((ip, port), disp)
    print(f"[OSC] Listening on {ip}:{port}")
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
