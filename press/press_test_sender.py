#!/usr/bin/env python3
"""縮む部屋 (press) のスタンドアロン・シミュレータ。

main.py (VPython) なしでモデルを回し、press_engine.scd へOSCを送って
音だけを開発・確認するためのツール。配置は territory/robots.json を共有。

使い方:
    python3 press_test_sender.py              # 音あり (OSC → :57120)、リアルタイム
    python3 press_test_sender.py --dry        # 音なし、イベントのログだけ
    python3 press_test_sender.py --speed 4    # 4倍速 (音は密度が変わるので目安)
    python3 press_test_sender.py --dur 300    # 300秒で終了 (省略時は無限)
"""

import argparse
import time

from press_client import PressClient, load_robots
from press_model import PressModel, SQUEEZE_S

FPS = 20.0

# iamas の高さ (main.py の VENUES と合わせる。他会場で試すならここを変える)
MIN_Z, MAX_Z = 0.39, 3.2
CRUSH_Z = 1.6


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dry", action="store_true", help="OSCを送らない")
    ap.add_argument("--speed", type=float, default=1.0, help="再生速度倍率")
    ap.add_argument("--dur", type=float, default=0.0, help="実行秒数 (0=無限)")
    ap.add_argument("--seed", type=int, default=None)
    args = ap.parse_args()

    robots = load_robots()
    pos = [(r["world_x"], r["world_y"]) for r in robots]
    cx = sum(x for x, _ in pos) / len(pos)
    cy = sum(y for _, y in pos) / len(pos)
    pos = [(x - cx, y - cy) for x, y in pos]
    z0 = [(MIN_Z + MAX_Z) / 2.0] * len(pos)

    model = PressModel(pos, z0, MIN_Z, MAX_Z, crush_z=CRUSH_Z, seed=args.seed)
    client = None
    if not args.dry:
        client = PressClient()
        client.scene(True)
        client.reset()
        print("OSC → press_engine.scd (57120)")

    dt = 1.0 / FPS
    t = 0.0
    frame = 0
    n_bounce = 0
    try:
        while args.dur <= 0 or t < args.dur:
            model.step(dt)
            n_bounce += len(model.bounces)
            if model.phase_event:
                print(f"[{t:7.1f}s] {model.phase_event:8s}  gap={model.gap_c:.2f}m "
                      f"p={model.progress:.2f}  bounces(通算)={n_bounce}")
            if client is not None:
                for bi, bvel, bgap, bwall, bhz in model.bounces:
                    client.bounce(bi, bvel, bgap, bwall, bhz)
                if model.phase_event == "squeeze":
                    client.squeeze(SQUEEZE_S, model.gap_c)
                elif model.phase_event == "release":
                    client.release()
                elif model.phase_event == "cycle":
                    client.reset()
                if frame % 4 == 0:
                    client.gap(model.gap_c, model.progress, model.top.zc, model.bot.zc)
                if frame % 100 == 0:
                    client.scene(True)
            t += dt
            frame += 1
            if args.speed > 0:
                time.sleep(dt / args.speed)
    except KeyboardInterrupt:
        pass
    finally:
        if client is not None:
            client.scene(False)
        print(f"終了: {t:.0f}s, バウンス{n_bounce}回")


if __name__ == "__main__":
    main()
