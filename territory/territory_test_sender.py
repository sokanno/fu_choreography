"""Standalone test sender for the FU territory engine.

Runs the full territory simulation (opinion vectors on the hue/fifths circle,
hidden trait vectors, drifting salient axis with auto-shift, fatigue,
torque-based gaze with scan/align/stare, height and pitch tilt) and streams
it to territory_engine.scd over OSC — so the sound can be developed and tuned
without robots or main.py. This is the same model as the design widgets.

Usage:
    python3 territory_test_sender.py              # auto: the society runs itself + REPL
    python3 territory_test_sender.py --no-auto    # auto-shift off (manual shifts only)

REPL commands (single line + Enter):
    s               trigger an axis shift (争点の転換) by hand
    auto <0|1>      auto-shift off/on
    scan <v>        wander spin torque      (default 0.09)
    ali <v>         align torque            (default 0.04)
    stare <v>       stare torque            (default 0.07)
    unif <v>        uniformity-fatigue gain (default 1.5)
    g <dB>          master gain
    q               fade out and quit
"""

import argparse
import json
import math
import os
import random
import select
import sys
import time

from territory_client import TerritoryClient, load_robots

_HERE = os.path.dirname(os.path.abspath(__file__))

TWO_PI = 2 * math.pi
FRAME_HZ = 20
GLOBAL_EVERY = 4          # send /terr/global every N frames (5 Hz)
MODEL_TICK = 0.08         # the widget's tick the constants were tuned at


def angd(a, b):
    d = a - b
    while d > math.pi:
        d -= TWO_PI
    while d < -math.pi:
        d += TWO_PI
    return d


def pitch_class(angle):
    """Opinion angle -> circle-of-fifths pitch class (matches the SC engine)."""
    return (round(angle / TWO_PI * 12) % 12 * 7) % 12


class Territory:
    """The full opinion/gaze model. Constants are per-0.08 s step (same values
    as the design widgets); step() rescales them to the actual dt."""

    def __init__(self, world_pos, seed=None):
        self.n = len(world_pos)
        self.pos = world_pos
        rng = random.Random(seed)
        self.rng = rng
        # neighbor graph: adjacent ring only (< 1.35 m)
        self.nbr = [[] for _ in range(self.n)]
        self.bear = [{} for _ in range(self.n)]
        for i in range(self.n):
            for j in range(self.n):
                if i == j:
                    continue
                dx = world_pos[j][0] - world_pos[i][0]
                dy = world_pos[j][1] - world_pos[i][1]
                if math.hypot(dx, dy) < 1.35:
                    self.nbr[i].append(j)
                    self.bear[i][j] = math.atan2(dy, dx)
        # tunable torques / gains
        self.scan = 0.09
        self.ali = 0.04
        self.stare = 0.07
        self.unif = 1.5
        self.auto_shift = True
        # model constants (per 0.08 s tick)
        self.k_att = 4.0    # attention sharpness
        self.eps = 0.9      # bounded confidence
        self.mu = 0.2       # conformity
        self.alpha = 0.12   # radicalization
        self.gam = 0.12     # fatigue pull to center
        self.beta = 0.02    # global homeostasis
        self.fr = 0.006     # fatigue rate
        self.kap = 0.02     # trait anchor
        self.eta = 0.03     # opinion noise
        self.drift = 0.001  # axis drift rad/tick
        self.reset()

    def reset(self):
        rng = self.rng
        n = self.n
        self.vx, self.vy = [], []
        self.tx0, self.ty0 = [], []
        self.w = []
        self.th, self.om, self.spin = [], [], []
        self.ht = [0.3] * n
        self.tilt = [0.0] * n
        for _ in range(n):
            a = rng.uniform(0, TWO_PI)
            m = 0.4 + 0.6 * rng.random()
            self.tx0.append(math.cos(a) * m)
            self.ty0.append(math.sin(a) * m)
            self.vx.append(self.tx0[-1] * 0.3)
            self.vy.append(0.0)
            self.w.append(rng.random() * 0.3)
            self.th.append(rng.uniform(0, TWO_PI))
            self.om.append(0.0)
            self.spin.append(1 if rng.random() < 0.5 else -1)
        self.psi = 0.0
        self.op_unwrap = [math.atan2(self.vy[i], self.vx[i]) for i in range(n)]
        self.talk = 1.0
        self.next_say = [1.0 + rng.expovariate(0.2) for _ in range(n)]
        self.R = 0.0
        self.C = 0.0
        self.camps = 0
        self.hi_time = 0.0
        self.last_shift = -999.0
        self.t = 0.0
        self.shift_count = 0

    def do_shift(self):
        jump = (math.pi / 3 + self.rng.random() * math.pi / 3)
        jump *= 1 if self.rng.random() < 0.5 else -1
        self.psi += jump
        for i in range(self.n):
            self.vx[i] *= 0.3
            self.vy[i] *= 0.3
            self.w[i] *= 0.5
            self.spin[i] = 1 if self.rng.random() < 0.5 else -1
            self.om[i] = self.spin[i] * (0.5 + self.rng.random()) * max(self.scan, 0.04)
        self.last_shift = self.t
        self.shift_count += 1
        return jump

    def step(self, dt):
        """Advance the model. Returns (ticks, auto_shifted): ticks is a list of
        (idx, vel 0..1) yaw half-turn crossings this step."""
        f = dt / MODEL_TICK
        n = self.n
        self.t += dt
        self.psi += self.drift * f
        ex, ey = math.cos(self.psi), math.sin(self.psi)

        mx = sum(self.vx) / n
        my = sum(self.vy) / n
        cx = sum(math.cos(t) for t in self.th) / n
        cy = sum(math.sin(t) for t in self.th) / n
        msum = sum(math.hypot(self.vx[i], self.vy[i]) for i in range(n))
        self.R = math.hypot(cx, cy)
        self.C = min(1.0, math.hypot(mx, my) * n / msum) if msum > 0.01 else 0.0

        nvx, nvy = self.vx[:], self.vy[:]
        for i in range(n):
            pxl = pyl = wsum = 0.0
            t_ali = t_stare = 0.0
            for j in self.nbr[i]:
                bij = self.bear[i][j]
                L = ((1 + math.cos(angd(bij, self.th[i]))) / 2) ** self.k_att
                P = ((1 + math.cos(angd(self.bear[j][i], self.th[j]))) / 2) ** self.k_att
                wt = L * (0.15 + 0.85 * P)
                dxo = self.vx[j] - self.vx[i]
                dyo = self.vy[j] - self.vy[i]
                d = math.hypot(dxo, dyo)
                if d < self.eps:
                    pxl += wt * dxo
                    pyl += wt * dyo
                    wsum += wt
                sim = math.exp(-d * d * 2)
                t_ali += self.ali * sim * math.sin(angd(self.th[j], self.th[i]))
                t_stare += self.stare * (d / 2) * math.sin(angd(bij, self.th[i]))
            if wsum > 0:
                pxl = self.mu * pxl / wsum
                pyl = self.mu * pyl / wsum
            m2 = self.vx[i] ** 2 + self.vy[i] ** 2
            mag = math.sqrt(m2)
            radf = self.alpha * (1 - 2 * self.w[i]) * (1 - m2)
            tproj = self.tx0[i] * ex + self.ty0[i] * ey
            vproj = self.vx[i] * ex + self.vy[i] * ey
            anch = self.kap * (tproj - vproj)
            nx = self.vx[i] + f * (
                pxl + radf * self.vx[i] - self.gam * self.w[i] * self.vx[i]
                + anch * ex - self.beta * mx
                + self.rng.uniform(-1, 1) * self.eta
            )
            ny = self.vy[i] + f * (
                pyl + radf * self.vy[i] - self.gam * self.w[i] * self.vy[i]
                + anch * ey - self.beta * my
                + self.rng.uniform(-1, 1) * self.eta
            )
            # damp the off-axis component (the salient axis owns the debate)
            perp = nx * (-ey) + ny * ex
            nx -= perp * 0.1 * f * (-ey)
            ny -= perp * 0.1 * f * ex
            nm = math.hypot(nx, ny)
            if nm > 1:
                nx /= nm
                ny /= nm
            nvx[i], nvy[i] = nx, ny
            self.w[i] = min(1.0, max(0.0, self.w[i] + f * self.fr * (m2 - 0.25)
                                     * (1 + self.unif * self.R * self.C)))
            engage = max(0.0, mag * (1 - self.w[i]))
            t_scan = 0.15 * (self.spin[i] * self.scan - self.om[i])
            self.om[i] += f * ((1 - engage) * t_scan + engage * (t_ali + t_stare)
                               + self.rng.uniform(-1, 1) * 0.004)
            self.om[i] *= 0.93 ** f
            self.om[i] = max(-0.15, min(0.15, self.om[i]))
        self.vx, self.vy = nvx, nvy

        # yaw integration + half-turn tick detection
        ticks = []
        for i in range(n):
            old = self.th[i]
            self.th[i] += self.om[i] * f
            if math.floor(old / math.pi) != math.floor(self.th[i] / math.pi):
                ticks.append((i, min(1.0, abs(self.om[i]) / 0.15)))

        # unwrapped opinion angle (continuous — the engine lags it into gliss)
        for i in range(n):
            a = math.atan2(self.vy[i], self.vx[i])
            cur = math.atan2(math.sin(self.op_unwrap[i]),
                             math.cos(self.op_unwrap[i]))
            self.op_unwrap[i] += angd(a, cur)

        # height / tilt targets (slewed): conviction descends, burnout retreats;
        # strong+fresh addresses the audience (down), burned-out looks away (up)
        for i in range(n):
            mag = math.hypot(self.vx[i], self.vy[i])
            ht_t = mag * (1 - 0.7 * self.w[i])
            tl_t = max(-1.0, min(1.0, 1.2 * mag * (1 - self.w[i]) - 0.9 * self.w[i]))
            self.ht[i] += (ht_t - self.ht[i]) * (1 - math.exp(-dt / 2.0))
            self.tilt[i] += (tl_t - self.tilt[i]) * (1 - math.exp(-dt / 1.5))

        # emergent camp count (same clustering as the widget)
        used = [False] * n
        camps = 0
        for i in range(n):
            if used[i] or math.hypot(self.vx[i], self.vy[i]) < 0.35:
                continue
            camps += 1
            stack = [i]
            used[i] = True
            while stack:
                a = stack.pop()
                for b in range(n):
                    if used[b] or math.hypot(self.vx[b], self.vy[b]) < 0.35:
                        continue
                    if math.hypot(self.vx[a] - self.vx[b],
                                  self.vy[a] - self.vy[b]) < 0.5:
                        used[b] = True
                        stack.append(b)
        self.camps = camps

        # auto-shift: sustained uniformity triggers a new salient axis
        auto_shifted = False
        if self.R > 0.85 or self.C > 0.85:
            self.hi_time += dt
        else:
            self.hi_time = 0.0
        if (self.auto_shift and self.hi_time > 8.0
                and self.t - self.last_shift > 12.0):
            self.hi_time = 0.0
            self.do_shift()
            auto_shifted = True
        return ticks, auto_shifted

    def utterances(self):
        """Speech scheduler: convinced tubes take the floor now and then;
        an utterance provokes earlier replies from opposing neighbors, so
        argument volleys form across the front line. Returns [(i, dur, amp)]."""
        now = self.t
        out = []
        for i in range(self.n):
            if now < self.next_say[i]:
                continue
            conv = math.hypot(self.vx[i], self.vy[i])
            lam = self.talk * (0.03 + 0.18 * conv * conv * (1 - 0.6 * self.w[i]))
            self.next_say[i] = now + min(60.0, self.rng.expovariate(max(lam, 1e-3)))
            if conv < 0.25:
                continue  # the undecided hold their tongue
            dur = 0.4 + 2.4 * self.rng.random() ** 2
            amp = 0.3 + 0.7 * conv
            out.append((i, dur, amp))
            for j in self.nbr[i]:
                dv = math.hypot(self.vx[i] - self.vx[j], self.vy[i] - self.vy[j])
                if dv > 0.8 and self.rng.random() < 0.5:
                    self.next_say[j] = min(self.next_say[j],
                                           now + 0.3 + self.rng.random() * 0.8)
        return out

    # --- state vectors for OSC ---
    def conviction(self):
        return [math.hypot(self.vx[i], self.vy[i]) for i in range(self.n)]

    def omega_rad_s(self):
        return [abs(o) / MODEL_TICK for o in self.om]

    def mean_op(self):
        mx = sum(self.vx) / self.n
        my = sum(self.vy) / self.n
        return math.atan2(my, mx)


class Sender:
    def __init__(self, args):
        self.client = TerritoryClient(host=args.host, port=args.port)
        robots = load_robots()
        world = [(r["world_x"], r["world_y"]) for r in robots]
        self.model = Territory(world, seed=args.seed)
        self.model.auto_shift = not args.no_auto
        self.gain_db = args.gain
        self.ali_hi = False
        self.coh_hi = False
        self.viz_path = os.path.join(_HERE, "viz_state.json") if args.viz else None
        self.viz_events = []

    def log(self, msg):
        t = self.model.t
        sys.stdout.write("\r\033[K" + f"[t={t:6.1f}s] " + msg + "\n")
        sys.stdout.flush()
        self.viz_events.append(f"[t={t:6.1f}s] {msg}")
        del self.viz_events[:-12]

    def write_viz(self):
        m = self.model
        state = {
            "t": round(m.t, 1), "R": round(m.R, 3), "C": round(m.C, 3),
            "camps": m.camps, "shifts": m.shift_count, "psi": round(m.psi, 4),
            "hi": round(m.hi_time, 1), "auto": m.auto_shift,
            "op": [round(math.atan2(m.vy[i], m.vx[i]), 3) for i in range(m.n)],
            "conv": [round(v, 3) for v in m.conviction()],
            "fat": [round(v, 3) for v in m.w],
            "ht": [round(v, 3) for v in m.ht],
            "tilt": [round(v, 3) for v in m.tilt],
            "th": [round(v % TWO_PI, 3) for v in m.th],
            "events": self.viz_events,
        }
        tmp = self.viz_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(state, f)
        os.replace(tmp, self.viz_path)

    def frame(self, dt):
        m = self.model
        ticks, auto_shifted = m.step(dt)
        if auto_shifted:
            self.client.shift(True)
            self.log("自動転換: 斉一化が持続、軸が跳躍 (通算%d回目)" % m.shift_count)
        self.client.send_state(
            m.op_unwrap, m.conviction(), m.w, m.ht, m.tilt, m.omega_rad_s()
        )
        for i, vel in ticks:
            self.client.tick(i, vel, pitch_class(m.op_unwrap[i]))
        for i, dur, amp in m.utterances():
            conv = math.hypot(m.vx[i], m.vy[i])
            self.client.say(i, dur, amp, m.op_unwrap[i], conv,
                            m.w[i], m.ht[i], m.tilt[i])
        # threshold event log (same hysteresis as the widget)
        if m.R > 0.8 and not self.ali_hi:
            self.ali_hi = True
            self.log(f"向きの整列 R={m.R:.2f} — 全体主義的整列を検出")
        if m.R < 0.6 and self.ali_hi:
            self.ali_hi = False
            self.log(f"整列の崩壊 R={m.R:.2f}")
        if m.C > 0.8 and not self.coh_hi:
            self.coh_hi = True
            self.log(f"意見の斉一化 C={m.C:.2f} — 満場一致に接近")
        if m.C < 0.6 and self.coh_hi:
            self.coh_hi = False
            self.log(f"斉一の解体 C={m.C:.2f}")

    def send_global(self):
        m = self.model
        self.client.send_global(m.R, m.C, m.camps, m.mean_op(), self.gain_db)

    def status_line(self):
        m = self.model
        mean_fat = sum(m.w) / m.n
        mean_om = sum(abs(o) for o in m.om) / m.n / MODEL_TICK
        mean_ht = sum(m.ht) / m.n
        parts = [
            f"⏱ {int(m.t) // 60}:{int(m.t) % 60:02d}",
            f"R {m.R:.2f}",
            f"C {m.C:.2f}",
            f"陣営 {m.camps}",
            f"疲弊 {mean_fat:.2f}",
            f"回転 {mean_om:.2f}rad/s",
            f"高さ {mean_ht:.2f}",
            f"転換 {m.shift_count}回",
        ]
        if m.hi_time > 0:
            parts.append(f"斉一化 {m.hi_time:.0f}s/8s")
        sys.stdout.write("\r\033[K" + " │ ".join(parts) + " ")
        sys.stdout.flush()

    def repl(self):
        if not select.select([sys.stdin], [], [], 0)[0]:
            return True
        line = sys.stdin.readline().strip().split()
        if not line:
            return True
        cmd, arg = line[0], (line[1] if len(line) > 1 else None)
        m = self.model
        try:
            if cmd == "q":
                return False
            elif cmd == "s":
                m.do_shift()
                self.client.shift(False)
                self.log("手動転換: 軸が跳躍")
            elif cmd == "auto":
                m.auto_shift = bool(int(arg))
                self.log(f"自動転換: {'ON' if m.auto_shift else 'OFF'}")
            elif cmd == "scan":
                m.scan = float(arg)
                self.log(f"徘徊スピン → {m.scan:.3f}")
            elif cmd == "ali":
                m.ali = float(arg)
                self.log(f"整列 → {m.ali:.3f}")
            elif cmd == "stare":
                m.stare = float(arg)
                self.log(f"凝視 → {m.stare:.3f}")
            elif cmd == "unif":
                m.unif = float(arg)
                self.log(f"斉一化疲弊 → {m.unif:.1f}")
            elif cmd == "talk":
                m.talk = float(arg)
                self.log(f"発言レート x{m.talk:.2f}")
            elif cmd == "g":
                self.gain_db = float(arg)
                self.send_global()
                self.log(f"ゲイン → {self.gain_db:.1f} dB")
            else:
                self.log("commands: s | auto <0|1> | scan <v> | ali <v> | stare <v> | unif <v> | g <dB> | q")
        except (ValueError, TypeError, IndexError):
            self.log("bad argument")
        return True

    def run(self, total=None):
        self.client.scene(True)
        self.send_global()
        t0 = time.monotonic()
        frame_i = 0
        next_t = t0
        try:
            while True:
                if total and self.model.t > total:
                    break
                self.frame(1.0 / FRAME_HZ)
                if frame_i % GLOBAL_EVERY == 0:
                    self.send_global()
                if self.viz_path and frame_i % 2 == 0:  # 10 Hz
                    self.write_viz()
                if frame_i % 10 == 0:  # 0.5 s
                    self.status_line()
                if not self.repl():
                    break
                frame_i += 1
                next_t += 1.0 / FRAME_HZ
                time.sleep(max(0.0, next_t - time.monotonic()))
        except KeyboardInterrupt:
            pass
        finally:
            self.log("fading out...")
            self.client.scene(False)
            time.sleep(3.5)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default=None)
    p.add_argument("--port", type=int, default=None)
    p.add_argument("--gain", type=float, default=0.0, help="master gain dB")
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--no-auto", action="store_true", help="disable auto axis shifts")
    p.add_argument("--viz", action="store_true",
                   help="write viz_state.json at 10 Hz for viz.html")
    p.add_argument("--duration", type=float, default=None)
    args = p.parse_args()

    sender = Sender(args)
    print("陣取りシミュレーション稼働 (社会は勝手に動きます、q で終了)")
    print("commands: s | auto <0|1> | scan <v> | ali <v> | stare <v> | unif <v> | g <dB> | q")
    sender.run(args.duration)


if __name__ == "__main__":
    main()
