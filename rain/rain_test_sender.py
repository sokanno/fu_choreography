"""Standalone test sender for the FU rain engine (M2).

Simulates everything the real choreography (main.py) will eventually send —
Kuramoto phases, a drifting rain-cell field, wind, sync anomalies, heat —
so the SC engine can be developed and tuned without robots or main.py.

Usage:
    python3 rain_test_sender.py                   # auto: weather wanders by itself + REPL
    python3 rain_test_sender.py steady            # manual only (REPL)
    python3 rain_test_sender.py arc               # the sec-7 ~6 min scene arc
    python3 rain_test_sender.py sync              # anomaly every 20 s (gate debugging)

All parameter changes glide smoothly over seconds (no jumps). A live status
line shows the current state in the terminal.

auto / steady modes read single-line commands from stdin:
    a               trigger a sync anomaly
    m <0..3>        material morph target
    w <0..1>        wind speed
    d <radians>     wind direction
    r <mult>        rain amount multiplier (1.0 = default)
    h <robot_id>    toggle heat (visitor) under robot 1..29
    hm <0|1>        heat mode: 0 umbrella, 1 material
    q               fade out and quit
(in auto mode a manual command holds that parameter for 90 s, then auto resumes)
"""

import argparse
import math
import random
import select
import sys
import time

import noise  # same lib main.py uses

from rain_client import RainClient, load_robots

TWO_PI = 2 * math.pi
FRAME_HZ = 30
GLOBAL_EVERY = 6  # send /rain/global every N frames (5 Hz)

MATERIAL_NAMES = ["コンクリート", "葉", "金属屋根", "水面"]


def material_name(v):
    v = max(0.0, min(3.0, v))
    lo, frac = int(v), v - int(v)
    if frac < 0.2 or lo >= 3:
        return MATERIAL_NAMES[lo]
    if frac > 0.8:
        return MATERIAL_NAMES[lo + 1]
    return f"{MATERIAL_NAMES[lo]}→{MATERIAL_NAMES[lo + 1]}"


class Slew:
    """Exponential glide toward a target — keeps every change gradual."""

    def __init__(self, value, tau):
        self.value = float(value)
        self.target = float(value)
        self.tau = tau

    def step(self, dt):
        self.value += (self.target - self.value) * (1 - math.exp(-dt / self.tau))
        return self.value


class Kuramoto:
    """Mean-field Kuramoto oscillators. K=0 -> independent drift (normal rain),
    K high -> locks within a couple of seconds (anomaly)."""

    def __init__(self, n, base_hz=0.8, spread_hz=0.06, seed=1):
        rng = random.Random(seed)
        self.n = n
        self.omega = [TWO_PI * (base_hz + rng.gauss(0, spread_hz)) for _ in range(n)]
        self.phase = [rng.uniform(0, TWO_PI) for _ in range(n)]
        self.coupling = 0.0
        self.noise_sigma = 0.4  # rad/sqrt(s) phase diffusion when uncoupled

    def step(self, dt):
        n = self.n
        sx = sum(math.sin(p) for p in self.phase) / n
        cx = sum(math.cos(p) for p in self.phase) / n
        r = math.hypot(sx, cx)
        psi = math.atan2(sx, cx)
        sig = self.noise_sigma * max(0.0, 1.0 - self.coupling) * math.sqrt(dt)
        for i in range(n):
            dphi = self.omega[i] + self.coupling * r * math.sin(psi - self.phase[i])
            self.phase[i] = (self.phase[i] + dphi * dt + random.gauss(0, sig)) % TWO_PI
        return r  # order parameter


class RainField:
    """Spatial base-rate field: Perlin rain cells drifting with the wind."""

    def __init__(self, positions, max_rate=60.0, contrast=1.6, cell_scale=1.2):
        self.pos = positions  # [(x, y)] normalized
        self.max_rate = max_rate
        self.contrast = contrast
        self.cell_scale = cell_scale
        self.offset = [random.uniform(0, 100), random.uniform(0, 100)]
        self.half_space = 0.0  # 0..1 amount of "downpour on one side only"

    def advect(self, dt, wind_speed, wind_dir):
        drift = 0.02 + 0.12 * wind_speed
        self.offset[0] += math.cos(wind_dir) * drift * dt
        self.offset[1] += math.sin(wind_dir) * drift * dt

    def rates(self, t):
        out = []
        for x, y in self.pos:
            v = noise.pnoise3(
                (x + self.offset[0]) * self.cell_scale,
                (y + self.offset[1]) * self.cell_scale,
                t * 0.03,
            )
            v = (v + 1.0) * 0.5  # -> [0, 1]
            v = v ** self.contrast
            if self.half_space > 0:
                side = 0.15 + 0.85 * (1 / (1 + math.exp(-(x - 0.5) * 10)))
                v = v * (1 - self.half_space) + v * side * self.half_space * 2.5
            out.append(min(self.max_rate * v, 200.0))
        return out


class Anomaly:
    """depth envelope: ramp up, hold locked, dissolve back (sec 7, adjustable)."""

    def __init__(self, attack=2.5, hold=5.0, release=5.0):
        self.attack, self.hold, self.release = attack, hold, release
        self.t0 = None

    def trigger(self, now):
        if self.t0 is None:
            self.t0 = now

    def depth(self, now):
        if self.t0 is None:
            return 0.0
        t = now - self.t0
        a, h, r = self.attack, self.hold, self.release
        if t < a:
            return t / a
        if t < a + h:
            return 1.0
        if t < a + h + r:
            return 1.0 - (t - a - h) / r
        self.t0 = None
        return 0.0

    @property
    def active(self):
        return self.t0 is not None


class AutoWeather:
    """Slowly wandering weather for auto mode: rain amount, wind and material
    drift on minute scales; sync anomalies fire on a random schedule."""

    def __init__(self, sender, t0=0.0, anomalies=True, seed=None):
        rng = random.Random(seed)
        self.rng = rng
        self.seed = rng.randint(0, 255)
        self.anomalies = anomalies
        self.next_anomaly = t0 + rng.uniform(90, 200) if anomalies else None
        self.next_material = t0 + rng.uniform(25, 70)
        self.dir0 = rng.uniform(0, TWO_PI)
        self.flags = set()
        # manual REPL override: param -> time until which auto leaves it alone
        self.hold_until = {}

    def held(self, sender, param, t):
        return t < self.hold_until.get(param, -1)

    def update(self, sender, t):
        # rain amount: ~2 min undulation, mostly modest with occasional heavy spells
        if not self.held(sender, "rain", t):
            v = noise.pnoise1(t * 0.008, base=self.seed)          # ~ -0.6..0.6
            sender.rate_mult.target = 0.15 + 2.0 * max(0.0, v + 0.55) ** 1.6
        # wind: calm periods, then it picks up
        if not self.held(sender, "wind", t):
            w = noise.pnoise1(t * 0.012, base=self.seed + 1)
            sender.wind_speed.target = min(0.85, max(0.0, (w + 0.35)) * 1.1)
        if not self.held(sender, "dir", t):
            sender.wind_dir.target = self.dir0 + 1.5 * noise.pnoise1(t * 0.004, base=self.seed + 2)

        # material: pick a new surface every few minutes, glide there slowly
        if t >= self.next_material and not self.held(sender, "material", t):
            # never re-pick where we are or where we're already heading
            prev = round(sender.material.target)
            choices = [m for m in range(4)
                       if m != prev and abs(m - sender.material.value) > 0.6]
            if not choices:
                choices = [m for m in range(4) if m != prev]
            sender.material.tau = 25.0  # auto morphs drift over ~a minute
            sender.material.target = float(self.rng.choice(choices))
            sender.log(f"素材がゆっくり {material_name(sender.material.target)} へモーフしていく…")
            self.next_material = t + self.rng.uniform(45, 120)

        # rare, short sync anomalies (the spec's "anomaly" dramaturgy)
        if self.anomalies and t >= self.next_anomaly:
            sender.anomaly.trigger(time.monotonic())
            sender.log("★ アノマリー — 雨が同期していく")
            self.next_anomaly = t + self.rng.uniform(120, 300)

        # narrate notable transitions (with hysteresis so it doesn't spam)
        self._event(sender, "heavy", sender.rate_mult.value > 1.4, "本降りになってきた",
                    sender.rate_mult.value < 1.0)
        self._event(sender, "light", sender.rate_mult.value < 0.35, "小雨になった",
                    sender.rate_mult.value > 0.6)
        self._event(sender, "gusty", sender.wind_speed.value > 0.45, "風が立ち上がってきた",
                    sender.wind_speed.value < 0.25)

    def _event(self, sender, key, condition, message, reset):
        if condition and key not in self.flags:
            self.flags.add(key)
            sender.log(message)
        elif reset and key in self.flags:
            self.flags.discard(key)


def lerp_keyframes(t, keys):
    """keys: [(time, value), ...] sorted; linear interpolation, clamped."""
    if t <= keys[0][0]:
        return keys[0][1]
    for (t0, v0), (t1, v1) in zip(keys, keys[1:]):
        if t < t1:
            return v0 + (v1 - v0) * (t - t0) / (t1 - t0)
    return keys[-1][1]


class Sender:
    def __init__(self, args):
        self.client = RainClient(host=args.host, port=args.port)
        robots = load_robots()
        self.n = len(robots)
        self.positions = [(r["x"], r["y"]) for r in robots]
        self.kuramoto = Kuramoto(self.n)
        self.field = RainField(self.positions, max_rate=args.max_rate)
        self.anomaly = Anomaly(args.attack, args.hold, args.release)
        self.heat = [0] * self.n
        # slewed parameters: REPL/auto set .target, the value glides there
        self.material = Slew(args.material, tau=5.0)
        self.wind_speed = Slew(args.wind, tau=8.0)
        self.wind_dir = Slew(args.wind_dir, tau=10.0)
        self.rate_mult = Slew(1.0, tau=6.0)
        self.gain_db = args.gain
        self.coupling_locked = 8.0
        self.auto = None  # set in auto mode
        # status bookkeeping
        self.avg_rate = 0.0
        self.order_r = 0.0
        self.depth = 0.0
        self.history = []  # (t, avg_rate) for the trend arrow

    def log(self, msg):
        sys.stdout.write("\r\033[K" + msg + "\n")
        sys.stdout.flush()

    def frame(self, t, dt):
        self.depth = self.anomaly.depth(t)
        for p in (self.material, self.wind_speed, self.wind_dir, self.rate_mult):
            p.step(dt)
        # couple the oscillators while the anomaly is up so the pulse is coherent
        self.kuramoto.coupling = self.coupling_locked * self.depth
        self.order_r = self.kuramoto.step(dt)
        self.field.advect(dt, self.wind_speed.value, self.wind_dir.value)
        rates = [r * self.rate_mult.value for r in self.field.rates(t)]
        self.avg_rate = sum(rates) / self.n
        self.history.append((t, self.avg_rate))
        while self.history and self.history[0][0] < t - 6:
            self.history.pop(0)
        self.client.send_frame(self.kuramoto.phase, rates, self.heat)

    def send_global(self):
        self.client.send_global(
            self.depth, self.material.value, self.wind_speed.value,
            self.wind_dir.value, self.gain_db,
        )

    def status_line(self, t):
        bar = "▁▂▃▄▅▆▇█"
        level = min(7, int(self.avg_rate / 12))
        rain_bar = bar[level] * (level + 1) + "·" * (7 - level)
        past = self.history[0][1] if self.history else self.avg_rate
        trend = "↗" if self.avg_rate > past * 1.15 + 1 else (
                "↘" if self.avg_rate < past * 0.85 - 1 else "→")
        parts = [
            f"⏱ {int(t) // 60}:{int(t) % 60:02d}",
            f"雨 {rain_bar} {self.avg_rate:4.0f}滴/s {trend}",
            f"風 {self.wind_speed.value:.2f}",
            f"素材 {material_name(self.material.value)}",
        ]
        if self.depth > 0.01:
            parts.append(f"★異変中 depth={self.depth:.2f} r={self.order_r:.2f}")
        elif self.auto and self.auto.next_anomaly:
            parts.append(f"次の異変 ~{max(0, int(self.auto.next_anomaly - t))}s")
        if any(self.heat):
            ids = [str(i + 1) for i, h in enumerate(self.heat) if h]
            parts.append("熱:" + ",".join(ids))
        sys.stdout.write("\r\033[K" + " │ ".join(parts) + " ")
        sys.stdout.flush()

    def run(self, total, on_time, stdin_repl=False):
        self.client.scene(True)
        self.send_global()
        t0 = time.monotonic()
        frame_i = 0
        next_t = t0
        try:
            while True:
                now = time.monotonic()
                t = now - t0
                if total and t > total:
                    break
                on_time(self, t)
                self.frame(t, 1.0 / FRAME_HZ)
                if frame_i % GLOBAL_EVERY == 0:
                    self.send_global()
                if frame_i % 15 == 0:  # 0.5 s
                    self.status_line(t)
                if stdin_repl and not self.repl(t):
                    break
                frame_i += 1
                next_t += 1.0 / FRAME_HZ
                time.sleep(max(0.0, next_t - time.monotonic()))
        except KeyboardInterrupt:
            pass
        finally:
            self.log("fading out...")
            self.depth = 0.0
            self.send_global()
            self.client.scene(False)
            time.sleep(2.5)

    def hold(self, t, param):
        if self.auto:
            self.auto.hold_until[param] = t + 90.0

    def repl(self, t):
        if not select.select([sys.stdin], [], [], 0)[0]:
            return True
        line = sys.stdin.readline().strip().split()
        if not line:
            return True
        cmd, arg = line[0], (line[1] if len(line) > 1 else None)
        try:
            if cmd == "q":
                return False
            elif cmd == "a":
                self.anomaly.trigger(time.monotonic())
                self.log("★ アノマリー!")
            elif cmd == "m":
                self.material.tau = 6.0  # manual: quicker than auto morphs
                self.material.target = float(arg)
                self.hold(t, "material")
                self.log(f"→ 素材を {material_name(float(arg))} へ(ゆっくり移行)")
            elif cmd == "w":
                self.wind_speed.target = float(arg)
                self.hold(t, "wind")
                self.log(f"→ 風 {float(arg):.2f} へ")
            elif cmd == "d":
                self.wind_dir.target = float(arg)
                self.hold(t, "dir")
            elif cmd == "r":
                self.rate_mult.target = float(arg)
                self.hold(t, "rain")
                self.log(f"→ 雨量 x{float(arg):.2f} へ")
            elif cmd == "h":
                i = int(arg) - 1
                self.heat[i] ^= 1
                self.log(f"熱 ロボット{arg}: {'IN' if self.heat[i] else 'OUT'}")
            elif cmd == "hm":
                self.client.set_heat_mode(int(arg))
                self.log(f"熱モード: {'umbrella(雨が止む)' if int(arg) == 0 else 'material(素材が変わる)'}")
            else:
                self.log("commands: a | m <0-3> | w <0-1> | d <rad> | r <mult> | h <id> | hm <0|1> | q")
        except (ValueError, TypeError, IndexError):
            self.log("bad argument")
        return True


# ---------------- modes ----------------

def mode_steady(sender, t):
    pass  # everything driven by the REPL


def make_mode_auto(sender, anomalies=True):
    sender.auto = AutoWeather(sender, anomalies=anomalies)
    sender.rate_mult.value = sender.rate_mult.target = 0.4  # start as light rain

    def on_time(sender, t):
        sender.auto.update(sender, t)

    return on_time


def make_mode_arc():
    """The sec-7 dramaturgy: ~6 min, normal rain throughout,
    two short sync anomalies at 3:10 and 5:00."""
    fired = set()

    def on_time(sender, t):
        sender.rate_mult.target = lerp_keyframes(
            t, [(0, 0.25), (60, 0.7), (120, 1.0), (310, 1.0), (330, 0.9), (355, 0.0)]
        )
        sender.field.contrast = lerp_keyframes(t, [(0, 2.2), (60, 1.3), (120, 1.3)])
        sender.wind_speed.target = lerp_keyframes(
            t, [(0, 0.0), (115, 0.0), (140, 0.6), (300, 0.4), (340, 0.1)]
        )
        sender.wind_dir.target = 0.4 + 0.3 * math.sin(t * 0.01)
        sender.material.target = lerp_keyframes(t, [(0, 0.0), (210, 0.0), (270, 3.0)])
        now = time.monotonic()
        if t >= 190 and "a1" not in fired:
            fired.add("a1")
            sender.anomaly.trigger(now)
            sender.log("★ アノマリー 1 (3:10)")
        if t >= 300 and "a2" not in fired:
            fired.add("a2")
            sender.anomaly.attack, sender.anomaly.hold, sender.anomaly.release = 1.5, 2.5, 4.0
            sender.anomaly.trigger(now)
            sender.log("★ アノマリー 2 (5:00)、短め")

    return on_time


def make_mode_sync():
    """Anomaly every 20 s — for debugging the phase gate."""
    state = {"next": 8.0}

    def on_time(sender, t):
        if t >= state["next"]:
            state["next"] += 20.0
            sender.anomaly.trigger(time.monotonic())
            sender.log(f"★ アノマリー t={t:.0f}s")

    return on_time


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mode", choices=["auto", "steady", "arc", "sync"], nargs="?", default="auto")
    p.add_argument("--host", default=None)
    p.add_argument("--port", type=int, default=None)
    p.add_argument("--max-rate", type=float, default=45.0, help="field peak drops/sec per robot")
    p.add_argument("--material", type=float, default=0.0)
    p.add_argument("--wind", type=float, default=0.0)
    p.add_argument("--wind-dir", type=float, default=0.0)
    p.add_argument("--gain", type=float, default=0.0, help="master gain dB")
    p.add_argument("--attack", type=float, default=2.5, help="anomaly ramp-up s")
    p.add_argument("--hold", type=float, default=5.0, help="anomaly hold s")
    p.add_argument("--release", type=float, default=5.0, help="anomaly dissolve s")
    p.add_argument("--no-anomaly", action="store_true", help="auto mode: never fire anomalies")
    p.add_argument("--duration", type=float, default=None)
    args = p.parse_args()

    sender = Sender(args)
    repl = False
    if args.mode == "auto":
        total, on_time = args.duration, make_mode_auto(sender, anomalies=not args.no_anomaly)
        repl = True
        print("自動モード: 雨量・風・素材が勝手に移り変わります(コマンドで介入も可、q で終了)")
    elif args.mode == "arc":
        total, on_time = args.duration or 360.0, make_mode_arc()
        print("running sec-7 scene arc (~6 min), Ctrl-C to stop")
    elif args.mode == "sync":
        total, on_time = args.duration, make_mode_sync()
        print("sync test: anomaly every 20 s, Ctrl-C to stop")
    else:
        total, on_time = args.duration, mode_steady
        repl = True
        print("steadyモード: 完全手動(素材の自動モーフ・自動アノマリーは起きません)")
        print("→ 自動で移り変わるモードは引数なしで:  python3 rain_test_sender.py")
        print("commands: a | m <0-3> | w <0-1> | d <rad> | r <mult> | h <id> | hm <0|1> | q")
    sender.run(total, on_time, stdin_repl=repl)


if __name__ == "__main__":
    main()
