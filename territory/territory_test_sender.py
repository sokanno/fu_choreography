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
# 観客側(正面)の方位角(rad, world座標系)。NMWの正面が合わない場合はここを回す
FRONT_ANGLE = 0.0
GLOBAL_EVERY = 4          # send /terr/global every N frames (5 Hz)
MODEL_TICK = 0.08         # the widget's tick the constants were tuned at
# 対立の前線の高さ・傾き (2026-09-24)
FRONT_HT = 1.0            # 前線の降下目標 (ht 1 = 最下 1.9 m)
FRONT_TILT = 0.9          # 前線の首の傾き (+1 = 60 度下向き)
FRONT_WEIGHT = (1.0, 0.4) # 前線からのホップ数ごとの効き (0=前線, 1=隣, それ以上は0)
REAR_HT_SCALE = 0.6       # 後方の筒の降下を控えめに (前線との高低差を出す)
# ---- アルゴリズム版 (2026-09-24): 演出タイマーを使わず局所ルールだけで回す ----
# ① 前線の消耗: 相手陣営を見つめる(注意を向ける)ほど疲れ、疲れるほど耳が開く
#    (bounded confidence が広がる)。影響力は相手の確信×元気さに比例 → 弱った側の
#    前線が強い側に引き込まれて塗り替わる。疲れた筒は上がり、暗くなる。
FRONT_FATIGUE = 0.010     # 対立への曝露による疲労 (per tick, 曝露1・最下で)
OPEN_K = 0.9              # 疲労による耳の開き (eps += OPEN_K * w)
FATIGUE_DIM = 0.6         # 疲れた筒の減光 (w=1 で 1-この値)
# ② 落ち着かなさ r: 陣営の中で秩序が続くと上がり、カオスの中で下がる。
#    r が高いと意見が揺らぎ(ノイズ↑・同調↓・過激化↓)、首が回り出す → 陣営が溶ける。
#    溶けて秩序が消えると r が抜け、また結晶化する (興奮性の周期)。
REST_UP = 0.25            # 秩序の中で r が上がる速さ [1/s] (相談・メンチ込みで調整: 結晶化は全体の約7割、3分のシーンで8割超は新しい2極化)
REST_DOWN = 0.02          # カオスの中で r が下がる速さ [1/s] (遅いほどカオスが長い)
ORDER_TH = 0.3            # 局所秩序 o がこれを超えると「秩序の中」
REST_NOISE = 7.0          # r=1 で意見ノイズ ×(1+この値)
REST_SPIN = 0.6           # r=1 で首の回転(スキャン) ×(1+この値) (大きいと全員が上限に寄って速さの差が消える)
FRONT_HOLD = 0.9          # 睨み合いの緊張が落ち着かなさを抑える度合い (大きいほど硬直が長い)
SPLIT_MIN = 5             # 結晶化 = 上位2陣営がそれぞれこの台数以上で…
SPLIT_SHARE = 0.5         # …合わせて全体のこの割合以上
CONSENSUS_SHARE = 0.7     # 最大陣営がこの割合以上 = 合意(斉一)
CRYST_ENTER = 3.0         # 2陣営がこの秒数続いたら結晶化とみなす
CRYST_EXIT = 8.0          # 2陣営でない状態がこの秒数続いたら溶解とみなす
CONSULT = 0.2             # 相談: 確信の弱い筒が意見の近い隣人の方を向くトルク (向き合う→耳を傾け合う→結晶化が進む)
CONSULT_REST = 0.5        # 落ち着かなさが相談を妨げる度合い (1 だとカオス中は相談しない)
MENCHI = 0.3              # メンチ: 降りている筒が一番意見の遠い隣(相手陣営)に正面を向けて首を止めるトルク
MENCHI_HT = (0.4, 0.8)    # この高さ(ht)の範囲で効き始め→全力 (降りるほど相手を真正面に)
CHATTER_RATE = 0.4        # がやがや: 降りていない筒の小声のおしゃべり [回/s/台] (ht 0 で最大、降りるほど減る)
CHATTER_AMP = (0.12, 0.3) # その音量の範囲 (通常の発言は 0.3〜1.0)
DWELL = (1.0, 3.0)        # 底に着いた筒がそこに留まる秒数の範囲 (着くたびにランダム)
LAND_HT = 0.85            # ht がこれを上に越えたら「底に着いた」(着地音)
LAND_COOLDOWN = 8.0       # 同じ筒の着地音の最短間隔 [s]


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
        self.eps = 0.65     # bounded confidence (0.9だと陣営が合流して斉一化しやすい)
        self.mu = 0.2       # conformity
        self.alpha = 0.12   # radicalization
        self.gam = 0.12     # fatigue pull to center
        self.beta = 0.05    # global homeostasis (多数派への逆風=均衡を保つ負帰還)
        self.fr = 0.006     # fatigue rate
        self.kap = 0.035    # trait anchor (素質が多数派に飲まれにくく)
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
        self.spd0, self.spd_n = [], []
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
            # 回転の個体差 (2026-09-24): 固有の速さ(対数正規 ~0.4〜2倍) + ゆっくり揺らぐ成分
            self.spd0.append(math.exp(rng.gauss(0.0, 0.6)))
            self.spd_n.append(0.0)
        self.psi = 0.0
        self.op_unwrap = [math.atan2(self.vy[i], self.vx[i]) for i in range(n)]
        self.talk = 2.0     # アルゴリズム版で確信が下がり発話が1/4に減ったので倍に (2026-09-24)
        self.next_say = [1.0 + rng.expovariate(0.2) for _ in range(n)]
        self.R = 0.0
        self.C = 0.0
        self.camps = 0
        self.hi_time = 0.0
        self.stale_time = 0.0
        self.stale_limit = 25.0 + self.rng.random() * 30.0
        self.melt = [0.0] * n
        self.melt_fired = False
        self.melt_active_until = -1.0
        self._melt_cap = [1.0] * n       # spatial attenuation from the melt seed
        # last_shift starts at 0 (not -inf): the deadlock guard must not fire
        # into the opening chaos before the first camps have even formed
        self.last_shift = 0.0
        self.prov = [0.0] * n            # provocation intensity per tube
        self.next_prov = 6.0 + self.rng.random() * 8.0
        self.prov_event = None           # ('lunge'|'defect', idx) this step
        self.clusters = []               # camp membership (filled each step)
        self.schism_side = [0.0] * n     # ±1 faction inside the splitting camp
        self.schism_active = False
        self.schism_t0 = 0.0
        self.schism_perp = 0.0
        self.schism_event = None         # ('begin',size)|('secede',)|('fizzle',)
        self.split_count = 0
        self._deaf = 0.0                 # cross-faction deafness during schism
        self._fizzle_streak = 0
        self.schism_mild = False         # 小派閥: grumbling that usually settles
        self._mild_dur = 0.0
        self.kap2 = 0.010                # 個性の常時圧: pull toward own 2D trait
        self.front_dist = [0] * n        # graph hops from the nearest front tube
        self.front_hops = [99] * n       # 同上だが前線が無いときは 99 (高さ・傾き用)
        self.brightness = [1.0] * n      # rear echelons dim as the standoff hardens
        self.rest = [self.rng.random() * 0.5 for _ in range(n)]  # 落ち着かなさ r (②)
        self.order = [0.0] * n           # 局所秩序 o (確信×近傍との一致)
        self.expo = [0.0] * n            # 対立への曝露 (①)
        self.camp_of = [-1] * n          # 所属陣営 (clusters の添字, -1 = 無所属)
        self.camp_sig = [None] * n       # 所属陣営の代表方向 (寝返り検出用)
        self.cryst = False               # 結晶化している (大きな2陣営が持続)
        self.state_now = 'chaos'
        self._cryst_t = 0.0
        self.phase_event = None          # 'crystallize' | 'dissolve' (検出のみ) this step
        self.converts = []               # この step で陣営を乗り換えた筒 [(i, 旧方向, 新方向)]
        self.break_event = None          # 旧・押し込み版との互換 (常に None)
        self.lands = []                  # この step で底に着いた筒
        self._last_land = [-99.0] * n
        self._last_conv = [-99.0] * n
        self._dwell_until = [-99.0] * n
        self.opp = [-1] * n              # メンチを切っている相手
        self.next_chat = [self.rng.random() * 4.0 for _ in range(n)]   # がやがやの次の発話時刻
        self.t = 0.0
        self.shift_count = 0

    def do_shift(self):
        jump = (math.pi / 3 + self.rng.random() * math.pi / 3)
        jump *= 1 if self.rng.random() < 0.5 else -1
        self.psi += jump
        for i in range(self.n):
            self.vx[i] *= 0.45
            self.vy[i] *= 0.45
            self.w[i] *= 0.5
            self.spin[i] = 1 if self.rng.random() < 0.5 else -1
            self.om[i] = self.spin[i] * (0.5 + self.rng.random()) * max(self.scan, 0.04)
        self.last_shift = self.t
        self.shift_count += 1
        self.stale_time = 0.0
        self.stale_limit = 25.0 + self.rng.random() * 30.0
        return jump

    def do_melt(self):
        """Front-line melt: pick one spot on the front and let the boundary
        liquefy from there (contagious via step()). Returns False if there is
        no front to melt."""
        front = []
        for i in range(self.n):
            for j in self.nbr[i]:
                if math.hypot(self.vx[i] - self.vx[j],
                              self.vy[i] - self.vy[j]) > 1.0:
                    front.append(i)
                    break
        if not front:
            return False
        seed = self.rng.choice(front)
        self.melt[seed] = 0.8
        for j in self.nbr[seed]:
            if j in front:
                self.melt[j] = max(self.melt[j], 0.45)
        # spatial attenuation: the liquefaction stays a local incident
        # (~3 rings around the seed), never a whole-floor dissolution
        hops = [99] * self.n
        hops[seed] = 0
        queue = [seed]
        qi = 0
        while qi < len(queue):
            a = queue[qi]
            qi += 1
            for b in self.nbr[a]:
                if hops[b] > hops[a] + 1:
                    hops[b] = hops[a] + 1
                    queue.append(b)
        self._melt_cap = [max(0.0, 1.0 - 0.28 * h) for h in hops]
        # contagion only lives for a bounded window, then the border heals —
        # without this the epidemic self-sustains and the society never reforms
        self.melt_active_until = self.t + 8.0 + self.rng.random() * 5.0
        self.last_shift = self.t          # shares the cooldown
        self.stale_time = 0.0
        self.stale_limit = 25.0 + self.rng.random() * 30.0
        return True

    def camp_pcs(self):
        """大きい2陣営の平均意見の音高 (pitch class)。無ければ None。"""
        cl = sorted(self.clusters, key=len, reverse=True)[:2]
        if len(cl) < 2:
            return None
        return [pitch_class(math.atan2(sum(self.vy[i] for i in c), sum(self.vx[i] for i in c)))
                for c in cl]

    def _front_agents(self):
        front = []
        for i in range(self.n):
            for j in self.nbr[i]:
                if math.hypot(self.vx[i] - self.vx[j],
                              self.vy[i] - self.vy[j]) > 1.0:
                    front.append(i)
                    break
        return front

    def do_provocation(self):
        """揺さぶり: during a standoff someone always tries something.
        lunge  — a convinced front tube surges (conviction burst, speaks at
                 once, the opposing neighbors sharpen in response)
        defect — a weak/tired front tube flips to the other side"""
        front = self._front_agents()
        if not front:
            return None
        conv = {i: math.hypot(self.vx[i], self.vy[i]) for i in front}
        roll = self.rng.random()
        if roll < 0.4 and not self.schism_active:
            # 小派閥: the majority starts grumbling — a gradient opens inside
            # its color and usually settles; a loose camp can tip into a real
            # secession by accident
            if self.begin_schism(min_size=5, mild=True):
                return None
            roll = 0.4 + self.rng.random() * 0.6
        if roll < 0.7:
            cands = [i for i in front if conv[i] > 0.5]
            if not cands:
                return None
            i = self.rng.choice(cands)
            self.prov[i] = 1.0
            for j in self.nbr[i]:
                if math.hypot(self.vx[i] - self.vx[j],
                              self.vy[i] - self.vy[j]) > 1.0:
                    self.prov[j] = max(self.prov[j], 0.5)
            self.next_say[i] = self.t    # the lunge speaks immediately
            return ('lunge', i)
        else:
            cands = [i for i in front if conv[i] < 0.5 or self.w[i] > 0.5]
            if not cands:
                return None
            i = self.rng.choice(cands)
            self.vx[i] *= -0.55
            self.vy[i] *= -0.55
            self.w[i] *= 0.5
            self.spin[i] = 1 if self.rng.random() < 0.5 else -1
            self.next_say[i] = self.t    # the defector announces itself
            return ('defect', i)

    def begin_schism(self, min_size=6, mild=False):
        """多数派分裂の始まり: factions form inside the largest camp along its
        latent (orthogonal) axis, sides chosen by each member's hidden trait.
        No jump — a growing factional pull first paints a gradient inside the
        camp's color; once the separation beats the confidence bound ε the
        subgroups stop hearing each other and secede on their own (handled in
        step()); if conformity wins instead, the schism fizzles (和解)."""
        if self.schism_active or not self.clusters:
            return False
        big = max(self.clusters, key=len)
        if len(big) < min_size:
            return False
        mvx = sum(self.vx[i] for i in big) / len(big)
        mvy = sum(self.vy[i] for i in big) / len(big)
        perp = math.atan2(mvy, mvx) + math.pi / 2 + self.rng.uniform(-0.3, 0.3)
        px_, py_ = math.cos(perp), math.sin(perp)
        self.schism_side = [0.0] * self.n
        for i in big:
            tp = self.tx0[i] * px_ + self.ty0[i] * py_
            if abs(tp) < 0.1:
                self.schism_side[i] = 1.0 if self.rng.random() < 0.5 else -1.0
            else:
                self.schism_side[i] = 1.0 if tp > 0 else -1.0
        self.schism_active = True
        self.schism_mild = mild
        self.schism_t0 = self.t
        self.schism_perp = perp
        if mild:
            # texture, not resolution: keep stalemate pressure running
            self._mild_dur = 12.0 + self.rng.random() * 10.0
            self.schism_event = ('grumble', len(big))
        else:
            self.last_shift = self.t
            self.stale_time = 0.0
            self.schism_event = ('begin', len(big))
        return True

    def _schism_update(self, f):
        """Escalating factional pull; detect secession or reconciliation."""
        px_, py_ = math.cos(self.schism_perp), math.sin(self.schism_perp)
        age = self.t - self.schism_t0
        # mild grumbling pulls weakly and never escalates; a real schism does
        g = 0.006 if self.schism_mild else (0.014 + 0.0015 * age)
        ax = ay = an = bx = by = bn = 0.0
        for i in range(self.n):
            s = self.schism_side[i]
            if s == 0.0:
                continue
            self.vx[i] += f * g * s * px_
            self.vy[i] += f * g * s * py_
            nm = math.hypot(self.vx[i], self.vy[i])
            if nm > 1:
                self.vx[i] /= nm
                self.vy[i] /= nm
            if s > 0:
                ax += self.vx[i]; ay += self.vy[i]; an += 1
            else:
                bx += self.vx[i]; by += self.vy[i]; bn += 1
        if an == 0 or bn == 0:
            self.schism_active = False
            self.schism_event = ('fizzle',)
            return
        sep = math.hypot(ax / an - bx / bn, ay / an - by / bn)
        if sep > 0.9:
            # secession: the factional rift becomes THE conflict
            self.psi = self.schism_perp
            self.schism_active = False
            self.schism_side = [0.0] * self.n
            self.split_count += 1
            self._fizzle_streak = 0
            self.last_shift = self.t
            self.stale_limit = 25.0 + self.rng.random() * 30.0
            self.schism_event = ('secede',)
        elif self.schism_mild and age > self._mild_dur:
            # the grumbling settles — shades close back into the party color
            self.schism_active = False
            self.schism_side = [0.0] * self.n
            self.schism_event = ('settle',)
        elif (not self.schism_mild) and age > 40.0:
            # conformity held: the party closes ranks
            self.schism_active = False
            self.schism_side = [0.0] * self.n
            self._fizzle_streak += 1
            self.last_shift = self.t
            self.schism_event = ('fizzle',)

    def step(self, dt):
        """Advance the model. Returns (ticks, auto_shifted): ticks is a list of
        (idx, vel 0..1) yaw half-turn crossings this step."""
        f = dt / MODEL_TICK
        n = self.n
        self.t += dt
        self.schism_event = None
        # factions grow deaf to each other as a REAL schism ages (echo
        # chambers); mild grumbling stays within earshot
        self._deaf = (0.6 * min(1.0, (self.t - self.schism_t0) / 20.0)
                      if (self.schism_active and not self.schism_mild) else 0.0)
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
            t_ali = t_stare = t_cons = 0.0
            opp, opp_d = -1, 0.8               # メンチの相手: 意見が一番遠い隣人
            # melting tubes hear across the front, stop radicalizing and
            # bleed conviction — the boundary liquefies locally
            mi = self.melt[i]
            ri = self.rest[i]
            # ① 疲れるほど耳が開く
            eps_i = self.eps + 1.2 * mi + OPEN_K * self.w[i]
            ex_sum = att_sum = agree = 0.0
            for j in self.nbr[i]:
                bij = self.bear[i][j]
                L = ((1 + math.cos(angd(bij, self.th[i]))) / 2) ** self.k_att
                P = ((1 + math.cos(angd(self.bear[j][i], self.th[j]))) / 2) ** self.k_att
                dxo = self.vx[j] - self.vx[i]
                dyo = self.vy[j] - self.vy[i]
                d = math.hypot(dxo, dyo)
                # ① 影響力は相手の確信×元気さに比例 (強い側が弱った相手を引き込む)
                mj = math.hypot(self.vx[j], self.vy[j])
                wt = L * (0.15 + 0.85 * P) * (0.25 + 0.75 * mj * (1 - self.w[j]))
                # 対立への曝露: 隣にいる相手陣営との意見の遠さ (向きに依らない。
                # 見つめる=メンチで疲れが加速すると戦いが一瞬で終わってしまうため)
                ex_sum += min(1.0, max(0.0, (d - 0.8) / 0.5))
                att_sum += 1.0
                if d < 0.4:
                    agree += 1.0                    # 意見の近い隣人
                eff_eps = eps_i
                if self.schism_side[i] * self.schism_side[j] < 0:
                    eff_eps *= (1 - self._deaf)
                if d < eff_eps:
                    pxl += wt * dxo
                    pyl += wt * dyo
                    wsum += wt
                sim = math.exp(-d * d * 2)
                t_ali += self.ali * sim * math.sin(angd(self.th[j], self.th[i]))
                t_stare += self.stare * (d / 2) * math.sin(angd(bij, self.th[i]))
                # 相談: 意見の近い相手の方を向く (相手もこちらを向けば見つめ合いになる)
                t_cons += CONSULT * sim * math.sin(angd(bij, self.th[i]))
                if d > opp_d:
                    opp, opp_d = j, d
            mu_i = self.mu * (1 - 0.6 * ri)          # ② 落ち着かない筒は同調しにくい
            if wsum > 0:
                pxl = mu_i * pxl / wsum
                pyl = mu_i * pyl / wsum
            m2 = self.vx[i] ** 2 + self.vy[i] ** 2
            mag = math.sqrt(m2)
            radf = self.alpha * (1 - 2 * self.w[i]) * (1 - m2) * (1 - mi) * (1 - ri)
            nb_n = max(1, len(self.nbr[i]))
            self.expo[i] = ex_sum / att_sum if att_sum > 0 else 0.0
            self.order[i] = mag * agree / nb_n     # 局所秩序: 確信 × 近い隣人の割合
            noise_i = self.eta * (1 + 1.5 * mi) * (1 + REST_NOISE * ri * ri)
            tproj = self.tx0[i] * ex + self.ty0[i] * ey
            vproj = self.vx[i] * ex + self.vy[i] * ey
            anch = self.kap * (tproj - vproj)
            # 個性の常時圧: everyone leans a little toward who they are, so a
            # camp never fully unifies — its interior keeps a faint gradient
            nx = self.vx[i] + f * (
                pxl + radf * self.vx[i] - self.gam * self.w[i] * self.vx[i]
                - 0.08 * mi * self.vx[i]
                + anch * ex - self.beta * mx
                + self.kap2 * (self.tx0[i] - self.vx[i])
                + self.rng.uniform(-1, 1) * noise_i
            )
            ny = self.vy[i] + f * (
                pyl + radf * self.vy[i] - self.gam * self.w[i] * self.vy[i]
                - 0.08 * mi * self.vy[i]
                + anch * ey - self.beta * my
                + self.kap2 * (self.ty0[i] - self.vy[i])
                + self.rng.uniform(-1, 1) * noise_i
            )
            # damp the off-axis component (the salient axis owns the debate);
            # schism members are exempt — their revolt IS off-axis
            perp = nx * (-ey) + ny * ex
            pdamp = 0.1 * (1 - 0.9 * abs(self.schism_side[i]))
            nx -= perp * pdamp * f * (-ey)
            ny -= perp * pdamp * f * ex
            nm = math.hypot(nx, ny)
            if nm > 1:
                nx /= nm
                ny /= nm
            nvx[i], nvy[i] = nx, ny
            self.w[i] = min(1.0, max(0.0, self.w[i] + f * self.fr * (m2 - 0.25)
                                     * (1 + self.unif * self.R * self.C)
                                     + f * FRONT_FATIGUE * self.expo[i] * mag * (0.5 + self.ht[i])))
            # a provoked tube surges: conviction pushes outward, gaze locks
            pi = self.prov[i]
            if pi > 0.0 and mag > 1e-4:
                nx += f * 0.14 * pi * self.vx[i] / mag
                ny += f * 0.14 * pi * self.vy[i] / mag
                nm = math.hypot(nx, ny)
                if nm > 1:
                    nx /= nm
                    ny /= nm
                nvx[i], nvy[i] = nx, ny
            engage = max(0.0, mag * (1 - self.w[i]) * (1 - 0.7 * mi))
            engage = min(1.0, engage * (1 + pi)) * (1 - 0.8 * ri)
            # 固有の速さ × ゆっくり揺らぐ速さ (OU過程) — 全員が同じ速さで回らないように
            self.spd_n[i] += -self.spd_n[i] * 0.02 * f + self.rng.gauss(0.0, 0.09) * math.sqrt(f)
            spd = self.spd0[i] * math.exp(max(-1.2, min(1.2, self.spd_n[i])))
            # ときどき回転の向きが変わる (落ち着かない筒ほど頻繁)
            if self.rng.random() < dt * (0.01 + 0.05 * ri):
                self.spin[i] = -self.spin[i]
            t_scan = 0.15 * (self.spin[i] * self.scan * spd * (1 + REST_SPIN * ri) - self.om[i])
            # 確信が弱い(=まだ結晶化していない)ほど相談、強いほど整列/睨み (落ち着かない筒は相談しない)
            cw = max(0.0, 1.0 - mag / 0.55) * (1 - CONSULT_REST * ri) * (1 - engage)
            torque = ((1 - engage) * (1 - cw) * t_scan + cw * t_cons
                      + engage * (t_ali + t_stare))
            # メンチ: 降りている(戦っている)筒は相手陣営の一人に真正面を向け、そこで首を止める
            mw = min(1.0, max(0.0, (self.ht[i] - MENCHI_HT[0]) / (MENCHI_HT[1] - MENCHI_HT[0])))
            # 一度睨んだ相手は、意見が離れている限り睨み続ける (相手を探して首が揺れないように)
            po = self.opp[i]
            if po >= 0 and po in self.bear[i] and \
                    math.hypot(self.vx[po] - self.vx[i], self.vy[po] - self.vy[i]) > 0.7:
                opp = po
            self.opp[i] = opp if mw > 0.0 else -1
            if opp >= 0 and mw > 0.0:
                t_men = MENCHI * math.sin(angd(self.bear[i][opp], self.th[i])) - 0.5 * self.om[i]
                torque = (1 - mw) * torque + mw * t_men
            self.om[i] += f * (torque + self.rng.uniform(-1, 1) * 0.004 * (1 - 0.8 * mw))
            self.om[i] *= 0.93 ** f
            self.om[i] = 0.15 * math.tanh(self.om[i] / 0.15)   # 上限は柔らかく (張り付いて全員同じ速さにならない)
        self.vx, self.vy = nvx, nvy

        # ② 落ち着かなさ: 秩序の中で溜まり、カオスの中で抜ける
        if self.auto_shift:
            for i in range(n):
                if self.order[i] > ORDER_TH:
                    # 睨み合っている筒(曝露が高い)は緊張で落ち着かなさが溜まりにくい
                    # → 対立は前線の緊張で硬直し、後方の退屈から崩れていく
                    self.rest[i] = min(1.0, self.rest[i] + dt * REST_UP
                                       * (self.order[i] - ORDER_TH) / (1 - ORDER_TH) * 2
                                       * (1 - FRONT_HOLD * min(1.0, self.expo[i] * 2)))
                else:
                    self.rest[i] = max(0.0, self.rest[i] - dt * REST_DOWN)
            # 隣どうしで少しだけ伝染 (溶け方・固まり方が面で広がる)
            r0 = self.rest[:]
            for i in range(n):
                nb = self.nbr[i]
                if nb:
                    self.rest[i] += (sum(r0[j] for j in nb) / len(nb) - r0[i]) * min(1.0, 0.02 * f)

        # ongoing schism: factional pull, secession / reconciliation check
        if self.schism_active:
            self._schism_update(f)

        # provocation decay (~3-4 s)
        if any(p > 0.0 for p in self.prov):
            for i in range(n):
                p = self.prov[i] * (0.985 ** f)
                self.prov[i] = p if p >= 0.01 else 0.0

        # melt contagion + decay: the liquefaction creeps to neighbors while
        # the active window lasts, then only heals and the border recrystallizes
        if any(m > 0.0 for m in self.melt):
            active = self.t < self.melt_active_until
            decay = 0.994 if active else 0.988
            old_melt = self.melt
            new_melt = old_melt[:]
            for i in range(n):
                grown = old_melt[i]
                if active:
                    nb = max((old_melt[j] for j in self.nbr[i]), default=0.0)
                    grown = min(self._melt_cap[i],
                                grown + f * 0.045 * nb * (1 - grown))
                grown *= decay ** f
                new_melt[i] = grown if grown >= 0.003 else 0.0
            self.melt = new_melt

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
        # 対立の前線 (2026-09-24): 前線の筒がいちばん下まで降り、首も鋭く傾く。
        # 前線から離れた後方は控えめに。front_hops は1ステップ前の値 (前線が無ければ全員 99)
        standoff = self.cryst and min(self.front_hops) == 0
        self.lands = []
        for i in range(n):
            mag = math.hypot(self.vx[i], self.vy[i])
            ht_t = mag * (1 - 0.7 * self.w[i])
            tl_t = max(-1.0, min(1.0, 1.2 * mag * (1 - self.w[i]) - 0.9 * self.w[i]))
            if standoff:
                fr = (FRONT_WEIGHT[self.front_hops[i]]
                      if self.front_hops[i] < len(FRONT_WEIGHT) else 0.0)
                fresh = 1 - 0.6 * self.w[i]          # 疲れた前線は降りきらない
                ht_t = (1 - fr) * ht_t * REAR_HT_SCALE + fr * max(ht_t, FRONT_HT * fresh)
                tl_t = (1 - fr) * tl_t + fr * max(tl_t, FRONT_TILT * fresh)
            if self.t < self._dwell_until[i]:
                ht_t = max(ht_t, 1.0)          # 底に着いたらしばらく留まる
            _h0 = self.ht[i]
            self.ht[i] += (ht_t - self.ht[i]) * (1 - math.exp(-dt / 2.0))
            if _h0 <= LAND_HT < self.ht[i]:
                # 底に着くたびに 1〜3 秒留まる (着地イベントのクールダウンとは無関係)
                self._dwell_until[i] = self.t + DWELL[0] + self.rng.random() * (DWELL[1] - DWELL[0])
                if self.t - self._last_land[i] > LAND_COOLDOWN:
                    self.lands.append(i)
                    self._last_land[i] = self.t
            self.tilt[i] += (tl_t - self.tilt[i]) * (1 - math.exp(-dt / 1.5))

        # emergent camps with membership (opinion-space clustering)
        used = [False] * n
        clusters = []
        for i in range(n):
            if used[i] or math.hypot(self.vx[i], self.vy[i]) < 0.35:
                continue
            cur = [i]
            used[i] = True
            stack = [i]
            while stack:
                a = stack.pop()
                for b in range(n):
                    if used[b] or math.hypot(self.vx[b], self.vy[b]) < 0.35:
                        continue
                    if math.hypot(self.vx[a] - self.vx[b],
                                  self.vy[a] - self.vy[b]) < 0.5:
                        used[b] = True
                        stack.append(b)
                        cur.append(b)
            clusters.append(cur)
        self.clusters = clusters
        self.camps = len(clusters)
        # 社会の状態: 'split'(2陣営に結晶化) / 'consensus'(斉一) / 'chaos'
        sz = sorted((len(c) for c in clusters), reverse=True) + [0, 0]
        if sz[0] >= CONSENSUS_SHARE * n:
            self.state_now = 'consensus'
        elif sz[1] >= SPLIT_MIN and sz[0] + sz[1] >= SPLIT_SHARE * n:
            self.state_now = 'split'
        else:
            self.state_now = 'chaos'

        # 揺さぶり(挑発)のタイマーは廃止 (アルゴリズム版)。寝返りは①で自然に起きる
        self.prov_event = None

        # 陣営の乗り換え(寝返り)を検出: 陣営の代表方向が大きく変わった筒
        self.converts = []
        camp_of = [-1] * n
        sig = [None] * n
        for ci, cl in enumerate(clusters):
            cxs = sum(self.vx[i] for i in cl) / len(cl)
            cys = sum(self.vy[i] for i in cl) / len(cl)
            for i in cl:
                camp_of[i] = ci
                sig[i] = math.atan2(cys, cxs)
        for i in range(n):
            # 寝返り = 陣営に属したまま、所属陣営の方向が120°以上変わった
            if (sig[i] is not None and self.camp_sig[i] is not None and self.cryst
                    and abs(angd(sig[i], self.camp_sig[i])) > 2.1
                    and math.hypot(self.vx[i], self.vy[i]) > 0.45
                    and self.t - self._last_conv[i] > 5.0):     # 前線での行ったり来たりは数えない
                self.converts.append((i, self.camp_sig[i], sig[i]))
                self._last_conv[i] = self.t
                self.next_say[i] = self.t          # 寝返った筒は声を上げる
            if sig[i] is not None and math.hypot(self.vx[i], self.vy[i]) > 0.45:
                self.camp_sig[i] = sig[i]
        self.camp_of = camp_of

        # rear-echelon dimming: the front keeps its light, the hinterland
        # fades as the standoff hardens (rig follows stale_time, 1 step behind)
        rig = min(1.0, self.stale_time / 30.0)
        front_now = self._front_agents() if self.cryst else []
        if front_now:
            dist = [99] * n
            queue = list(front_now)
            for i in front_now:
                dist[i] = 0
            qi = 0
            while qi < len(queue):
                a = queue[qi]
                qi += 1
                for b in self.nbr[a]:
                    if dist[b] > dist[a] + 1:
                        dist[b] = dist[a] + 1
                        queue.append(b)
            self.front_dist = dist
        else:
            self.front_dist = [0] * n
        self.front_hops = self.front_dist[:] if front_now else [99] * n
        for i in range(n):
            if front_now:
                bt = 1.0 - 0.8 * rig * min(1.0, self.front_dist[i] * 0.35)
            else:
                bt = 1.0
            bt *= 1.0 - FATIGUE_DIM * self.w[i] * self.w[i]    # 疲れた筒は暗くなる
            self.brightness[i] += (bt - self.brightness[i]) * (1 - math.exp(-dt / 2.0))

        # 演出タイマー(斉一化→分裂, 膠着→溶解/分裂, 強制分裂)は廃止 (アルゴリズム版)。
        # 結晶化・溶解は起きたことを検出するだけ (音の合図用)
        auto_shifted = False
        self.melt_fired = False
        self.phase_event = None
        if self.R > 0.85 or self.C > 0.85:
            self.hi_time += dt
        else:
            self.hi_time = 0.0
        if self.camps >= 2:
            self.stale_time += dt
        else:
            self.stale_time = max(0.0, self.stale_time - 2.0 * dt)
        want = self.state_now == 'split'
        if want != self.cryst:
            self._cryst_t += dt
            if self._cryst_t > (CRYST_ENTER if want else CRYST_EXIT):
                self.cryst = want
                self._cryst_t = 0.0
                self.phase_event = 'crystallize' if want else 'dissolve'
        else:
            self._cryst_t = 0.0
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
        # がやがや: 降りていない筒は小声でおしゃべりする (迷っている筒も。言い返しの連鎖は起こさない)
        for i in range(self.n):
            if now < self.next_chat[i]:
                continue
            up = 1.0 - self.ht[i]
            lam = CHATTER_RATE * up * up
            self.next_chat[i] = now + min(30.0, self.rng.expovariate(max(lam, 1e-3)))
            if up < 0.3:
                continue
            dur = 0.3 + 0.9 * self.rng.random()
            amp = CHATTER_AMP[0] + (CHATTER_AMP[1] - CHATTER_AMP[0]) * self.rng.random() * up
            out.append((i, dur, amp))
        return out

    # --- state vectors for OSC ---
    def facing(self, front_angle=0.0):
        """0..1 per tube: how much the head points toward the audience side.
        Drives the voice directivity (bright when facing, veiled when turned
        away) — rotation becomes audible as a sweeping beam of voice."""
        return [(1 + math.cos(self.th[i] - front_angle)) / 2
                for i in range(self.n)]

    def fat_out(self):
        """Fatigue as sent to the engine: melting voices also destabilize."""
        return [min(1.0, self.w[i] + 0.5 * self.melt[i]) for i in range(self.n)]

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
        if m.melt_fired:
            self.client.melt()
            self.log("前線溶解: 境界が液状化していく")
        if m.phase_event == 'crystallize':
            pcs = m.camp_pcs()
            if pcs:
                self.client.braam(*pcs)
            self.log(f"結晶化: {m.camps}陣営に割れた")
        elif m.phase_event == 'dissolve':
            self.client.melt()
            self.log("溶解: 陣営がほどけてカオスへ")
        for i in m.lands:
            self.client.land(i, m.op_unwrap[i])
        for i, _a, _b in m.converts:
            self.log("寝返り: 筒%dが相手陣営へ" % (i + 1))
        if m.prov_event:
            kind, idx = m.prov_event
            self.log(("挑発: 筒%dが突出" if kind == 'lunge'
                      else "寝返り: 筒%dが転向") % (idx + 1))
        if m.schism_event:
            ev = m.schism_event
            if ev[0] == 'begin':
                self.client.split(0)
                self.log(f"派閥形成: 多数派{ev[1]}体の内部に亀裂")
            elif ev[0] == 'grumble':
                self.client.split(0)
                self.log(f"小派閥: {ev[1]}体の党内に不満がくすぶる")
            elif ev[0] == 'secede':
                self.client.split(1)
                self.log("離脱: 新党が結成され、争点が置き換わった")
            elif ev[0] == 'settle':
                self.log("小派閥は収まり、色が閉じていく")
            else:
                self.log("分裂回避: 党は結束を取り戻した")
        self.client.send_state(
            m.op_unwrap, m.conviction(), m.fat_out(), m.ht, m.tilt,
            m.omega_rad_s(), m.facing(FRONT_ANGLE)
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
