"""縮む部屋 (press) — パントマイム・コレオグラフィのモデル。

上と下から見えない壁が迫る。筒はその間を上下にバウンスして壁を可視化し、
バウンスのたびに音が鳴る。壁は時々 ±30° まで傾き、行ったり来たりしながら
少しずつ閉じ、最後は平らに戻って筒を潰す。2秒ほど微振動して固まったら
壁は急に消え、筒はふわっとそれぞれの高さへ離散する。

純Python・依存なし (20 Hz想定)。main.py と press_test_sender.py の両方から使う。
座標系はメートル、(x, y) は作品中心からの相対位置。
"""

import math
import random

# ---------------- 定数 (チューニングはここ) ----------------
# 壁の接近 = 知覚一定: CLOSE_PCT_T 秒ごとに「現在の隙間」の CLOSE_PCT を詰める。
# 遠いほど速く (開始時 ~0.17 m/s)、狭いほど遅い指数接近。
# 指数だけだと永遠に交わらないので CLOSE_V_MIN の床で必ず閉じ切る
CLOSE_PCT      = 0.055        # 詰める割合 (closing≈45秒になる値。2026-11-03指示)
CLOSE_PCT_T    = 2.0          # その基準時間 [s]
CLOSE_V_MIN    = 0.058        # 接近速度の下限 [m/s] (closing45秒化に合わせ比率維持で縮小)
WALL_VMAX      = 0.18         # 壁速度の上限メモ [m/s] (開始時でも超えない設計)
WALL_TILT      = True         # 壁の傾斜。上下の壁は常に同一傾斜 = 必ず平行 (2026-11-03復活)
TILT_MAX_DEG   = 20.0         # 壁の最大傾斜 [deg]
TILT_MIN_DEG   = 6.0          # 傾くときの最小傾斜 [deg] (中途半端な傾きは読めない)
TILT_FLAT_P    = 0.55         # サイクル開始時に「平ら」を引く確率 (角度は開始時に決めて最後まで固定)
V_MIN, V_MAX   = 0.15, 0.20   # バウンスごとに引き直す速度 [m/s] (上限=ウィンチ全速。_slew が最終的に守る)
CHAR_SPREAD    = (0.55, 1.0)  # 筒ごとの「性格」= サイクル中ずっと持続する速さの個体差
SPEED_NOISE    = 0.35         # 飛行中の速度ゆらぎ (±35%、滑らかなノイズ)
EXCITE_HIT     = 0.35         # 1ヒットで上がる興奮 (打たれるほど興奮して速くなる)
EXCITE_TAU     = 3.0          # 興奮の冷め [s]
EXCITE_GAIN    = 0.0          # 興奮の速度倍率への寄与。0 = 速度と無関係 (色・音程にだけ効く)
HEAT_HIT       = 0.05         # 興奮したまま打たれるたび積もる白熱 (赤を超えて白へ)
HEAT_FADE_S    = 2.5          # 解放後に白熱が冷めるフェードアウト時定数 [s]
TURN_P_CALM    = 0.10         # 空中で気まぐれに向きを変える確率 [1/s] (ストレス0 = 自由遊泳)
TURN_P_PANIC   = 0.02         # ストレス最大時 (壁まで一直線 → バウンスが密になる)
TURN_CLEAR_M   = 0.25         # 反転は壁からこれ以上離れているときだけ (壁際では鳴らさず反転しない)
# 反転は1往復(壁→壁)につき最大1回。壁に必ず届くので壁の可視化が薄まらない
SPEED_CALM     = 0.85         # ストレス0のときの速度倍率 (ガンガン動く 2026-11-03)。焦るほど1.0へ
STRESS_TAU     = 1.5          # ストレス(壁の閉じ具合由来)の平滑時定数 [s]
STRESS_GAMMA   = 1.3          # ストレスの立ち上がりカーブ (>1 = 終盤に一気に焦る)
DWELL_PROB     = 0.45         # バウンス時に壁へ貼り付く(マイムの手)確率 (ストレスで減る)
DWELL_S        = (0.4, 1.6)   # 貼り付き時間 [s] (ストレスで短くなる)。壁の存在を見せる主役
IMPACT_HOLD_S  = 1.2          # 衝突後、壁の傾きに姿勢を合わせておく時間 [s]
# (目線・うなずき・倒れ込みのティルト演出は撤回 2026-11-03: 潰され中も直立のまま=平行)
SQUEEZE_S      = 2.0          # 挟まれてから弾けるまでの溜め [s] (壁は同じ法則のまま閉じ続ける)
CRUSH_GAP_M    = 0.10         # バーチャルに潰し切る壁間隔 [m] (筒30cmを貫いて潰す)
SLOT_MIN_M     = 0.008        # 壁が閉じ切っても残る微スロット半幅 [m] (跳ね続けの下限)
SND_MIN_DT     = 0.28         # 1本あたりの打音の最短間隔 [s] (動きは全ヒット、音だけ間引く)
PITCH0_HZ      = (45.0, 110.0)  # 各筒の初期音程 [Hz] (サイクル開始時にまばらな低音を配る)
PITCH_STEP_ST  = 1.5          # 1発鳴るごとに自分の音程が上がる半音数 (個体のCDJロール。上限なし)
PITCH_STEP_HOT = 3.0          # 興奮1のとき上乗せされる半音数 (終盤は1発4.5半音 → 不可聴域へ抜ける)
RELEASE_VMAX   = 0.20         # 解放後の離散の最大速度 [m/s] (弾ける。上限=ウィンチ全速)
RELEASE_K      = 0.35         # 落ち着き先への追従ゲイン [1/s] (冷めるほど効いてくる)
RELEASE_COOL_S = 9.0          # 乱雑さ(温度)の冷却時定数 [s] = エコー ~prRevOpen と同じ長さ
RELEASE_BURST  = 0.18         # 解放直後の初速 [m/s] (押しつぶされて弾ける)
RELEASE_JIT_A  = 0.22         # 温度1のときのランダム加速度 [m/s^2] (ブラウン運動の熱)
RELEASE_DAMP   = 0.35         # 速度の粘性減衰 [1/s]
RELEASE_MAX_S  = 28.0         # 離散の最長時間 [s] (保険)
SCATTER_S      = 12.0         # バラけ時間の上限 [s]。全員が行き先に着けば早く切り上げる
REST_S         = 6.0          # 離散後、次のサイクルまでの休息 [s] (バラけ時間を足した分短く)
REST_BOB_M     = 0.15         # 休息中のゆっくりした呼吸振幅 [m] (止まって見えない大きさ)
SCATTER_BOB    = 0.12         # バラけ待機中の呼吸振幅 [m] (筒ごとに周波数・位相が違う)
SCATTER_STAGGER_S = 5.0       # バラけへの出発の時差 [s] (一斉に動き出さない)
WAKE_STAGGER_S = 6.0          # closing開始後、各筒が飛行を始めるまでの時差 [s]
WAKE_RAMP_S    = 3.0          # 目覚めてから全速までのフェードイン [s]
FLASH_TAU      = 0.30         # バウンス発光の減衰 [s]
YAW_WANDER_DPS = 20.0         # 飛行中のヨー徘徊の最大角速度 [deg/s]


# 指数接近の係数: gap の相対減少率 [1/s]
CLOSE_K = -math.log(1.0 - CLOSE_PCT) / CLOSE_PCT_T


def _smoothstep(u):
    u = max(0.0, min(1.0, u))
    return u * u * (3.0 - 2.0 * u)


class _Noise1:
    """滑らかな [-1,1] ノイズ: 一定間隔で目標を引き直し、3次イージングで補間。"""

    def __init__(self, rng, t_min, t_max, start=0.0):
        self.rng, self.t_min, self.t_max = rng, t_min, t_max
        self.a, self.b = start, rng.uniform(-1.0, 1.0)
        self.t, self.dur = 0.0, rng.uniform(t_min, t_max)

    def step(self, dt):
        self.t += dt
        while self.t >= self.dur:
            self.t -= self.dur
            self.a, self.b = self.b, self.rng.uniform(-1.0, 1.0)
            self.dur = self.rng.uniform(self.t_min, self.t_max)
        return self.a + (self.b - self.a) * _smoothstep(self.t / self.dur)


class _Wall:
    """片側の壁 = 中心高さ zc + 傾斜 (ex, ey) [dz/m]。
    高さは _close_walls が、傾斜はサイクル開始時に PressModel が与える (以後固定)。"""

    def __init__(self, rng, z_start, z_end):
        self.z_start, self.z_end = z_start, z_end   # 接近の始点と潰れ切り前の基準
        self.zc = z_start                            # 現在の中心高さ
        self.sx = self.sy = 0.0                      # 傾斜 (sx,sy と ex,ey は同値)
        self.ex = self.ey = 0.0

    def z_at(self, x, y):
        return self.zc + self.ex * x + self.ey * y

    def slope_deg(self):
        return math.degrees(math.atan(math.hypot(self.ex, self.ey)))

    def azimuth_deg(self):
        return math.degrees(math.atan2(self.ey, self.ex))


class PressModel:
    """phase: scatter → closing → squeeze → release → rest → (cycle) scatter …

    step(dt) 後に読むもの:
      z/yaw/pitch/flash[i] … 筒の状態 (m / deg / deg / 0..1)
      bounces              … このフレームのバウンス [(i, vel01, gap_m, wall 1=上 0=下, hz)]
                             hz = その筒の現在の音程。鳴るたびに PITCH_STEP_ST 半音上がる
      phase_event          … None | 'squeeze' | 'release' | 'rest' | 'cycle'
      gap_c, progress      … 壁の中心間隔 [m] と閉じ具合 0..1 (音響へ)
      stress               … 0=のんびり(青) → 1=焦り(赤)。速度と色の抑揚の源
      excite[i]            … 個体の興奮 0..1 (青→赤)。heat[i] … 白熱 0..1 (赤→白、
                             興奮したまま打たれると積もり、潰しで最大、解放で冷める)
      wall_planes()        … 3D表示用 [(zc, sx, sy), (zc, sx, sy)] (上, 下) / None (非表示)
    """

    def __init__(self, positions, z0, min_z, max_z, crush_z=1.6,
                 winch_speed=0.2, agent_length=0.30, seed=None):
        self.rng = random.Random(seed)
        self.n = len(positions)
        self.px = [p[0] for p in positions]
        self.py = [p[1] for p in positions]
        self.r_max = max(1e-6, max(math.hypot(x, y) for x, y in positions))
        self.min_z, self.max_z = min_z, max_z
        self.crush_z = crush_z
        self.winch = winch_speed
        self.margin = agent_length / 2.0 + 0.04      # 筒の半分 + 余白 = 壁から筒中心までの最小距離
        self.hold_gap = 2.0 * self.margin - 0.02     # 潰し切ったときの壁間隔 (筒がわずかに食い込む)

        self.z = list(z0)
        self.yaw = [self.rng.uniform(0.0, 360.0) for _ in range(self.n)]
        self.pitch = [0.0] * self.n
        self.flash = [0.0] * self.n
        self.stress = 0.0
        self._start_cycle(first=True)

    # ---------------- サイクル管理 ----------------
    def _start_cycle(self, first=False):
        self.phase = "scatter"
        self.t = 0.0
        # バラけ先: 空間に均等 (層状スロットをシャッフル)。
        # 漂い・呼吸(±0.3m)で可動域の端に張り付いて静止しないよう内側に寄せる
        lo, hi = self.min_z + 0.35, self.max_z - 0.35
        slots = [lo + (hi - lo) * (k + 0.5) / self.n for k in range(self.n)]
        self.rng.shuffle(slots)
        self.scatter_tgt = slots
        self.sc_f = [self.rng.uniform(0.12, 0.25) for _ in range(self.n)]
        self.sc_ph = [self.rng.uniform(0.0, 2.0 * math.pi) for _ in range(self.n)]
        # 一斉に動き出さないための時差: バラけへの出発と、closing での飛行開始
        self.cycle_z0 = list(self.z)
        self.sc_go = [self.rng.uniform(0.0, SCATTER_STAGGER_S) for _ in range(self.n)]
        self.wake_t = None   # closing 開始時に各筒の目覚め時刻を配る
        top_end = self.crush_z + self.hold_gap / 2.0
        bot_end = self.crush_z - self.hold_gap / 2.0
        self.top = _Wall(self.rng, self.max_z + 0.25, top_end)
        self.bot = _Wall(self.rng, self.min_z - 0.25, bot_end)
        # 壁は _close_walls が駆動: 最初の隙間 gap0 から CRUSH_GAP_M の貫入位置まで、
        # 両壁が同時に着くよう比例配分で動く
        self.gap0 = self.top.zc - self.bot.zc
        self.top_crush = self.crush_z + CRUSH_GAP_M / 2.0
        self.bot_crush = self.crush_z - CRUSH_GAP_M / 2.0
        # 共有ティルト: サイクル開始時に1回だけ決め、潰れるまで固定。
        # 上下の壁に同じ傾き = 必ず平行。面は部屋中心(=crush_z)を支点に傾く。
        # 上限は幾何から: 潰し面の端(最遠の筒)が可動域 [min_z, max_z] を出ない角度。
        # それが TILT_MIN_DEG 未満しか取れない配置なら傾けない (平らで潰す)
        tilt_room = min(self.crush_z - self.min_z, self.max_z - self.crush_z)
        tilt_cap = math.degrees(math.atan(max(0.0, tilt_room) / self.r_max))
        a_hi = min(TILT_MAX_DEG, tilt_cap)
        if WALL_TILT and a_hi >= TILT_MIN_DEG \
                and self.rng.random() >= TILT_FLAT_P:
            a = math.radians(self.rng.uniform(TILT_MIN_DEG, a_hi))
            ph = self.rng.uniform(0.0, 2.0 * math.pi)
            self.tilt_x = math.tan(a) * math.cos(ph)
            self.tilt_y = math.tan(a) * math.sin(ph)
        else:
            self.tilt_x = self.tilt_y = 0.0
        for w in (self.top, self.bot):
            w.sx = w.ex = self.tilt_x
            w.sy = w.ey = self.tilt_y
        self.crush_done_t = None   # 閉じ切った時刻 (SQUEEZE_S の溜めの起点)
        # 飛行 = 向き fdir × 都度の速度 fmag × 性格 char × ストレス倍率 × ゆらぎ
        self.fdir = [self.rng.choice((-1.0, 1.0)) for _ in range(self.n)]
        self.fmag = [self.rng.uniform(V_MIN, V_MAX) for _ in range(self.n)]
        self.char = [self.rng.uniform(*CHAR_SPREAD) for _ in range(self.n)]
        self.v_noise = [_Noise1(self.rng, 1.5, 4.0) for _ in range(self.n)]
        self.turn_used = [False] * self.n   # 気まぐれ反転は1往復(壁→壁)に1回まで
        self.excite = [0.0] * self.n        # 興奮: ヒットで上がり冷める → 速度へ
        self.heat = [0.0] * self.n          # 白熱: 興奮状態の打撃で積もる → 色の白へ
        self.snd_last = [-1.0] * self.n     # 打音の間引き用 (最後に鳴らした時刻)
        # 個体の音程: まばらな低音からスタートし、鳴るたびに上がる (自律的なロール)
        self.snd_hz = [math.exp(self.rng.uniform(
            math.log(PITCH0_HZ[0]), math.log(PITCH0_HZ[1]))) for _ in range(self.n)]
        self.vz = [0.0] * self.n   # release フェーズ専用の積分器
        self.dwell_until = [0.0] * self.n
        self.dwell_wall = [0] * self.n
        self.impact_hold = [0.0] * self.n
        self.pitch_tgt = [0.0] * self.n
        self.yaw_tgt = list(self.yaw)
        self.yaw_noise = [_Noise1(self.rng, 2.5, 6.0) for _ in range(self.n)]
        self.bounces = []
        self.phase_event = None if first else "cycle"
        self.gap_c = self.top.zc - self.bot.zc
        self.progress = 0.0

    # ---------------- ヘルパ ----------------
    def _slew(self, i, tgt, dt, vmax=None):
        """筒 i の z を tgt へ、ウィンチ上限以内で寄せる。
        クランプは目標側に掛ける: 結果をクランプすると、前のシーンで
        可動域の外(床1.8より下など)にいた筒が初手でテレポートしてしまう。"""
        tgt = max(self.min_z, min(self.max_z, tgt))
        v = (vmax if vmax is not None else self.winch) * dt
        self.z[i] += max(-v, min(v, tgt - self.z[i]))

    def _approach_yaw(self, i, tgt, dt, tau):
        d = (tgt - self.yaw[i] + 180.0) % 360.0 - 180.0
        self.yaw[i] += d * min(1.0, dt / tau)

    # ---------------- メイン ----------------
    def step(self, dt):
        self.bounces = []
        self.phase_event = None
        self.t += dt

        for i in range(self.n):
            self.flash[i] = max(0.0, self.flash[i] - self.flash[i] * dt / FLASH_TAU)

        # ストレス: 壁の閉じ具合 (gap) から。揺り戻しで壁が開けば少しほっとする
        if self.phase == "closing":
            full = self.max_z - self.min_z
            s_raw = 1.0 - (self.gap_c - self.hold_gap) / max(1e-6, full - self.hold_gap)
            s_tgt = max(0.0, min(1.0, s_raw)) ** STRESS_GAMMA
        elif self.phase == "squeeze":
            s_tgt = 1.0
        else:
            s_tgt = 0.0
        self.stress += (s_tgt - self.stress) * min(1.0, dt / STRESS_TAU)

        if self.phase == "scatter":
            self._step_scatter(dt)
        elif self.phase == "closing":
            self._step_closing(dt)
        elif self.phase == "squeeze":
            self._step_squeeze(dt)
        elif self.phase == "release":
            self._step_release(dt)
        else:
            self._step_rest(dt)

        self.gap_c = self.top.zc - self.bot.zc
        return self.phase_event

    def _close_walls(self, dt):
        """知覚一定の接近 (closing / squeeze 共通の唯一の壁駆動):
        CLOSE_PCT_T 秒ごとに現在の隙間の CLOSE_PCT を詰める指数接近。
        遠いほど速く、狭いほど遅い。下限 CLOSE_V_MIN の床で必ず閉じ切り、
        壁は筒を貫いて CRUSH_GAP_M で止まる。"""
        gap = self.top.zc - self.bot.zc
        v = max(CLOSE_K * gap, CLOSE_V_MIN)
        gap = max(CRUSH_GAP_M, gap - v * dt)
        u = (self.gap0 - gap) / max(1e-6, self.gap0 - CRUSH_GAP_M)
        self.top.zc = self.top.z_start + (self.top_crush - self.top.z_start) * u
        self.bot.zc = self.bot.z_start + (self.bot_crush - self.bot.z_start) * u
        self.progress = max(0.0, min(1.0, u))

    def _step_scatter(self, dt):
        # サイクル冒頭: 筒が空間に均等にバラけるまでの時間。壁はまだ動かない。
        # 行き先そのものがゆっくり漂うので、着いた後も止まらない
        for i in range(self.n):
            bob = SCATTER_BOB * math.sin(2.0 * math.pi * self.sc_f[i] * self.t
                                         + self.sc_ph[i])
            # 出発は時差つき: まだの筒はその場で呼吸を続ける
            base = self.cycle_z0[i] if self.t < self.sc_go[i] else self.scatter_tgt[i]
            self._slew(i, base + bob, dt, vmax=self.fmag[i])
            self.pitch[i] += (0.0 - self.pitch[i]) * min(1.0, dt / 1.2)
            self.yaw[i] += YAW_WANDER_DPS * 0.6 * self.yaw_noise[i].step(dt) * dt
        if self.t >= SCATTER_S:
            self.phase = "closing"
            # 飛行の目覚めも時差つき (一斉にスピードが変わるとわざとらしい)
            self.wake_t = [self.t + self.rng.uniform(0.0, WAKE_STAGGER_S)
                           for _ in range(self.n)]

    def _step_closing(self, dt):
        self._close_walls(dt)

        # 速度の抑揚: のんびり(SPEED_CALM) → ストレスで全開
        self._fly(dt, SPEED_CALM + (1.0 - SPEED_CALM) * self.stress)

        # もう誰もバウンスできない隙間になったら「挟まれ」の溜めへ (SQUEEZE_S 秒)
        if (self.top.zc - self.bot.zc) <= self.hold_gap + 0.06:
            self.phase = "squeeze"
            self.t_phase0 = self.t
            self.phase_event = "squeeze"

    def _fly(self, dt, speed_mult):
        """壁の間の飛行 = このシーンの唯一の運動則。closing でも squeeze でも同じ。
        壁が近づけばスロットが縮み、跳ねる周期が勝手に震えになる (特別処理なし)。"""
        for i in range(self.n):
            x, y = self.px[i], self.py[i]
            # 壁が実機の可動域の外にある間は可動域の端が実効的な壁になる。
            # 壁が閉じ切っても SLOT_MIN_M の微スロットが残り、跳ね続けられる
            top_lim = min(self.top.z_at(x, y) - self.margin, self.max_z)
            bot_lim = max(self.bot.z_at(x, y) + self.margin, self.min_z)
            mid = (top_lim + bot_lim) / 2.0
            half = max((top_lim - bot_lim) / 2.0, SLOT_MIN_M)
            lo, hi = mid - half, mid + half
            # 音のピッチは実際の壁間隔から (潰しで筒を貫いて狭まるほど高く鳴る)
            local_gap = max(self.top.z_at(x, y) - self.bot.z_at(x, y), 0.05)

            # 興奮はゆっくり冷める (ヒットで上がる)
            self.excite[i] -= self.excite[i] * min(1.0, dt / EXCITE_TAU)

            if self.wake_t is not None and self.t < self.wake_t[i]:
                # まだ目覚めていない: バラけの呼吸のまま。壁に触れられたら目を覚ます
                bob = SCATTER_BOB * math.sin(
                    2.0 * math.pi * self.sc_f[i] * self.t + self.sc_ph[i])
                tgt = self.scatter_tgt[i] + bob
                if tgt >= hi or tgt <= lo:
                    self.wake_t[i] = self.t
                    tgt = min(max(tgt, lo), hi)
                self._slew(i, tgt, dt, vmax=self.fmag[i])
            elif self.t < self.dwell_until[i]:
                # 壁に貼り付き中 (マイムの手)。壁が動けばついていく
                self._slew(i, hi if self.dwell_wall[i] else lo, dt)
                if self.t + dt >= self.dwell_until[i]:
                    self.fdir[i] = -1.0 if self.dwell_wall[i] else 1.0
                    self.fmag[i] = self.rng.uniform(V_MIN, V_MAX)
            else:
                nmul = 1.0 + SPEED_NOISE * self.v_noise[i].step(dt)
                # 目覚めてから WAKE_RAMP_S かけて全速へフェードイン
                wake = 1.0
                if self.wake_t is not None:
                    wake = _smoothstep((self.t - self.wake_t[i]) / WAKE_RAMP_S)
                v = self.fdir[i] * self.fmag[i] * self.char[i] * speed_mult * nmul \
                    * (1.0 + EXCITE_GAIN * self.excite[i]) * wake
                z_new = self.z[i] + v * dt
                wall = None
                if v > 0 and z_new >= hi:
                    wall, lim = 1, hi
                elif v < 0 and z_new <= lo:
                    wall, lim = 0, lo
                if wall is None:
                    # 壁に触れない自由遊泳: 気まぐれに向きを変える (音は鳴らない)。
                    # のんびり時ほどよく翻り、焦るほど壁まで一直線 (打音が密になる)。
                    # 1往復1回まで: 必ずどちらかの壁に届く = 壁の可視化は薄まらない
                    turn_p = TURN_P_CALM + (TURN_P_PANIC - TURN_P_CALM) * self.stress
                    if (not self.turn_used[i]
                            and (top_lim - z_new) > TURN_CLEAR_M
                            and (z_new - bot_lim) > TURN_CLEAR_M
                            and self.rng.random() < turn_p * dt):
                        self.turn_used[i] = True
                        self.fdir[i] *= -1.0
                        self.fmag[i] = self.rng.uniform(V_MIN, V_MAX)
                    self._slew(i, z_new, dt)
                else:
                    self._slew(i, lim, dt)
                    self.turn_used[i] = False   # 壁に着いたので次の往復でまた1回使える
                    self.excite[i] = min(1.0, self.excite[i] + EXCITE_HIT)
                    # 興奮が高いまま打たれると白熱が積もる (赤を超えて白へ)
                    self.heat[i] = min(1.0, self.heat[i] + HEAT_HIT * self.excite[i])
                    vel01 = max(0.0, min(1.0, abs(v) / V_MAX))
                    # 動きは全ヒット、音だけ1本あたり SND_MIN_DT で間引く。
                    # 鳴った筒は自分の音程を半音 PITCH_STEP_ST だけ上げる
                    if self.t - self.snd_last[i] >= SND_MIN_DT:
                        self.bounces.append(
                            (i, vel01, local_gap, wall, self.snd_hz[i]))
                        self.snd_last[i] = self.t
                        self.snd_hz[i] *= 2.0 ** (
                            (PITCH_STEP_ST + PITCH_STEP_HOT * self.excite[i]) / 12.0)
                    self.flash[i] = 1.0
                    w = self.top if wall else self.bot
                    if w.slope_deg() > 5.0:
                        self.pitch_tgt[i] = w.slope_deg() * (1.0 if wall else -1.0)
                        self.yaw_tgt[i] = w.azimuth_deg()
                    else:
                        self.pitch_tgt[i] = 0.0
                    self.impact_hold[i] = self.t + IMPACT_HOLD_S
                    # 焦っているときは貼り付いて休む余裕がない
                    if self.rng.random() < DWELL_PROB * (1.0 - 0.7 * self.stress):
                        self.dwell_until[i] = self.t + \
                            self.rng.uniform(*DWELL_S) * (1.0 - 0.5 * self.stress)
                        self.dwell_wall[i] = wall
                    else:
                        self.fdir[i] = -1.0 if wall else 1.0
                        self.fmag[i] = self.rng.uniform(V_MIN, V_MAX)

            # 姿勢: 衝突直後は壁に合わせ、あとは直立へ戻る。
            # 潰され中は回転しない (ピッチ0へ、ヨーはその場のまま)
            if self.phase == "squeeze":
                self.pitch[i] += (0.0 - self.pitch[i]) * min(1.0, dt / 0.4)
            elif self.t < self.impact_hold[i] or self.t < self.dwell_until[i]:
                self.pitch[i] += (self.pitch_tgt[i] - self.pitch[i]) * min(1.0, dt / 0.25)
                self._approach_yaw(i, self.yaw_tgt[i], dt, 0.4)
            else:
                self.pitch[i] += (0.0 - self.pitch[i]) * min(1.0, dt / 1.2)
                self.yaw[i] += YAW_WANDER_DPS * self.yaw_noise[i].step(dt) * dt

    def _step_squeeze(self, dt):
        # 挟まれの溜め: 壁は closing と同じ法則のまま閉じ続け、筒は捕まって震える
        self._close_walls(dt)
        self._fly(dt, SPEED_CALM + (1.0 - SPEED_CALM) * self.stress)
        if self.t - self.t_phase0 >= SQUEEZE_S:
            self._begin_release()

    def _begin_release(self):
        # 解放: 壁は消え、各筒はそれぞれの高さへ弾けて離散
        self.phase = "release"
        self.t_phase0 = self.t
        self.phase_event = "release"
        # 休息の呼吸(±0.15m)でも端に張り付かない帯域に離散する
        lo, hi = self.min_z + 0.25, self.max_z - 0.25
        slots = [lo + (hi - lo) * (k + 0.5) / self.n for k in range(self.n)]
        self.rng.shuffle(slots)
        self.release_tgt = slots
        # 潰しのエネルギーが飛散する初速 (熱い = 乱雑)
        self.vz = [self.rng.uniform(-RELEASE_BURST, RELEASE_BURST)
                   for _ in range(self.n)]

    def _step_release(self, dt):
        # エントロピーが減っていく拡散: 解放直後は熱い乱雑なブラウン運動、
        # 温度 T が冷めるにつれ揺らぎが衰え、落ち着き先への引力だけが残って整列する。
        # 冷却時定数はエコーの残響 (9s) と同じ = 音と一緒に静まる
        ts = self.t - self.t_phase0
        T = math.exp(-ts / RELEASE_COOL_S)
        vcap = RELEASE_VMAX * (0.25 + 0.75 * T)
        done = True
        for i in range(self.n):
            err = self.release_tgt[i] - self.z[i]
            a = RELEASE_K * err * (1.0 - 0.6 * T)              # 熱いうちは目標に縛られない
            a += RELEASE_JIT_A * T * self.v_noise[i].step(dt)  # 熱 = ランダムな揺さぶり
            a -= RELEASE_DAMP * self.vz[i]                     # 粘性で冷める
            self.vz[i] += a * dt
            self.vz[i] = max(-vcap, min(vcap, self.vz[i]))
            self.z[i] += self.vz[i] * dt
            if self.z[i] <= self.min_z or self.z[i] >= self.max_z:
                self.z[i] = max(self.min_z, min(self.max_z, self.z[i]))
                self.vz[i] *= -0.4   # 可動域の端ではやわらかく跳ね返る (音は鳴らない)
            self.pitch[i] += (0.0 - self.pitch[i]) * min(1.0, dt / 1.5)
            # 白熱は解放とともに冷めてフェードアウト
            self.heat[i] -= self.heat[i] * min(1.0, dt / HEAT_FADE_S)
            # 回転も熱いうちは速く、冷めるとゆっくり
            self.yaw[i] += YAW_WANDER_DPS * (0.4 + 2.2 * T) \
                * self.yaw_noise[i].step(dt) * dt
            if abs(err) > 0.10:
                done = False
        if (done and T < 0.2) or ts > RELEASE_MAX_S:
            self.phase = "rest"
            self.t_phase0 = self.t
            self.phase_event = "rest"
            self.rest_z0 = list(self.z)
            self.rest_f = [self.rng.uniform(0.12, 0.22) for _ in range(self.n)]
            self.rest_ph = [self.rng.uniform(0.0, 2.0 * math.pi) for _ in range(self.n)]

    def _step_rest(self, dt):
        ts = self.t - self.t_phase0
        for i in range(self.n):
            bob = REST_BOB_M * math.sin(2.0 * math.pi * self.rest_f[i] * ts + self.rest_ph[i])
            self._slew(i, self.rest_z0[i] + bob, dt)
            self.heat[i] -= self.heat[i] * min(1.0, dt / HEAT_FADE_S)
            self.yaw[i] += YAW_WANDER_DPS * 0.4 * self.yaw_noise[i].step(dt) * dt
        if ts >= REST_S:
            self._start_cycle()

    # ---------------- 表示用 ----------------
    def wall_planes(self):
        """壁が存在する間だけ [(zc, sx, sy) 上, (zc, sx, sy) 下] を返す。"""
        if self.phase in ("closing", "squeeze"):
            return [(self.top.zc, self.top.ex, self.top.ey),
                    (self.bot.zc, self.bot.ex, self.bot.ey)]
        return None
