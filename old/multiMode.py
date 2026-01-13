from vpython import *
scene.visible = False
scene.width = scene.height = 0

import math, random, struct, csv
from noise import pnoise2
import paho.mqtt.client as mqtt

import colorsys
import random
from math import radians, degrees, sin, cos


# ─── モード切替トランジション用 ─────────────────────
is_transitioning   = False
transition_start   = 0.0
transition_duration = 2.0   # 切り替えを何秒かけるか

# ========================================================
# グローバルパラメータ（初期値）
# ========================================================
separationFactor = 0.14
cohesionFactor   = 1.76
rotationSpeed    = 0.001
radius           = 5.5
cameraHeight     = 1.0

noiseScale   = 0.01
noiseSpeed   = 0.01
waveScale    = 0.2
waveStrength = 0.1
flow_target_height = 2.5

showPole = True

# LEDs は白一色
led_color = vector(1,1,1)

# 新規追加パラメータ（Appearance Control）
bg3d_brightness = 0.1   # 3D背景の明るさ (0=真っ黒, 1=真っ白)
bg2d_brightness = 0.1   # 2D背景の明るさ
led_amp         = 1.0   # LED 波の振幅スケーラ

# ========================================================
# HSB カラー制御パラメータ（0.0～1.0）
# ========================================================
base_hue   = 0.2   # 虹の開始位置 (0=赤, 0.33=緑, 0.66=青, …)
hue_span   = 0.07   # 使う虹の幅 (1.0なら全域、0.5なら半分だけ)
hue_vari   = 0.1   # 波動による微揺らぎ（既存）
hue_speed  = 1.0   # 波動の速さ（既存）
waveScale  = 0.2   # 波の空間スケール（既存）
noise_time = 0.0   # 時間カウンタ（既存）
led_amp    = 1.0   # 波の振幅（既存）
base_sat   = 1.0   # ← 追加：彩度のデフォルト

# ========================================================
# 定数
# ========================================================
ROWS, COLS     = 7, 7
minZ, maxZ     = 0.0, 2.8
sepDist        = 0.4
agent_length   = 0.30
agent_radius   = 0.075/2
cable_radius   = 0.01
led_radius     = agent_radius

# ========================================================
# 天上天下唯我独尊モード用パラメータ（グローバル初期値）
# ========================================================
# groupA_count     = 1                   # A グループの台数（残りは B）
sine_freq   = 0.3                 # 正弦波の周波数 [Hz]
sine_amp   = (maxZ - minZ) / 4   # 正弦波の振幅
color_speed      = 0.1                 # LED 色変化の速度

# スライダーから直接書き換える「目標値」
target_sine_amp  = sine_amp
target_sine_freq = sine_freq

# update_tenge 内で実際に使う「現在値」
current_sine_amp  = target_sine_amp
current_sine_freq = target_sine_freq

# ========================================================
# 回る天井モード用パラメータ
# ========================================================
tilt_angle_deg   = 10.0    # 平面の傾き（°）
plane_rot_speed  = 1.0     # 回転速度（rad/s）
plane_angle      = 0.0     # フレームごとに増加
plane_height     = 2.5     # 平面中心の高さ



# ========================================================
# MQTT セットアップ
# ========================================================
mqtt_client = mqtt.Client()
mqtt_client.connect("127.0.0.1", 1883, 60)
mqtt_client.loop_start()

# ========================================================
# 3Dビュー（左上）
# ========================================================
scene3d = canvas(
    width=800, height=533,
    background=color.gray(0.2),
    caption="",
    align='right'
)
scene3d.up       = vector(0,0,1)
scene3d.forward  = vector(-1,-1,-0.2)
scene3d.userspin = False

# ========================================================
# 上からビュー（右上）
# ========================================================
scene2d = canvas(
    width=600, height=400,
    background=color.gray(0.2),
    caption="",
    align='right'
)
scene2d.camera.projection = "orthographic"
scene2d.up            = vector(1,0,0)
scene2d.camera.axis   = vector(0,0,-1)
scene2d.lights        = []

# ========================================================
# 床 (floor) の作成
# ========================================================
# 画面上の全エージェント範囲をあとで計算して centerX/centerY/span をセット
# → 先にダミーで作成しておき、後でサイズと位置を調整します
floor = box(canvas=scene3d,
            pos=vector(0,0,-0.01),
            size=vector(6.4,7.6,0.02),
            color=color.gray(0.5),
            opacity=1.0)

ceiling = box(canvas=scene3d,
            pos=vector(0,0, maxZ + 0.21),
            size=vector(6.4,7.6,0.02),
            color=color.gray(0.5),
            opacity=1.0,
            shininess=0)

# ========================================================
# UI（下段にまとめる）
# ========================================================
ui = canvas(
    width=0, height=0,
    background=color.white,
    caption="",
    align='right'
)

# ドロップダウンメニューの作成
modes = ["フローモード", "天上天下唯我独尊モード", "回る天井"]
# ドロップダウン作成
def on_mode_select(m):
    global is_transitioning, transition_start
    # 既存の visibility 切り替え処理
    select = m.selected
    for mode, widgets in slider_groups.items():
        for w in widgets:
            w.visible = (mode == select)

    # ここからトランジション開始処理
    is_transitioning   = True
    transition_start   = sim_time
    # 各エージェントの「切り替え前ステート」を保存
    for ag in agents:
        ag.prev_z     = ag.z
        ag.prev_yaw   = ag.yaw
        ag.prev_pitch = ag.pitch
        ag.prev_color = ag.current_color


mode_menu = menu(choices=modes, index=0, bind=on_mode_select, canvas=ui)
ui.append_to_caption("\n\n")  # 少し余白

# 各モードのスライダーを登録しておく辞書
slider_groups = { mode: [] for mode in modes }

# add_slider をラップして、mode を指定できるようにする
def add_mode_slider(mode, cnv, label, mn, mx, val, fmt, setter):
    # ラベル
    txt0 = wtext(text=f"{label}: ", canvas=cnv)
    # 値表示
    txt1 = wtext(text=f"{fmt.format(val)}  ", canvas=cnv)
    # スライダー本体
    sld = slider(min=mn, max=mx, value=val, length=200,
                 bind=lambda s, txt=txt1, fmt=fmt, st=setter: (
                     st(s.value), txt.__setattr__("text", f"{fmt.format(s.value)}  ")
                 ),
                 canvas=cnv)
    cnv.append_to_caption("<br>")
    # ウィジェット一覧に登録（visibility 切り替え用）
    slider_groups[mode] += [txt0, txt1, sld]
    return sld


# ─── Behavior Control ─────────────────────────────────
ui.append_to_caption("<b>フローパラメータ</b><br><br>")
add_mode_slider("フローモード", ui, "Separation",    0,   2,   separationFactor, "{:.2f}",
           lambda v: globals().update(separationFactor=v))
add_mode_slider("フローモード", ui, "Cohesion",      0,   2,   cohesionFactor,   "{:.2f}",
           lambda v: globals().update(cohesionFactor=v))
add_mode_slider("フローモード", ui, "Noise Scale",   .01, .5,  noiseScale,      "{:.2f}",
           lambda v: globals().update(noiseScale=v))
add_mode_slider("フローモード", ui, "Wave Scale",    .01, 1.0, waveScale,       "{:.2f}",
           lambda v: globals().update(waveScale=v))
add_mode_slider("フローモード", ui, "Wave Strength", 0,   1,   waveStrength,    "{:.2f}",
           lambda v: globals().update(waveStrength=v))
add_mode_slider("フローモード", ui, "Noise Speed",   0,   .1,  noiseSpeed,      "{:.3f}",
           lambda v: globals().update(noiseSpeed=v))
# LED 波の振幅
add_mode_slider("フローモード", ui, "LED Amplitude", 0, 2, led_amp, "{:.2f}",
           lambda v: globals().update(led_amp=v))

# ─── HSB Color Control ───────────────────────────────
# 3) UI（add_mode_slider／"フローモード", ui.append_to_caption済みと仮定）
add_mode_slider("フローモード", ui, "Hue Offset", 0.0, 1.0, base_hue, "{:.2f}",
           lambda v: globals().update(base_hue=v))
add_mode_slider("フローモード", ui, "Hue Span",   0.0, 1.0, hue_span, "{:.2f}",
           lambda v: globals().update(hue_span=v))
ui.append_to_caption("<br>")

# ========================================================
# UI：モード別スライダー登録（天上天下唯我独尊モード）
# ========================================================
ui.append_to_caption("<b>天上天下唯我独尊パラメータ</b><br><br>")

add_mode_slider("天上天下唯我独尊モード", ui,
    "Sine Amplitude", 0.0, 2.0, target_sine_amp, "{:.2f}",
    lambda v: globals().update(target_sine_amp=v)
)
add_mode_slider("天上天下唯我独尊モード", ui,
    "Sine Frequency", 0.0, 2.0, target_sine_freq, "{:.2f}",
    lambda v: globals().update(target_sine_freq=v)
)

add_mode_slider(
    "天上天下唯我独尊モード", ui,
    "Color Speed", 0.0, 2.0, color_speed, "{:.2f}",
    lambda v: globals().update(color_speed=v)
)
ui.append_to_caption("<br>")

# ========================================================
# ─── 回る天井モード用パラメータ ───────────────────
# ========================================================
ui.append_to_caption("<b>回る天井パラメータ</b><br><br>")
# 角度（°）
add_mode_slider("回る天井", ui,
    "Tilt Angle", 0.0, 90.0, tilt_angle_deg, "{:.1f}",
    lambda v: globals().update(tilt_angle_deg=v)
)
# 回転速度（rad/s）
add_mode_slider("回る天井", ui,
    "Rotation Speed", 0.0, 2.0, plane_rot_speed, "{:.3f}",
    lambda v: globals().update(plane_rot_speed=v)
)
# 平面高さ（m）
add_mode_slider("回る天井", ui,
    "Plane Height", minZ, maxZ, plane_height, "{:.2f}",
    lambda v: globals().update(plane_height=v)
)


# ========================================================
# Agent クラス
# ========================================================
class Agent:
    def __init__(self,i,j,x,y,z,nid,idx):
        self.i, self.j, self.x, self.y, self.z = i, j, x, y, z
        self.node_id = nid
        self.vz = 0
        self.idx = idx
        self.neighbors = []
        self.group = None  # 天上天下唯我独尊モード用

        # Boids LFO
        self.baseSepBias = random.uniform(0.8,1.2)
        self.baseCohBias = random.uniform(0.8,1.2)
        self.freqSep     = random.uniform(0.2,2.0)
        self.freqCoh     = random.uniform(0.2,2.0)
        self.ampSep      = random.uniform(0.1,0.5)
        self.ampCoh      = random.uniform(0.1,0.5)

        self.yaw   = random.uniform(0,360)
        self.pitch = random.uniform(-60,60)

        # 3D body + cable
        ax  = self.compute_axis() * agent_length
        ctr = vector(x,y,z)
        self.body = cylinder(canvas=scene3d,
                     pos=ctr-ax/2, axis=ax,
                     radius=agent_radius,
                     color=color.red,        # ← 初期色は赤
                     shininess=0)            # ← これでマット感アップ

        self.cable= cylinder(canvas=scene3d,
                             pos=ctr,
                             axis=vector(0,0,maxZ-z),
                             radius=cable_radius, color=color.gray(0.5))
        # 2D dot
        self.dot2d = sphere(canvas=scene2d,
                            pos=vector(x,y,0),
                            radius=agent_radius,
                            color=color.blue)
        # LEDs
        self.leds = []
        u = ax.norm()
        for t in (-0.5, 0.0, 0.5):
            p3 = ctr + u*(agent_length*t)
            ld3 = sphere(canvas=scene3d, pos=p3,
                         radius=led_radius,
                         emissive=True, color=color.black)
            ld2 = sphere(canvas=scene2d,
                         pos=vector(x+u.x*agent_length*t,
                                    y+u.y*agent_length*t, 0),
                         radius=led_radius,
                         emissive=True, color=color.black)
            self.leds.append((ld3, ld2, t))

        # RGB LED の色  
        self.current_color = vector(0,0,0)  # ← 追加
        self.target_color  = vector(0,0,0)   # 目標色（グループごとに再設定）

        # トランジション用
        self.prev_z     = self.z
        self.prev_yaw   = self.yaw
        self.prev_pitch = self.pitch
        self.prev_color = vector(0,0,0)

    def compute_axis(self):
        pr, yr = math.radians(self.pitch), math.radians(self.yaw)
        return vector(math.cos(pr)*math.cos(yr),
                      math.cos(pr)*math.sin(yr),
                      math.sin(pr))

    def update_boids(self, t):
        fs = fc = cnt = 0
        for idx in self.neighbors:
            if 0 <= idx < len(agents):
                nb   = agents[idx]
                dz   = self.z - nb.z
                dist = abs(dz)
                if dist < sepDist:
                    fs += (sepDist - dist)*(1 if dz>=0 else -1)
                fc += nb.z; cnt += 1
        dynS = self.baseSepBias * (1 + self.ampSep * math.sin(2*math.pi*self.freqSep*t))
        dynC = self.baseCohBias * (1 + self.ampCoh * math.sin(2*math.pi*self.freqCoh*t))
        fs *= separationFactor * dynS
        coh = ((fc/cnt - self.z)*cohesionFactor * dynC) if cnt>0 else 0
        self.vz = max(-1, min(1, self.vz + (fs+coh)*0.01))*0.95
        self.z  = max(minZ, min(maxZ, self.z + self.vz))

    def update(self, ty, tp, t, dt):
        # Boids Z
        self.update_boids(t)
        # Wave Z
        w = pnoise2(self.i*waveScale + t, self.j*waveScale + t)
        tgtZ = (w + 1)/2 * (maxZ - minZ) + minZ
        self.z += (tgtZ - self.z) * waveStrength * dt
        self.z = max(minZ, min(maxZ, self.z))
        # FlowNoise yaw/pitch
        tp = max(-60, min(60, tp))
        self.yaw   += ((ty - self.yaw + 540)%360 - 180)*0.1
        self.pitch += ((tp - self.pitch + 540)%360 - 180)*0.1
        # Update geometry
        ax  = self.compute_axis()*agent_length
        ctr = vector(self.x, self.y, self.z)
        self.body.pos  = ctr - ax/2;  self.body.axis = ax
        self.cable.pos = ctr;         self.cable.axis = vector(0,0,maxZ-self.z)
        self.dot2d.pos = vector(self.x, self.y, 0)
        # LEDs follow position
        u = ax.norm()
        for ld3, ld2, t in self.leds:
            ld3.pos = ctr + u*(agent_length*t)
            ld2.pos = vector(self.x + u.x*agent_length*t,
                             self.y + u.y*agent_length*t, 0)

    def display(self):
        self.body.visible  = True
        self.cable.visible = showPole


    def set_leds(self):
        # 1) 波の明るさ（0..1）
        b_noise = (pnoise2(self.i*waveScale + noise_time,
                           self.j*waveScale + noise_time) + 1)/2
        brightness = min(1.0, max(0.0, b_noise * led_amp))

        # 2) HSV→RGB で「純色」を作る
        hue = (base_hue
               + (self.j/(COLS-1)) * hue_span
               + hue_vari * math.sin(noise_time*hue_speed)) % 1.0
        sat = base_sat
        val = 1.0   # 明度は常に最大１にしておく
        r0, g0, b0 = colorsys.hsv_to_rgb(hue, sat, val)
        pure_color = vector(r0, g0, b0)

        # 3) 明るさを掛ける → 最終色
        final_color = pure_color * brightness
        self.current_brightness = brightness


        # 4) シリンダー・LED球体に反映
        self.body.color = final_color
        for ld3, ld2, _ in self.leds:
            ld3.color = ld2.color = final_color

        # 5) 自分の最新色を保存しておく
        self.current_color = final_color
        self.current_brightness = brightness

    def update_tenge(self, t, dt):
        global current_sine_freq

        # 1) 周波数を滑らかに追従（時定数 τ=1秒）
        tau   = 1.0
        alpha = dt / (tau + dt)
        current_sine_freq += (target_sine_freq - current_sine_freq) * alpha

        # 2) グループでフェーズを決定（A=0, B=π）
        phase = 0.0 if self.group == "A" else math.pi

        # 3) 正弦波値を取得
        sinv = math.sin(2 * math.pi * current_sine_freq * t + phase)

        # 4) グループごとの高さレンジを設定
        if self.group == "A":
            min_h, max_h = 1.5, maxZ
        else:
            min_h, max_h = 2.0, maxZ

        # 5) 中央と振幅を計算して z を更新
        mid_h = (min_h + max_h) / 2
        amp_h = (max_h - min_h) / 2
        self.z = mid_h + amp_h * sinv
        # 念のため clamp
        self.z = max(min_h, min(self.z, maxZ))

        # 6) Group B のみ向き（yaw/pitch）をイージング
        if self.group == "B":
            others = [o for o in agents if o.group != self.group]
            if others:
                target = min(others, key=lambda o: (o.x-self.x)**2 + (o.y-self.y)**2)
                dx, dy, dz = target.x-self.x, target.y-self.y, target.z-self.z
                target_yaw   = math.degrees(math.atan2(dy, dx))
                horiz_dist   = math.hypot(dx, dy)
                target_pitch = math.degrees(math.atan2(dz, horiz_dist))

                # yaw 差を -180..+180° に正規化
                dyaw = ((target_yaw - self.yaw + 540) % 360) - 180
                k = min(1.0, ease_speed * dt)
                self.yaw   += dyaw   * k
                self.pitch += (target_pitch - self.pitch) * k

        # 7) 3D ジオメトリ更新
        ax  = self.compute_axis() * agent_length
        ctr = vector(self.x, self.y, self.z)
        self.body.pos   = ctr - ax/2
        self.body.axis  = ax
        self.cable.pos  = ctr
        self.cable.axis = vector(0, 0, maxZ - self.z)

        # 8) 2D ドット更新
        self.dot2d.pos = vector(self.x, self.y, 0)

        # 9) LEDs を本体に追従させる
        u = ax.norm()
        for ld3, ld2, offset in self.leds:
            ld3.pos = ctr + u * (agent_length * offset)
            ld2.pos = vector(
                self.x + u.x * (agent_length * offset),
                self.y + u.y * (agent_length * offset),
                0
            )


    # def update_tenge(self, t, dt):
    #     global current_sine_amp, current_sine_freq

    #     # 1) 「目標値」→「現在値」へ滑らかに追従（時定数 tau[s]）
    #     tau = 1.0  # 秒
    #     alpha = dt / (tau + dt)
    #     current_sine_amp  += (target_sine_amp  - current_sine_amp)  * alpha
    #     current_sine_freq += (target_sine_freq - current_sine_freq) * alpha

    #     # 2) ─── グループは外部でセット済み ─────────────────────────────
    #     phase = 0.0 if self.group == "A" else math.pi

    #     # 3) 正弦波で高さ z を計算
    #     midZ = (maxZ + minZ) / 2
    #     self.z = midZ + current_sine_amp * math.sin(
    #         2 * math.pi * current_sine_freq * t + phase
    #     )

    #     # ─── ターゲットとの向き合わせをイージングで───────────
    #     # Group B は自動でターゲットを探す
    #     if self.group == "B":
    #         # 異グループ内で最も近いエージェントを取得
    #         others = [o for o in agents if o.group != self.group]
    #         if others:
    #             target = min(
    #                 others,
    #                 key=lambda o: (o.x-self.x)**2 + (o.y-self.y)**2
    #             )
    #             dx, dy, dz = target.x - self.x, target.y - self.y, target.z - self.z

    #             # 目標 yaw, pitch を計算
    #             target_yaw   = math.degrees(math.atan2(dy, dx))
    #             horiz_dist   = math.hypot(dx, dy)
    #             target_pitch = math.degrees(math.atan2(dz, horiz_dist))

    #             # yaw 差を -180..+180° に正規化
    #             dyaw = ((target_yaw - self.yaw + 540) % 360) - 180
    #             # イージング係数（フレーム時間 dt に比例、最大 1.0）
    #             k = min(1.0, ease_speed * dt)

    #             # 滑らかに追従
    #             self.yaw   += dyaw   * k
    #             self.pitch += (target_pitch - self.pitch) * k


    #     # 4) 3D ジオメトリの更新
    #     ax  = self.compute_axis() * agent_length
    #     ctr = vector(self.x, self.y, self.z)
    #     self.body.pos  = ctr - ax/2
    #     self.body.axis = ax
    #     self.cable.pos = ctr
    #     self.cable.axis= vector(0, 0, maxZ - self.z)

    #     # 5) 2D ドットの更新（上からビュー）
    #     self.dot2d.pos = vector(self.x, self.y, 0)

    #     # 6) LEDs を本体に追従させる
    #     u = ax.norm()
    #     for ld3, ld2, offset in self.leds:
    #         # 3D LED
    #         new3 = ctr + u * (agent_length * offset)
    #         ld3.pos = new3
    #         # 2D LED (上からビュー)
    #         new2 = vector(self.x + u.x * (agent_length * offset),
    #                       self.y + u.y * (agent_length * offset),
    #                       0)
    #         ld2.pos = new2

    def set_leds_tenge(self, t, dt):
        # 1) グループで色相を決定
        base_h = (t * color_speed) % 1.0
        if self.group == "A":
            hue = base_h
        else:
            hue = (base_h + 0.5) % 1.0

        # 2) HSV→RGB
        r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
        self.target_color = vector(r, g, b)

        # 3) イージングで現在色を更新
        k = min(1.0, ease_color_speed * dt)
        self.current_color += (self.target_color - self.current_color) * k

        # 4) 3D 本体と LEDs に反映
        self.body.color = self.current_color
        for ld3, ld2, _ in self.leds:
            ld3.color = self.current_color
            ld2.color = self.current_color

    def on_mode_select(m):
        global is_transitioning, transition_start
        # --- 既存の UI 表示制御はそのまま ---
        # トランジション開始
        is_transitioning = True
        transition_start = sim_time
        # 各エージェントの現在ステートを保存
        for ag in agents:
            ag.prev_z     = ag.z
            ag.prev_yaw   = ag.yaw
            ag.prev_pitch = ag.pitch
            ag.prev_color = ag.current_color



# ========================================================
# Agent 作成＋隣接設定
# ========================================================
agents = []
with open("node.csv", newline="") as f:
    for k, row in enumerate(csv.DictReader(f)):
        i, j = divmod(k, COLS)
        z = random.uniform(minZ + agent_length/2, maxZ - agent_length/2)
        # row["id"] に加えて、enumerate の k を idx として渡す
        agents.append(Agent(i, j,
                            float(row["x"]), float(row["y"]),
                            z,
                            row["id"],
                            idx=k))
for ag in agents:
    i, j = ag.i, ag.j
    nbrs = [(i-1,j), (i+1,j), (i,j-1), (i,j+1)]
    nbrs += ([(i-1,j-1),(i+1,j-1)] if i%2==0 else [(i-1,j+1),(i+1,j+1)])
    ag.neighbors = [ni*COLS + nj for ni,nj in nbrs]

# ========================================================
# カメラ中心＆2Dビュー調整
# ========================================================
minX, maxX = min(a.x for a in agents), max(a.x for a in agents)
minY, maxY = min(a.y for a in agents), max(a.y for a in agents)
centerX, centerY = (minX+maxX)/2, (minY+maxY)/2
span = max(maxX-minX, maxY-minY)

scene3d.center = vector(centerX, centerY, (minZ+maxZ)/2)
scene2d.center = vector(centerX, centerY, 0)
scene2d.range  = span * 0.4

# ========================================================
# メインループ
# ========================================================
sim_time = noise_time = angle = 0.0
dt = 1/60
current_groupA_idx = random.randrange(len(agents))
prev_z_diff       = None
epsilon           = 0.05   # 高さ差トリガーの許容幅
# ─── イージング速度 ───────────────────────────────
ease_speed = 3.0   # 大きいほど速く追従（0.0～10.0 くらいが調整幅の目安）
ease_color_speed = 1.0   # 1秒でどれだけ追いつくか（大きいほど速い）

while True:
    rate(20)
    sim_time   += dt
    noise_time += noiseSpeed
    angle      += rotationSpeed
    plane_angle += plane_rot_speed * dt   # ← 平面回転角を更新

    if mode_menu.selected == "回る天井":
        # ─── 1) 傾斜面の法線ベクトル ───────────────────────────
        a = radians(tilt_angle_deg)
        # X軸回転で傾けた初期法線（Y+方向が上るようにマイナス符号）
        n0 = vector(0, -sin(a), cos(a))
        # Z軸回りに回転
        ca, sa = cos(plane_angle), sin(plane_angle)
        nx = n0.x*ca - n0.y*sa
        ny = n0.x*sa + n0.y*ca
        nz = n0.z
        normal = vector(nx, ny, nz).norm()

        # ─── 2) 各エージェントの平面上 Z を先に計算 ───────────────
        plane_zs = []
        for ag in agents:
            dx = ag.x - centerX
            dy = ag.y - centerY
            z_p = plane_height - (normal.x*dx + normal.y*dy) / normal.z
            plane_zs.append(min(max(minZ, z_p), maxZ))
        minZp = min(plane_zs)
        maxZp = max(plane_zs)

        # ─── 3) エージェントごとに配置・向き・色を更新 ────────────
        for ag, z in zip(agents, plane_zs):
            # (a) 坂の上向きベクトルを算出
            g_dot_n = -normal.z
            uphill = (g_dot_n*normal - vector(0,0,-1)).norm()
            axis = uphill * agent_length

            # (b) ジオメトリ更新
            ctr = vector(ag.x, ag.y, z)
            ag.body.pos   = ctr - axis/2
            ag.body.axis  = axis
            ag.cable.pos  = ctr
            ag.cable.axis = vector(0,0, maxZ - z)
            ag.dot2d.pos  = vector(ag.x, ag.y, 0)

            # (c) LEDs 位置更新
            u = axis.norm()
            for ld3, ld2, offset in ag.leds:
                ld3.pos = ctr + u * (agent_length * offset)
                ld2.pos = vector(
                    ag.x + u.x * (agent_length * offset),
                    ag.y + u.y * (agent_length * offset),
                    0
                )

            # (d) 色の計算：全体Hueをサイクル＋高さで明度とHueオフセット
            global_hue   = (sim_time * color_speed) % 1.0
            height_ratio = (z - minZp) / (maxZp - minZp)
            # 高さに応じてHueを1/8サイクルだけずらす
            hue = (global_hue + height_ratio * (1.0/8.0)) % 1.0
            # 一番低いところを明度1.0、一番高いところを明度0.5にマッピング
            brightness = 1.0 - 0.5 * height_ratio
            r, g, b = colorsys.hsv_to_rgb(hue, 1.0, brightness)
            col = vector(r, g, b)

            # (e) 本体とLEDに色を適用
            ag.body.color = col
            for ld3, ld2, _ in ag.leds:
                ld3.color = ld2.color = col

            
    elif mode_menu.selected == "フローモード":
        # 1) 全エージェントの動きをまず計算（z, pitch, yaw）
        for ag in agents:
            ty = pnoise2(ag.i*noiseScale + noise_time,
                        ag.j*noiseScale + noise_time) * 360
            tp = pnoise2(ag.i*noiseScale + noise_time + 100,
                        ag.j*noiseScale + noise_time + 100) * 120 - 60
            ag.update(ty, tp, sim_time, dt)
            # ─── z の上限を 3m（maxZ）に固定 ─────────────────
            ag.z = min(ag.z, maxZ)

        # 2) 平均高さを flow_target_height に合わせるため、一括シフト
        current_mean = sum(a.z for a in agents) / len(agents)
        offset = flow_target_height - current_mean
        for ag in agents:
            ag.z = min(max(minZ, ag.z + offset), maxZ)

        # 3) 画面と LED、MQTT 出力
        for ag in agents:
            ag.display()
            ag.set_leds()
            mqtt_client.publish(f"ps/{ag.node_id}",
                struct.pack("<3f",
                            round(ag.z,2),
                            round(ag.pitch,2),
                            round(ag.yaw,2)))
            r_byte = int(ag.current_color.x * 255)
            g_byte = int(ag.current_color.y * 255)
            b_byte = int(ag.current_color.z * 255)
            mqtt_client.publish(f"cl/{ag.node_id}", bytes([r_byte,g_byte,b_byte]))


    elif mode_menu.selected == "天上天下唯我独尊モード":
        # ─── 1) 高さ差の検出＆Group A の再割り当て ───────────
        za     = agents[current_groupA_idx].z
        zb     = sum(a.z for i,a in enumerate(agents) if i != current_groupA_idx) / (len(agents)-1)
        z_diff = za - zb

        if prev_z_diff is not None and abs(z_diff) < epsilon:
            # 閾値内に入った瞬間だけ一度切り替え
            choices = list(range(len(agents)))
            choices.remove(current_groupA_idx)
            new_idx = random.choice(choices)
            # print(f"[DEBUG] GroupA: {current_groupA_idx} → {new_idx} at t={sim_time:.2f} z_diff={z_diff:.3f}")
            current_groupA_idx = new_idx

        prev_z_diff = z_diff

        # ─── 2) 外側で group を設定 ────────────────────────
        for idx, ag in enumerate(agents):
            ag.group = "A" if idx == current_groupA_idx else "B"

        # ─── 3) 各エージェント更新 ─────────────────────────
        for ag in agents:
            ag.update_tenge(sim_time, dt)
            ag.display()
            ag.set_leds_tenge(sim_time, dt)

            # —— ここから Flow モードと同じ MQTT 配信 —— 
            mqtt_client.publish(
                f"ps/{ag.node_id}",
                struct.pack("<3f",
                            round(ag.z, 2),
                            round(ag.pitch, 2),
                            round(ag.yaw, 2))
            )
            r_byte = int(ag.current_color.x * 255)
            g_byte = int(ag.current_color.y * 255)
            b_byte = int(ag.current_color.z * 255)
            mqtt_client.publish(
                f"cl/{ag.node_id}",
                bytes([r_byte, g_byte, b_byte])
            )

        # ――――― トランジション適用 ―――――
        if is_transitioning:
            p = min(1.0, (sim_time - transition_start) / transition_duration)
            for ag in agents:
                # 座標・向き・色の補間（既存）
                z = ag.prev_z * (1-p) + ag.z * p
                yaw   = ag.prev_yaw   * (1-p) + ag.yaw   * p
                pitch = ag.prev_pitch * (1-p) + ag.pitch * p
                col = ag.prev_color * (1-p) + ag.current_color * p

                # 1) 軸ベクトル再生成
                pr, yr = math.radians(pitch), math.radians(yaw)
                axis = vector(math.cos(pr)*math.cos(yr),
                            math.cos(pr)*math.sin(yr),
                            math.sin(pr)) * agent_length
                ctr = vector(ag.x, ag.y, z)

                # 2) 本体・ケーブル・2Dドット
                ag.body.pos  = ctr - axis/2
                ag.body.axis = axis
                ag.cable.pos = ctr
                ag.cable.axis= vector(0,0,maxZ-z)
                ag.dot2d.pos = vector(ag.x, ag.y, 0)

                # 3) LED の 3D/2D 両方を再配置
                u = axis.norm()
                for ld3, ld2, offset in ag.leds:
                    # 3D LED
                    ld3.pos = ctr + u * (agent_length * offset)
                    # 2D LED
                    ld2.pos = vector(
                        ag.x + u.x * (agent_length * offset),
                        ag.y + u.y * (agent_length * offset),
                        0
                    )

                # 4) 色の補間反映
                ag.body.color = col
                for ld3, ld2, _ in ag.leds:
                    ld3.color = ld2.color = col

            if p >= 1.0:
                is_transitioning = False

    # 3Dカメラ軌道更新
    camX = centerX + radius * math.cos(angle)
    camY = centerY + radius * math.sin(angle)
    scene3d.camera.pos  = vector(camX, camY, cameraHeight)
    scene3d.camera.axis = vector(centerX-camX,
                                 centerY-camY,
                                 ((minZ+maxZ)/2)-cameraHeight)
