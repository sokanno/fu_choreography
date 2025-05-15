from vpython import *
scene.visible = False
scene.width = scene.height = 0

import math, random, struct, csv
from noise import pnoise2, pnoise1
import paho.mqtt.client as mqtt

import colorsys
import random
from math import radians, degrees, sin, cos

from osc_listener import start_osc_listener, params

from mqtt_listener import start, fetch_messages, stop
from pythonosc import udp_client
import struct
import threading
import queue

# SuperCollider サーバーのホストとポート
SC_HOST = "127.0.0.1"
SC_PORT = 57121
osc_client = udp_client.SimpleUDPClient(SC_HOST, SC_PORT)

# ─── モード切替トランジション用 ─────────────────────
is_transitioning   = False
transition_start   = 0.0
transition_duration = 2.0   # 切り替えを何秒かけるか

# ========================================================
# グローバルパラメータ（初期値）
# ========================================================
paused = False          # 一時停止フラグ

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
sine_freq   = 0.1                 # 正弦波の周波数 [Hz]
sine_amp   = (maxZ - minZ) / 4   # 正弦波の振幅
color_speed      = 0.03                 # LED 色変化の速度

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
plane_rot_speed  = 0.3     # 回転速度（rad/s）
plane_angle      = 0.0     # フレームごとに増加
plane_height     = 2.5     # 平面中心の高さ

# Audience の人数設定（0～10）
audience_count = 2
audiences = []   # リストは後で埋める
# ─── グローバルにノイズ用パラメータを追加 ─────────────────
audience_noise_scale = 0.5    # 空間解像度（大きいほどゆっくり変化）
audience_noise_speed = 0.2    # 時間変化の速さ
audience_speed_amp = 0.5   # 0.0＝固定速度、1.0＝±50%の変動幅

# 人が近くにいたら優先で向く検出半径[m]
detect_radius = 1.0

# ─── ② 鬼さんこちらモード用グローバルパラメータ ────────────────────────
oni_empty_radius     = 2.5    # 空き半径（スライダーで可変）
oni_min_select_dist  = 2.5    # 降下候補最小距離
oni_max_select_dist  = 4.0    # 降下候補最大距離
oni_descent_speed    = 0.2    # 降下速度[m/s]
oni_return_speed     = 1.0    # 緊急上昇速度[m/s]
oni_target_z         = 1.8    # 降下目標Z[m]

# ステートマシン用
oni_state            = "idle" # "idle","descending","waiting","ascending"
oni_next_time        = 0.0    # 次の降下開始時間
oni_current_idx      = None   # 選択中のAgentインデックス
oni_target_aud       = None   # 選択中のAudience
oni_wait_start       = 0.0    # 停止開始時刻

# ─── グローバル変数 ─────────────────────────────────────
oni_color_start     = 0.0            # 降下開始時刻（色イージング用）
oni_initial_color   = vector(1,1,1)  # 降下開始時の色
oni_target_color    = vector(1,1,1)  # 2 秒後に向かう反対色

# --------------------------------------------------------
# Audience 表示モード
#   False  … 既存のランダム移動シミュレーション
#   True   … MQTT “ca” 座標で直接配置（センサーモード）
# --------------------------------------------------------
sensor_mode        = False      # 既存
sensor_person      = None       # 1 人だけ使い回す
last_aud_msg_time  = 0.0        # 最後に ca を受信した時刻
PRESENCE_TIMEOUT   = 0.8        # 秒：これを過ぎたら非表示

# ─── Sound‑reaction globals ──────────────────────────────────
last_sound_coords   = None     # (x, y) in m
last_sound_time     = -1.0     # sim_time at reception
SOUND_REACT_TIME    = 1.0   # 振り向きが効いているピーク時間
SOUND_REACT_DECAY   = 1.0   # フェードアウト
SOUND_TURN_TIME     = 0.5   # ← ★ 完全に振り向くまでに使える時間

# ==== Fish‑school mode params ====
fish_align_factor  = 0.5    # 向きそろえ強度
fish_sep_dist      = 0.35   # 距離しきい値
fish_coh_factor    = 1.2    # 群れへの吸引
fin_osc_amp_deg    = 4.0    # 尾びれ振幅 (deg)
fin_osc_freq_hz    = 1.5    # 尾びれ周波数

# ==== Shimmer mode params ====
# ==== Shimmer mode params ====
# shim_base_freq_hz  = 5.0          # 基本垂直振動 (Hz)
# shim_base_amp_m    = 0.01         # 振幅 ±1 cm
# shim_wave_speed    = 1.2          # 伝播速度 (m/s)
# shim_trigger_mean  = 3.0          # ★ 平均トリガ間隔 [s]（以前 3s）
# # shim_kick_pitch_deg = 6.0         # ★ ケツ振り横揺れ振幅 (deg)
# # shim_kick_decay     = 0.6         # ★ 横揺れが0になるまでの減衰時間 [s]
# shim_kick_yaw_deg   = 20.0   # ★ 水平回転の振幅 (deg)
# shim_kick_decay     = 0.6    # 減衰時間 [s] （前回と同じで可）
shim_base_freq_hz = 10.0      # 垂直バイブ＝ 10 Hz（以前 5 Hz）
shim_base_amp_m   = 0.01      # ±1 cm
shim_wave_speed   = 1.2       # m/s
shim_trigger_mean = 6.0       # 波トリガ平均間隔[s]
shim_front_tol    = 0.35      # 波前線の厚み[m]

# 水平回転キック
shim_kick_yaw_deg = 60.0      # ピーク ±90°
shim_kick_rise    = 0.5       # 0→MAX 0.5 s
shim_kick_decay   = 0.8       # その後指数減衰 τ

# Shimmer wave bookkeeping
next_shim_time = 0.0      # 次トリガー時刻
wave_fronts = []          # [(origin_i, origin_j, start_time)]
color_rise_time   = 0.5      # 色をグラデーションする秒数


# ========================================================
# MQTT セットアップ
# ========================================================
mqtt_client = mqtt.Client()
# mqtt_client.connect("127.0.0.1", 1883, 60)
mqtt_client.connect("192.168.1.2", 1883, 60)
mqtt_client.loop_start()

# ========================================================
# 3Dビュー（左上）
# ========================================================
scene3d = canvas(
    width=1200, height=800, #     width=800, height=533,
    background=color.gray(0.2),
    caption="",
    align='right',
    margin=10
)
scene3d.up       = vector(0,0,1)
scene3d.forward  = vector(-1,-1,-0.2)
scene3d.userspin = False

# ========================================================
# 上からビュー（右上）
# ========================================================
scene2d = canvas(
    width=300, height=200,
    background=color.gray(0.15),
    caption="",
    align='right',
    margin=10
)
scene2d.camera.projection = "orthographic"
scene2d.up            = vector(1,0,0)
scene2d.camera.axis   = vector(0,0,-1)
scene2d.lights = []

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
# UI 用キャンバス（サイズゼロにしても OK）
ui = canvas(
    width=0, height=0,
    background=color.white,
    caption="",
    align='left'
)

# ① 先に切り替え用コールバックを定義
def on_mode_select(m):
    global is_transitioning, transition_start
    # …既存のモード切り替えロジック（transition_start とか agents.prev_* の保存など）…

# ② ドロップダウンを 1 回だけ生成し、すぐ隠す
modes = ["フローモード",
         "天上天下モード",
         "回る天井",
         "鬼さんこちらモード",
         "魚群モード",
         "蜂シマーモード"]
mode_menu = menu(
    choices=modes,
    index=0,
    bind=on_mode_select,
    canvas=ui
)
mode_menu.visible = False  # ←これだけで UI に表示されなくなる

# （以降はメインループ内で mode_menu.index を参照して on_mode_select を手動呼び出し）
wtext(text='<br><br>')

def toggle_sensor(b):
    global sensor_mode, sensor_person, audiences
    sensor_mode = not sensor_mode
    b.text = "Sensor ON" if sensor_mode else "Sensor OFF"
    print("Sensor mode =", sensor_mode)

    if sensor_mode:
        # ---- これからセンサー入力に切り替える ----
        # 既存観客オブジェクトは全部消す
        for p in audiences:
            if hasattr(p, "body"):
                p.body.visible  = False
                p.head.visible  = False
                p.dot2d.visible = False
        audiences.clear()
        sensor_person = None       # 新しく作り直させる
    else:
        # ---- シミュレーションに戻る ----
        sensor_person = None       # もう使わない

sensor_btn = button(
    canvas=ui,
    text="Sensor OFF",        # 初期表示
    bind=toggle_sensor
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
        # ─── Audience クラスの 2D 部分 ─────────────────
        # self.dot2d = sphere(
        #     canvas=scene2d,
        #     pos=vector(self.x, self.y, 0),
        #     radius=agent_radius,
        #     color=color.white,
        #     emissive=True,    # ← ここを追加
        #     opacity=0.8)   

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

        # トラッキング用ターゲット角度（Noneなら検出ナシ）
        self.target_yaw   = None
        self.target_pitch = None

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
        # self.dot2d.pos = vector(self.x, self.y, 0)
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
        # self.dot2d.pos = vector(self.x, self.y, 0)

        # 9) LEDs を本体に追従させる
        u = ax.norm()
        for ld3, ld2, offset in self.leds:
            ld3.pos = ctr + u * (agent_length * offset)
            ld2.pos = vector(
                self.x + u.x * (agent_length * offset),
                self.y + u.y * (agent_length * offset),
                0
            )

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


# ─── Audience クラス定義（頭に球、円柱を 0.3m 短く、2D は下レイヤー） ─────────
class Audience:
    def __init__(self, canvas3d, canvas2d, x_range, y_range,
                 height=1.6, radius=0.2, speed=0.5):
        """
        canvas3d: 3Dシーン
        canvas2d: 2D上からビュー
        x_range, y_range: (min, max) 座標域
        height: 全体の高さ[m]
        radius: 半径[m]
        speed: 移動速度[m/s]
        """
        self.xmin, self.xmax = x_range
        self.ymin, self.ymax = y_range
        self.height = height
        self.radius = radius
        self.speed  = speed

        # 円柱本体は head の分だけ短く
        self.cyl_h = self.height - 0.3

        # 初期位置
        self.x = random.uniform(self.xmin, self.xmax)
        self.y = random.uniform(self.ymin, self.ymax)

        # 3D 円柱（体幹）
        self.body = cylinder(
            canvas=canvas3d,
            pos=vector(self.x, self.y, 0),
            axis=vector(0, 0, self.cyl_h),
            radius=self.radius,
            color=color.white,
            opacity=0.8
        )
        # 3D 頭部：球の中心を円柱の天端 + 半径 に
        self.head = sphere(
            canvas=canvas3d,
            pos=vector(self.x, self.y, self.cyl_h + self.radius),
            radius=self.radius,
            color=color.white,
            opacity=0.8
        )
        # 2D 表示（下レイヤーにするため z<0 に）
        self.dot2d = sphere(
            canvas=canvas2d,
            pos=vector(self.x, self.y, -0.1),
            radius=self.radius,  # ← 半径を 2 倍に
            color=color.white,           # ← 鮮やかな赤に
            emissive=True,             # ← 自発光に
            opacity=0.5                # ← 透過なしに
        )
        self.noise_t = random.random() * 1000  # 個体差をつけるオフセット

    def update(self, dt):
        # 1) ノイズ時間を進める
        self.noise_t += dt * audience_noise_speed

        # 1.5) 速度変動（Perlinノイズ＋振幅パラメータ）
        delta = pnoise1(self.noise_t * 0.5)  # -1..+1
        self.current_speed = self.speed * (1 + audience_speed_amp * delta * 0.5)

        # 2) 移動方向のノイズ取得
        n = pnoise2(self.x * audience_noise_scale, self.noise_t)
        angle = n * 2 * math.pi

        # 3) 基本移動量
        dx = math.cos(angle) * self.current_speed * dt
        dy = math.sin(angle) * self.current_speed * dt

        # 4) 壁反発：壁に近いほど跳ね返る成分を加える
        margin_x = min(self.x - self.xmin, self.xmax - self.x)
        margin_y = min(self.y - self.ymin, self.ymax - self.y)
        threshold = self.radius * 2
        if margin_x < threshold:
            repel = (threshold - margin_x) / threshold
            dx += repel * (-math.cos(angle)) * self.current_speed * dt
        if margin_y < threshold:
            repel = (threshold - margin_y) / threshold
            dy += repel * (-math.sin(angle)) * self.current_speed * dt

        # 5) 他の観客との衝突回避
        min_sep = self.radius * 2
        for other in audiences:
            if other is self: continue
            dist = math.hypot(self.x - other.x, self.y - other.y)
            if dist < min_sep and dist > 1e-3:
                # お互い離れるベクトルを加算
                ux = (self.x - other.x) / dist
                uy = (self.y - other.y) / dist
                factor = (min_sep - dist) / min_sep
                dx += ux * factor * self.current_speed * dt
                dy += uy * factor * self.current_speed * dt

        # 6) 移動量を反映＆クランプ
        new_x = self.x + dx
        new_y = self.y + dy
        self.x = min(max(new_x, self.xmin), self.xmax)
        self.y = min(max(new_y, self.ymin), self.ymax)

        # 7) 3D 体幹・頭部更新
        self.body.pos = vector(self.x, self.y, 0)
        self.head.pos = vector(self.x, self.y, self.cyl_h + self.radius)

        # 8) 2D ドット更新
        self.dot2d.pos = vector(self.x, self.y, -0.1)






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

# カメラ中心＆2Dビュー調整のあと
# Audience の初期化
x_range = (centerX - span/2, centerX + span/2)
y_range = (centerY - span/2, centerY + span/2)
audiences = []  # 最初は空


# ───────────────────────────────────────────────
# 受信した座標をどう使うかはプロジェクト側で実装
# ───────────────────────────────────────────────
sensor_person = None        # 1 人だけ描画する想定（複数なら list に）

def handle_audience(coords):
    global sensor_person, audiences, last_aud_msg_time
    x, y = coords
    last_aud_msg_time = sim_time          # ← 受信時刻を記録

    if not sensor_mode:
        return

    # ----- 1人目を生成（まだ無い or 削除済みのときだけ） -----
    if (sensor_person is None or
        not hasattr(sensor_person, "body")):

        # 既存のリストを空にしてから 1 人追加
        for p in audiences:
            if hasattr(p, "body"):
                p.body.visible  = p.head.visible = False
                p.dot2d.visible = False
        audiences.clear()

        x_range = (centerX - span/2, centerX + span/2)
        y_range = (centerY - span/2, centerY + span/2)
        sensor_person = Audience(scene3d, scene2d,
                                 x_range, y_range,
                                 height=1.7, radius=0.15, speed=0.0)
        audiences.append(sensor_person)

    # ----- 位置だけ更新 -----
    sensor_person.x, sensor_person.y = x, y
    sensor_person.body.pos = vector(x, y, 0)
    sensor_person.head.pos = vector(x, y,
                                    sensor_person.cyl_h + sensor_person.radius)
    sensor_person.dot2d.pos = vector(x, y, -0.1)
    # 可視化を確実に ON
    sensor_person.body.visible  = True
    sensor_person.head.visible  = True
    sensor_person.dot2d.visible = True




def handle_sound_source(coords):
    x, y = coords
    # 例: ロボットの向きを音源方向に
    print(f"[SOUND]    x={x:.2f} m, y={y:.2f} m")

def update_scene(dt):
    """
    1 フレーム分のシミュレーション / レンダリング / 通信 をまとめて行う。
    既存の VPython 描画や MQTT publish があるならここに入れる。
    """
    pass  # ←既存の描画・物理計算などに置き換える

def toggle_sensor(b):
    global sensor_mode, sensor_person
    sensor_mode = not sensor_mode
    b.text = "Sensor ON" if sensor_mode else "Sensor OFF"
    print("Sensor mode =", sensor_mode)

    if not sensor_mode and sensor_person:
        # センサーモードを OFF にしたら観客オブジェクトを消す
        sensor_person.body.visible  = False
        sensor_person.head.visible  = False
        sensor_person.dot2d.visible = False
        sensor_person = None

# ─── ジオメトリ＆LED 同期関数 ─────────────────────────────
def update_geometry(ag):
    axis = ag.compute_axis() * agent_length
    ctr  = vector(ag.x, ag.y, ag.z)
    ag.body.pos,  ag.body.axis  = ctr - axis/2, axis
    ag.cable.pos, ag.cable.axis = ctr, vector(0,0, maxZ - ag.z)
    u = axis.norm()
    for ld3, ld2, off in ag.leds:
        ld3.pos = ctr + u * (agent_length * off)
        ld2.pos = vector(ag.x + u.x * (agent_length * off),
                         ag.y + u.y * (agent_length * off),
                         0)
# 補助関数：frange（任意の間隔でグリッドサーチ）
def frange(start, stop, step):
    x = start
    while x <= stop:
        yield x
        x += step


def apply_sound_reaction():
    """
    ― 音源 (ss) 受信から一定時間、全エージェントが音源方向へ
      ゆっくり首を振り（SOUND_TURN_TIME 秒でほぼ到達）、
      ピーク維持後はフェードアウトして元の動きへ戻る。
    """
    if last_sound_coords is None:
        return

    elapsed = sim_time - last_sound_time
    total   = SOUND_REACT_TIME + SOUND_REACT_DECAY
    if elapsed > total:
        return  # 反応終了

    # --- フェード係数 k: 1 → 0 へ線形 ---------------------------
    if elapsed <= SOUND_REACT_TIME:
        k_fade = 1.0
    else:
        k_fade = 1.0 - (elapsed - SOUND_REACT_TIME) / SOUND_REACT_DECAY

    sx, sy = last_sound_coords
    for ag in agents:
        # 目標方向
        dx, dy = sx - ag.x, sy - ag.y
        dz     = 0.0 - ag.z
        tgt_yaw   = math.degrees(math.atan2(dy, dx))
        tgt_pitch = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
        tgt_pitch = max(-60, min(60, tgt_pitch))

        # 現在との差 (±180° 正規化)
        dyaw = ((tgt_yaw - ag.yaw + 540) % 360) - 180
        dpit = tgt_pitch - ag.pitch

        # --- “0.5 秒で追いつく” 回転ステップ --------------------
        step = min(1.0, dt / SOUND_TURN_TIME)   # dt はグローバルで 1/20 (=0.05)

        ag.yaw   += dyaw * step * k_fade        # フェードも掛ける
        ag.pitch += dpit * step * k_fade

        # 仕上げ：ジオメトリ更新
        update_geometry(ag)

def clamp(v, lo, hi):        # ★ 追加 ★
    return max(lo, min(hi, v))


def send_robot_data(agent):
    """
    agent: 各エージェントオブジェクト
    - node_id: int
    - z, pitch, yaw: float
    - current_color.x/y/z: 0.0–1.0
    """
    
    # ---- MQTT 出力 ----
    # 位置・向き
    payload_pos = struct.pack("<3f",
                              round(agent.z, 2),
                              round(agent.pitch, 2),
                              round(agent.yaw,   2))
    mqtt_client.publish(f"ps/{agent.node_id}", payload_pos)
    # 色
    r = int(agent.current_color.x * 255)
    g = int(agent.current_color.y * 255)
    b = int(agent.current_color.z * 255)
    mqtt_client.publish(f"cl/{agent.node_id}", bytes([r, g, b]))

    # ---- OSC 出力 ----
    # SuperCollider の OSCFunc('/robot') に合わせて、
    # [id, r, g, b, angle, height]
    # angle は 0–360deg のまま、height は 0.0–1.0 正規化済みと仮定
    # osc_client.send_message(
    #     "/robot",
    #     [agent.node_id,
    #      agent.current_color.x,
    #      agent.current_color.y,
    #      agent.current_color.z,
    #      agent.yaw,
    #      (agent.z - minZ) / (maxZ - minZ)]
    # )

# 1) 送信用キューを用意
send_queue = queue.Queue()

# 2) ネットワークワーカーを起動
def network_worker():
    while True:
        agent = send_queue.get()  # エージェントオブジェクトを受け取る
        try:
            send_robot_data(agent)
        except Exception as e:
            print(f"[network_worker] 送信エラー for agent {agent.node_id}: {e}")
        finally:
            send_queue.task_done()

threading.Thread(target=network_worker, daemon=True).start()

# def send_group_heights(agents, minZ, maxZ):
#     """各グループの平均高さを 0-1 に正規化して送信"""
#     def norm(height):
#         return (height - minZ) / (maxZ - minZ) if maxZ != minZ else 0.5

#     zs_A = [ag.z for ag in agents if ag.group == "A"]
#     zs_B = [ag.z for ag in agents if ag.group == "B"]

#     if zs_A:
#         osc_client.send_message('/groupA', norm(sum(zs_A)/len(zs_A)))
#     if zs_B:
#         osc_client.send_message('/groupB', norm(sum(zs_B)/len(zs_B)))
    
def send_group_stats(agents, minZ, maxZ):
    """
    各グループの平均高さ (0.0–1.0 正規化) に加えて
    groupA の平均 X/Y 座標も送信する。
    """
    def norm_height(h):
        return (h - minZ) / (maxZ - minZ) if maxZ != minZ else 0.5

    # グループごとに集める
    groupA = [ag for ag in agents if ag.group == "A"]
    groupB = [ag for ag in agents if ag.group == "B"]

    # 高さの送信
    if groupA:
        avg_z_A = sum(ag.z for ag in groupA) / len(groupA)
        osc_client.send_message('/groupA_height', norm_height(avg_z_A))
    if groupB:
        avg_z_B = sum(ag.z for ag in groupB) / len(groupB)
        osc_client.send_message('/groupB_height', norm_height(avg_z_B))

    # groupA の X/Y も送る
    if groupA:
        avg_x_A = sum(ag.x for ag in groupA) / len(groupA)
        avg_y_A = sum(ag.y for ag in groupA) / len(groupA)
        osc_client.send_message('/groupA_x', avg_x_A)
        osc_client.send_message('/groupA_y', avg_y_A)


# ========================================================
# メインループ
# ========================================================

print(">>> [main] about to call start_osc_listener()")
start_osc_listener(ip="0.0.0.0", port=8000)
print(">>> [main] returned from start_osc_listener()")

# mqtt起動
# client = start(broker_host="localhost", broker_port=1883)
client = start(broker_host="192.168.1.2", broker_port=1883)

sim_time = noise_time = angle = 0.0
dt = 1/20
current_groupA_idx = random.randrange(len(agents))
prev_z_diff       = None
epsilon           = 0.05   # 高さ差トリガーの許容幅
# ─── イージング速度 ───────────────────────────────
ease_speed = 1.0   # 大きいほど速く追従（0.0～10.0 くらいが調整幅の目安）
ease_color_speed = 1.0   # 1秒でどれだけ追いつくか（大きいほど速い）




while True:
    rate(20)
    if params["pause"] == 1.0:
        continue        # 一時停止中はループ先頭に戻る
    sim_time   += dt
    noise_time += noiseSpeed
    angle      += rotationSpeed
    plane_angle += plane_rot_speed * dt   # ← 平面回転角を更新

    # MaxからのOSCメッセージを更新
    separationFactor     = params["separation"]
    cohesionFactor       = params["cohesion"]
    noiseScale           = params["noise_scale"]
    waveScale            = params["wave_scale"]
    waveStrength         = params["wave_strength"]
    noiseSpeed           = params["noise_speed"]
    led_amp              = params["led_amplitude"]
    base_hue             = params["hue_offset"]
    hue_span             = params["hue_span"]
    target_sine_amp      = params["sine_amplitude"]
    target_sine_freq     = params["sine_frequency"]
    color_speed          = params["color_speed"]
    tilt_angle_deg       = params["tilt_angle"]
    plane_rot_speed      = params["rotation_speed"]
    plane_height         = params["plane_height"]
    audience_count       = int(params["audience_count"])
    audience_speed_amp   = params["audience_movement"]
    detect_radius        = params["detect_radius"]
    oni_empty_radius     = params["empty_radius"]
    raw_index            = params["menu"]
    paused               = bool(params["pause"])
    # 既存の OSC → 変数割当ブロックに追記
    fish_align_factor = params["fish_align"]
    fish_sep_dist     = params["fish_sep"]
    fish_coh_factor   = params["fish_coh"]
    fin_osc_amp_deg   = params["fin_amp"]
    fin_osc_freq_hz   = params["fin_freq"]


    # 必ず整数化
    try:
        menu_index = int(raw_index)
    except (ValueError, TypeError):
        print(f"Invalid menu index received: {raw_index}")
        menu_index = 0

    # OSC から受け取った menu インデックスを反映
    if menu_index != mode_menu.index:
        mode_menu.index = menu_index      # ドロップダウンの選択を切り替え
        on_mode_select(mode_menu)         # 必要なら切り替え処理を手動呼び出し


    # ★★★ MQTT 受信処理を追加 ★★★
    for msg in fetch_messages():
        if msg["topic"] == "ca":
            handle_audience(msg["coords"])
        elif msg["topic"] == "ss":
            # ← ここでグローバルに記録
            last_sound_coords = msg["coords"]
            last_sound_time   = sim_time


    # ─── Audience の再生成チェック ───────────────────
    # ※ センサーモード中は人数をいじらない
    if (not sensor_mode) and (len(audiences) != audience_count):
        # ① 既存のオブジェクトを完全に消去
        for person in audiences:
            if hasattr(person, "body"):
                person.body.visible  = False
                person.head.visible  = False
                person.dot2d.visible = False
                # 参照を切ってガベージコレクト
                del person.body, person.head, person.dot2d
        audiences.clear()

        # ② 新しい人数分だけ生成（ランダム歩行用）
        x_range = (centerX - span/2, centerX + span/2)
        y_range = (centerY - span/2, centerY + span/2)
        audiences = [
            Audience(
                scene3d, scene2d, x_range, y_range,
                height=random.uniform(1.5, 1.8),   # 身長
                radius=0.15,
                speed=random.uniform(0.5, 1.5)     # 速度 0.5〜1.5 m/s
            )
            for _ in range(audience_count)
        ]



    # ─── Audience の動作更新 ─────────────────────────
    if sensor_mode:
        # センサーモードでは自動移動させない
        pass
    else:
        # 既存のランダム移動シミュレーション
        for person in audiences:
            person.update(dt)



    # ─── 人検出でターゲット向きだけ計算 ────────────────────
    for ag in agents:
        # (1) まず毎フレーム必ず両方をクリア
        ag.target_yaw   = None
        ag.target_pitch = None

        closest = None
        md = detect_radius
        for p in audiences:
            d = math.hypot(p.x - ag.x, p.y - ag.y)
            if d < md:
                md = d
                closest = p

        if closest:
            dx, dy = closest.x - ag.x, closest.y - ag.y
            dz = closest.height - ag.z
            # (2) 見つかったら両方に値をセット
            ag.target_yaw   = math.degrees(math.atan2(dy, dx))
            ag.target_pitch = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
        # else 部分は不要です（上でクリア済み）



    if mode_menu.selected == "回る天井":
        # 1) 傾斜面の法線ベクトル
        a = radians(tilt_angle_deg)
        n0 = vector(0, -sin(a), cos(a))
        ca, sa = cos(plane_angle), sin(plane_angle)
        normal = vector(n0.x*ca - n0.y*sa,
                        n0.x*sa + n0.y*ca,
                        n0.z).norm()

        # 2) 平面上 Z のリストを先に作る ← 必ずここで定義
        plane_zs = []
        for ag in agents:
            dx, dy = ag.x - centerX, ag.y - centerY
            z_p = plane_height - (normal.x*dx + normal.y*dy) / normal.z
            plane_zs.append(min(max(minZ, z_p), maxZ))
        minZp, maxZp = min(plane_zs), max(plane_zs)

        # ★★★ ここに追加 ★★★
        sin_val = math.sin(sim_time)   # 任意で周波数掛け算
        cos_val = math.cos(sim_time)
        osc_client.send_message('/sin', sin_val)
        osc_client.send_message('/cos', cos_val)
        # ★★★★★★★★★★★★★

        # 3) 各エージェントごとに高さ固定＆向きイージング
        for ag, z in zip(agents, plane_zs):
            # (A) 高さは平面揃え
            ag.z = z

            # (B) ターゲット yaw/pitch
            closest = None
            md = detect_radius
            for p in audiences:
                d = math.hypot(p.x - ag.x, p.y - ag.y)
                if d < md:
                    md = d; closest = p
            if closest:
                dx, dy = closest.x - ag.x, closest.y - ag.y
                dz = closest.height - ag.z
                target_yaw   = degrees(atan2(dy, dx))
                target_pitch = degrees(atan2(dz, hypot(dx, dy)))
            else:
                g_dot_n = -normal.z
                uphill  = (g_dot_n*normal - vector(0,0,-1)).norm()
                target_yaw   = degrees(atan2(uphill.y, uphill.x))
                target_pitch = degrees(asin(uphill.z))

            # (C) イージング適用
            k = min(1.0, ease_speed * dt)
            dyaw = ((target_yaw - ag.yaw + 540) % 360) - 180
            ag.yaw   += dyaw * k
            ag.pitch += (target_pitch - ag.pitch) * k

            # (D) 軸再生成～ジオメトリ反映
            ax  = ag.compute_axis() * agent_length
            ctr = vector(ag.x, ag.y, ag.z)
            ag.body.pos   = ctr - ax/2
            ag.body.axis  = ax
            ag.cable.pos  = ctr
            ag.cable.axis = vector(0,0, maxZ - ag.z)
            # ag.dot2d.pos  = vector(ag.x, ag.y, -0.1)
            u = ax.norm()
            for ld3, ld2, off in ag.leds:
                ld3.pos = ctr + u*(agent_length*off)
                ld2.pos = vector(ag.x + u.x*(agent_length*off),
                                ag.y + u.y*(agent_length*off),
                                0)
            # ─── 7) 色更新（従来通り）────────────────────────
            global_hue   = (sim_time * color_speed) % 1.0
            height_ratio = (ag.z - minZp) / (maxZp - minZp)
            if ag.id == 1:
                # osc_client.send_message('/id1/z',       ag.z)          # 実座標
                osc_client.send_message('/id1/ratio',   height_ratio)  # 0-1 正規化
            hue        = (global_hue + height_ratio * 0.125) % 1.0
            brightness = 1.0 - 0.5 * height_ratio
            r, g, b    = colorsys.hsv_to_rgb(hue, 1.0, brightness)
            osc_client.send_message('/hue', hue)   # ★ここを追加★
            col        = vector(r, g, b)
            ag.body.color = col
            for ld3, ld2, _ in ag.leds:
                ld3.color = ld2.color = col
            
            # ─── 5) 描画・LED・MQTT 出力 ───────────────────────────────
            # for ag in agents:
            #     ag.display()
            #     ag.set_leds()
            send_queue.put(ag)

            
    elif mode_menu.selected == "フローモード":
        # ─── 1) 全エージェントの Z 更新（Boids＋Wave） ─────────────────
        for ag in agents:
            # Boids の高さ更新だけ
            ag.update_boids(sim_time)
            # Wave Z
            w = pnoise2(ag.i*waveScale + sim_time, ag.j*waveScale + sim_time)
            tgtZ = (w + 1)/2 * (maxZ - minZ) + minZ
            ag.z += (tgtZ - ag.z) * waveStrength * dt
            ag.z = min(max(minZ, ag.z), maxZ)

        # ─── 2) 平均高さを flow_target_height に合わせる ───────────────
        mean_h = sum(a.z for a in agents) / len(agents)
        offset = flow_target_height - mean_h
        for ag in agents:
            ag.z = min(max(minZ, ag.z + offset), maxZ)

        # ─── 3) 向きのターゲット計算 + イージング ───────────────────
        for ag in agents:
            # 3-A) Perlin ノイズ由来の「流れ方向」
            flow_yaw   = pnoise2(ag.i*noiseScale + noise_time,
                                ag.j*noiseScale + noise_time) * 360
            flow_pitch = (pnoise2(ag.i*noiseScale + noise_time + 100,
                                ag.j*noiseScale + noise_time + 100) * 120 - 60)

            # 3-B) 検出範囲内に観客がいれば、その方向を target に
            target_yaw   = flow_yaw
            target_pitch = flow_pitch
            closest = None
            md = detect_radius
            for p in audiences:
                d = math.hypot(p.x - ag.x, p.y - ag.y)
                if d < md:
                    md = d; closest = p
            if closest:
                dx, dy = closest.x - ag.x, closest.y - ag.y
                dz = closest.height - ag.z
                target_yaw   = math.degrees(math.atan2(dy, dx))
                target_pitch = math.degrees(math.atan2(dz,
                                                    math.hypot(dx, dy)))
                # ★ここで必ず ±60°にクランプ
                target_pitch = max(-60, min(60, target_pitch))

            # 3-C) イージングで現在値を更新
            k = min(1.0, ease_speed * dt)
            dyaw = ((target_yaw - ag.yaw + 540) % 360) - 180
            ag.yaw   += dyaw * k
            ag.pitch += (target_pitch - ag.pitch) * k
            # ★念のため、イージング後の ag.pitch もクランプしてしまうと安全
            ag.pitch = max(-60, min(60, ag.pitch))

            # ─── 4) ジオメトリ更新 ───────────────────────────────────
            ax  = ag.compute_axis() * agent_length
            ctr = vector(ag.x, ag.y, ag.z)
            ag.body.pos   = ctr - ax/2
            ag.body.axis  = ax
            ag.cable.pos  = ctr
            ag.cable.axis = vector(0,0, maxZ - ag.z)
            # ag.dot2d.pos  = vector(ag.x, ag.y, -0.1)
            u = ax.norm()
            for ld3, ld2, off in ag.leds:
                ld3.pos = ctr + u*(agent_length*off)
                ld2.pos = vector(
                    ag.x + u.x*(agent_length*off),
                    ag.y + u.y*(agent_length*off),
                    0
                )

             # ─── 5) 描画・LED・MQTT 出力 ───────────────────────────────
            for ag in agents:
                ag.display()
                ag.set_leds()
            
            # ─── 6) MQTT 出力──────────────────────────────
            send_queue.put(ag)





    elif mode_menu.selected == "天上天下モード":
        # ─────────────────────────────────────────────
        # 1) Group-A の切り替え判定
        # ─────────────────────────────────────────────
        za = agents[current_groupA_idx].z
        zb = sum(a.z for i, a in enumerate(agents) if i != current_groupA_idx) / (len(agents)-1)
        z_diff = za - zb

        if prev_z_diff is not None and abs(z_diff) < epsilon:
            # 平衡状態なら別のエージェントをリーダーに
            choices = list(range(len(agents)))
            choices.remove(current_groupA_idx)
            current_groupA_idx = random.choice(choices)
        prev_z_diff = z_diff

        # ─────────────────────────────────────────────
        # 2) グループ属性と色の決定（変更検知あり）
        # ─────────────────────────────────────────────
        group_changed = False
        for idx, ag in enumerate(agents):
            new_group = "A" if idx == current_groupA_idx else "B"
            if new_group != getattr(ag, "group", None):
                # スナップショットを取ってトランジション準備
                ag.prev_color  = getattr(ag, "current_color", vector(1, 1, 1))
                ag.prev_z      = ag.z
                ag.prev_yaw    = ag.yaw
                ag.prev_pitch  = ag.pitch
                ag.current_color = vector(1.0, 0.3, 0.3) if new_group == "A" else vector(0.25, 0.6, 1.0)
                ag.group = new_group
                group_changed = True

        if group_changed:
            is_transitioning   = True
            transition_start   = sim_time
            transition_duration= 1.0     # フェード時間 [s]

        # ─────────────────────────────────────────────
        # 3) 上下運動だけ更新
        # ─────────────────────────────────────────────
        for ag in agents:
            ag.update_tenge(sim_time, dt)      # ← ここで ag.z が更新される

        # ─────────────────────────────────────────────
        # 4) 向きとジオメトリ
        # ─────────────────────────────────────────────
        for ag in agents:
            # A) Group-B はリーダー (A) を向く
            if ag.group == "B":
                lead = agents[current_groupA_idx]
                dx, dy = lead.x - ag.x, lead.y - ag.y
                dz     = lead.z - ag.z
                base_yaw   = math.degrees(math.atan2(dy, dx))
                base_pitch = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
            else:
                base_yaw, base_pitch = ag.yaw, ag.pitch  # Group-A は向きを維持

            # B) 近くに観客がいればそちらを優先
            closest, md = None, detect_radius
            for p in audiences:
                d = math.hypot(p.x - ag.x, p.y - ag.y)
                if d < md:
                    md, closest = d, p
            if closest:
                dx2, dy2 = closest.x - ag.x, closest.y - ag.y
                dz2      = closest.height - ag.z
                tgt_yaw   = math.degrees(math.atan2(dy2, dx2))
                tgt_pitch = math.degrees(math.atan2(dz2, math.hypot(dx2, dy2)))
            else:
                tgt_yaw, tgt_pitch = base_yaw, base_pitch

            # C) イージング
            k = min(1.0, ease_speed * dt)
            dyaw = ((tgt_yaw - ag.yaw + 540) % 360) - 180
            ag.yaw   += dyaw * k
            ag.pitch += (tgt_pitch - ag.pitch) * k

            # D) ジオメトリ
            axis = ag.compute_axis() * agent_length
            ctr  = vector(ag.x, ag.y, ag.z)
            ag.body.pos  = ctr - axis/2
            ag.body.axis = axis
            ag.cable.pos = ctr
            ag.cable.axis = vector(0, 0, maxZ - ag.z)
            # 天上天下モードのメインループ内、位置を更新し終えた直後で呼び出す
            send_group_stats(agents, minZ, maxZ)  

        # ─────────────────────────────────────────────
        # 5) 色フェード or 通常 LED
        # ─────────────────────────────────────────────
        if is_transitioning:
            p = min(1.0, (sim_time - transition_start) / transition_duration)
            for ag in agents:
                col = ag.prev_color * (1 - p) + ag.current_color * p
                ag.body.color = col
                for ld3, ld2, _ in ag.leds:
                    ld3.color = ld2.color = col
            if p >= 1.0:
                is_transitioning = False
        else:
            for ag in agents:
                ag.set_leds_tenge(sim_time, dt)   # 時間・高さ依存の発光

        # ─────────────────────────────────────────────
        # 6) 描画 & MQTT 送信
        # ─────────────────────────────────────────────
        for ag in agents:
            ag.display()
            send_queue.put(ag)


    elif mode_menu.selected == "鬼さんこちらモード":
        # 1) 混雑チェック
        is_crowded = True
        step = 0.5
        for cx in frange(x_range[0], x_range[1], step):
            for cy in frange(y_range[0], y_range[1], step):
                if all(math.hypot(p.x-cx, p.y-cy) > oni_empty_radius
                       for p in audiences):
                    is_crowded = False
                    break
            if not is_crowded: break
        if is_crowded:
            print("人多すぎ")

        # 2) その他の筒は 2.5~maxZ をゆらゆら＆クランプ
        for i, ag in enumerate(agents):
            if i == oni_current_idx: continue
            ag.update_boids(sim_time)
            tgt_h = random.uniform(2.5, maxZ)
            ag.z += (tgt_h - ag.z) * waveStrength * dt
            ag.z = max(2.5, min(maxZ, ag.z))
            update_geometry(ag)
            ag.set_leds()

        # 3) ステートマシン
        if oni_state == "idle":
            if sim_time >= oni_next_time and audiences:
                cands = [
                    idx for idx, ag in enumerate(agents)
                    if any(
                        oni_min_select_dist
                          < math.hypot(p.x-ag.x, p.y-ag.y)
                          < oni_max_select_dist
                        for p in audiences
                    )
                ]
                if cands:
                    oni_current_idx      = random.choice(cands)
                    oni_target_aud       = random.choice(audiences)
                    # ── ここで降下停止高さを毎回ランダムに決定 ──
                    oni_target_z         = random.uniform(1.25, 1.6)
                    oni_state            = "descending"
                    # 色イージング初期化（降下する筒の色相を反対に、明度を最大に）
                    oni_color_start   = sim_time
                    oni_initial_color = agents[oni_current_idx].current_color
                    # HSV に変換（元の明度は無視）
                    h0, s0, _ = colorsys.rgb_to_hsv(
                        oni_initial_color.x,
                        oni_initial_color.y,
                        oni_initial_color.z
                    )
                    # 反対の色相、元の彩度、明度は常に 1.0
                    h1 = (h0 + 0.5) % 1.0
                    r1, g1, b1 = colorsys.hsv_to_rgb(h1, s0, 1.0)
                    oni_target_color = vector(r1, g1, b1)


        elif oni_state == "descending":
            ag = agents[oni_current_idx]
            # 近づいてきた人がいれば最も近い人にターゲット変更
            closest = None
            md = 2.0
            for p in audiences:
                d = math.hypot(p.x-ag.x, p.y-ag.y)
                if d < md:
                    md = d
                    closest = p
            if closest:
                oni_target_aud = closest

            # 高さを下げる（1.25mまで）、速度倍速
            ag.z = max(oni_target_z, ag.z - (oni_descent_speed * 2) * dt)
            update_geometry(ag)

            # 常に観客を向く
            dx, dy = oni_target_aud.x - ag.x, oni_target_aud.y - ag.y
            dz     = oni_target_aud.height - ag.z
            ag.yaw   = math.degrees(math.atan2(dy, dx))
            ag.pitch = max(-60, min(60,
                math.degrees(math.atan2(dz, math.hypot(dx, dy)))
            ))

            # 色イージング（2秒で反対色へ）
            tcol = min((sim_time - oni_color_start) / 2.0, 1.0)
            raw_col = oni_initial_color * (1 - tcol) + oni_target_color * tcol
            new_col = vector(
                max(0.0, min(1.0, raw_col.x)),
                max(0.0, min(1.0, raw_col.y)),
                max(0.0, min(1.0, raw_col.z))
            )
            ag.body.color = new_col
            for ld3, ld2, _ in ag.leds:
                ld3.color = ld2.color = new_col
            ag.current_color = new_col

            if md < 2.0 or ag.z <= 1.25 + 1e-3:
                oni_wait_start    = sim_time
                oni_wait_duration = random.uniform(2.0, 6.0)
                oni_state         = "waiting"

        elif oni_state == "waiting":
            ag = agents[oni_current_idx]
            # 近づいてきた人がいればターゲット更新
            closest = None
            md = 2.0
            for p in audiences:
                d = math.hypot(p.x-ag.x, p.y-ag.y)
                if d < md:
                    md = d
                    closest = p
            if closest:
                oni_target_aud = closest

            update_geometry(ag)
            # 常に観客を向く
            dx, dy = oni_target_aud.x - ag.x, oni_target_aud.y - ag.y
            dz     = oni_target_aud.height - ag.z
            ag.yaw   = math.degrees(math.atan2(dy, dx))
            ag.pitch = max(-60, min(60,
                math.degrees(math.atan2(dz, math.hypot(dx, dy)))
            ))

            # 待機解除→上昇時の色戻しイージング初期化
            if md < 2.0 or sim_time - oni_wait_start >= oni_wait_duration:
                oni_state               = "ascending"
                oni_return_start        = sim_time
                oni_return_start_color  = ag.current_color
                update_geometry(ag)
                ag.set_leds()
                oni_return_end_color    = ag.current_color

        elif oni_state == "ascending":
            ag = agents[oni_current_idx]
            # 近づいてきた人がいればターゲット更新
            closest = None
            md = 2.0
            for p in audiences:
                d = math.hypot(p.x-ag.x, p.y-ag.y)
                if d < md:
                    md = d
                    closest = p
            if closest:
                oni_target_aud = closest

            # 高さ更新、速度倍速
            sp = (oni_return_speed * 2
                  if any(math.hypot(p.x-ag.x, p.y-ag.y) < 2 for p in audiences)
                  else (oni_descent_speed * 2))
            ag.z = min(maxZ, ag.z + sp * dt)
            update_geometry(ag)

            # 常に観客を向く
            dx, dy = oni_target_aud.x - ag.x, oni_target_aud.y - ag.y
            dz     = oni_target_aud.height - ag.z
            ag.yaw   = math.degrees(math.atan2(dy, dx))
            ag.pitch = max(-60, min(60,
                math.degrees(math.atan2(dz, math.hypot(dx, dy)))
            ))

            # 色戻しイージング（1秒＋ノージング）
            tback = min((sim_time - oni_return_start) / 1.0, 1.0)
            noise = random.uniform(-0.05, 0.05) * (1 - tback)
            raw_back = oni_return_start_color * (1 - tback) + oni_return_end_color * tback
            back_col = vector(
                max(0.0, min(1.0, raw_back.x + noise)),
                max(0.0, min(1.0, raw_back.y + noise)),
                max(0.0, min(1.0, raw_back.z + noise))
            )
            ag.body.color = back_col
            for ld3, ld2, _ in ag.leds:
                ld3.color = ld2.color = back_col
            ag.current_color = back_col

            if ag.z >= maxZ - 1e-3:
                # 前の上昇開始時刻から 2~8秒 のランダム待機
                oni_next_time    = oni_return_start + random.uniform(1.0, 4.0)
                oni_current_idx  = None
                oni_target_aud   = None
                oni_state        = "idle"

        # ─── 8) 描画・LED・MQTT 出力 ───────────────────────
        for ag in agents:
            ag.display()
            ag.set_leds()
        send_queue.put(ag)

    elif mode_menu.selected == "魚群モード":
        # 0) 流れベクトル（逆向き）を先に求める
        flow_vecs = []
        for ag in agents:
            fx = pnoise2(ag.i*noiseScale + noise_time,
                        ag.j*noiseScale + noise_time)
            fy = pnoise2(ag.i*noiseScale + noise_time + 50,
                        ag.j*noiseScale + noise_time + 50)
            v = vector(-fx, -fy, 0).norm()  # 逆向き
            flow_vecs.append(v)

        # 1) Boids in XY 平面
        for idx, ag in enumerate(agents):
            sep = coh = align = vector(0,0,0)
            cnt = 0
            for jdx in ag.neighbors:
                if 0 <= jdx < len(agents):     # ★ガードを追加★
                    nb = agents[jdx]
                    dxy   = vector(nb.x - ag.x, nb.y - ag.y, 0)
                    dist  = dxy.mag
                    if dist < fish_sep_dist:
                        sep -= dxy / (dist**2 + 1e-3)
                    coh   += dxy
                    align += flow_vecs[jdx]
                    cnt   += 1

            if cnt:
                coh /= cnt
                align /= cnt

            steer = (sep*1.8 +
                    coh * fish_coh_factor +
                    align * fish_align_factor +
                    flow_vecs[idx])

            # 目標方向 → yaw/pitch
            steer = steer.norm()
            tgt_yaw   = math.degrees(math.atan2(steer.y, steer.x))
            tgt_pitch = 0  # 水平泳ぎ

            # 尾びれオシレータ
            phase = ag.idx * 0.7  # 個体差
            osc   = fin_osc_amp_deg * math.sin(2*math.pi*fin_osc_freq_hz*sim_time + phase)
            tgt_yaw += osc

            # イージング
            k = min(1.0, ease_speed * dt)
            dyaw = ((tgt_yaw - ag.yaw + 540) % 360) - 180
            ag.yaw   += dyaw   * k
            ag.pitch += (tgt_pitch - ag.pitch) * k

            # Z の“押し返し”
            z_wave = pnoise2(ag.i*waveScale + sim_time*0.3,
                            ag.j*waveScale + sim_time*0.3)
            ag.z = flow_target_height + z_wave*0.3   # 30 cm 振幅

            update_geometry(ag)

            # LED のきらめき
            hue = (0.55 + steer.x*0.15) % 1.0        # 青緑系中心
            flick = 0.7 + 0.3*math.sin(sim_time*10 + phase)
            r,g,b = colorsys.hsv_to_rgb(hue, 0.8, flick)
            col   = vector(r,g,b)
            ag.current_color = col
            for ld3, ld2, _ in ag.leds:
                ld3.color = ld2.color = col
            
            # ─── 8) 描画・LED・MQTT 出力 ───────────────────────
            # for ag in agents:
                # ag.display()
                # ag.set_leds()
            send_queue.put(ag)

    # ------------------------------------------------------------
    elif mode_menu.selected == "蜂シマーモード":
    # ------------------------------------------------------------
        # 基本
        shim_base_freq_hz = 1.0
        shim_base_amp_m   = 0.005
        shim_wave_speed   = 1.2
        shim_trigger_mean = 6.0
        shim_front_tol    = 0.35

        # 水平回転
        yaw_peak_deg  = 90.0
        yaw_rise_s    = 0.5
        yaw_decay_tau = 0.8

        # ドロップ通常
        drop_m_norm   = 0.15
        drop_down_s_n = 0.2
        drop_up_s_n   = 1.0
        drop_total_n  = drop_down_s_n + drop_up_s_n

        # レアドロップ (0.5 %)
        rare_prob     = 0.002
        rare_factor   = 6
        drop_m_rare   = drop_m_norm * rare_factor          # 0.9 m
        drop_down_s_r = drop_down_s_n * rare_factor        # 1.2 s
        drop_up_s_r   = drop_up_s_n   * rare_factor        # 6 s
        drop_total_r  = drop_down_s_r + drop_up_s_r

        # 色 & フラッシュ
        color_rise_s   = 0.5
        flash_rise_s   = 0.2
        flash_decay_s  = 1.5
        white_mix_peak = 0.5
        bright_peak    = 0.5

        # 0) 波トリガ（震源→中心 方角を保存）
        if sim_time >= next_shim_time:
            hue = random.random()
            r,g,b = colorsys.hsv_to_rgb(hue, 1, 1)
            wave_col = vector(r,g,b)
            ori = random.choice(agents)
            wdir = math.degrees(math.atan2(centerY-ori.y, centerX-ori.x))
            wave_fronts.append((ori.x, ori.y, sim_time, wave_col, wdir))
            next_shim_time = sim_time + random.expovariate(1/shim_trigger_mean)

        # 1) エージェント更新
        for ag in agents:
            z_off = math.sin(2*math.pi*shim_base_freq_hz*sim_time + ag.idx*0.7) * shim_base_amp_m

            # 初期化
            if not hasattr(ag,"yaw0"):
                ag.yaw0 = ag.yaw
                ag.z0   = ag.z
                ag.face_dir = ag.yaw0
                ag.kick_time = ag.drop_time = ag.color_t0 = -999.0
                ag.kick_peak = 0.0
                ag.drop_mode = "norm"          # "norm" or "rare"
                ag.current_color = vector(0.5,0.5,0.0)
                ag.color_from = ag.color_to = ag.current_color
                #  ▼▼ ここを追加 ▼▼
                ag.drop_mode  = "norm"
                ag.drop_down_s = drop_down_s_n
                ag.drop_up_s   = drop_up_s_n
                ag.drop_total  = drop_total_n
                ag.drop_m      = drop_m_norm
                #  ▲▲ ここまで ▲▲


            # レアドロップが完了するまで波を無視
            if ag.drop_mode == "rare" and sim_time - ag.drop_time < drop_total_r:
                wave_hit_allowed = False
            else:
                wave_hit_allowed = True
                ag.drop_mode = "norm"          # レア終了後リセット

            # 波ヒット判定
            if wave_hit_allowed:
                for ox,oy,t0,wcol,wdir in wave_fronts:
                    tau   = sim_time - t0
                    front = shim_wave_speed * tau
                    if 0 < tau < 3 and abs(math.hypot(ag.x-ox,ag.y-oy)-front) < shim_front_tol:
                        # ─ 選択：レア or 通常 ─
                        if random.random() < rare_prob:
                            # --------  レアドロップ  --------
                            ag.drop_mode = "rare"
                            ag.drop_m      = random.uniform(0.45, 1.20)   # 45–120 cm
                            ag.drop_down_s = drop_down_s_n * rare_factor  # 1.2 s
                            ag.drop_up_s   = drop_up_s_n   * rare_factor  # 6.0 s
                            ag.drop_total  = ag.drop_down_s + ag.drop_up_s
                        else:
                            # --------  通常ドロップ  --------
                            ag.drop_mode = "norm"
                            ag.drop_m      = drop_m_norm
                            ag.drop_down_s = drop_down_s_n
                            ag.drop_up_s   = drop_up_s_n
                            ag.drop_total  = drop_total_n


                        

                        z_off       += shim_base_amp_m * 4
                        ag.kick_time = sim_time
                        ag.kick_peak = yaw_peak_deg
                        ag.drop_time = sim_time
                        ag.face_dir  = wdir
                        ag.color_from = ag.current_color
                        ag.color_to   = wcol * 0.5
                        ag.color_t0   = sim_time
                        break   # 1 波で十分

            # ─ 回転 ─
            k_age = sim_time - ag.kick_time
            yaw_kick = (0 if k_age < 0 else
                        yaw_peak_deg * k_age/yaw_rise_s if k_age<=yaw_rise_s else
                        yaw_peak_deg * math.exp(-(k_age-yaw_rise_s)/yaw_decay_tau))
            desired_yaw = ag.face_dir + yaw_kick
            dyaw = ((desired_yaw - ag.yaw + 540)%360) - 180
            ag.yaw += dyaw * 0.1

            # ─ ドロップ ─
            d_age = sim_time - ag.drop_time
            if d_age < 0:
                drop_off = 0
            elif d_age <= ag.drop_down_s:
                drop_off = -ag.drop_m * (d_age / ag.drop_down_s)
            elif d_age <= ag.drop_total:
                drop_off = -ag.drop_m * (1 - (d_age - ag.drop_down_s)/ag.drop_up_s)
            else:
                drop_off = 0

            # ─ 色基本 ─
            if sim_time - ag.color_t0 < color_rise_s:
                t = (sim_time - ag.color_t0)/color_rise_s
                base_col = ag.color_from*(1-t) + ag.color_to*t
            else:
                base_col = ag.color_to

            # ─ フラッシュ ─
            f_age = sim_time - ag.drop_time
            if 0 <= f_age <= flash_rise_s + flash_decay_s:
                if f_age <= flash_rise_s:
                    r = f_age/flash_rise_s
                else:
                    r = 1 - (f_age - flash_rise_s)/flash_decay_s
                mix = white_mix_peak*r
                inc = 1 + bright_peak*r
                col = (base_col*(1-mix) + vector(1,1,1)*mix) * inc
                col = vector(min(1,col.x),min(1,col.y),min(1,col.z))
            else:
                col = base_col

            ag.current_color = col

            # ─ 近接観客 強調 (通常のみ) ─
            if ag.drop_mode == "norm":
                for p in audiences:
                    if math.hypot(p.x-ag.x,p.y-ag.y) < 1:
                        z_off  *= 2
                        break

            # ─ 反映 ─
            ag.z   = clamp(ag.z0 + z_off + drop_off, minZ, maxZ)
            ag.pitch = 0
            update_geometry(ag)
            for l3,l2,_ in ag.leds:
                l3.color = l2.color = ag.current_color

            # ─── 8) 描画・LED・MQTT 出力 ───────────────────────
            for ag in agents:
                ag.display()
                ag.set_leds()
            send_queue.put(ag)

        # 2) 古い波 8 s で削除
        wave_fronts[:] = [(x,y,t,c,d) for (x,y,t,c,d) in wave_fronts
                          if sim_time - t < 8]
    # ------------------------------------------------------------
    

    apply_sound_reaction()     # ★追加★

    # 3Dカメラ軌道更新
    camX = centerX + radius * math.cos(angle)
    camY = centerY + radius * math.sin(angle)
    scene3d.camera.pos  = vector(camX, camY, cameraHeight)
    scene3d.camera.axis = vector(centerX-camX,
                                 centerY-camY,
                                 ((minZ+maxZ)/2)-cameraHeight)
    
    # ループの最後のほう、筒の処理が終わった後などに追加
    if sensor_mode and sensor_person is not None:
        if sim_time - last_aud_msg_time > PRESENCE_TIMEOUT:
            # しばらく信号が来ていない → 隠す
            sensor_person.body.visible  = False
            sensor_person.head.visible  = False
            sensor_person.dot2d.visible = False

    