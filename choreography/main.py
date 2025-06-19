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

#=========================================================
# ここはどこか
place = "berlin"  # "venue" or else
#=========================================================

# SuperCollider サーバーのホストとポート
HOST = "127.0.0.1"
SC_PORT = 57120 # 57120 is SupperCollider, max uses 57121
MX_PORT = 57121 # 57120 is SupperCollider, max uses 57121
osc_client_sc = udp_client.SimpleUDPClient(HOST, SC_PORT)
osc_client_max = udp_client.SimpleUDPClient(HOST, MX_PORT)

# ─── モード切替トランジション用 ─────────────────────
is_transitioning   = False
transition_start   = 0.0
transition_duration = 2.0   # 切り替えを何秒かけるか

# ========================================================
# グローバルパラメータ（初期値）
# ========================================================
paused = False          # 一時停止フラグ


# ========================================================
# マニュアルモード用のグローバル変数
# ========================================================
manual_commands = {}  # {target_id: {"z": float, "yaw": float, ...}}
manual_global = {}    # 空の辞書で初期化（Noneではなく）


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
#   False … ランダム歩行（既存）
#   True  … MQTT “ca” 座標で直接配置（センサーモード）
# --------------------------------------------------------
sensor_mode         = False         # ← 変更なし
sensor_people: list['Audience'] = []   # ★ New: 複数人用リスト
last_aud_msg_time   = -999.0
PRESENCE_TIMEOUT    = 1.0           # 秒：信号が途絶えてから非表示まで


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
wave_events = []


#   人が来なくなってから “何秒” で全員を隠すか
PRESENCE_TIMEOUT = 1.0        # ★ 1 秒
last_aud_msg_time = -999.0


# ========================================================
# MQTT セットアップ
# ========================================================
mqtt_client = mqtt.Client()
if place == "venue":
    mqtt_client.connect("192.168.1.2", 1883, 60)
else:
    mqtt_client.connect("127.0.0.1", 1883, 60)
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
    
    # マニュアルモードから他のモードに切り替わる場合
    if mode_menu.selected == "マニュアルモード":
        clear_manual_commands()
    
    # ★追加：全エージェントの自律モードを解除し、リングを非表示に
    for ag in agents:
        if hasattr(ag, 'autonomous_mode'):
            ag.autonomous_mode = False
            ag.actual_pitch = ag.pitch
        if hasattr(ag, 'update_autonomous_indicator'):
            ag.update_autonomous_indicator()  # これで赤いリングが消える
    
    # 既存のトランジション処理
    is_transitioning = True
    transition_start = sim_time
    for ag in agents:
        ag.prev_z = ag.z
        ag.prev_yaw = ag.yaw
        ag.prev_pitch = ag.pitch
        ag.prev_color = ag.current_color

def clear_manual_commands():
    """
    マニュアルモードから他のモードに切り替わる際に
    手動制御のコマンドをすべてクリアする
    """
    global manual_commands, manual_global
    manual_commands.clear()
    manual_global.clear()
    print("[Manual] Commands cleared due to mode change")


# ② ドロップダウンを 1 回だけ生成し、すぐ隠す
modes = ["マニュアルモード",
         "魚群モード",
         "蜂シマーモード",
         "天上天下モード",
         "回る天井"]
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
    global sensor_mode, sensor_people, audiences
    sensor_mode = not sensor_mode
    b.text = "Sensor ON" if sensor_mode else "Sensor OFF"
    print("Sensor mode =", sensor_mode)

    if sensor_mode:
        # 既存オブジェクトをすべて不可視にしてリストを空に
        for p in audiences + sensor_people:
            if hasattr(p, "body"):
                p.body.visible  = p.head.visible = False
                p.dot2d.visible = False
        audiences.clear()
        sensor_people.clear()       # まっさらに
    else:
        # シミュレーションに戻るとき：センサー用オブジェクトを消す
        for p in sensor_people:
            p.body.visible = p.head.visible = p.dot2d.visible = False
        sensor_people.clear()


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

        self.drop_time  = float('-inf')   # ← 追加：これで常に存在
        self.drop_total = 0.0

        # ダウンライトの明るさ変数を追加
        self.downlight_brightness = 0.0  # 0.0-1.0の範囲
        self.target_downlight = 0.0      # イージング用の目標値

        # 自律制御モード用
        self.autonomous_mode = False  # 自律制御中かどうか
        self.actual_pitch = self.pitch  # 実際の表示用pitch

        # 筒の端の表示用リング（赤い輪）
        self.end_ring_front = ring(
            canvas=scene3d,
            pos=vector(x, y, z),
            axis=self.compute_axis(),
            radius=agent_radius * 1.2,  # 少し大きめ
            thickness=agent_radius * 0.2,
            color=color.red,
            visible=False  # 初期は非表示
        )
        # self.end_ring_back = ring(
        #     canvas=scene3d,
        #     pos=vector(x, y, z),
        #     axis=self.compute_axis(),
        #     radius=agent_radius * 1.2,
        #     thickness=agent_radius * 0.2,
        #     color=color.red,
        #     visible=False
        # )
        # 2Dビューでの自律モード表示（オプション）
        self.auto_indicator_2d = sphere(
            canvas=scene2d,
            pos=vector(self.x, self.y, 0.1),  # LEDより上のレイヤー
            radius=agent_radius * 0.5,
            color=color.red,
            emissive=True,
            visible=False
        )
        # 2Dビューにダウンライト表示用の円を追加
        # LEDよりも下のレイヤー（z=-0.2）に配置
        self.downlight_2d = cylinder(
            canvas=scene2d,
            pos=vector(self.x, self.y, -0.2),  # LEDより下
            axis=vector(0, 0, 0.01),  # 薄い円柱
            radius=agent_radius * 3,   # エージェントより大きめの円
            color=vector(1, 1, 0.8),   # 暖色系の光
            opacity=0.0,               # 初期は透明
            emissive=True              # 発光
        )

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

    # def compute_axis(self):
    #     pr, yr = math.radians(self.pitch), math.radians(self.yaw)
    #     return vector(math.cos(pr)*math.cos(yr),
    #                   math.cos(pr)*math.sin(yr),
    #                   math.sin(pr))
    def compute_axis(self):
        # 自律モード時は実際の表示用pitchを使用
        if hasattr(self, 'autonomous_mode') and self.autonomous_mode:
            pr = math.radians(self.actual_pitch)
        else:
            pr = math.radians(self.pitch)
        
        yr = math.radians(self.yaw)
        return vector(math.cos(pr)*math.cos(yr),
                      math.cos(pr)*math.sin(yr),
                      math.sin(pr))
    
    def update_autonomous_indicator(self):
        """自律モードのビジュアル表示を更新"""
        if self.autonomous_mode:
            # 筒の両端に赤いリングを表示
            ax = self.compute_axis() * agent_length
            ctr = vector(self.x, self.y, self.z)
            
            # 前端と後端の位置
            front_pos = ctr + ax/2
            back_pos = ctr - ax/2
            
            self.end_ring_front.pos = front_pos
            self.end_ring_front.axis = ax.norm() * 0.01  # 薄いリング
            self.end_ring_front.visible = True
            
            # self.end_ring_back.pos = back_pos
            # self.end_ring_back.axis = ax.norm() * 0.01
            # self.end_ring_back.visible = True
            
            # 2Dビューの更新
            self.auto_indicator_2d.visible = True
            self.auto_indicator_2d.pos = vector(self.x, self.y, 0.1)
        else:
            # 通常モード：リングを非表示
            self.end_ring_front.visible = False
            # self.end_ring_back.visible = False
            self.auto_indicator_2d.visible = False
    
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

    # def update(self, dt):
    #     # 1) ノイズ時間を進める
    #     self.noise_t += dt * audience_noise_speed

    #     # 1.5) 速度変動（Perlinノイズ＋振幅パラメータ）
    #     delta = pnoise1(self.noise_t * 0.5)  # -1..+1
    #     self.current_speed = self.speed * (1 + audience_speed_amp * delta * 0.5)

    #     # 2) 移動方向のノイズ取得
    #     n = pnoise2(self.x * audience_noise_scale, self.noise_t)
    #     angle = n * 2 * math.pi

    #     # 3) 基本移動量
    #     dx = math.cos(angle) * self.current_speed * dt
    #     dy = math.sin(angle) * self.current_speed * dt

    #     # 4) 壁反発：壁に近いほど跳ね返る成分を加える
    #     margin_x = min(self.x - self.xmin, self.xmax - self.x)
    #     margin_y = min(self.y - self.ymin, self.ymax - self.y)
    #     threshold = self.radius * 2
    #     if margin_x < threshold:
    #         repel = (threshold - margin_x) / threshold
    #         dx += repel * (-math.cos(angle)) * self.current_speed * dt
    #     if margin_y < threshold:
    #         repel = (threshold - margin_y) / threshold
    #         dy += repel * (-math.sin(angle)) * self.current_speed * dt

    #     # 5) 他の観客との衝突回避
    #     min_sep = self.radius * 2
    #     for other in audiences:
    #         if other is self: continue
    #         dist = math.hypot(self.x - other.x, self.y - other.y)
    #         if dist < min_sep and dist > 1e-3:
    #             # お互い離れるベクトルを加算
    #             ux = (self.x - other.x) / dist
    #             uy = (self.y - other.y) / dist
    #             factor = (min_sep - dist) / min_sep
    #             dx += ux * factor * self.current_speed * dt
    #             dy += uy * factor * self.current_speed * dt

    #     # 6) 移動量を反映＆クランプ
    #     new_x = self.x + dx
    #     new_y = self.y + dy
    #     self.x = min(max(new_x, self.xmin), self.xmax)
    #     self.y = min(max(new_y, self.ymin), self.ymax)

    #     # 7) 3D 体幹・頭部更新
    #     self.body.pos = vector(self.x, self.y, 0)
    #     self.head.pos = vector(self.x, self.y, self.cyl_h + self.radius)

    #     # 8) 2D ドット更新
    #     self.dot2d.pos = vector(self.x, self.y, -0.1)
    def update(self, dt):
        # 1) ノイズ時間を進める
        self.noise_t += dt * audience_noise_speed

        # 1.5) 速度変動（Perlinノイズ＋振幅パラメータ）
        delta = pnoise1(self.noise_t * 0.5)  # -1..+1
        self.current_speed = self.speed * (1 + audience_speed_amp * delta * 0.5)

        # 2) 移動方向のノイズ取得（スケールを調整して振動を減らす）
        n = pnoise2(self.x * audience_noise_scale * 0.5, self.noise_t)
        base_angle = n * 2 * math.pi

        # ★ 3) 改善版：中心への緩やかな引力を追加
        center_x = (self.xmin + self.xmax) / 2
        center_y = (self.ymin + self.ymax) / 2
        to_center_x = center_x - self.x
        to_center_y = center_y - self.y
        dist_from_center = math.hypot(to_center_x, to_center_y)
        
        # エリアの半径
        area_radius = min((self.xmax - self.xmin) / 2, (self.ymax - self.ymin) / 2)
        
        # 中心から離れるほど強くなる引力（0.0〜0.2に減少）
        if dist_from_center > 0.01:
            center_pull = min(0.2, (dist_from_center / area_radius) * 0.2)
            center_angle = math.atan2(to_center_y, to_center_x)
            
            # ベース角度と中心への角度をブレンド
            angle = base_angle * (1 - center_pull) + center_angle * center_pull
        else:
            angle = base_angle

        # 4) 基本移動量の初期化
        dx = 0.0
        dy = 0.0

        # ★ 5) 他の観客との衝突回避（先に計算して振動を防ぐ）
        separation_radius = self.radius * 3.0  # 分離を始める距離
        min_sep = self.radius * 2.2  # 最小許容距離
        
        for other in audiences:
            if other is self: continue
            dist = math.hypot(self.x - other.x, self.y - other.y)
            
            if dist < separation_radius and dist > 1e-3:
                # 反発ベクトル
                ux = (self.x - other.x) / dist
                uy = (self.y - other.y) / dist
                
                # 距離に応じた反発力（近いほど強い）
                if dist < min_sep:
                    # 強い反発
                    force = 2.0 * (1 - dist / min_sep)
                else:
                    # 緩やかな反発
                    force = 0.5 * (1 - (dist - min_sep) / (separation_radius - min_sep))
                
                dx += ux * force * self.current_speed * dt
                dy += uy * force * self.current_speed * dt

        # 6) 基本的な移動を追加（衝突回避の後）
        dx += math.cos(angle) * self.current_speed * dt * 0.7  # 基本移動を少し抑制
        dy += math.sin(angle) * self.current_speed * dt * 0.7

        # ★ 7) 壁の処理（シンプル化）
        wall_threshold = self.radius * 2.5
        wall_force = 1.5
        
        # 各壁からの距離
        dist_left = self.x - self.xmin
        dist_right = self.xmax - self.x
        dist_bottom = self.y - self.ymin
        dist_top = self.ymax - self.y
        
        # 壁に近い時の反発
        if dist_left < wall_threshold:
            dx += (wall_threshold - dist_left) / wall_threshold * wall_force * dt
        if dist_right < wall_threshold:
            dx -= (wall_threshold - dist_right) / wall_threshold * wall_force * dt
        if dist_bottom < wall_threshold:
            dy += (wall_threshold - dist_bottom) / wall_threshold * wall_force * dt
        if dist_top < wall_threshold:
            dy -= (wall_threshold - dist_top) / wall_threshold * wall_force * dt

        # ★ 8) 速度制限（振動防止）
        speed_limit = self.current_speed * dt * 1.5
        current_speed = math.hypot(dx, dy)
        if current_speed > speed_limit:
            dx = dx / current_speed * speed_limit
            dy = dy / current_speed * speed_limit

        # 9) 移動量を反映
        new_x = self.x + dx
        new_y = self.y + dy
        
        # ★ 10) 最終的な衝突チェック（他の観客と重ならないように）
        position_valid = True
        for other in audiences:
            if other is self: continue
            future_dist = math.hypot(new_x - other.x, new_y - other.y)
            if future_dist < self.radius * 2:
                position_valid = False
                break
        
        # 位置が有効な場合のみ更新
        if position_valid:
            self.x = new_x
            self.y = new_y
        
        # 壁の境界内に収める
        margin = self.radius
        self.x = min(max(self.x, self.xmin + margin), self.xmax - margin)
        self.y = min(max(self.y, self.ymin + margin), self.ymax - margin)

        # 11) 3D 体幹・頭部更新
        self.body.pos = vector(self.x, self.y, 0)
        self.head.pos = vector(self.x, self.y, self.cyl_h + self.radius)

        # 12) 2D ドット更新
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
                            int(row["id"]),
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


def handle_audience(coords_list: list[tuple[float, float]]):
    """
    coords_list … [(x,y), (x,y), …]  0 人なら空リスト
    """
    global sensor_people, audiences, last_aud_msg_time
    last_aud_msg_time = sim_time

    if not sensor_mode:
        return                       # OFF のときは無視

    # (A) 人数合わせ
    n_recv, n_curr = len(coords_list), len(sensor_people)
    if n_recv > n_curr:                      # 追加生成
        x_range = (centerX - span/2, centerX + span/2)
        y_range = (centerY - span/2, centerY + span/2)
        for _ in range(n_recv - n_curr):
            sensor_people.append(
                Audience(scene3d, scene2d,
                         x_range, y_range,
                         height=1.7, radius=0.15, speed=0.0)
            )
    elif n_recv < n_curr:                    # 余分を隠す
        for p in sensor_people[n_recv:]:
            p.body.visible = p.head.visible = p.dot2d.visible = False
        sensor_people[:] = sensor_people[:n_recv]

    # (B) 座標更新
    for (x, y), person in zip(coords_list, sensor_people):
        person.x, person.y = x, y
        person.body.pos = vector(x, y, 0)
        person.head.pos = vector(x, y, person.cyl_h + person.radius)
        person.dot2d.pos = vector(x, y, -0.1)
        person.body.visible = person.head.visible = person.dot2d.visible = True

    # (C) ほかのロジックが audiences を見る場合に備えて同期
    audiences = sensor_people



# 2. ダウンライト表示を更新する関数：
def update_downlight_display(agent):
    """
    ダウンライトの明るさを2Dビューに可視化
    """
    # 明るさに応じて不透明度を設定（0-1の範囲）
    opacity = agent.downlight_brightness * 0.8  # 最大でも80%の不透明度
    
    # 2D表示の更新
    agent.downlight_2d.opacity = opacity
    
    # 明るさに応じて色も少し変化（暗い時は赤っぽく、明るい時は白っぽく）
    agent.downlight_2d.color = vector(
        1.0,
        0.8 + 0.2 * agent.downlight_brightness,
        0.6 + 0.4 * agent.downlight_brightness
    )


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
        # --- 旧 ---
        sensor_person = None
        # --- 新 ---
        sensor_people: list[Audience] = []

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

# ========================================================
# 自律制御判定用の関数（グローバル）
# ========================================================

def check_autonomous_mode(agent, audiences, autonomous_threshold=1.5):
    """
    エージェントが観客に近く、かつ正面を向いている場合に自律モードを有効化
    
    Parameters:
    - agent: エージェントオブジェクト
    - audiences: 観客リスト
    - autonomous_threshold: 自律モードに入る距離閾値[m]
    
    Returns:
    - should_be_autonomous: 自律モードにすべきかどうか
    - target_pitch: 自律制御時の目標pitch角度（通常時はNone）
    """
    closest = None
    min_dist = float('inf')
    
    # 最も近い観客を探す
    for person in audiences:
        dist = math.hypot(person.x - agent.x, person.y - agent.y)
        if dist < min_dist:
            min_dist = dist
            closest = person
    
    if closest and min_dist < autonomous_threshold:
        # 観客への方向を計算
        dx = closest.x - agent.x
        dy = closest.y - agent.y
        target_yaw = math.degrees(math.atan2(dy, dx))
        
        # 現在の向きと目標方向の差
        yaw_diff = abs(((target_yaw - agent.yaw + 540) % 360) - 180)
        
        # 正面を向いている（±30度以内）場合に自律モードON
        if yaw_diff < 30:
            # 観客の高さを考慮したpitch計算
            dz = closest.height - agent.z
            horiz_dist = math.hypot(dx, dy)
            target_pitch = math.degrees(math.atan2(dz, horiz_dist))
            target_pitch = max(-60, min(60, target_pitch))
            
            return True, target_pitch
    
    return False, None


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
    - autonomous_mode: bool (追加)
    - actual_pitch: float (追加)
    """
    # ---- MQTT 出力 ----
    # 位置・向き
    z_m = max(minZ, min(maxZ, agent.z))
    yaw_deg = (agent.yaw % 360.0 + 360.0) % 360.0  # 0–360
    
    # Pitch: 自律モードなら90度を送信、そうでなければ実際の値
    if agent.autonomous_mode:
        pitch_deg = 90.0  # 自律制御の信号
    else:
        pitch_deg = max(-60.0, min(60.0, agent.pitch))
    
    mm = int(z_m * 1000 + 0.5)  # 0–2800 → uint16_t
    pitchC = int(pitch_deg * 100 + 0.5)  # -6000〜+6000 or 9000 → int16_t
    yawC = int(yaw_deg * 100 + 0.5)  # 0–35999 → uint16_t
    
    # パック (6 B)
    payload_pos = struct.pack("<HhH", mm, pitchC, yawC)
    mqtt_client.publish(f"ps/{agent.node_id}", payload_pos)
    
    # 色は従来どおり
    r = int(agent.current_color.x * 255)
    g = int(agent.current_color.y * 255)
    b = int(agent.current_color.z * 255)
    mqtt_client.publish(f"cl/{agent.node_id}", bytes([r, g, b]))
    
    # ダウンライト
    dl = int(agent.downlight_brightness * 255)
    mqtt_client.publish(f"dl/{agent.node_id}", bytes([dl]))


# def send_robot_data(agent):
#     """
#     agent: 各エージェントオブジェクト
#     - node_id: int
#     - z, pitch, yaw: float
#     - current_color.x/y/z: 0.0–1.0
#     """
#     # ---- MQTT 出力 ----
#     # 位置・向き
#     # ─ ① スケール＆クリップ ──────────────────────────
#     z_m       = max(minZ, min(maxZ, agent.z))
#     pitch_deg = max(-60.0, min(60.0, agent.pitch))
#     yaw_deg   = (agent.yaw % 360.0 + 360.0) % 360.0      # 0–360

#     mm     = int(z_m * 1000 + 0.5)          # 0–2800 → uint16_t
#     pitchC = int(pitch_deg * 100 + 0.5)     # -6000〜+6000 → int16_t
#     yawC   = int(yaw_deg   * 100 + 0.5)     # 0–35999 → uint16_t

#     # ─ ② パック (6 B) ────────────────────────────────
#     payload_pos = struct.pack("<HhH", mm, pitchC, yawC)
#     mqtt_client.publish(f"ps/{agent.node_id}", payload_pos)

#     # ─ ③ 色は従来どおり 1 バイト ×3 ────────────────
#     r = int(agent.current_color.x * 255)
#     g = int(agent.current_color.y * 255)
#     b = int(agent.current_color.z * 255)
#     mqtt_client.publish(f"cl/{agent.node_id}", bytes([r, g, b]))
#     # mqtt_client.publish(f"dl/{agent.node_id}", bytes([int(round((r+g+b)/3))]))

#     # ─ ④ ダウンライト送信（新規追加）────────────────
#     dl = int(agent.downlight_brightness * 255)
#     mqtt_client.publish(f"dl/{agent.node_id}", bytes([dl]))

    # ---- OSC 出力 ----
    # SuperCollider の OSCFunc('/robot') に合わせて、
    # [id, r, g, b, angle, height]
    # angle は 0–360deg のまま、height は 0.0–1.0 正規化済みと仮定
    # osc_client.send_message(
    #     "/robot",
    #     [agent.node_id,
    #      agent.current_color.x,s
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
        osc_client_max.send_message('/groupA_height', norm_height(avg_z_A))
    if groupB:
        avg_z_B = sum(ag.z for ag in groupB) / len(groupB)
        osc_client_max.send_message('/groupB_height', norm_height(avg_z_B))

    # groupA の X/Y も送る
    if groupA:
        avg_x_A = sum(ag.x for ag in groupA) / len(groupA)
        avg_y_A = sum(ag.y for ag in groupA) / len(groupA)
        osc_client_max.send_message('/groupA_x', avg_x_A)
        osc_client_max.send_message('/groupA_y', avg_y_A)


def process_manual_commands():
    """
    改良版：MaxMSPからのOSCパラメータを即座に反映
    """
    global manual_commands, manual_global  # ← これが重要！
    
    # OSCからパラメータを取得
    target_id = int(params.get("manual_target", 0))
    z_cmd = params.get("manual_z", None)
    yaw_cmd = params.get("manual_yaw", None)
    pitch_cmd = params.get("manual_pitch", None)
    r_cmd = params.get("manual_r", None)
    g_cmd = params.get("manual_g", None)
    b_cmd = params.get("manual_b", None)
    dl_cmd = params.get("manual_dl", None)  # ← 追加
    
    # 値の変化を検出するための初期化
    if not hasattr(process_manual_commands, 'last_values'):
        process_manual_commands.last_values = {}
        process_manual_commands.last_target = -1
    
    # ターゲットが変わったかチェック
    target_changed = (target_id != process_manual_commands.last_target)
    if target_changed:
        print(f"[Manual] Target changed: {process_manual_commands.last_target} → {target_id}")
        process_manual_commands.last_target = target_id
    
    # 現在の値をまとめる
    current_values = {
        'z': z_cmd, 'yaw': yaw_cmd, 'pitch': pitch_cmd,
        'r': r_cmd, 'g': g_cmd, 'b': b_cmd,
        'dl': dl_cmd  # ← 追加
    }
    
    # 値が変化したかチェック
    value_changed = False
    for key, val in current_values.items():
        if val is not None:
            last_val = process_manual_commands.last_values.get(f"{target_id}_{key}")
            if last_val != val:
                value_changed = True
                process_manual_commands.last_values[f"{target_id}_{key}"] = val
    
    # コマンドオブジェクトを構築
    command = {}
    
    # 位置・姿勢のコマンド
    if z_cmd is not None:
        command["z"] = max(minZ, min(maxZ, float(z_cmd)))
    if yaw_cmd is not None:
        command["yaw"] = (float(yaw_cmd) % 360.0 + 360.0) % 360.0
    if pitch_cmd is not None:
        command["pitch"] = max(-60.0, min(60.0, float(pitch_cmd)))
    
    # 色のコマンド
    if r_cmd is not None:
        command["r"] = max(0.0, min(1.0, float(r_cmd)))
    if g_cmd is not None:
        command["g"] = max(0.0, min(1.0, float(g_cmd)))
    if b_cmd is not None:
        command["b"] = max(0.0, min(1.0, float(b_cmd)))
    if dl_cmd is not None:
        command["dl"] = max(0.0, min(1.0, float(dl_cmd)))

    # コマンドがある場合のみ処理
    if command:
        if target_id == 0:
            # 全体制御 - グローバル変数を更新
            manual_global.update(command)  # ← 代入ではなくupdate()を使用
            if value_changed:
                print(f"[Manual] Global command: {command}")
        else:
            # 個別制御
            manual_commands[target_id] = command.copy()
            if value_changed:
                print(f"[Manual] ID {target_id} command: {command}")
                # デバッグ：コマンドが設定されたことを確認
                print(f"[Manual] manual_commands now contains: {list(manual_commands.keys())}")


def apply_manual_mode():
    """
    マニュアルモードでの各エージェントの制御（デバッグ強化版）
    """
    global manual_commands, manual_global
    
    target_id = int(params.get("manual_target", 0))
    
    # 一度だけデバッグ情報を表示
    if not hasattr(apply_manual_mode, 'debug_done'):
        print(f"\n[Debug] Total agents: {len(agents)}")
        print(f"[Debug] Agent node_ids: {[ag.node_id for ag in agents[:10]]}...")
        print(f"[Debug] Current target_id: {target_id}")
        print(f"[Debug] manual_commands keys: {list(manual_commands.keys())}")
        apply_manual_mode.debug_done = True
    
    # 適用されたエージェントの数をカウント
    applied_count = 0
    
    for ag in agents:
        cmd = None
        
        # 制御コマンドの選択
        if target_id == 0:
            # 全体制御モード
            if manual_global:
                cmd = manual_global
        else:
            # 個別制御モード
            if ag.node_id == target_id and target_id in manual_commands:
                cmd = manual_commands[target_id]
                applied_count += 1
                # 初回適用時のみデバッグ出力
                if not hasattr(ag, '_manual_debug_printed'):
                    print(f"[Manual] Found matching agent: node_id={ag.node_id}, applying command: {cmd}")
                    ag._manual_debug_printed = True
        
        if cmd:
            # 位置・姿勢の更新（スムーズなイージング）
            ease_factor = 0.1
            
            if "z" in cmd:
                ag.z += (cmd["z"] - ag.z) * ease_factor
            
            if "yaw" in cmd:
                dyaw = ((cmd["yaw"] - ag.yaw + 540) % 360) - 180
                ag.yaw += dyaw * ease_factor
            
            if "pitch" in cmd:
                ag.pitch += (cmd["pitch"] - ag.pitch) * ease_factor
            
            # 色の更新
            if "r" in cmd and "g" in cmd and "b" in cmd:
                target_color = vector(cmd["r"], cmd["g"], cmd["b"])
                ag.current_color += (target_color - ag.current_color) * ease_factor
            
            # ダウンライトの更新（追加）
            if "dl" in cmd:
                ag.target_downlight = cmd["dl"]
                ag.downlight_brightness += (ag.target_downlight - ag.downlight_brightness) * ease_factor
            
            # ジオメトリの更新
            ax = ag.compute_axis() * agent_length
            ctr = vector(ag.x, ag.y, ag.z)
            ag.body.pos = ctr - ax/2
            ag.body.axis = ax
            ag.cable.pos = ctr
            ag.cable.axis = vector(0, 0, maxZ - ag.z)
            
            # LEDの更新
            u = ax.norm()
            for ld3, ld2, off in ag.leds:
                ld3.pos = ctr + u * (agent_length * off)
                ld2.pos = vector(ag.x + u.x * (agent_length * off),
                               ag.y + u.y * (agent_length * off), 0)
                ld3.color = ld2.color = ag.current_color
            
            # 本体の色も更新
            ag.body.color = ag.current_color
            
            # MQTT送信キューに追加
            send_queue.put(ag)
    
    # デバッグ：個別制御時に適用されたエージェント数を表示
    if target_id > 0 and applied_count == 0 and random.random() < 0.05:  # 5%の確率で
        print(f"[Warning] No agent found with node_id={target_id}")
        print(f"[Warning] Available node_ids: {sorted([ag.node_id for ag in agents])}")
# ========================================================
# メインループ
# ========================================================

print(">>> [main] about to call start_osc_listener()")
start_osc_listener(ip="0.0.0.0", port=8000)
print(">>> [main] returned from start_osc_listener()")

# mqtt起動
if place == "venue":
    client = start(broker_host="192.168.1.2", broker_port=1883)
else:
    client = start(broker_host="localhost", broker_port=1883)


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
    # デバッグ用：マニュアルモードのパラメータを確認（オプション）
    if mode_menu.selected == "マニュアルモード" and random.random() < 0.01:  # 1%の確率で
        manual_target = int(params.get("manual_target", 0))
        manual_z = params.get("manual_z", None)
        print(f"[Debug] Manual params: target={manual_target}, z={manual_z}")
    

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


###回る天井モード--------------------------------------------------------------------   
    if mode_menu.selected == "回る天井":
        # ========================================================
        # 回る天井モードの初期化とトランジション
        # ========================================================
        
        # モード切り替え検出
        if not hasattr(mode_menu, 'ceiling_mode_initialized') or not mode_menu.ceiling_mode_initialized:
            print(f"[回る天井モード] 初期化開始")
            mode_menu.ceiling_mode_initialized = True
            mode_menu.ceiling_transition_start = sim_time
            mode_menu.ceiling_transition_duration = 2.0  # 2秒のトランジション
            
            # 平面の初期角度を設定（現在の状態から開始）
            if not hasattr(mode_menu, 'plane_angle_saved'):
                mode_menu.plane_angle_saved = plane_angle
            
            # 各エージェントの開始状態を保存
            for ag in agents:
                ag.ceiling_start_z = ag.z
                ag.ceiling_start_yaw = ag.yaw
                ag.ceiling_start_pitch = ag.pitch
                ag.ceiling_start_color = vector(ag.current_color.x, ag.current_color.y, ag.current_color.z)
                
                # 自律モードの状態も保存
                ag.ceiling_start_autonomous = getattr(ag, 'autonomous_mode', False)
                
                print(f"  Agent {ag.node_id}: z={ag.ceiling_start_z:.2f}, "
                      f"yaw={ag.ceiling_start_yaw:.1f}, pitch={ag.ceiling_start_pitch:.1f}")
        
        # 他のモードの初期化フラグをリセット
        if hasattr(mode_menu, 'global_prev_mode') and mode_menu.global_prev_mode != "回る天井":
            mode_menu.fish_mode_initialized = False
            mode_menu.shimmer_initialized = False
            mode_menu.tenge_initialized = False
        
        # トランジション進行度を計算
        transition_elapsed = sim_time - getattr(mode_menu, 'ceiling_transition_start', sim_time)
        transition_progress = min(1.0, transition_elapsed / getattr(mode_menu, 'ceiling_transition_duration', 2.0))
        
        # イージング関数（easeInOutCubic）
        if transition_progress < 0.5:
            eased_progress = 4 * transition_progress * transition_progress * transition_progress
        else:
            eased_progress = 1 - pow(-2 * transition_progress + 2, 3) / 2
        
        # トランジション中かどうか
        in_transition = transition_progress < 1.0
        
        # ========================================================
        # 傾斜面の計算
        # ========================================================
        # 1) 傾斜面の法線ベクトル
        a = radians(tilt_angle_deg)
        n0 = vector(0, -sin(a), cos(a))
        ca, sa = cos(plane_angle), sin(plane_angle)
        normal = vector(n0.x*ca - n0.y*sa,
                        n0.x*sa + n0.y*ca,
                        n0.z).norm()

        # 2) 平面上 Z のリストを先に作る
        plane_zs = []
        for ag in agents:
            dx, dy = ag.x - centerX, ag.y - centerY
            z_p = plane_height - (normal.x*dx + normal.y*dy) / normal.z
            plane_zs.append(min(max(minZ, z_p), maxZ))
        minZp, maxZp = min(plane_zs), max(plane_zs)

        # OSCメッセージ送信
        sin_val = math.sin(sim_time)
        cos_val = math.cos(sim_time)
        osc_client_max.send_message('/sin', sin_val)
        osc_client_max.send_message('/cos', cos_val)

        # ========================================================
        # 各エージェントごとの処理
        # ========================================================
        for ag, target_z_plane in zip(agents, plane_zs):
            # (A) 高さの処理（トランジション対応）
            if in_transition:
                # トランジション中：開始位置から平面位置へ補間
                ag.z = ag.ceiling_start_z + (target_z_plane - ag.ceiling_start_z) * eased_progress
            else:
                # 通常時：平面に固定
                ag.z = target_z_plane

            # (B) 観客検出（人検出優先）
            closest = None
            md = detect_radius
            for p in audiences:
                d = math.hypot(p.x - ag.x, p.y - ag.y)
                if d < md:
                    md = d
                    closest = p
            
            # (C) ターゲット yaw/pitch 計算
            if closest:
                # 観客がいる場合：観客を向く
                dx, dy = closest.x - ag.x, closest.y - ag.y
                dz = closest.height - ag.z
                target_yaw = degrees(atan2(dy, dx))
                target_pitch = degrees(atan2(dz, hypot(dx, dy)))
                target_pitch = max(-60, min(60, target_pitch))  # クランプ
                
                # 自律モードチェック
                yaw_diff = abs(((target_yaw - ag.yaw + 540) % 360) - 180)
                if md < 1.5 and yaw_diff < 30:  # 1.5m以内かつ±30度以内
                    ag.autonomous_mode = True
                else:
                    ag.autonomous_mode = False
            else:
                # 観客がいない場合：斜面を登る方向を向く
                ag.autonomous_mode = False
                
                g_dot_n = -normal.z
                uphill = (g_dot_n*normal - vector(0,0,-1)).norm()
                target_yaw = degrees(atan2(uphill.y, uphill.x))
                target_pitch = degrees(asin(uphill.z))

            # (D) 向きの更新（トランジション対応）
            if in_transition:
                # トランジション中は徐々に速度を上げる
                k = min(1.0, ease_speed * dt * eased_progress)
                
                # Pitchは0に向かって補間
                ag.pitch = ag.ceiling_start_pitch + (target_pitch - ag.ceiling_start_pitch) * eased_progress
            else:
                # 通常のイージング
                k = min(1.0, ease_speed * dt)
                
                # Pitchのイージング
                dpitch = target_pitch - ag.pitch
                ag.pitch += dpitch * k
            
            # Yawのイージング（トランジション中も通常も同じ処理）
            dyaw = ((target_yaw - ag.yaw + 540) % 360) - 180
            ag.yaw += dyaw * k
            
            # actual_pitchを更新（自律モード用の表示値）
            ag.actual_pitch = ag.pitch

            # (E) 自律モードのビジュアル更新
            # トランジション中は自律モードの表示を抑制
            if in_transition:
                ag.autonomous_mode = False
            ag.update_autonomous_indicator()

            # (F) ジオメトリ更新
            ax = ag.compute_axis() * agent_length
            ctr = vector(ag.x, ag.y, ag.z)
            
            ag.body.pos = ctr - ax/2
            ag.body.axis = ax
            ag.cable.pos = ctr
            ag.cable.axis = vector(0,0, maxZ - ag.z)
            
            # LEDを筒と一緒に動かす
            u = ax.norm()
            for ld3, ld2, off in ag.leds:
                ld3.pos = ctr + u*(agent_length*off)
                ld2.pos = vector(ag.x + u.x*(agent_length*off),
                                ag.y + u.y*(agent_length*off),
                                0)
            
            # (H) 色更新（トランジション対応）
            global_hue = (sim_time * color_speed) % 1.0
            height_ratio = (ag.z - minZp) / (maxZp - minZp) if maxZp != minZp else 0.5
            
            # 特定IDのOSC送信
            if ag.node_id == 1:        
                osc_client_max.send_message('/id1/z', ag.z)
            if ag.node_id == 46:
                osc_client_max.send_message('/id46/z', ag.z)
            
            # 目標色の計算
            hue = (global_hue + height_ratio * 0.125) % 1.0
            brightness = 1.0 - 0.5 * height_ratio
            r, g, b = colorsys.hsv_to_rgb(hue, 1.0, brightness)
            osc_client_max.send_message('/hue', hue)
            
            # 自律モード時は色を少し変える（トランジション完了後のみ）
            if ag.autonomous_mode and not in_transition:
                r = min(1.0, r * 0.8 + 0.2)
            
            target_color = vector(r, g, b)
            
            # トランジション中の色補間
            if in_transition:
                # 開始色から目標色へ補間
                ag.current_color = vector(
                    ag.ceiling_start_color.x + (target_color.x - ag.ceiling_start_color.x) * eased_progress,
                    ag.ceiling_start_color.y + (target_color.y - ag.ceiling_start_color.y) * eased_progress,
                    ag.ceiling_start_color.z + (target_color.z - ag.ceiling_start_color.z) * eased_progress
                )
            else:
                ag.current_color = target_color
            
            # 色を適用
            ag.body.color = ag.current_color
            for ld3, ld2, _ in ag.leds:
                ld3.color = ld2.color = ag.current_color
            
            # (I) MQTT送信
            send_queue.put(ag)


    # ─── 天上天下モード ────────────────────────────────
    elif mode_menu.selected == "天上天下モード":
        # ========================================================
        # 天上天下モードの初期化とトランジション
        # ========================================================
        
        # モード切り替え検出
        if not hasattr(mode_menu, 'tenge_initialized') or not mode_menu.tenge_initialized:
            print(f"[天上天下モード] 初期化開始")
            mode_menu.tenge_initialized = True
            mode_menu.tenge_transition_start = sim_time
            mode_menu.tenge_transition_duration = 2.0  # 2秒のトランジション
            
            # 初期パラメータ設定
            mode_menu.tenge_amplitude = 0.35  # 初期振幅
            mode_menu.tenge_speed = 1.0  # ラジアン/秒
            
            # モード切り替え検出
            if not hasattr(mode_menu, 'last_crossing_phase'):
                mode_menu.last_crossing_phase = 0.0
            
            # 各エージェントの開始状態を保存
            for ag in agents:
                ag.tenge_start_z = ag.z
                ag.tenge_start_yaw = ag.yaw
                ag.tenge_start_pitch = ag.pitch
                ag.tenge_start_color = vector(ag.current_color.x, ag.current_color.y, ag.current_color.z)
                
                # 位相の初期化
                ag.tenge_phase = 0.0
                
                # グループをリセット
                ag.group = None
                
                print(f"  Agent {ag.node_id}: z={ag.tenge_start_z:.2f}→2.35(中間), "
                      f"yaw={ag.tenge_start_yaw:.1f}, pitch={ag.tenge_start_pitch:.1f}")
        
        # 他のモードの初期化フラグをリセット
        if hasattr(mode_menu, 'global_prev_mode') and mode_menu.global_prev_mode != "天上天下モード":
            mode_menu.fish_mode_initialized = False
            mode_menu.shimmer_initialized = False
            mode_menu.ceiling_mode_initialized = False
        
        # トランジション進行度を計算
        transition_elapsed = sim_time - getattr(mode_menu, 'tenge_transition_start', sim_time)
        transition_progress = min(1.0, transition_elapsed / getattr(mode_menu, 'tenge_transition_duration', 2.0))
        
        # イージング関数（easeInOutCubic）
        if transition_progress < 0.5:
            eased_progress = 4 * transition_progress * transition_progress * transition_progress
        else:
            eased_progress = 1 - pow(-2 * transition_progress + 2, 3) / 2
        
        # トランジション中かどうか
        in_transition = transition_progress < 1.0
        
        # ─────────────────────────────────────────────
        # パラメータ設定
        # ─────────────────────────────────────────────
        min_height = 2.0
        max_height = 2.7
        center_z = 2.35  # 中間点（すれ違いポイント）

        # ─────────────────────────────────────────────
        # 1) 位相を進める（トランジション完了後のみ）
        # ─────────────────────────────────────────────
        if not in_transition:
            for ag in agents:
                ag.tenge_phase += mode_menu.tenge_speed * dt
        else:
            # トランジション中は位相を0に保つ
            for ag in agents:
                ag.tenge_phase = 0.0
        
        # ─────────────────────────────────────────────
        # 2) 高さを計算
        # ─────────────────────────────────────────────
        # 振幅の段階的増加（トランジション後3秒かけて）
        amplitude_ramp_duration = 3.0
        time_since_transition = sim_time - getattr(mode_menu, 'tenge_transition_start', sim_time) - getattr(mode_menu, 'tenge_transition_duration', 2.0)
        
        if in_transition:
            # トランジション中：全員を中間高さに集める
            target_z_a = center_z
            target_z_b = center_z
            current_amplitude = 0.0
        elif time_since_transition < amplitude_ramp_duration:
            # トランジション直後：振幅を徐々に増やす
            ramp_progress = time_since_transition / amplitude_ramp_duration
            current_amplitude = mode_menu.tenge_amplitude * ramp_progress
            
            phase_a = agents[current_groupA_idx].tenge_phase
            target_z_a = center_z + current_amplitude * math.cos(phase_a)
            target_z_b = center_z - current_amplitude * math.cos(phase_a)
        else:
            # 通常時：フル振幅
            phase_a = agents[current_groupA_idx].tenge_phase
            target_z_a = center_z + mode_menu.tenge_amplitude * math.cos(phase_a)
            target_z_b = center_z - mode_menu.tenge_amplitude * math.cos(phase_a)
        
        # ─────────────────────────────────────────────
        # 3) すれ違い検出（トランジション完了後のみ）
        # ─────────────────────────────────────────────
        crossing = False
        if not in_transition:
            phase_a = agents[current_groupA_idx].tenge_phase
            phase_mod = phase_a % (2 * math.pi)
            
            # π/2付近または3π/2付近
            if (1.4 < phase_mod < 1.7) or (4.6 < phase_mod < 4.8):
                if not hasattr(mode_menu, 'last_crossing_phase') or abs(phase_a - mode_menu.last_crossing_phase) > 1.0:
                    crossing = True
                    mode_menu.last_crossing_phase = phase_a
            
            if crossing:
                # 新しいGroup Aを選択
                choices = list(range(len(agents)))
                choices.remove(current_groupA_idx)
                current_groupA_idx = random.choice(choices)
                # osc_client_max.send_message('/trig', 0)
                selected_node_id = agents[current_groupA_idx].node_id
                osc_client_max.send_message('/trig', int(selected_node_id))
                # トランジション中はすぐに反映                
                # 新しいパラメータ
                mode_menu.tenge_amplitude = random.uniform(0.1, 0.35)
                mode_menu.tenge_speed = random.uniform(0.8, 1.8)
                
                print(f"すれ違い! 新Group A: {current_groupA_idx}")

        # ─────────────────────────────────────────────
        # 4) 各エージェントに高さを設定
        # ─────────────────────────────────────────────
        for idx, ag in enumerate(agents):
            # グループ割り当て
            if idx == current_groupA_idx:
                new_group = "A"
                target_color = vector(1.0, 0.3, 0.3)  # 赤系
            else:
                new_group = "B"
                target_color = vector(0.25, 0.6, 1.0)  # 青系
            
            # 高さの設定
            if in_transition:
                # トランジション中：開始位置から中間位置へ（全員同じ高さ）
                ag.z = ag.tenge_start_z + (center_z - ag.tenge_start_z) * eased_progress
            else:
                # 通常時：グループに応じた高さ
                if new_group == "A":
                    ag.z = target_z_a
                else:
                    ag.z = target_z_b
            
            # グループ変更の処理
            if new_group != getattr(ag, "group", None):
                ag.prev_color = getattr(ag, "current_color", vector(1, 1, 1))
                if in_transition:
                    # トランジション中は徐々に色を変える
                    ag.target_tenge_color = target_color
                else:
                    ag.current_color = vector(1.0, 0.3, 0.3) if new_group == "A" else vector(0.25, 0.6, 1.0)
                ag.group = new_group

        # ─────────────────────────────────────────────
        # 5) 向きとジオメトリ
        # ─────────────────────────────────────────────
        
        # 向きとジオメトリ
        for ag in agents:
            # A) Group Aは最も近い観客を探して自律モード
            if ag.group == "A":
                closest = None
                min_dist = float('inf')
                for p in audiences:
                    d = math.hypot(p.x - ag.x, p.y - ag.y)
                    if d < min_dist:
                        min_dist = d
                        closest = p
                
                if closest:
                    dx, dy = closest.x - ag.x, closest.y - ag.y
                    dz = closest.height - ag.z
                    tgt_yaw = math.degrees(math.atan2(dy, dx))
                    tgt_pitch = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
                    tgt_pitch = max(-60, min(60, tgt_pitch))
                    
                    # Group Aは常に自律モード（トランジション完了後）
                    ag.autonomous_mode = True if not in_transition else False
                else:
                    ag.autonomous_mode = False
                    tgt_yaw, tgt_pitch = ag.yaw, ag.pitch
            
            # B) Group Bはリーダーを向く or 近くの観客を優先
            else:  # ag.group == "B"
                # まずリーダー（Group A）の方向を計算
                lead = agents[current_groupA_idx]
                dx, dy = lead.x - ag.x, lead.y - ag.y
                dz = lead.z - ag.z
                base_yaw = math.degrees(math.atan2(dy, dx))
                base_pitch = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
                
                # 近くに観客がいるかチェック
                closest, md = None, detect_radius
                for p in audiences:
                    d = math.hypot(p.x - ag.x, p.y - ag.y)
                    if d < md:
                        md, closest = d, p
                
                if closest:
                    dx2, dy2 = closest.x - ag.x, closest.y - ag.y
                    dz2 = closest.height - ag.z
                    tgt_yaw = math.degrees(math.atan2(dy2, dx2))
                    tgt_pitch = math.degrees(math.atan2(dz2, math.hypot(dx2, dy2)))
                    tgt_pitch = max(-60, min(60, tgt_pitch))
                    
                    # 自律モードチェック（トランジション完了後）
                    if not in_transition:
                        yaw_diff = abs(((tgt_yaw - ag.yaw + 540) % 360) - 180)
                        if md < 1.5 and yaw_diff < 30:
                            ag.autonomous_mode = True
                        else:
                            ag.autonomous_mode = False
                    else:
                        ag.autonomous_mode = False
                else:
                    ag.autonomous_mode = False
                    tgt_yaw, tgt_pitch = base_yaw, base_pitch

            # C) 向きの更新（トランジション対応）
            if in_transition:
                # トランジション中は徐々に動きを開始
                k = min(1.0, ease_speed * dt * eased_progress)
                # Pitchは0に向かって補間
                ag.pitch = ag.tenge_start_pitch + (0 - ag.tenge_start_pitch) * eased_progress
            else:
                # 通常のイージング
                k = min(1.0, ease_speed * dt)
                dpitch = tgt_pitch - ag.pitch
                ag.pitch += dpitch * k
            
            # Yawのイージング
            dyaw = ((tgt_yaw - ag.yaw + 540) % 360) - 180
            ag.yaw += dyaw * k
            
            # actual_pitchを更新
            ag.actual_pitch = ag.pitch

            # D) 自律モードのビジュアル更新
            ag.update_autonomous_indicator()

            # E) ジオメトリ
            axis = ag.compute_axis() * agent_length
            ctr  = vector(ag.x, ag.y, ag.z)
            ag.body.pos  = ctr - axis/2
            ag.body.axis = axis
            ag.cable.pos = ctr
            ag.cable.axis = vector(0, 0, maxZ - ag.z)
            
            # LEDを筒と一緒に動かす
            u = axis.norm()
            for ld3, ld2, offset in ag.leds:
                ld3.pos = ctr + u * (agent_length * offset)
                ld2.pos = vector(
                    ag.x + u.x * (agent_length * offset),
                    ag.y + u.y * (agent_length * offset),
                    0
                )
        
        # ─────────────────────────────────────────────
        # 6) 色の処理（トランジション対応）
        # ─────────────────────────────────────────────
        send_group_stats(agents, minZ, maxZ)
        
        for ag in agents:
            if in_transition:
                # トランジション中：開始色から目標色へ補間
                if hasattr(ag, 'target_tenge_color'):
                    ag.current_color = vector(
                        ag.tenge_start_color.x + (ag.target_tenge_color.x - ag.tenge_start_color.x) * eased_progress,
                        ag.tenge_start_color.y + (ag.target_tenge_color.y - ag.tenge_start_color.y) * eased_progress,
                        ag.tenge_start_color.z + (ag.target_tenge_color.z - ag.tenge_start_color.z) * eased_progress
                    )
                
                # 色を直接適用
                ag.body.color = ag.current_color
                for ld3, ld2, _ in ag.leds:
                    ld3.color = ld2.color = ag.current_color
            else:
                # 通常時：既存の色フェード処理
                ag.set_leds_tenge(sim_time, dt)

        # 描画 & MQTT 送信
        for ag in agents:
            ag.display()
            send_queue.put(ag)  


    elif mode_menu.selected == "魚群モード":
            # ========================================================
            # 魚群モードの初期化とトランジション
            # ========================================================
            
            # グローバルなprev_modeの確認（デバッグ用）
            current_prev_mode = getattr(mode_menu, 'global_prev_mode', None)
            
            # モード切り替え検出（より確実な方法）
            if not hasattr(mode_menu, 'fish_mode_initialized') or not mode_menu.fish_mode_initialized:
                print(f"[魚群モード] 初期化開始 (前のモード: {current_prev_mode})")
                mode_menu.fish_mode_initialized = True
                mode_menu.fish_transition_start = sim_time
                mode_menu.fish_transition_duration = 2.0  # 2秒のトランジション
                
                # 各エージェントの開始状態を保存
                for ag in agents:
                    # 現在の状態を確実に保存
                    ag.fish_start_z = ag.z
                    ag.fish_start_yaw = ag.yaw
                    ag.fish_start_pitch = ag.pitch
                    ag.fish_start_color = vector(ag.current_color.x, ag.current_color.y, ag.current_color.z)
                    
                    # ダウンライトの状態も保存
                    ag.fish_start_downlight = getattr(ag, 'downlight_brightness', 0.0)
                    
                    # きらめき用の初期化
                    ag.fish_flicker_phase = random.random() * 2 * math.pi  # 位相をランダム化
                    ag.fish_flicker_frequency = 0.1  # 初期は低周波（ゆっくり）
                    
                    # 目標値を設定
                    ag.fish_target_z = 2.0  # 基準高さ
                    ag.fish_target_pitch = 0.0  # 水平
                    ag.fish_target_color = vector(0.0, 0.7, 0.8)  # 青緑系の初期色
                    
                    # 波動の初期値を保存（ジャンプ防止用）
                    initial_wave = pnoise2(ag.i*waveScale + sim_time*0.3,
                                        ag.j*waveScale + sim_time*0.3)
                    ag.fish_initial_wave = initial_wave
                    
                    print(f"  Agent {ag.node_id}: z={ag.fish_start_z:.2f}→{ag.fish_target_z}, "
                        f"pitch={ag.fish_start_pitch:.1f}→{ag.fish_target_pitch}, wave={initial_wave:.3f}")
            
            # 他のモードから切り替わった時のフラグリセット
            if hasattr(mode_menu, 'global_prev_mode') and mode_menu.global_prev_mode != "魚群モード":
                # 他のモードの初期化フラグをリセット
                mode_menu.shimmer_initialized = False
                mode_menu.tenge_initialized = False
                mode_menu.ceiling_initialized = False
                # 必要に応じて他のモードのフラグも追加
            
            # トランジション進行度を計算
            transition_elapsed = sim_time - getattr(mode_menu, 'fish_transition_start', sim_time)
            transition_progress = min(1.0, transition_elapsed / getattr(mode_menu, 'fish_transition_duration', 2.0))
            
            # イージング関数（easeInOutCubic）
            if transition_progress < 0.5:
                eased_progress = 4 * transition_progress * transition_progress * transition_progress
            else:
                eased_progress = 1 - pow(-2 * transition_progress + 2, 3) / 2
            
            # トランジション中かどうか
            in_transition = transition_progress < 1.0
            
            # ========================================================
            # 流れベクトルの計算（トランジション中も必要）
            # ========================================================
            flow_vecs = []
            for ag in agents:
                fx = pnoise2(ag.i*noiseScale + noise_time,
                            ag.j*noiseScale + noise_time)
                fy = pnoise2(ag.i*noiseScale + noise_time + 50,
                            ag.j*noiseScale + noise_time + 50)
                v = vector(-fx, -fy, 0).norm()  # 逆向き
                flow_vecs.append(v)

            # ========================================================
            # 各エージェントの更新
            # ========================================================
            for idx, ag in enumerate(agents):
                # Boids計算（トランジション中も行う）
                sep = coh = align = vector(0,0,0)
                cnt = 0
                for jdx in ag.neighbors:
                    if 0 <= jdx < len(agents):
                        nb = agents[jdx]
                        dxy = vector(nb.x - ag.x, nb.y - ag.y, 0)
                        dist = dxy.mag
                        if dist < fish_sep_dist:
                            sep -= dxy / (dist**2 + 1e-3)
                        coh += dxy
                        align += flow_vecs[jdx]
                        cnt += 1

                if cnt:
                    coh /= cnt
                    align /= cnt

                steer = (sep*1.8 +
                        coh * fish_coh_factor +
                        align * fish_align_factor +
                        flow_vecs[idx])

                # 目標方向
                steer = steer.norm()
                tgt_yaw = math.degrees(math.atan2(steer.y, steer.x))
                tgt_pitch = 0  # 水平泳ぎ

                # 尾びれオシレータ
                phase = ag.idx * 0.7
                osc = fin_osc_amp_deg * math.sin(2*math.pi*fin_osc_freq_hz*sim_time + phase)
                
                # トランジション中は振幅を抑える
                if in_transition:
                    osc *= eased_progress
                
                tgt_yaw += osc

                # ========================================================
                # 位置・姿勢の更新（トランジション処理込み）
                # ========================================================
                
                if in_transition:
                    # Z座標のトランジション（確実に開始値から目標値へ）
                    ag.z = ag.fish_start_z + (ag.fish_target_z - ag.fish_start_z) * eased_progress
                    
                    # Pitchのトランジション（確実に開始値から目標値へ）
                    ag.pitch = ag.fish_start_pitch + (ag.fish_target_pitch - ag.fish_start_pitch) * eased_progress
                    
                    # Yawのトランジション（目標方向に向かって徐々に調整）
                    dyaw = ((tgt_yaw - ag.yaw + 540) % 360) - 180
                    # トランジション中は回転速度を調整
                    k = min(1.0, ease_speed * dt * eased_progress)
                    ag.yaw += dyaw * k
                    
                    # 色のトランジション（確実に開始色から目標色へ）
                    base_color = vector(
                        ag.fish_start_color.x + (ag.fish_target_color.x - ag.fish_start_color.x) * eased_progress,
                        ag.fish_start_color.y + (ag.fish_target_color.y - ag.fish_start_color.y) * eased_progress,
                        ag.fish_start_color.z + (ag.fish_target_color.z - ag.fish_start_color.z) * eased_progress
                    )
                else:
                    # 通常のイージング
                    k = min(1.0, ease_speed * dt)
                    dyaw = ((tgt_yaw - ag.yaw + 540) % 360) - 180
                    ag.yaw += dyaw * k
                    ag.pitch += (tgt_pitch - ag.pitch) * k
                    
                    # 基本色
                    base_color = ag.fish_target_color

                # ========================================================
                # Z座標の波動（人を避ける動作）
                # ========================================================
                z_wave = pnoise2(ag.i*waveScale + sim_time*0.3,
                                ag.j*waveScale + sim_time*0.3)
                
                # 人との距離に基づいて振幅と中心高さを計算
                min_amplitude = 0.25
                max_amplitude = 0.4
                avoid_radius = 1.5
                
                base_height_with_person = 2.5
                base_height_without_person = 2.0
                
                # 各観客からの影響を計算
                amplitude_factor = 1.0
                height_factor = 1.0
                
                for person in audiences:
                    dist = math.hypot(person.x - ag.x, person.y - ag.y)
                    
                    if dist < avoid_radius:
                        t = dist / avoid_radius
                        smooth_t = t * t * (3 - 2 * t)
                        person_factor = smooth_t
                        amplitude_factor = min(amplitude_factor, person_factor)
                        height_factor = min(height_factor, person_factor)
                
                # 最終的な振幅を計算
                wave_amplitude = min_amplitude + (max_amplitude - min_amplitude) * amplitude_factor
                
                # 基準高さを計算
                current_base_height = base_height_without_person + (base_height_with_person - base_height_without_person) * (1 - height_factor)
                
                # 目標のZ座標（波動を含む最終位置）
                target_z_with_wave = current_base_height + z_wave * wave_amplitude
                
                # 波動効果の段階的適用（トランジション後も3秒かけて）
                wave_ramp_duration = 3.0
                time_since_transition = sim_time - getattr(mode_menu, 'fish_transition_start', sim_time) - getattr(mode_menu, 'fish_transition_duration', 2.0)
                
                if in_transition:
                    # トランジション中：開始位置から基準高さへ
                    ag.z = ag.fish_start_z + (ag.fish_target_z - ag.fish_start_z) * eased_progress
                elif time_since_transition < wave_ramp_duration:
                    # トランジション直後：基準高さから波動込みの高さへ徐々に移行
                    wave_progress = time_since_transition / wave_ramp_duration
                    # easeInOutでスムーズに
                    if wave_progress < 0.5:
                        wave_eased = 2 * wave_progress * wave_progress
                    else:
                        wave_eased = 1 - pow(-2 * wave_progress + 2, 2) / 2
                    
                    # 初期波動値から現在の波動値へ補間（ジャンプ防止）
                    initial_wave = getattr(ag, 'fish_initial_wave', 0.0)
                    interpolated_wave = initial_wave + (z_wave - initial_wave) * wave_eased
                    
                    # 波動効果も徐々に適用
                    ag.z = ag.fish_target_z + interpolated_wave * wave_amplitude * wave_eased
                else:
                    # 通常時：完全な波動効果
                    ag.z = target_z_with_wave
                
                # 高さの制限
                ag.z = max(minZ, min(maxZ, ag.z))

                # ジオメトリ更新
                update_geometry(ag)

                # ========================================================
                # LED色のきらめき（徐々に周波数を上げる）
                # ========================================================
                
                # きらめきの周波数を徐々に上げる
                if in_transition:
                    # 0.1Hz（ゆっくり）から10Hz（速い）へ
                    target_frequency = 10.0
                    ag.fish_flicker_frequency = 0.1 + (target_frequency - 0.1) * eased_progress
                    
                    # 色相の変化幅も徐々に増やす
                    hue_variation = 0.02 + 0.13 * eased_progress  # 0.02→0.15
                    brightness_variation = 0.05 + 0.25 * eased_progress  # 0.05→0.3
                else:
                    ag.fish_flicker_frequency = 10.0
                    hue_variation = 0.15
                    brightness_variation = 0.3
                
                # きらめきの計算（各エージェントの位相を使用）
                hue = (0.55 + steer.x * hue_variation) % 1.0
                flick = 0.7 + brightness_variation * math.sin(ag.fish_flicker_frequency * sim_time + ag.fish_flicker_phase)
                
                # 明度を制限（暗くなりすぎない）
                flick = max(0.5, min(1.0, flick))
                
                r, g, b = colorsys.hsv_to_rgb(hue, 0.8, flick)
                flicker_color = vector(r, g, b)
                
                # トランジション中は基本色ときらめき色をブレンド
                if in_transition:
                    # きらめきの影響を徐々に強くする
                    blend_factor = eased_progress * 0.7  # 最大70%までブレンド
                    final_color = base_color * (1 - blend_factor) + flicker_color * blend_factor
                else:
                    final_color = flicker_color
                
                ag.current_color = final_color
                
                # ========================================================
                # ダウンライトのランダムきらめき
                # ========================================================
                if not hasattr(ag, 'downlight_flicker_time'):
                    ag.downlight_flicker_time = -999.0
                    ag.downlight_base = 0.0
                
                flicker_duration = 0.5
                
                # トランジション中はきらめきを抑制
                trigger_probability = 0.017 if not in_transition else 0.017 * eased_progress
                
                if random.random() < trigger_probability:
                    ag.downlight_flicker_time = sim_time
                
                flicker_age = sim_time - ag.downlight_flicker_time
                if 0 <= flicker_age <= flicker_duration:
                    t = flicker_age / flicker_duration
                    intensity = math.sin(t * math.pi)
                    max_intensity = 0.3 if not in_transition else 0.3 * eased_progress
                    ag.downlight_brightness = intensity * max_intensity
                else:
                    # トランジション中は徐々に消灯
                    if in_transition:
                        ag.downlight_brightness = ag.fish_start_downlight * (1 - eased_progress)
                    else:
                        ag.downlight_brightness = 0.0
                
                # ダウンライト表示更新
                update_downlight_display(ag)
            
            # ========================================================
            # 描画・LED・MQTT出力
            # ========================================================
            for ag in agents:
                ag.display()
                ag.body.color = ag.current_color
                for ld3, ld2, _ in ag.leds:
                    ld3.color = ld2.color = ag.current_color
                send_queue.put(ag)

    # ------------------------------------------------------------
    elif mode_menu.selected == "蜂シマーモード":
    # ------------------------------------------------------------
        # --- 1) 各種パラメータ（先に定義） ---
        shim_base_freq_hz = 1.0
        shim_base_amp_m   = 0.005
        shim_wave_speed   = 1.2
        shim_trigger_mean = 3.0
        shim_front_tol    = 0.35

        yaw_peak_deg  = 90.0
        yaw_rise_s    = 0.5
        yaw_decay_tau = 0.8

        drop_m_norm   = 0.15
        drop_down_s_n = 0.2
        drop_up_s_n   = 1.0
        drop_total_n  = drop_down_s_n + drop_up_s_n

        rare_prob     = 0.02 # was 0.002
        rare_factor   = 6
        drop_m_rare   = drop_m_norm * rare_factor
        drop_down_s_r = drop_down_s_n * rare_factor
        drop_up_s_r   = drop_up_s_n   * rare_factor
        drop_total_r  = drop_down_s_r + drop_up_s_r
        
        # 逃げ判定を行う時間窓（レアドロップ開始から何秒まで？）
        escape_window_s = 10.0      # ← レアドロップ全期間をカバーするように延長
        escape_radius = 2.0         # 逃げる判定半径
        
        color_rise_s   = 0.5
        flash_rise_s   = 0.2
        flash_decay_s  = 1.5
        white_mix_peak = 0.5
        bright_peak    = 0.5
        
        # ★改善：モード開始時の初期化チェック
        if not hasattr(mode_menu, 'shimmer_initialized') or not mode_menu.shimmer_initialized:
            print(f"[シマーモード] 初期化開始")
            mode_menu.shimmer_initialized = True
            mode_menu.shimmer_transition_start = sim_time
            mode_menu.shimmer_transition_duration = 2.0  # 2秒のトランジション（他モードと統一）
            
            # 各エージェントの現在位置を記憶
            for ag in agents:
                ag.shimmer_start_z = ag.z
                ag.shimmer_start_yaw = ag.yaw
                ag.shimmer_start_pitch = ag.pitch
                ag.shimmer_start_color = vector(ag.current_color.x, ag.current_color.y, ag.current_color.z)
                
                # デフォルトの正位置を設定
                ag.shimmer_default_z = 2.5  # シマーモードの基準高さ
                ag.shimmer_default_pitch = 0.0
                ag.shimmer_default_color = vector(0.5, 0.5, 0.0)  # 黄色系
                
                # シマーモード用の初期化
                ag.yaw0 = ag.yaw
                ag.z0 = ag.shimmer_default_z  # 正位置をベースに
                ag.face_dir = ag.yaw0
                ag.actual_face_dir = ag.yaw0
                ag.last_had_audience = False  # 観客検出状態を初期化
                ag.kick_time = ag.drop_time = ag.color_t0 = -999.0
                ag.kick_peak = 0.0
                ag.drop_mode = "norm"
                ag.current_color = ag.shimmer_default_color
                ag.color_from = ag.color_to = ag.current_color
                ag.drop_down_s = drop_down_s_n
                ag.drop_up_s = drop_up_s_n
                ag.drop_total = drop_total_n
                ag.drop_m = drop_m_norm
                
                print(f"  Agent {ag.node_id}: z={ag.shimmer_start_z:.2f}→{ag.shimmer_default_z}, "
                      f"yaw={ag.shimmer_start_yaw:.1f}, pitch={ag.shimmer_start_pitch:.1f}")
        
        # 他のモードの初期化フラグをリセット
        if hasattr(mode_menu, 'global_prev_mode') and mode_menu.global_prev_mode != "蜂シマーモード":
            mode_menu.fish_mode_initialized = False
            mode_menu.tenge_mode_initialized = False
            mode_menu.ceiling_mode_initialized = False
            # 波イベントもクリア
            wave_events.clear()
        
        # ★トランジション進行度を計算
        transition_elapsed = sim_time - getattr(mode_menu, 'shimmer_transition_start', sim_time)
        transition_progress = min(1.0, transition_elapsed / getattr(mode_menu, 'shimmer_transition_duration', 2.0))
        
        # イージング関数（easeInOutCubic）
        if transition_progress < 0.5:
            eased_progress = 4 * transition_progress * transition_progress * transition_progress
        else:
            eased_progress = 1 - pow(-2 * transition_progress + 2, 3) / 2
        
        # トランジション中かどうか
        in_transition = transition_progress < 1.0
        
        # ★トランジション中は波を発生させない
        if in_transition:
            next_shim_time = sim_time + mode_menu.shimmer_transition_duration - transition_elapsed + 0.5
        
        # --- 2) 波発生処理（トランジション完了後のみ） ---
        if not in_transition and sim_time >= next_shim_time:
            # 波の色・震源ノード・進行方向を決定
            hue = random.random()
            r, g, b = colorsys.hsv_to_rgb(hue, 1, 1)
            wave_col = vector(r, g, b)

            ori = random.choice(agents)
            wdir = math.degrees(math.atan2(centerY - ori.y, centerX - ori.x))

            # 各ノードについて"トリガー時刻"を計算してリストに入れる
            events = []
            for ag in agents:
                # 震源→エージェント間の距離
                dist = math.hypot(ag.x - ori.x, ag.y - ori.y)

                # 正確な到達時刻
                t_trigger = sim_time + dist / shim_wave_speed

                # 同一距離上のノードが完全同時に鳴らないように小さな乱数を足す（既存）
                jitter = random.uniform(0.0, 0.05)
                t_trigger += jitter
                
                # 新規：順序が入れ替わるほどの大きなランダム遅延
                if dist < 2.0:  # 震源近く（2m以内）
                    random_delay = random.uniform(0, 0.5)
                elif dist < 5.0:  # 中距離（2-5m）
                    random_delay = random.uniform(0, 1.0)
                else:  # 遠距離（5m以上）
                    random_delay = random.uniform(0, 1.5)
                
                t_trigger += random_delay

                # 各イベントには「t_trigger / 該当エージェント / wave_color / wave_dir」をまとめておく
                events.append((t_trigger, ag, wave_col, wdir))

            # 時刻順にソートしてから wave_events にプッシュ
            events.sort(key=lambda x: x[0])
            wave_events.append({
                'origin': (ori.x, ori.y),
                'origin_id': ori.node_id,  # 震源ノードのIDを追加
                't0': sim_time,
                'events': events
            })

            # 次の波を発生させるタイミングを決定
            next_shim_time = sim_time + random.expovariate(1/shim_trigger_mean)

        # --- 3) 各ノードの共通更新処理 ---
        for ag in agents:
            # ★トランジション中の処理
            if in_transition:
                # 位置の補間
                ag.z = ag.shimmer_start_z + (ag.shimmer_default_z - ag.shimmer_start_z) * eased_progress
                ag.pitch = ag.shimmer_start_pitch + (ag.shimmer_default_pitch - ag.shimmer_start_pitch) * eased_progress
                # yawは維持または少しずつ調整
                
                # 色の補間
                ag.current_color = vector(
                    ag.shimmer_start_color.x + (ag.shimmer_default_color.x - ag.shimmer_start_color.x) * eased_progress,
                    ag.shimmer_start_color.y + (ag.shimmer_default_color.y - ag.shimmer_start_color.y) * eased_progress,
                    ag.shimmer_start_color.z + (ag.shimmer_default_color.z - ag.shimmer_start_color.z) * eased_progress
                )
                
                # z0を現在位置に設定（トランジション完了時にスムーズに繋がるように）
                ag.z0 = ag.z
                
                # ベースの振動を徐々に開始
                z_off = math.sin(2*math.pi*shim_base_freq_hz*sim_time + ag.idx*0.7) * shim_base_amp_m * eased_progress
                ag.z = clamp(ag.z + z_off, minZ, maxZ)
                
                # ジオメトリ更新
                update_geometry(ag)
                
                # 色を適用
                ag.body.color = ag.current_color
                for ld3, ld2, _ in ag.leds:
                    ld3.color = ld2.color = ag.current_color
                    
                # 自律モード表示を更新（トランジション中はオフ）
                ag.autonomous_mode = False
                ag.update_autonomous_indicator()
                
                continue  # トランジション中は通常処理をスキップ
            
            # ★以下、通常のシマーモード処理
            # z_off のベース（サイン波で上下振動）
            z_off = math.sin(2*math.pi*shim_base_freq_hz*sim_time + ag.idx*0.7) * shim_base_amp_m

            # (1) 初期化（既に初期化済みなのでスキップ可能）
            if not hasattr(ag, "yaw0"):
                ag.yaw0       = ag.yaw
                ag.z0         = ag.z
                ag.face_dir   = ag.yaw0
                ag.actual_face_dir = ag.yaw0
                ag.kick_time  = ag.drop_time = ag.color_t0 = -999.0
                ag.kick_peak  = 0.0
                ag.drop_mode  = "norm"
                ag.current_color = vector(0.5, 0.5, 0.0)
                ag.color_from = ag.color_to = ag.current_color

                # ここで通常ドロップのパラメータをセットしておく
                ag.drop_mode   = "norm"
                ag.drop_down_s = drop_down_s_n
                ag.drop_up_s   = drop_up_s_n
                ag.drop_total  = drop_total_n
                ag.drop_m      = drop_m_norm

            # (2) レアドロップ中は「波の判定」を受けないフラグを立てる
            if ag.drop_mode == "rare" and sim_time - ag.drop_time < drop_total_r:
                wave_hit_allowed = False
            else:
                wave_hit_allowed = True
                ag.drop_mode = "norm"  # レア期間を過ぎたら通常に戻す

            # ★(3) 観客検出と自律モード判定
            closest = None
            min_dist = float('inf')
            
            # レアドロップ中は距離制限なしで最も近い観客を探す
            if ag.drop_mode == "rare":
                for p in audiences:
                    d = math.hypot(p.x - ag.x, p.y - ag.y)
                    if d < min_dist:
                        min_dist = d
                        closest = p
            else:
                # 通常時は検出半径内の観客を探す
                for p in audiences:
                    d = math.hypot(p.x - ag.x, p.y - ag.y)
                    if d < detect_radius and d < min_dist:
                        min_dist = d
                        closest = p
            
            # 目標方向の計算
            if closest:
                dx, dy = closest.x - ag.x, closest.y - ag.y
                dz = closest.height - ag.z
                target_yaw = math.degrees(math.atan2(dy, dx))
                target_pitch = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
                target_pitch = max(-60, min(60, target_pitch))
                
                # 自律モードの判定
                yaw_diff = abs(((target_yaw - ag.actual_face_dir + 540) % 360) - 180)
                if ag.drop_mode == "rare":
                    # レアドロップ時は常に自律モード
                    ag.autonomous_mode = True
                elif min_dist < 1.5 and yaw_diff < 30:
                    # 通常時は1.5m以内かつ正面向きで自律モード
                    ag.autonomous_mode = True
                else:
                    ag.autonomous_mode = False
            else:
                # 観客がいない場合
                ag.autonomous_mode = False
                target_yaw = ag.actual_face_dir
                target_pitch = 0

            # ★(4) 回転処理（修正版）
            k_age = sim_time - ag.kick_time
            
            # 波のキックによる回転
            yaw_kick = (0 if k_age < 0 else
                        yaw_peak_deg * k_age / yaw_rise_s if k_age <= yaw_rise_s else
                        yaw_peak_deg * math.exp(-(k_age - yaw_rise_s) / yaw_decay_tau))
            
            # 観客がいる場合はイージングで向きを変える
            if closest:
                # 観客への方向にイージング
                dyaw = ((target_yaw - ag.yaw + 540) % 360) - 180
                ease_factor = 0.1
                if ag.drop_mode == "rare":
                    ease_factor = 0.15  # レアドロップ時は少し速く反応
                ag.yaw += dyaw * ease_factor
                ag.actual_face_dir = ag.yaw  # ★実際の向きを更新
                
                # Pitchもイージング
                dpitch = target_pitch - ag.pitch
                ag.pitch += dpitch * ease_factor
            else:
                # ★観客がいない場合の処理を改善
                # actual_face_dirに向かってイージングで戻る
                if hasattr(ag, 'last_had_audience') and ag.last_had_audience:
                    # 観客がいなくなった直後：現在のyawをactual_face_dirとして保存
                    ag.actual_face_dir = ag.yaw
                    ag.last_had_audience = False
                
                # 波のキックを適用した目標方向
                desired_yaw = ag.actual_face_dir + yaw_kick
                dyaw = ((desired_yaw - ag.yaw + 540) % 360) - 180
                ag.yaw += dyaw * 0.1  # イージングで滑らかに
                
                # actual_face_dirは波の影響を受けて徐々に更新
                if abs(yaw_kick) > 1:  # 波のキックが有効な時のみ更新
                    ag.actual_face_dir += yaw_kick * 0.02  # ゆっくり更新
                    
                # Pitchは0に向かってイージング
                ag.pitch += (0 - ag.pitch) * 0.1
            
            # 観客検出状態を記録
            ag.last_had_audience = (closest is not None)
            
            # actual_pitchを更新
            ag.actual_pitch = ag.pitch

            # ★(5) 自律モードのビジュアル更新
            ag.update_autonomous_indicator()

            # ── ドロップ処理（改良版：鬼さんこちら） ─────────────────────────────────────────────────
            d_age = sim_time - ag.drop_time
            
            # ★ レアドロップ時の特別な処理
            if ag.drop_mode == "rare":
                # 最新の距離を計算
                current_dist = float('inf')
                if closest:  # closestは上で計算済み
                    current_dist = min_dist
                
                # 逃げ判定と速度調整
                escape_active = False
                if current_dist < escape_radius and d_age < drop_total_r:
                    escape_active = True
                    # 下降中に人が近づいた → 即座に上昇フェーズへ
                    if d_age < ag.drop_down_s:
                        ag.drop_time = sim_time - ag.drop_down_s
                        d_age = ag.drop_down_s
                        print(f"鬼さん逃げる! ID {ag.node_id}, 距離: {current_dist:.1f}m")
                
                # ドロップ計算
                if d_age < 0:
                    drop_off = 0
                elif d_age <= ag.drop_down_s:
                    # 下降フェーズ
                    drop_off = -ag.drop_m * (d_age / ag.drop_down_s)
                elif d_age <= ag.drop_total:
                    # 上昇フェーズ
                    progress = (d_age - ag.drop_down_s) / ag.drop_up_s
                    
                    # 逃げモード時は速度を上げる
                    if escape_active:
                        # 2〜3倍速で上昇
                        speed_factor = 2.0 + (escape_radius - current_dist) / escape_radius
                        progress = min(1.0, progress * speed_factor)
                    
                    drop_off = -ag.drop_m * (1 - progress)
                else:
                    drop_off = 0
                    # レアドロップ終了
                    ag.drop_mode = "norm"
                    ag.autonomous_mode = False
            else:
                # 通常のドロップ処理
                if d_age < 0:
                    drop_off = 0
                elif d_age <= ag.drop_down_s:
                    drop_off = -ag.drop_m * (d_age / ag.drop_down_s)
                elif d_age <= ag.drop_total:
                    drop_off = -ag.drop_m * (1 - (d_age - ag.drop_down_s) / ag.drop_up_s)
                else:
                    drop_off = 0

            # ── 色のベース計算 ─────────────────────────────────────────────
            if sim_time - ag.color_t0 < color_rise_s:
                t = (sim_time - ag.color_t0) / color_rise_s
                base_col = ag.color_from * (1 - t) + ag.color_to * t
            else:
                base_col = ag.color_to

            # ── フラッシュ（光る）演出 ───────────────────────────────────────
            f_age = sim_time - ag.drop_time
            if 0 <= f_age <= flash_rise_s + flash_decay_s:
                if f_age <= flash_rise_s:
                    r = f_age / flash_rise_s
                else:
                    r = 1 - (f_age - flash_rise_s) / flash_decay_s
                mix = white_mix_peak * r
                inc = 1 + bright_peak * r
                col = (base_col * (1 - mix) + vector(1, 1, 1) * mix) * inc
                col = vector(min(1, col.x), min(1, col.y), min(1, col.z))
            else:
                col = base_col

            ag.current_color = col

            # ── 近接観客強調 (通常ドロップのみ) ─────────────────────────────────
            if ag.drop_mode == "norm":
                for p in audiences:
                    if math.hypot(p.x - ag.x, p.y - ag.y) < 1:
                        z_off *= 2
                        break

            # ── 各エージェントの最終的な z 座標 / 表示更新 ──────────────
            final_z = ag.z0 + z_off + drop_off
            
            # ★安全性チェック：急激な変化を防ぐ
            max_z_change = 0.5  # 1フレームでの最大変化量
            if hasattr(ag, 'prev_z'):
                z_change = final_z - ag.prev_z
                if abs(z_change) > max_z_change:
                    # 変化量を制限
                    final_z = ag.prev_z + max_z_change * (1 if z_change > 0 else -1)
                    print(f"[安全性] Agent {ag.node_id}: z変化を制限 {z_change:.2f} -> {final_z - ag.prev_z:.2f}")
            
            ag.z = clamp(final_z, minZ, maxZ)
            ag.prev_z = ag.z  # 次フレーム用に保存
            
            update_geometry(ag)
            for l3, l2, _ in ag.leds:
                l3.color = l2.color = ag.current_color

        # --- 4) wave_events をチェックして、『sim_time >= t_trigger』のイベントだけを順次発火する ---
        new_wave_events = []
        for wave in wave_events:
            remaining = []
            for (t_trigger, ag, wcol, wdir) in wave['events']:
                if sim_time >= t_trigger:
                    # ★ レアドロップ中かチェック
                    if ag.drop_mode == "rare" and sim_time - ag.drop_time < drop_total_r:
                        # レアドロップ中は波を無視
                        remaining.append((t_trigger, ag, wcol, wdir))
                        continue
                    
                    # (a) ── 波にぶつかった瞬間の処理 ───────────────────────────
                    
                    # レアドロップの判定を追加
                    if random.random() < rare_prob:  # 0.02 = 2%の確率
                        ag.drop_mode = "rare"
                        ag.escape_triggered = False  # リセット
                        ag.drop_m = drop_m_rare       # 通常の6倍の深さ
                        ag.drop_down_s = drop_down_s_r # 通常の6倍の時間
                        ag.drop_up_s = drop_up_s_r     # 通常の6倍の時間
                        ag.drop_total = drop_total_r   # 合計時間も6倍
                        rel = 8.0  # 音も長く
                    else:
                        ag.drop_mode = "norm"
                        ag.drop_m = drop_m_norm
                        ag.drop_down_s = drop_down_s_n
                        ag.drop_up_s = drop_up_s_n
                        ag.drop_total = drop_total_n
                        rel = random.uniform(1.0, 2.0)
                    
                    atk = random.uniform(0.001, 0.009)
                    fAtk = random.uniform(0.001, 0.02)
                    vibRate = random.uniform(0.1, 10)
                    vibDepth = random.uniform(0.01, 0.5)
                    amp = random.uniform(0.03, 0.08)

                    # 震源からの距離を計算
                    distance = math.hypot(ag.x - wave['origin'][0], ag.y - wave['origin'][1])
                    
                    # 震源ノードのIDを送信
                    osc_client_max.send_message(
                        "/shim",
                        [int(wave['origin_id']), atk, rel, fAtk, vibRate, vibDepth, amp, distance]
                    )

                    # z_off を強制的に大きくして跳ね上げ
                    ag.z = clamp(ag.z + shim_base_amp_m * 4, minZ, maxZ)
                    
                    # ★重要：基準高さは維持（元の高さに戻るように）
                    # ag.z0 = ag.z  # これを削除！z0は更新しない

                    # ノードのステートを初期化／更新
                    ag.kick_time = sim_time
                    ag.kick_peak = yaw_peak_deg
                    ag.drop_time = sim_time
                    ag.face_dir = wdir  # ★注意：face_dirは波の方向を保持（参考値）
                    # actual_face_dirは観客がいる時は更新しない
                    if not closest:
                        ag.actual_face_dir = wdir
                    ag.color_from = ag.current_color
                    ag.color_to = wcol * 0.5
                    ag.color_t0 = sim_time

                    # ────────────────────────────────────────────────────────────
                else:
                    remaining.append((t_trigger, ag, wcol, wdir))

            # まだ消費していないイベントが残っていれば続行
            if remaining:
                wave['events'] = remaining
                new_wave_events.append(wave)

        wave_events = new_wave_events

        # ─── 8) 描画・LED・MQTT 出力 ───────────────────────
        for ag in agents:
            ag.display()
            # ag.set_leds()
            # ─ OSC 出力 ─────────────────────────────────────────
            if 0 <= sim_time - ag.drop_time <= ag.drop_total:
                # ★ 明るさは RGB の最大値をそのまま 0.0-1.0 で
                brightness = max(ag.current_color.x,
                                ag.current_color.y,
                                ag.current_color.z)
            send_queue.put(ag)

    # メインループの既存のelif文の後に追加：
    elif mode_menu.selected == "マニュアルモード":
        # マニュアルモードの処理
        process_manual_commands()
        apply_manual_mode()
        
        # 表示更新
        for ag in agents:
            ag.display()
            update_downlight_display(ag)  # ← ダウンライト表示を更新
            send_queue.put(ag)


    # 音に対するインタラクティブモード
    apply_sound_reaction()     # ★追加★

    # 3Dカメラ軌道更新
    camX = centerX + radius * math.cos(angle)
    camY = centerY + radius * math.sin(angle)
    scene3d.camera.pos  = vector(camX, camY, cameraHeight)
    scene3d.camera.axis = vector(centerX-camX,
                                 centerY-camY,
                                 ((minZ+maxZ)/2)-cameraHeight)

    # --- after -------------------------------------------------
    if sensor_mode and sensor_people:
        if sim_time - last_aud_msg_time > PRESENCE_TIMEOUT:
            for p in sensor_people:
                p.body.visible = p.head.visible = p.dot2d.visible = False
            # 2) 実体をリストから外す  ←★追加★
            sensor_people.clear()
            audiences.clear()          # audiences は検出ループで使われる

    # 各モード処理の後、メインループの最後に
    mode_menu.global_prev_mode = mode_menu.selected

    # モード変更時のフラグリセット（on_mode_select関数内に追加）
    if mode_menu.selected != "回る天井":
        mode_menu.ceiling_mode_initialized = False

    if mode_menu.selected != "魚群モード":
        mode_menu.fish_mode_initialized = False
        mode_menu.fish_reset_start = 0.0
        mode_menu.fish_reset_duration = 1.0

    if mode_menu.selected != "蜂シマーモード":
        # シマーモードの初期化フラグをリセット
        mode_menu.shimmer_initialized = False
    
    if mode_menu.selected != "天上天下モード":
        mode_menu.tenge_initialized = False