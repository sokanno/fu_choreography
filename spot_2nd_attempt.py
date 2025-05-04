from vpython import *
scene.visible = False
scene.width = scene.height = 0

import math, random, struct, csv
from noise import pnoise2
import paho.mqtt.client as mqtt

import colorsys

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

# スポットライト制御パラメータ
spot_brightness   = 1.0   # 0〜1 の全体明るさ
spot_center_x     = 0.0   # フォーカス中心 X
spot_center_y     = 0.0   # フォーカス中心 Y
spot_spread       = 0.5   # 0→シャープ、1→フラット
spot_exponent_max = 10    # spread=0 のときの最大シャープ指数

# ========================================================
# 定数
# ========================================================
ROWS, COLS     = 7, 7
minZ, maxZ     = 0.0, 3.0
sepDist        = 0.4
agent_length   = 0.30
agent_radius   = 0.075/2
cable_radius   = 0.01
led_radius     = agent_radius

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
    width=600, height=400,
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

# ========================================================
# UI（下段にまとめる）
# ========================================================
ui = canvas(
    width=0, height=0,
    background=color.white,
    caption="",
    align='right'
)

def add_slider(cnv, label, mn, mx, val, fmt, setter,
               length=200, label_w=150, spacing=1):
    # ラベル部を inline-block にして下にマージンを入れる
    cnv.append_to_caption(
        f'<div style="display:inline-block; width:{label_w}px; '
        f'margin-bottom:{spacing}px;">{label}:</div>'
    )
    # 数値表示
    txt = wtext(text=f"{fmt.format(val)}  ", canvas=cnv)
    # スライダー本体
    def on_slide(s):
        setter(s.value)
        txt.text = f"{fmt.format(s.value)}  " 
    slider(min=mn, max=mx, value=val, length=length,
           bind=on_slide, canvas=cnv)
    # 行間をさらに空ける
    cnv.append_to_caption("<br>")


# ─── Behavior Control ─────────────────────────────────
ui.append_to_caption("<b>Behavior Control</b><br><br>")
add_slider(ui, "Separation",    0,   2,   separationFactor, "{:.2f}",
           lambda v: globals().update(separationFactor=v))
add_slider(ui, "Cohesion",      0,   2,   cohesionFactor,   "{:.2f}",
           lambda v: globals().update(cohesionFactor=v))
add_slider(ui, "Noise Scale",   .01, .5,  noiseScale,      "{:.2f}",
           lambda v: globals().update(noiseScale=v))
add_slider(ui, "Wave Scale",    .01, 1.0, waveScale,       "{:.2f}",
           lambda v: globals().update(waveScale=v))
add_slider(ui, "Wave Strength", 0,   1,   waveStrength,    "{:.2f}",
           lambda v: globals().update(waveStrength=v))
add_slider(ui, "Noise Speed",   0,   .1,  noiseSpeed,      "{:.3f}",
           lambda v: globals().update(noiseSpeed=v))
ui.append_to_caption("<br><br>")

# ─── Viewport Control ─────────────────────────────────
ui.append_to_caption("<b>Viewport Control</b><br><br>")
add_slider(ui, "Rot Speed",     0,   .02, rotationSpeed,   "{:.3f}",
           lambda v: globals().update(rotationSpeed=v))
add_slider(ui, "Radius",        2,   20,  radius,          "{:.1f}",
           lambda v: globals().update(radius=v))
add_slider(ui, "Cam Height",    0,   10,  cameraHeight,    "{:.1f}",
           lambda v: globals().update(cameraHeight=v))
ui.append_to_caption("Show Poles ")
checkbox(bind=lambda c: globals().update(showPole=c.checked),
         text="ON", canvas=ui, checked=showPole)
ui.append_to_caption("<br><br>")

# ─── Appearance Control ──────────────────────────────
ui.append_to_caption("<b>Appearance Control</b><br><br>")
# 3Dビュー背景の明るさ
add_slider(ui, "BG3D Bright", 0, 1, bg3d_brightness, "{:.2f}",
           lambda v: (
               globals().update(bg3d_brightness=v),
               setattr(scene3d, "background", color.gray(v))
           ))

# 2Dビュー背景の明るさ
add_slider(ui, "BG2D Bright", 0, 1, bg2d_brightness, "{:.2f}",
           lambda v: (
               globals().update(bg2d_brightness=v),
               setattr(scene2d, "background", color.gray(v))
           ))

# LED 波の振幅
add_slider(ui, "LED Amplitude", 0, 2, led_amp, "{:.2f}",
           lambda v: globals().update(led_amp=v))

ui.append_to_caption("<br>")  # appearance control の終わり


# ─── HSB Color Control ───────────────────────────────
# 3) UI（add_slider／ui.append_to_caption済みと仮定）
ui.append_to_caption("<b>Wave Hue Control</b><br><br>")
add_slider(ui, "Hue Offset", 0.0, 1.0, base_hue, "{:.2f}",
           lambda v: globals().update(base_hue=v))
add_slider(ui, "Hue Span",   0.0, 1.0, hue_span, "{:.2f}",
           lambda v: globals().update(hue_span=v))
ui.append_to_caption("<br>")


# ========================================================
# Agent クラス
# ========================================================
class Agent:
    def __init__(self,i,j,x,y,z,nid):
        self.i, self.j, self.x, self.y, self.z = i, j, x, y, z
        self.node_id = nid
        self.vz = 0
        self.neighbors = []

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
        # --- スポットライト用の薄い円盤 ---
        # 床面 z=0 より少し上 (0.01m) に配置
        self.spot = cylinder(canvas=scene3d,
                             pos=vector(x,y,0.01),
                             axis=vector(0,0,0.002),
                             radius=1.0,       # 半径 1m ぐらい
                             color=color.white,
                             opacity=0.0)      # 初期はオフ
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

    def compute_axis(self):
        pr = radians(self.pitch) if hasattr(self, 'pitch') else 0
        yr = radians(self.yaw)   if hasattr(self, 'yaw')   else 0
        return vector(cos(pr)*cos(yr),
                      cos(pr)*sin(yr),
                      sin(pr))

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

    def set_spotlight(self):
        dx = self.x - spot_center_x
        dy = self.y - spot_center_y
        d  = math.hypot(dx, dy)
        nd = min(1.0, d / maxDist)               # 0..1 に正規化
        exponent = spot_exponent_max * (1 - spot_spread)
        opacity  = spot_brightness * ((1 - nd) ** exponent)
        self.spot.opacity = opacity

# ========================================================
# Agent 作成＋隣接設定
# ========================================================
agents = []
with open("node.csv", newline="") as f:
    for k, row in enumerate(csv.DictReader(f)):
        i, j = divmod(k, COLS)
        z    = random.uniform(minZ+agent_length/2, maxZ-agent_length/2)
        agents.append(Agent(i, j, float(row["x"]), float(row["y"]), z, row["id"]))
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
maxDist = math.hypot(maxX-minX, maxY-minY)

# UI用のデフォルトスポット位置もグリッド中央に
spot_center_x = centerX
spot_center_y = centerY

scene3d.center = vector(centerX, centerY, (minZ+maxZ)/2)
scene2d.center = vector(centerX, centerY, 0)
scene2d.range  = span * 0.4

# ─── Spotlight パラメータ ─────────────────────────────
add_slider(ui, "Spot Brightness", 0.0, 1.0, spot_brightness, "{:.2f}",
           lambda v: globals().update(spot_brightness=v))
add_slider(ui, "Spot Center X", minX, maxX, spot_center_x, "{:.2f}",
           lambda v: globals().update(spot_center_x=v))
add_slider(ui, "Spot Center Y", minY, maxY, spot_center_y, "{:.2f}",
           lambda v: globals().update(spot_center_y=v))
add_slider(ui, "Spot Spread",     0.0,  1.0, spot_spread, "{:.2f}",
           lambda v: globals().update(spot_spread=v))


# ========================================================
# メインループ
# ========================================================
sim_time = noise_time = angle = 0.0
dt = 1/60

while True:
    rate(20)
    sim_time   += dt
    noise_time += noiseSpeed
    angle      += rotationSpeed

    for ag in agents:
        ty = pnoise2(ag.i*noiseScale + noise_time,
                     ag.j*noiseScale + noise_time) * 360
        tp = pnoise2(ag.i*noiseScale + noise_time + 100,
                     ag.j*noiseScale + noise_time + 100) * 120 - 60
        ag.update(ty, tp, sim_time, dt)
        ag.display()
        ag.set_leds()
        # MQTT配信
        mqtt_client.publish(f"ps/{ag.node_id}",
            struct.pack("<3f",
                        round(ag.z,2),
                        round(ag.pitch,2),
                        round(ag.yaw,2)))
        
        # r0 = ag.current_color.x  # 0..1
        # g0 = ag.current_color.y
        # b0 = ag.current_color.z

        # brightness_byte = int(ag.current_brightness * 255)
        # r_byte = int(ag.current_color.x * brightness_byte / 255)
        # g_byte = int(ag.current_color.y * brightness_byte / 255)
        # b_byte = int(ag.current_color.z * brightness_byte / 255)


        r_byte = int(ag.current_color.x * 255)
        g_byte = int(ag.current_color.y * 255)
        b_byte = int(ag.current_color.z * 255)
        color_payload = bytes([r_byte, g_byte, b_byte])
        mqtt_client.publish(f"cl/{ag.node_id}", color_payload)

    # 3Dカメラ軌道更新
    camX = centerX + radius * math.cos(angle)
    camY = centerY + radius * math.sin(angle)
    scene3d.camera.pos  = vector(camX, camY, cameraHeight)
    scene3d.camera.axis = vector(centerX-camX,
                                 centerY-camY,
                                 ((minZ+maxZ)/2)-cameraHeight)
