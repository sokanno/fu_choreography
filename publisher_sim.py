from vpython import *
import math, random, struct, csv
from noise import pnoise2   # pip install noise
import paho.mqtt.client as mqtt

# ========================================================
# グローバルパラメータ（初期値／添付画像に合わせて変更）
# ========================================================
separationFactor = 0.14   # 分離力 (global) デフォルト 0.14
cohesionFactor   = 1.76   # 結合力 (global) デフォルト 1.76
rotationSpeed    = 0.001  # カメラ回転速度 デフォルト 0.001
radius           = 6.9    # カメラから中心までの距離 デフォルト 6.9
cameraHeight     = 1.0    # カメラ高さ デフォルト 1.0
noiseScale       = 0.01   # フローノイズのスケール デフォルト 0.01
showPole         = True   # ケーブル（ポール）の描画オン/オフ デフォルトON
waveScale    = 0.2    # 波の空間スケール
waveStrength = 0.1    # 波の強さ（0～1程度）
noiseSpeed = 0.01    # FlowNoise 時間進行速度

# --- グローバルパラメータ／LED 用 ---
waveScale    = 0.2
waveStrength = 0.1
noiseScale   = 0.05
noiseSpeed   = 0.01
rotationSpeed= 0.005
radius       = 8.0
cameraHeight = 4.0
separationFactor = 1.0
cohesionFactor   = 0.2

# LED の見た目
led_radius = 0.02

# ========================================================
# グリッド／シミュレーション定数（単位: m）
# ========================================================
ROWS = 7
COLS = 7
minZ = 0.0      # ノード高さ下限
maxZ = 3.0      # ノード高さ上限
separationDist = 0.4  # Z方向の分離判定距離

# エージェント（筒）のサイズ
agent_length   = 0.30    # 30cm
agent_diameter = 0.075   # 7.5cm
agent_radius   = agent_diameter/2

# ケーブル（ポール）の太さ
cable_radius = 0.01

# ========================================================
# MQTT のセットアップ
# ========================================================
mqtt_client = mqtt.Client()
mqtt_client.connect("127.0.0.1", 1883, 60)
mqtt_client.loop_start()

# ========================================================
# VPython シーン設定
# ========================================================
# scene.width = 800
# scene.height = 600
# scene.background = color.gray(0.8)
# scene.forward = vector(-1, -1, -0.2)  # 初期カメラ向き（後で毎フレーム更新）
scene.width = 600
scene.height = 400
scene.background = color.gray(0.8)
scene.forward = vector(-1, -1, -0.2)  # 初期カメラ向き
scene.up = vector(0, 0, 1)             # これで回転軸が Z 軸に固定されます

# ========================================================
# Pause ボタン設定
# ========================================================

paused = False

def toggle_pause(btn):
    global paused
    paused = not paused
    if paused:
        btn.text = "Resume"
    else:
        btn.text = "Pause"


# ========================================================
# VPython GUI ウィジェット（スライダー・チェックボックス）
# ========================================================

scene.append_to_caption("\nSeparation Force (global): ")
def update_separation(s):
    global separationFactor
    separationFactor = s.value
    separation_text.text = f" {s.value:.2f}\n"
separation_slider = slider(min=0, max=2, value=separationFactor, length=220, bind=update_separation)
separation_text = wtext(text=f" {separationFactor:.2f}\n")

scene.append_to_caption("\nCohesion Force (global): ")
def update_cohesion(s):
    global cohesionFactor
    cohesionFactor = s.value
    cohesion_text.text = f" {s.value:.2f}\n"
cohesion_slider = slider(min=0, max=2, value=cohesionFactor, length=220, bind=update_cohesion)
cohesion_text = wtext(text=f" {cohesionFactor:.2f}\n")

scene.append_to_caption("\nCamera Rotation Speed: ")
def update_rotation(s):
    global rotationSpeed
    rotationSpeed = s.value
    rotation_text.text = f" {s.value:.3f}\n"
rotation_slider = slider(min=0, max=0.02, value=rotationSpeed, length=220, bind=update_rotation)
rotation_text = wtext(text=f" {rotationSpeed:.3f}\n")

scene.append_to_caption("\nCamera Distance (Radius): ")
def update_zoom(s):
    global radius
    radius = s.value
    zoom_text.text = f" {s.value:.1f}\n"
zoom_slider = slider(min=2, max=20, value=radius, length=220, bind=update_zoom)
zoom_text = wtext(text=f" {radius:.1f}\n")

scene.append_to_caption("\nCamera Height: ")
def update_camHeight(s):
    global cameraHeight
    cameraHeight = s.value
    camHeight_text.text = f" {s.value:.1f}\n"
camHeight_slider = slider(min=0, max=10, value=cameraHeight, length=220, bind=update_camHeight)
camHeight_text = wtext(text=f" {cameraHeight:.1f}\n")

scene.append_to_caption("\nFlow Noise Scale: ")
def update_noiseScale(s):
    global noiseScale
    noiseScale = s.value
    noiseScale_text.text = f" {s.value:.2f}\n"
noiseScale_slider = slider(min=0.01, max=0.5, value=noiseScale, length=220, bind=update_noiseScale)
noiseScale_text = wtext(text=f" {noiseScale:.2f}\n")

scene.append_to_caption("\nWave Scale: ")
def update_waveScale(s):
    global waveScale
    waveScale = s.value
    waveScale_text.text = f" {s.value:.2f}\n"
waveScale_slider = slider(min=0.01, max=1.0, value=waveScale, length=220, bind=update_waveScale)
waveScale_text   = wtext(text=f" {waveScale:.2f}\n")

scene.append_to_caption("\nWave Strength: ")
def update_waveStrength(s):
    global waveStrength
    waveStrength = s.value
    waveStrength_text.text = f" {s.value:.2f}\n"
waveStrength_slider = slider(min=0.0, max=1.0, value=waveStrength, length=220, bind=update_waveStrength)
waveStrength_text   = wtext(text=f" {waveStrength:.2f}\n")

scene.append_to_caption("\nNoise Time Speed: ")
def update_noiseSpeed(s):
    global noiseSpeed
    noiseSpeed = s.value
    noiseSpeed_text.text = f" {s.value:.3f}\n"
noiseSpeed_slider = slider(min=0.0, max=0.1, value=noiseSpeed,
                           length=220, bind=update_noiseSpeed)
noiseSpeed_text   = wtext(text=f" {noiseSpeed:.3f}\n")

scene.append_to_caption("\nShow Poles (Cables): ")
def update_showPole(c):
    global showPole
    showPole = c.checked
showPole_checkbox = checkbox(bind=update_showPole, text=" Show Poles", checked=showPole)
scene.append_to_caption("\n\n")

scene.append_to_caption("\n")  # 改行
pause_button = button(text="Pause", bind=toggle_pause)
scene.append_to_caption("\n\n")

# ========================================================
# 補助関数：角度をスムーズに近づける（度数法）
# ========================================================
def approach_angle_degrees(current, target, ratio):
    d = target - current
    d = ((d + 540) % 360) - 180
    return current + ratio * d

# ========================================================
# Agent クラス
# ========================================================
class Agent:
    def __init__(self, i, j, x, y, z, node_id):
        self.i = i
        self.j = j
        self.x = x
        self.y = y
        self.z = z
        self.node_id = node_id
        self.vz = 0.0
        self.neighbors = []  # 後で設定
        # LFO 用パラメータ（個体差）
        self.baseSepBias = random.uniform(0.8, 1.2)
        self.baseCohBias = random.uniform(0.8, 1.2)
        self.freqSep = random.uniform(0.2, 2.0)
        self.freqCoh = random.uniform(0.2, 2.0)
        self.ampSep  = random.uniform(0.1, 0.5)
        self.ampCoh  = random.uniform(0.1, 0.5)
        # 初期の向き
        self.yaw   = random.uniform(0, 360)
        self.pitch = random.uniform(-60, 60)
        # --- シリンダー(body) を「中心位置 self.z」から半長さだけオフセットして生成 ---
        axis_vec = self.compute_axis()
        center = vector(self.x, self.y, self.z)  # ここが筒の中心
        self.body = cylinder(
            pos=center - axis_vec * (agent_length/2),  # 中心から下方向へ半長さだけ動かす
            axis=axis_vec * agent_length,
            radius=agent_radius,
            color=color.red
        )
        # ケーブル(cable)は常に「中心位置」から天井まで垂直に伸びる
        self.cable = cylinder(
            pos=center,
            axis=vector(0, 0, maxZ - self.z),
            radius=cable_radius,
            color=color.gray(0.5)
        )
    
    def compute_axis(self):
        pitch_rad = math.radians(self.pitch)
        yaw_rad   = math.radians(self.yaw)
        return vector(math.cos(pitch_rad)*math.cos(yaw_rad),
                      math.cos(pitch_rad)*math.sin(yaw_rad),
                      math.sin(pitch_rad))
    
    def update_boids_forces_z(self, separationFactor, cohesionFactor, sim_time):
        # ...（既存の内容）
        forceSeparation = 0.0
        forceCohesion   = 0.0
        sumZ = 0.0
        countN = 0
        for idx in self.neighbors:
            if idx < 0 or idx >= len(agents):
                continue
            nb = agents[idx]
            dz = self.z - nb.z
            dist = abs(dz)
            if dist < separationDist:
                overlap = (separationDist - dist)
                direction = 1 if dz >= 0 else -1
                forceSeparation += overlap * direction
            sumZ += nb.z
            countN += 1
        dynSep = self.baseSepBias * (1 + self.ampSep * math.sin(2*math.pi * self.freqSep * sim_time))
        dynCoh = self.baseCohBias * (1 + self.ampCoh * math.sin(2*math.pi * self.freqCoh * sim_time))
        forceSeparation *= (separationFactor * dynSep)
        if countN > 0:
            avgZ = sumZ / countN
            diffZ = avgZ - self.z
            forceCohesion = diffZ * (cohesionFactor * dynCoh)
        force = (forceSeparation + forceCohesion) * 0.01
        self.vz += force
        self.vz = max(-1, min(1, self.vz))
        self.vz *= 0.95
        self.z += self.vz
        self.z = max(minZ, min(maxZ, self.z))
    
    def update_from_flow_fields(self, targetYaw, targetPitch):
        # ターゲットのpitchを-60°～+60°にクランプ
        targetPitch = max(-60, min(60, targetPitch))
        self.yaw   = approach_angle_degrees(self.yaw, targetYaw, 0.1)
        self.pitch = approach_angle_degrees(self.pitch, targetPitch, 0.1)
        # 更新後のpitchも確実に-60°～+60°の範囲にする
        self.pitch = max(-60, min(60, self.pitch))
    
    def update(self, separationFactor, cohesionFactor,
               targetYaw, targetPitch, sim_time, dt):
        # 1) Boids 力で self.vz／self.z を更新
        self.update_boids_forces_z(separationFactor, cohesionFactor, sim_time)

        # 2) 波動（FlowNoise）による追従：Perlin ノイズを Z へミックス
        wave = pnoise2(self.i * waveScale + sim_time,
                       self.j * waveScale + sim_time)  # -1..+1
        # マップして 0..1 に → minZ..maxZ へ
        targetZ = (wave + 1)/2 * (maxZ - minZ) + minZ
        # dt ごとにスムーズに近づける
        self.z += (targetZ - self.z) * waveStrength * dt
        # 境界クリップ
        self.z = max(minZ, min(maxZ, self.z))

        # 3) Flow ノイズ由来 yaw/pitch 更新
        self.update_from_flow_fields(targetYaw, targetPitch)
        # 4) VPython オブジェクト位置更新
        axis_vec = self.compute_axis()
        center   = vector(self.x, self.y, self.z)
        self.body.pos  = center - axis_vec * (agent_length/2)
        self.body.axis = axis_vec * agent_length
        self.cable.pos  = center
        self.cable.axis = vector(0, 0, maxZ - self.z)

    def display(self, showPole):
        self.cable.visible = showPole
        self.body.visible = True

# ========================================================
# CSV からノード情報を読み込み（"node.csv"：カラム "id", "x", "y"）
# ========================================================
nodes = []
with open("node.csv", newline="") as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        node_id = row["id"].strip()
        x = float(row["x"])
        y = float(row["y"])
        nodes.append((node_id, x, y))

if len(nodes) != ROWS * COLS:
    print("Warning: CSV内のノード数が ROWS×COLS と一致していません。")

# CSV の順番を 7x7 の行列（row-major order）とみなし、各エージェントに i,j を割り当てる
agents = []
for index, (node_id, x, y) in enumerate(nodes):
    i = index // COLS
    j = index % COLS
    # 筒の中心が minZ ～ maxZ の範囲になるように  
    z = random.uniform(minZ + agent_length/2, maxZ - agent_length/2)
    agents.append(Agent(i, j, x, y, z, node_id))

# ========================================================
# 各エージェントに対して、隣接セル（ハニカム）のインデックスを設定
# ※ CSVのノード数が不足している場合、計算上の隣接インデックスが範囲外になる可能性があるため、
#    update_boids_forces_z 内でチェックを行っています。
# ========================================================
for agent in agents:
    i = agent.i
    j = agent.j
    neighbor_indices = []
    if i % 2 == 0:
        neighbor_coords = [(i-1, j), (i+1, j), (i, j-1), (i, j+1), (i-1, j-1), (i+1, j-1)]
    else:
        neighbor_coords = [(i-1, j), (i+1, j), (i, j-1), (i, j+1), (i-1, j+1), (i+1, j+1)]
    for (ni, nj) in neighbor_coords:
        idx = ni * COLS + nj
        neighbor_indices.append(idx)
    agent.neighbors = neighbor_indices

# ========================================================
# シーン内のエージェント群の中心を計算（カメラ注視点）
# ========================================================
minX = min(agent.x for agent in agents)
maxX = max(agent.x for agent in agents)
minY = min(agent.y for agent in agents)
maxY = max(agent.y for agent in agents)
centerX = (minX + maxX) / 2
centerY = (minY + maxY) / 2
scene.center = vector(centerX, centerY, (minZ+maxZ)/2)

# --- ループ前に置く ---
sim_time   = 0.0   # Boids＋波動用時間
noise_time = 0.0   # FlowNoise用時間
angle      = 0.0   # カメラ用回転角
dt         = 1/30  # フレーム時間（約5FPS想定でもrate(30)に合わせるなら1/30）

# --- メインループ ---
while True:
    rate(30)

    # Pause 中は時間を進めず描画もスキップ
    if paused:
        continue

    # 時間更新
    sim_time   += dt
    noise_time += noiseSpeed    # yaw/pitch 用ノイズの時間増分
    angle      += rotationSpeed

    # カメラを円軌道
    camX = centerX + radius * math.cos(angle)
    camY = centerY + radius * math.sin(angle)
    scene.camera.pos  = vector(camX, camY, cameraHeight)
    scene.camera.axis = vector(centerX - camX,
                               centerY - camY,
                               ((minZ+maxZ)/2) - cameraHeight)

    # 各エージェントの更新
    for agent in agents:
        # FlowNoise 由来の yaw/pitch
        ty = pnoise2(agent.i*noiseScale + noise_time,
                     agent.j*noiseScale + noise_time) * 360
        tp = pnoise2(agent.i*noiseScale + noise_time + 100,
                     agent.j*noiseScale + noise_time + 100) * 120 - 60

        # Boids＋波動＋FlowNoise 更新
        agent.update(separationFactor, cohesionFactor,
                     ty, tp, sim_time, dt)

        # 描画ON/OFF
        agent.display(showPole)

        # MQTT 送信
        h = round(agent.z, 2)
        p = round(agent.pitch, 2)
        y = round(agent.yaw, 2)
        payload = struct.pack("<3f", h, p, y)
        mqtt_client.publish(f"ps/{agent.node_id}", payload)
