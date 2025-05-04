from vpython import *
import math, random, struct, csv
from noise import pnoise2   # pip install noise
import paho.mqtt.client as mqtt

# ----------------------------
# グローバルパラメータ
# ----------------------------
ROWS, COLS     = 7, 7
minZ, maxZ     = 0.0, 3.0
sepDist        = 0.4
agent_length   = 0.30
agent_radius   = 0.075/2
cable_radius   = 0.01
led_radius     = agent_radius

separationFactor = 0.14
cohesionFactor   = 1.76
rotationSpeed    = 0.001
radius           = 5.5
cameraHeight     = 1.0

noiseScale   = 0.01
noiseSpeed   = 0.01
waveScale    = 0.2
waveStrength = 0.1

led_amp      = 1.0          # LED 波の振幅スケーラ
led_color    = vector(1,1,1)  # 白

showPole = True

# MQTT セットアップ
mqtt_client = mqtt.Client()
mqtt_client.connect("127.0.0.1", 1883, 60)
mqtt_client.loop_start()

# ========================================================
# VPython キャンバス設定
# ========================================================
scene3d = canvas(title="3D View",
                 width=600, height=400,
                 background=color.gray(0.2),
                 align='left')
scene3d.up       = vector(0,0,1)
scene3d.forward  = vector(-1,-1,-0.2)
scene3d.userspin = False

scene2d = canvas(title="Top-Down View",
                 width=600, height=400,
                 background=color.gray(0.2),
                 align='right')
scene2d.camera.projection = "orthographic"
scene2d.up           = vector(1,0,0)
scene2d.camera.axis  = vector(0,0,-1)
scene2d.lights       = []

# ========================================================
# 床 (floor) の作成
# ========================================================
# 画面上の全エージェント範囲をあとで計算して centerX/centerY/span をセット
# → 先にダミーで作成しておき、後でサイズと位置を調整します
floor = box(canvas=scene3d,
            pos=vector(0,0,-0.01),
            size=vector(1,1,0.02),
            color=color.white,
            opacity=1.0)

# --------------------------------------------------------
# Agent クラス
# --------------------------------------------------------
class Agent:
    def __init__(self, i, j, x, y, z, nid):
        self.i, self.j = i, j
        self.x, self.y, self.z = x, y, z
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

        # 初期角度
        self.yaw   = random.uniform(0,360)
        self.pitch = random.uniform(-60,60)

        # 3D body + cable
        ax = self.compute_axis() * agent_length
        ctr = vector(x,y,z)
        self.body = cylinder(canvas=scene3d,
                             pos=ctr-ax/2,
                             axis=ax,
                             radius=agent_radius,
                             color=color.red,
                             shininess=0,      # マットに
                             emissive=False,
                             opacity=1.0)
        self.cable = cylinder(canvas=scene3d,
                              pos=ctr,
                              axis=vector(0,0,maxZ-z),
                              radius=cable_radius,
                              color=color.gray(0.5))

        # 2D dot
        self.dot2d = sphere(canvas=scene2d,
                            pos=vector(x,y,0),
                            radius=agent_radius,
                            color=color.blue)

        # LEDs（球体）
        self.leds = []
        u = ax.norm()
        for t in (-0.5, 0.0, 0.5):
            p3 = ctr + u*(agent_length*t)
            ld3 = sphere(canvas=scene3d,
                         pos=p3,
                         radius=led_radius,
                         emissive=True,
                         color=color.black)
            ld2 = sphere(canvas=scene2d,
                         pos=vector(x+u.x*agent_length*t,
                                    y+u.y*agent_length*t, 0),
                         radius=led_radius,
                         emissive=True,
                         color=color.black)
            self.leds.append((ld3, ld2, t))

        # スポットライト用薄筒 (床照明)
        spot_r = agent_radius * 2
        self.spot3d = cylinder(canvas=scene3d,
                               pos=vector(x, y, 0),
                               axis=vector(0,0,0.001),
                               radius=spot_r,
                               color=color.white,
                               emissive=True,
                               opacity=0.0)

    def compute_axis(self):
        pr = math.radians(self.pitch)
        yr = math.radians(self.yaw)
        return vector(math.cos(pr)*math.cos(yr),
                      math.cos(pr)*math.sin(yr),
                      math.sin(pr))

    def update_boids(self, t):
        fs = fc = cnt = 0
        for idx in self.neighbors:
            if 0 <= idx < len(agents):
                nb = agents[idx]
                dz = self.z - nb.z
                d  = abs(dz)
                if d < sepDist:
                    fs += (sepDist - d) * (1 if dz>=0 else -1)
                fc += nb.z; cnt += 1
        dynS = self.baseSepBias*(1 + self.ampSep*math.sin(2*math.pi*self.freqSep*t))
        dynC = self.baseCohBias*(1 + self.ampCoh*math.sin(2*math.pi*self.freqCoh*t))
        fs *= separationFactor * dynS
        coh = ((fc/cnt - self.z)*cohesionFactor * dynC) if cnt>0 else 0
        self.vz = max(-1, min(1, self.vz + (fs+coh)*0.01)) * 0.95
        self.z += self.vz
        self.z = max(minZ, min(maxZ, self.z))

    def update(self, ty, tp, t, dt):
        # Boids + Wave 高さ
        self.update_boids(t)
        w = pnoise2(self.i*waveScale + t, self.j*waveScale + t)
        tgtZ = (w+1)/2*(maxZ-minZ) + minZ
        self.z += (tgtZ - self.z)*waveStrength*dt
        self.z = max(minZ, min(maxZ, self.z))

        # FlowNoise で角度
        tp = max(-60, min(60, tp))
        self.yaw   += ((ty - self.yaw + 540)%360 - 180)*0.1
        self.pitch += ((tp - self.pitch + 540)%360 - 180)*0.1

        # ジオメトリ更新
        ax  = self.compute_axis()*agent_length
        ctr = vector(self.x, self.y, self.z)
        self.body.pos  = ctr - ax/2
        self.body.axis = ax
        self.cable.pos = ctr
        self.cable.axis= vector(0,0,maxZ-self.z)
        self.dot2d.pos = vector(self.x, self.y, 0)
        u = ax.norm()
        for ld3, ld2, t in self.leds:
            ld3.pos = ctr + u*(agent_length*t)
            ld2.pos = vector(self.x + u.x*agent_length*t,
                             self.y + u.y*agent_length*t, 0)

    def display(self):
        self.body.visible  = True
        self.cable.visible = showPole

    def set_leds(self):
        # 1) 明るさ波 (0..1)
        b = (pnoise2(self.i*waveScale + noise_time,
                     self.j*waveScale + noise_time) + 1)/2
        brightness = min(1.0, max(0.0, b * led_amp))

        # 2) 最終色を計算
        final_color = led_color * brightness

        # 3) 反映
        self.body.color = final_color
        for ld3, ld2, _ in self.leds:
            ld3.color = ld2.color = final_color

        # 4) 床スポットライトを点灯
        self.spot3d.opacity = brightness

# ========================================================
# エージェント作成 & 隣接設定
# ========================================================
agents = []
with open("node.csv", newline="") as f:
    rows = list(csv.DictReader(f))
for k, row in enumerate(rows):
    i, j = divmod(k, COLS)
    x, y = float(row["x"]), float(row["y"])
    z    = random.uniform(minZ+agent_length/2, maxZ-agent_length/2)
    agents.append(Agent(i, j, x, y, z, row["id"]))

for ag in agents:
    i,j = ag.i, ag.j
    nbrs = [(i-1,j),(i+1,j),(i,j-1),(i,j+1)]
    nbrs += ([(i-1,j-1),(i+1,j-1)] if i%2==0 else [(i-1,j+1),(i+1,j+1)])
    ag.neighbors = [ni*COLS+nj for ni,nj in nbrs]

# カメラ中心 & 床のサイズ調整
minX,maxX = min(a.x for a in agents), max(a.x for a in agents)
minY,maxY = min(a.y for a in agents), max(a.y for a in agents)
centerX,centerY = (minX+maxX)/2, (minY+maxY)/2
spanX = maxX-minX + agent_radius*2
spanY = maxY-minY + agent_radius*2

scene3d.center = vector(centerX, centerY, (minZ+maxZ)/2)
scene2d.center = vector(centerX, centerY, 0)
scene2d.range  = max(spanX, spanY)/2

floor.pos   = vector(centerX, centerY, -0.01)
floor.size  = vector(spanX, spanY, 0.02)

# メインループ
sim_time = noise_time = angle = 0.0
dt = 1/60

while True:
    rate(60)
    sim_time   += dt
    noise_time += noiseSpeed
    angle      += rotationSpeed

    # 各エージェント更新
    for ag in agents:
        ty = pnoise2(ag.i*noiseScale + noise_time,
                     ag.j*noiseScale + noise_time) * 360
        tp = pnoise2(ag.i*noiseScale + noise_time + 100,
                     ag.j*noiseScale + noise_time + 100) * 120 - 60
        ag.update(ty, tp, sim_time, dt)
        ag.display()
        ag.set_leds()
        # MQTT 送信
        mqtt_client.publish(f"ps/{ag.node_id}",
            struct.pack("<3f", round(ag.z,2), round(ag.pitch,2), round(ag.yaw,2)))

    # カメラ軌道
    camX = centerX + radius * math.cos(angle)
    camY = centerY + radius * math.sin(angle)
    scene3d.camera.pos  = vector(camX, camY, cameraHeight)
    scene3d.camera.axis = vector(centerX-camX,
                                 centerY-camY,
                                 ((minZ+maxZ)/2)-cameraHeight)
