# file: agent.py
import math, random, colorsys
from noise import pnoise2
from vpython import vector, cylinder, sphere, color
import config
import scenes

# グローバル変数 "agents" は loader.py で定義されます

class Agent:
    def __init__(self, i, j, x, y, z, nid):
        self.i, self.j, self.x, self.y, self.z = i, j, x, y, z
        self.node_id = nid
        self.vz = 0
        self.neighbors = []

        # Boids LFO
        self.baseSepBias = random.uniform(0.8, 1.2)
        self.baseCohBias = random.uniform(0.8, 1.2)
        self.freqSep     = random.uniform(0.2, 2.0)
        self.freqCoh     = random.uniform(0.2, 2.0)
        self.ampSep      = random.uniform(0.1, 0.5)
        self.ampCoh      = random.uniform(0.1, 0.5)

        self.yaw   = random.uniform(0, 360)
        self.pitch = random.uniform(-60, 60)

        # 3D body + ケーブル
        ax  = self.compute_axis() * config.agent_length
        ctr = vector(x, y, z)
        self.body = cylinder(
            canvas=scenes.scene3d,
            pos=ctr - ax/2,
            axis=ax,
            radius=config.agent_radius,
            color=color.red,
            shininess=0
        )
        self.cable = cylinder(
            canvas=scenes.scene3d,
            pos=ctr,
            axis=vector(0, 0, config.maxZ - z),
            radius=config.cable_radius,
            color=color.gray(0.5)
        )
        # 2D dot
        self.dot2d = sphere(
            canvas=scenes.scene2d,
            pos=vector(x, y, 0),
            radius=config.agent_radius,
            color=color.blue
        )
        # LEDs
        self.leds = []
        u = ax.norm()
        for t in (-0.5, 0.0, 0.5):
            p3 = ctr + u * (config.agent_length * t)
            ld3 = sphere(canvas=scenes.scene3d, pos=p3,
                         radius=config.led_radius,
                         emissive=True, color=color.black)
            ld2 = sphere(canvas=scenes.scene2d,
                         pos=vector(x + u.x * config.agent_length * t,
                                    y + u.y * config.agent_length * t, 0),
                         radius=config.led_radius,
                         emissive=True, color=color.black)
            self.leds.append((ld3, ld2, t))

        # RGB LED の色
        self.current_color = vector(0, 0, 0)
        self.current_brightness = 0

    def compute_axis(self):
        pr = math.radians(self.pitch)
        yr = math.radians(self.yaw)
        return vector(
            math.cos(pr) * math.cos(yr),
            math.cos(pr) * math.sin(yr),
            math.sin(pr)
        )

    def update_boids(self, t):
        fs = fc = cnt = 0
        for idx in self.neighbors:
            if 0 <= idx < len(agents):
                nb = agents[idx]
                dz = self.z - nb.z
                dist = abs(dz)
                if dist < config.sepDist:
                    fs += (config.sepDist - dist) * (1 if dz >= 0 else -1)
                fc += nb.z
                cnt += 1
        dynS = self.baseSepBias * (1 + self.ampSep * math.sin(2 * math.pi * self.freqSep * t))
        dynC = self.baseCohBias * (1 + self.ampCoh * math.sin(2 * math.pi * self.freqCoh * t))
        fs *= config.separationFactor * dynS
        coh = ((fc / cnt - self.z) * config.cohesionFactor * dynC) if cnt > 0 else 0
        self.vz = max(-1, min(1, self.vz + (fs + coh) * 0.01)) * 0.95
        self.z  = max(config.minZ, min(config.maxZ, self.z + self.vz))

    def update(self, ty, tp, t, dt):
        # Boids Z
        self.update_boids(t)
        # Wave Z
        w = pnoise2(self.i * config.waveScale + t,
                    self.j * config.waveScale + t)
        tgtZ = (w + 1) / 2 * (config.maxZ - config.minZ) + config.minZ
        self.z += (tgtZ - self.z) * config.waveStrength * dt
        self.z = max(config.minZ, min(config.maxZ, self.z))
        # FlowNoise yaw/pitch
        tp = max(-60, min(60, tp))
        self.yaw   += ((ty - self.yaw + 540) % 360 - 180) * 0.1
        self.pitch += ((tp - self.pitch + 540) % 360 - 180) * 0.1
        # ジオメトリ更新
        ax  = self.compute_axis() * config.agent_length
        ctr = vector(self.x, self.y, self.z)
        self.body.pos  = ctr - ax / 2
        self.body.axis = ax
        self.cable.pos = ctr
        self.cable.axis= vector(0, 0, config.maxZ - self.z)
        self.dot2d.pos = vector(self.x, self.y, 0)
        # LEDs follow position
        u = ax.norm()
        for ld3, ld2, t in self.leds:
            ld3.pos = ctr + u * (config.agent_length * t)
            ld2.pos = vector(
                self.x + u.x * config.agent_length * t,
                self.y + u.y * config.agent_length * t, 0
            )

    def display(self):
        self.body.visible  = True
        self.cable.visible = config.showPole

    def set_leds(self):
        # 1) 波の明るさ（0..1）
        b_noise = (pnoise2(
            self.i * config.waveScale + config.noise_time,
            self.j * config.waveScale + config.noise_time
        ) + 1) / 2
        brightness = min(1.0, max(0.0, b_noise * config.led_amp))

        # 2) HSV→RGB で「純色」を作る
        hue = (
            config.base_hue
            + (self.j / (config.COLS - 1)) * config.hue_span
            + config.hue_vari * math.sin(config.noise_time * config.hue_speed)
        ) % 1.0
        sat = config.base_sat
        val = 1.0
        r0, g0, b0 = colorsys.hsv_to_rgb(hue, sat, val)
        pure_color = vector(r0, g0, b0)

        # 3) 明るさを掛ける → 最終色
        final_color = pure_color * brightness
        self.current_brightness = brightness

        # 4) 色をオブジェクトに反映
        self.body.color = final_color
        for ld3, ld2, _ in self.leds:
            ld3.color = ld2.color = final_color

        # 5) 最新の色と明るさを保存
        self.current_color = final_color
        self.current_brightness = brightness
