# file: agent.py
import math, random, colorsys
from noise import pnoise2
from vpython import vector, cylinder, sphere, color
import config
import scenes

# グローバル変数 'agents' は loader.py で設定されます

class Agent:
    def __init__(self, i, j, x, y, z, nid, idx=None):
        self.i, self.j, self.x, self.y, self.z = i, j, x, y, z
        self.node_id = nid
        self.idx = idx
        self.vz = 0
        self.neighbors = []
        self.group = None

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
            nb = scenes.__dict__.get('agents', [])[idx] if 'agents' in scenes.__dict__ else None
            if nb:
                dz = self.z - nb.z
                dist = abs(dz)
                if dist < config.sepDist:
                    fs += (config.sepDist - dist) * (1 if dz >= 0 else -1)
                fc += nb.z; cnt += 1
        dynS = self.baseSepBias * (1 + self.ampSep * math.sin(2 * math.pi * self.freqSep * t))
        dynC = self.baseCohBias * (1 + self.ampCoh * math.sin(2 * math.pi * self.freqCoh * t))
        fs *= config.separationFactor * dynS
        coh = ((fc / cnt - self.z) * config.cohesionFactor * dynC) if cnt > 0 else 0
        self.vz = max(-1, min(1, self.vz + (fs + coh) * 0.01)) * 0.95
        self.z  = max(config.minZ, min(config.maxZ, self.z + self.vz))

    def update(self, ty, tp, t, dt):
        self.update_boids(t)
        w = pnoise2(self.i * config.waveScale + t, self.j * config.waveScale + t)
        tgtZ = (w + 1) / 2 * (config.maxZ - config.minZ) + config.minZ
        self.z += (tgtZ - self.z) * config.waveStrength * dt
        self.z = max(config.minZ, min(config.maxZ, self.z))
        tp = max(-60, min(60, tp))
        self.yaw   += ((ty - self.yaw + 540) % 360 - 180) * 0.1
        self.pitch += ((tp - self.pitch + 540) % 360 - 180) * 0.1
        ax  = self.compute_axis() * config.agent_length
        ctr = vector(self.x, self.y, self.z)
        self.body.pos  = ctr - ax/2; self.body.axis = ax
        self.cable.pos = ctr; self.cable.axis = vector(0,0,config.maxZ-self.z)
        self.dot2d.pos = vector(self.x, self.y, 0)
        u = ax.norm()
        for ld3, ld2, off in self.leds:
            ld3.pos = ctr + u * (config.agent_length * off)
            ld2.pos = vector(self.x + u.x * config.agent_length * off,
                             self.y + u.y * config.agent_length * off, 0)

    def display(self):
        self.body.visible  = True
        self.cable.visible = config.showPole

    def set_leds(self):
        b_noise = (pnoise2(self.i * config.waveScale + config.noise_time,
                           self.j * config.waveScale + config.noise_time) + 1) / 2
        brightness = min(1.0, max(0.0, b_noise * config.led_amp))
        hue = (config.base_hue + (self.j / (config.COLS - 1)) * config.hue_span +
               config.hue_vari * math.sin(config.noise_time * config.hue_speed)) % 1.0
        r0, g0, b0 = colorsys.hsv_to_rgb(hue, config.base_sat, 1.0)
        final_color = vector(r0, g0, b0) * brightness
        self.body.color = final_color
        for ld3, ld2, _ in self.leds:
            ld3.color = ld2.color = final_color
        self.current_color = final_color
        self.current_brightness = brightness

    def update_tenge(self, t, dt):
        # 1) 目標値から現在値へ追従
        alpha = dt / (1.0 + dt)
        config.current_sine_amp  += (config.target_sine_amp  - config.current_sine_amp)  * alpha
        config.current_sine_freq += (config.target_sine_freq - config.current_sine_freq) * alpha
        # 2) グループ設定
        if self.idx is not None:
            self.group = 'A' if self.idx < config.groupA_count else 'B'
        # 3) 正弦波で高さ更新
        midZ = (config.maxZ + config.minZ) / 2
        self.z = midZ + config.current_sine_amp * math.sin(2 * math.pi * config.current_sine_freq * t + (0 if self.group=='A' else math.pi))
        # 4) 他グループの最も近いエージェントを向く
        others = [o for o in scenes.__dict__.get('agents', []) if hasattr(o, 'group') and o.group != self.group]
        if others:
            target = min(others, key=lambda o: (o.x - self.x)**2 + (o.y - self.y)**2)
            dx, dy, dz = target.x - self.x, target.y - self.y, target.z - self.z
            self.yaw = math.degrees(math.atan2(dy, dx))
            self.pitch = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
        # 5) ジオメトリ更新
        ax  = self.compute_axis() * config.agent_length
        ctr = vector(self.x, self.y, self.z)
        self.body.pos  = ctr - ax/2; self.body.axis = ax
        self.cable.pos = ctr; self.cable.axis = vector(0,0,config.maxZ-self.z)
        self.dot2d.pos = vector(self.x, self.y, 0)

    def set_leds_tenge(self, t):
        base = (t * config.color_speed) % 1.0
        hue = base if self.group=='A' else (base + 0.5) % 1.0
        r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
        col = vector(r, g, b)
        self.body.color = col
        for ld3, ld2, _ in self.leds:
            ld3.color = ld2.color = col
