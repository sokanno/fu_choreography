# Fragmentations of Unity — Choreography System

Ceiling-mounted robotic cylinder installation driven by real-time choreography software. 29 motorized tubes hang from the ceiling, each capable of vertical movement, yaw rotation, pitch tilt (+-60 deg), and independent RGB LED control. The system orchestrates multiple choreographic scenes through MQTT, with an accompanying audio layer synthesized in Max/MSP.

## System Architecture

```
                  ┌──────────────────┐
                  │   UI.maxpat      │
                  │  (Max/MSP GUI)   │
                  │   OSC control    │
                  └───────┬──────────┘
                          │ OSC :8000
                          ▼
┌─────────────────────────────────────────────────┐
│                  main.py                         │
│         Choreography Engine (VPython)            │
│                                                  │
│  Scenes:                                         │
│   - 回る天井       (Rotating Ceiling)            │
│   - 天上天下モード  (Tenjo Tenge)                │
│   - 魚群モード     (Fish Swarm)                  │
│   - 蜂シマーモード  (Bee Shimmer)                │
│   - 舞台挨拶モード  (Stage Greeting)             │
│   - 向き合うモード  (Face Each Other)            │
│   - ホタルモード    (Firefly)                    │
│   - 見えない蝶々    (Invisible Butterfly)        │
│   - The two of us                                │
│   - マニュアルモード (Manual)                    │
│                                                  │
│  MQTT heartbeat monitoring                       │
│  Audience detection (GridEye sensor fusion)      │
│  3D + 2D VPython visualizer                      │
└──────┬──────────────┬──────────────┬─────────────┘
       │ MQTT         │ OSC :57121   │ OSC :57120
       ▼              ▼              ▼
┌──────────┐  ┌──────────────┐  ┌──────────┐
│  29 Tubes │  │  Max/MSP     │  │SuperCol- │
│  (ESP32)  │  │  Audio       │  │lider     │
│           │  │              │  │(optional)│
│  ps/ cl/  │  │ hotaru.maxpat│  │          │
│  dl/ at/  │  │ twofus_synth │  │          │
│           │  │ tenjotenge   │  │          │
└──────────┘  │ shimmer etc. │  └──────────┘
              └──────────────┘
```

## Directory Structure

```
mqtt_python/
├── README.md
├── UI.maxpat                  # Max/MSP GUI — scene switching & parameter control
├── choreography/
│   ├── main.py                # Core choreography engine (~6000 lines)
│   ├── osc_listener.py        # OSC parameter receiver (port 8000)
│   ├── mqtt_listener.py       # Audience/sound-source MQTT subscriber
│   ├── node.csv               # 29-tube layout (id, x, y) in meters
│   ├── node10.csv             # 10-tube subset layout
│   ├── node15.csv             # 15-tube subset layout
│   ├── hotaru.maxpat          # Firefly audio engine (poly~ 29 voices)
│   ├── hotaruVoice.maxpat     # Single firefly voice (FM synthesis + auto-mute)
│   └── twofus_synth.maxpat    # "The two of us" synth (RGB-to-timbre mapping)
```

## Prerequisites

| Software | Version | Purpose |
|----------|---------|---------|
| Python | 3.11+ | Choreography engine |
| Mosquitto | 2.x | MQTT broker |
| Max/MSP | 8+ | Audio synthesis & UI |

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/sokanno/fu_choreography.git
cd fu_choreography
```

### 2. Create a Python virtual environment

```bash
python3 -m venv venv
source venv/bin/activate   # macOS / Linux
```

### 3. Install Python dependencies

```bash
pip install -r requirements.txt
```

This installs:
- `vpython` — 3D/2D real-time visualizer
- `paho-mqtt` — MQTT client for tube communication
- `python-osc` — OSC send/receive (Max/MSP, SuperCollider)
- `noise` — Perlin noise for organic motion

### 4. Install and start the MQTT broker

**macOS (Homebrew):**
```bash
brew install mosquitto
brew services start mosquitto
```

**Ubuntu / Raspberry Pi:**
```bash
sudo apt install mosquitto mosquitto-clients
sudo systemctl enable mosquitto
sudo systemctl start mosquitto
```

Confirm the broker is running:
```bash
mosquitto_sub -t "test" &
mosquitto_pub -t "test" -m "hello"
# Should print "hello"
```

### 5. Configure the network mode

Edit `choreography/main.py`, line ~30:

```python
place = "venue"   # production: MQTT broker at 192.168.1.2
```
```python
place = "local"   # development: MQTT broker at localhost
```

- **`"venue"`** — connects to `192.168.1.2:1883` (the production network with real tubes)
- **anything else** — connects to `localhost:1883` (local development / simulation)

### 6. Open the Max/MSP patches

1. Open `UI.maxpat` — main control interface for scene switching and parameter tuning
2. Open the audio patches as needed from `../FU_audio/`:
   - `_FU_audioMain.maxpat` — master audio router
   - `hotaru.maxpat` (or `choreography/hotaru.maxpat`) — firefly audio
   - `twofus_synth.maxpat` (or `choreography/twofus_synth.maxpat`) — "The two of us" audio
3. Ensure Max is listening on **UDP port 57121** (set automatically by the patches)

## Running

### Start the choreography engine

```bash
cd choreography
python main.py
```

A VPython visualizer opens in your browser showing:
- **Left panel**: 3D view with orbiting camera (tubes, cables, audience markers)
- **Right panel**: 2D top-down view (LED colors, dead-node indicators)

### Start a scene

From `UI.maxpat`, select a scene from the dropdown menu. The scene index is sent via OSC to `main.py` on port 8000.

### Verify everything is connected

| Check | How |
|-------|-----|
| MQTT broker | `mosquitto_sub -t "#" -v` — should show `ps/`, `cl/`, `dl/` messages flowing |
| Tube heartbeats | Red rings/cables disappear in the visualizer as tubes come online |
| Audio | Sounds play in Max when scenes trigger (firefly flashes, tube interactions) |
| OSC control | Changing parameters in `UI.maxpat` immediately affects the visualizer |

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `ModuleNotFoundError: vpython` | Virtual environment not activated | `source venv/bin/activate` |
| `ConnectionRefusedError` on startup | MQTT broker not running | `brew services start mosquitto` |
| All tubes show red rings | No heartbeat data (normal in dev mode without real tubes) | Expected behavior; all nodes treated as alive for choreography |
| Some tubes red despite working | `heartbeat_timeout` too short | Increase `heartbeat_timeout` in `main.py` (default: 30s) |
| No sound from Max | OSC port mismatch or Max patch not open | Ensure Max patches are open and DSP is on |
| `DeprecationWarning: Callback API version 1` | paho-mqtt 2.x warning | Cosmetic only; does not affect functionality |

## Communication Protocol

### MQTT Topics (Python -> Tubes)

| Topic | Payload | Description |
|-------|---------|-------------|
| `ps/{id}` | `<HhH` (6 bytes) | Position: height(mm), pitch(x100), yaw(x100) |
| `cl/{id}` | 3 bytes | Color: R, G, B (0-255) |
| `dl/{id}` | 1 byte | Downlight brightness (0-255) |
| `at/{id}` | 1 byte | Autonomous mode flag (0/1) |

### MQTT Topics (Tubes -> Python)

| Topic | Description |
|-------|-------------|
| `fu/device/{MAC}/heartbeat` | JSON with `id` field. 30s timeout for alive detection |
| `ca` | Audience position (int16 x2, cm) |
| `ss` | Sound source position |

### OSC (UI -> Python, port 8000)

Scene selection, Boids parameters, height control, manual overrides. See `osc_listener.py` for the full parameter list.

### OSC (Python -> Max, port 57121)

| Address | Args | Description |
|---------|------|-------------|
| `/firefly_flash` | id, x, y, z, isolation | Firefly sound trigger (200ms delayed) |
| `/twofus/tube` | id, z_norm, bri, r, g, b | RGB timbre control for "The two of us" |
| `/trig` | node_id | Tenjo Tenge crossing trigger |
| `/groupA_height` | float | Group A average height (normalized) |
| `/groupB_height` | float | Group B average height (normalized) |

## Scenes

### Rotating Ceiling (回る天井)
Tubes sway as a unified surface with Perlin noise waves and shadow waves rippling across the grid.

### Tenjo Tenge (天上天下モード)
One tube (Group A) opposes all others (Group B) in a sinusoidal vertical oscillation. On each crossing, Group A is reassigned to a new alive node with a guaranteed distinct color (minimum 72 deg hue shift). Group B displays the complementary color with brightness decay.

### Fish Swarm (魚群モード)
Boids-based flocking behavior. Tubes act as a school of fish with separation, cohesion, and alignment forces.

### Bee Shimmer (蜂シマーモード)
Coordinated shimmering patterns across the tube grid.

### Firefly (ホタルモード)
Kuramoto-model synchronization. Each tube has an independent phase oscillator that gradually synchronizes through visual coupling. Sound triggers 200ms after each flash for a natural light-before-sound feel. Isolated fireflies produce blue-shifted tones; synchronized ones glow yellow-green.

### The Two of Us
Two tubes descend and interact through choreographed movements and color exchange. Uses RGB-to-timbre crossmodal mapping:
- **Red** channel -> PWM oscillator (bright, aggressive)
- **Green** channel -> 2-op FM synthesis (organic, natural)
- **Blue** channel -> Pure sine (ethereal, boosted 3x for perceptual balance)

After interaction, one tube departs and a new partner arrives, carrying color traces from previous encounters.

### Invisible Butterfly (見えない蝶々モード)
An invisible butterfly agent flies through the space; tubes react to its proximity with height changes and color shifts.

## Alive Detection

Tubes periodically send MQTT heartbeats. The system tracks the last heartbeat time per node ID with a 30-second timeout window.

- **Visualizer**: Dead nodes show a red ring (2D) and red cable (3D)
- **Scene logic**: Dead nodes are excluded from individual assignments (Tenjo Tenge Group A, The Two of Us pair selection). If an active node goes dead mid-scene, it is immediately replaced with a living node
- **Firefly**: Dead nodes do not flash or produce sound, but remain in the phase simulation

## Audio (Max/MSP)

### Firefly (`hotaru.maxpat` + `hotaruVoice.maxpat`)
29-voice poly~ with FM synthesis. Each flash triggers a note with:
- Pitch mapped from node position
- Equal-power stereo panning from Y coordinate
- Auto-mute via `thispoly~` (voices mute when silent)
- 5-second watchdog timer mutes all voices on prolonged silence

### The Two of Us (`twofus_synth.maxpat`)
29-voice subtractive/FM/additive hybrid synth with:
- Per-voice RGB timbre blending (PWM + FM + Sine)
- Brightness-driven vibrato and pulse width modulation
- Position-based stereo panning (full L-R spread)
- Noise + filtered wind layer with LFO-driven filter sweep
- Stereo reverb and limiter

## Physical Tube Specifications

| Parameter | Value |
|-----------|-------|
| Count | 29 (grid layout, 4-5 per row, 7 rows) |
| Grid span | 3.6m (x) x 5.4m (y) |
| Vertical range | 0.0 - 2.8m |
| Pitch range | -60 to +60 deg |
| Yaw | 360 deg continuous |
| LEDs | 4 ring LEDs (front/back, inner/outer) |
| Controller | ESP32 |
| Communication | MQTT over WiFi |

## Node Layout

```
Row  y=+2.70:    1     2     3     4         (x: -1.35 to +1.35)
Row  y=+1.80:  5   6   7   8   9             (x: -1.80 to +1.80)
Row  y=+0.90:   10   11   12   13            (x: -1.35 to +1.35)
Row  y= 0.00:  14   15   16                  (x: -1.80 to  0.00)
Row  y=-0.90:   17   18   19   20            (x: -1.35 to +1.35)
Row  y=-1.80:  21   22   23   24   25        (x: -1.80 to +1.80)
Row  y=-2.70:   26   27   28   29            (x: -1.35 to +1.35)
```

## License

Private repository. All rights reserved.
