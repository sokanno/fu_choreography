# Directory structure:
# 
# config.py         – グローバルパラメータ定義
# mqtt_setup.py     – MQTT クライアント初期化
# scenes.py         – VPython シーン（3D/2D）と床／天井作成
# ui.py             – UI（スライダー・メニュー）定義
# agent.py          – Agent クラス定義
# loader.py         – node.csv から agents 読み込み・隣接設定・カメラ中心計算
# main.py           – メインループ実行

# ---------------------------------
# file: config.py
# ---------------------------------
from vpython import *
import math, random, struct, csv
from noise import pnoise2
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
# 天上天下唯我独尊モード用パラメータ
# ========================================================
groupA_count      = 1                   # A グループの台数
sine_freq         = 0.2                 # 正弦波の周波数 [Hz]
sine_amp          = (maxZ - minZ) / 4   # 正弦波の振幅
color_speed       = 0.1                 # LED 色変化の速度

target_sine_amp  = sine_amp
target_sine_freq = sine_freq
current_sine_amp  = target_sine_amp
current_sine_freq = target_sine_freq