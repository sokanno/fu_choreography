#!/usr/bin/env python3
# Drag = ca, Shift‑click (or right‑click) = ss  publisher

import pygame, struct
import paho.mqtt.client as mqtt

# ─── 展示室寸法 (m) ─────────────────────────────────────────────
ROOM_X_MIN, ROOM_X_MAX = -10.9,  3.2
ROOM_Y_MIN, ROOM_Y_MAX = -3.8,   3.8

SCALE = 50   # px / m
W = int((ROOM_X_MAX - ROOM_X_MIN) * SCALE)
H = int((ROOM_Y_MAX - ROOM_Y_MIN) * SCALE)

# ─── MQTT ─────────────────────────────────────────────────────
BROKER = "localhost"
TOPIC_CA = "ca"
TOPIC_SS = "ss"

# ─── PyGame 初期化 ────────────────────────────────────────────
pygame.init()
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption("Drag = ca,  Shift‑click / Right‑click = ss")

clock = pygame.time.Clock()
dragging = False
marker_ca = None           # 円を描く用 (ca)
marker_ss = None           # クリック瞬間に点滅 (ss)

# MQTT
client = mqtt.Client()
client.connect(BROKER, 1883, 60)
client.loop_start()

# util: 画面座標 → (m)
def screen_to_room(px, py):
    x_m = ROOM_X_MIN + px / SCALE
    y_m = ROOM_Y_MAX - py / SCALE     # Y 軸反転
    return x_m, y_m

# ─── メインループ ───────────────────────────────────────────
running = True
while running:
    for ev in pygame.event.get():
        if ev.type == pygame.QUIT:
            running = False

        # ---- 左ボタン押下 → ca ドラッグ開始 -----------------
        elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
            mods = pygame.key.get_mods()
            if mods & pygame.KMOD_SHIFT:          # ← Shift 押しながら左クリック ＝ ss
                x_m, y_m = screen_to_room(*ev.pos)
                payload = struct.pack(">hh",
                                       int(round(x_m*100)),
                                       int(round(y_m*100)))
                client.publish(TOPIC_SS, payload, qos=0)
                marker_ss = ev.pos        # 一瞬だけ描画
            else:
                dragging = True
                marker_ca = ev.pos        # ドラッグ用マーカー

        # ---- 右クリック (button==3) → ss -------------------
        elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 3:
            x_m, y_m = screen_to_room(*ev.pos)
            payload = struct.pack(">hh",
                                   int(round(x_m*100)),
                                   int(round(y_m*100)))
            client.publish(TOPIC_SS, payload, qos=0)
            marker_ss = ev.pos

        # ---- ドラッグ移動 -------------------------------
        elif ev.type == pygame.MOUSEMOTION and dragging:
            marker_ca = ev.pos
            x_m, y_m = screen_to_room(*ev.pos)
            payload = struct.pack(">hh",
                                   int(round(x_m*100)),
                                   int(round(y_m*100)))
            client.publish(TOPIC_CA, payload, qos=0)

        # ---- 左ボタン離し → ドラッグ終了 ----------------
        elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
            dragging = False
            marker_ca = None

    # ---- 描画 ----------------------------------------------
    screen.fill((255, 255, 255))
    pygame.draw.rect(screen, (0,0,0), (0,0,W,H), width=2)

    if marker_ca:
        pygame.draw.circle(screen, (255,0,0), marker_ca, 5)
    if marker_ss:
        pygame.draw.circle(screen, (0,0,255), marker_ss, 5)
        marker_ss = None                 # 1 フレームだけ表示

    pygame.display.flip()
    clock.tick(60)

# ---- 終了処理 ----
client.loop_stop()
client.disconnect()
pygame.quit()
