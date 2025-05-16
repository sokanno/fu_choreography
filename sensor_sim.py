#!/usr/bin/env python3
"""
Drag: publish audience (“ca”) coordinates
    * Hold numeric key 1‑5 while dragging to send that many people
      − 2 → +1 m on +X  (x+1,  y   )
      − 3 → +1 m on +Y  (x+0,  y+1 )
      − 4 → +1 m on –X  (x‑1,  y   )
      − 5 → +1 m on –Y  (x+0,  y‑1 )

      Payload order: x1 y1 x2 y2 …  (big‑endian int16 cm)

Shift‑click or Right‑click: publish sound‑source (“ss”) single coordinate

Keys are checked *while the mouse button is held*, so you can change the
number on the fly.
"""

import pygame
import struct
import paho.mqtt.client as mqtt

# ───────────────────────────────────────────────────────────
# Exhibition room field size [m]
# ───────────────────────────────────────────────────────────
ROOM_X_MIN, ROOM_X_MAX = -10.9, 3.2
ROOM_Y_MIN, ROOM_Y_MAX = -3.8, 3.8

SCALE = 50  # px / m
W = int((ROOM_X_MAX - ROOM_X_MIN) * SCALE)
H = int((ROOM_Y_MAX - ROOM_Y_MIN) * SCALE)

# ───────────────────────────────────────────────────────────
# MQTT
# ───────────────────────────────────────────────────────────
# BROKER = "192.168.1.2"
BROKER = "localhost"
TOPIC_CA = "ca"
TOPIC_SS = "ss"

# ───────────────────────────────────────────────────────────
# PyGame setup
# ───────────────────────────────────────────────────────────
pygame.init()
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption("Drag=ca  |  Shift/Right‑click=ss  |  1‑5 people")
clock = pygame.time.Clock()

# MQTT
client = mqtt.Client()
client.connect(BROKER, 1883, 60)
client.loop_start()

# ───────────────────────────────────────────────────────────
# Helpers
# ───────────────────────────────────────────────────────────

def screen_to_room(px: int, py: int):
    """Convert screen pixels → metres (room coords)"""
    x_m = ROOM_X_MIN + px / SCALE
    y_m = ROOM_Y_MAX - py / SCALE  # flip Y
    return x_m, y_m

# Offsets pattern (m)
PATTERN = [(0, 0),  # always the first person (cursor)
           (1, 0),  # +X
           (0, 1),  # +Y
           (-1, 0), # –X
           (0, -1)] # –Y

def current_count():
    """Return 1‑5 depending on which numeric key is held (default 1)."""
    keys = pygame.key.get_pressed()
    for n, key in ((5, pygame.K_5), (4, pygame.K_4),
                   (3, pygame.K_3), (2, pygame.K_2)):
        if keys[key]:
            return n
    return 1

def build_ca_payload(x0_m: float, y0_m: float, n: int) -> bytes:
    """Pack (x1,y1,x2,y2,…) big‑endian int16 cm"""
    parts = []
    for dx, dy in PATTERN[:n]:
        x_cm = int(round((x0_m + dx) * 100))
        y_cm = int(round((y0_m + dy) * 100))
        parts.extend([x_cm, y_cm])
    fmt = ">" + "hh" * n
    return struct.pack(fmt, *parts)

# ───────────────────────────────────────────────────────────
# State vars
# ───────────────────────────────────────────────────────────

dragging = False
base_pos_px: tuple[int, int] | None = None  # cursor pos during drag

running = True
while running:
    for ev in pygame.event.get():
        if ev.type == pygame.QUIT:
            running = False

        # ─── Left‑button press → start ca drag (unless Shift) ───
        elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
            mods = pygame.key.get_mods()
            if mods & pygame.KMOD_SHIFT:               # Shift‑click = ss
                x_m, y_m = screen_to_room(*ev.pos)
                payload = struct.pack(">hh", int(round(x_m*100)), int(round(y_m*100)))
                client.publish(TOPIC_SS, payload, qos=0)
            else:
                dragging = True
                base_pos_px = ev.pos
                # send first point immediately
                x_m, y_m = screen_to_room(*ev.pos)
                n = current_count()
                client.publish(TOPIC_CA, build_ca_payload(x_m, y_m, n), qos=0)

        # ─── Right‑click = ss ───
        elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 3:
            x_m, y_m = screen_to_room(*ev.pos)
            payload = struct.pack(">hh", int(round(x_m*100)), int(round(y_m*100)))
            client.publish(TOPIC_SS, payload, qos=0)

        # ─── Drag movement → continuous ca publish ───
        elif ev.type == pygame.MOUSEMOTION and dragging:
            base_pos_px = ev.pos
            x_m, y_m = screen_to_room(*ev.pos)
            n = current_count()
            client.publish(TOPIC_CA, build_ca_payload(x_m, y_m, n), qos=0)

        # ─── Left‑button release ───
        elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
            dragging = False
            base_pos_px = None

    # ─── Draw — simple visual feedback ───
    screen.fill((255, 255, 255))
    pygame.draw.rect(screen, (0, 0, 0), (0, 0, W, H), width=2)

    if base_pos_px:
        # Draw one circle per person at offset locations
        x0_px, y0_px = base_pos_px
        n = current_count()
        for dx, dy in PATTERN[:n]:
            px = x0_px + int(dx * SCALE)
            py = y0_px - int(dy * SCALE)  # minus because screen Y downwards
            pygame.draw.circle(screen, (255, 0, 0), (px, py), 5)

    pygame.display.flip()
    clock.tick(60)

# ───────────────────────────────────────────────────────────
# Cleanup
# ───────────────────────────────────────────────────────────
client.loop_stop()
client.disconnect()
pygame.quit()
