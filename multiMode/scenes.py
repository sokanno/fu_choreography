# file: scenes.py
from vpython import canvas, box, color, vector
import config

# ---------------------------------
# 3Dビュー（scene3d）
# ---------------------------------
scene3d = canvas(
    width=600,
    height=400,
    background=color.gray(config.bg3d_brightness),
    caption="",
    align='right'
)
scene3d.up = vector(0, 0, 1)
scene3d.forward = vector(-1, -1, -0.2)
scene3d.userspin = False

# ---------------------------------
# 上からビュー（scene2d）
# ---------------------------------
scene2d = canvas(
    width=600,
    height=400,
    background=color.gray(config.bg2d_brightness),
    caption="",
    align='right'
)
scene2d.camera.projection = "orthographic"
scene2d.up = vector(1, 0, 0)
scene2d.camera.axis = vector(0, 0, -1)
scene2d.lights = []

# ---------------------------------
# 床 (floor) と 天井 (ceiling)
# ---------------------------------
floor = box(
    canvas=scene3d,
    pos=vector(0, 0, -0.01),
    size=vector(6.4, 7.6, 0.02),
    color=color.gray(0.5),
    opacity=1.0
)

ceiling = box(
    canvas=scene3d,
    pos=vector(0, 0, config.maxZ + 0.01),
    size=vector(6.4, 7.6, 0.02),
    color=color.gray(0.5),
    opacity=1.0,
    shininess=0
)
