# file: ui.py
from vpython import canvas, menu, wtext, slider, color
import config
import scenes

# ---------------------------------
# UI キャンバス
# ---------------------------------
ui = canvas(
    width=0,
    height=0,
    background=color.white,
    caption="",
    align='right'
)

# モード定義
modes = ["流れモード", "天上天下唯我独尊モード"]
slider_groups = {mode: [] for mode in modes}

# モード切替ハンドラ
def on_mode_select(m):
    # 選択中のモード名を取得
    select = m.choices[m.index]
    for mode, widgets in slider_groups.items():
        for w in widgets:
            w.visible = (mode == select)

mode_menu = menu(
    choices=modes,
    index=0,
    bind=on_mode_select,
    canvas=ui
)
ui.append_to_caption("\n\n")

# スライダー追加ユーティリティ
def add_mode_slider(mode, cnv, label, mn, mx, val, fmt, setter):
    txt0 = wtext(text=f"{label}: ", canvas=cnv)
    txt1 = wtext(text=f"{fmt.format(val)}  ", canvas=cnv)
    def on_slide(s, txt=txt1, fmt=fmt, st=setter):
        st(s.value)
        txt.text = f"{fmt.format(s.value)}  "
    sld = slider(
        min=mn,
        max=mx,
        value=val,
        length=200,
        bind=on_slide,
        canvas=cnv
    )
    cnv.append_to_caption("<br>")
    slider_groups[mode].extend([txt0, txt1, sld])
    return sld

# ─── 流れモード用スライダー ─────────────────────────────
ui.append_to_caption("<b>流れモード</b><br><br>")
add_mode_slider(
    "流れモード", ui, "Separation", 0, 2,
    config.separationFactor, "{:.2f}",
    lambda v: setattr(config, 'separationFactor', v)
)
add_mode_slider(
    "流れモード", ui, "Cohesion", 0, 2,
    config.cohesionFactor, "{:.2f}",
    lambda v: setattr(config, 'cohesionFactor', v)
)
add_mode_slider(
    "流れモード", ui, "Noise Scale", 0.01, 0.5,
    config.noiseScale, "{:.2f}",
    lambda v: setattr(config, 'noiseScale', v)
)
add_mode_slider(
    "流れモード", ui, "Wave Scale", 0.01, 1.0,
    config.waveScale, "{:.2f}",
    lambda v: setattr(config, 'waveScale', v)
)
add_mode_slider(
    "流れモード", ui, "Wave Strength", 0, 1,
    config.waveStrength, "{:.2f}",
    lambda v: setattr(config, 'waveStrength', v)
)
add_mode_slider(
    "流れモード", ui, "Noise Speed", 0, 0.1,
    config.noiseSpeed, "{:.3f}",
    lambda v: setattr(config, 'noiseSpeed', v)
)
ui.append_to_caption("<br><br>")

# ─── Appearance Control ──────────────────────────────
ui.append_to_caption("<b>Appearance Control</b><br><br>")
add_mode_slider(
    "流れモード", ui, "BG3D Bright", 0, 1,
    config.bg3d_brightness, "{:.2f}",
    lambda v: (
        setattr(config, 'bg3d_brightness', v),
        setattr(scenes.scene3d, 'background', color.gray(v))
    )
)
add_mode_slider(
    "流れモード", ui, "BG2D Bright", 0, 1,
    config.bg2d_brightness, "{:.2f}",
    lambda v: (
        setattr(config, 'bg2d_brightness', v),
        setattr(scenes.scene2d, 'background', color.gray(v))
    )
)
add_mode_slider(
    "流れモード", ui, "LED Amplitude", 0, 2,
    config.led_amp, "{:.2f}",
    lambda v: setattr(config, 'led_amp', v)
)
ui.append_to_caption("<br>")

# ─── Wave Hue Control ───────────────────────────────
ui.append_to_caption("<b>Wave Hue Control</b><br><br>")
add_mode_slider(
    "流れモード", ui, "Hue Offset", 0.0, 1.0,
    config.base_hue, "{:.2f}",
    lambda v: setattr(config, 'base_hue', v)
)
add_mode_slider(
    "流れモード", ui, "Hue Span", 0.0, 1.0,
    config.hue_span, "{:.2f}",
    lambda v: setattr(config, 'hue_span', v)
)
ui.append_to_caption("<br><br>")

# ─── 天上天下唯我独尊モード用スライダー ──────────────────
ui.append_to_caption("<b>天上天下唯我独尊モード</b><br><br>")
add_mode_slider(
    "天上天下唯我独尊モード", ui,
    "Group A Count", 1, config.ROWS*config.COLS-1,
    config.groupA_count, "{:.0f}",
    lambda v: setattr(config, 'groupA_count', int(v))
)
add_mode_slider(
    "天上天下唯我独尊モード", ui,
    "Sine Amplitude", 0.0, (config.maxZ-config.minZ)/2,
    config.target_sine_amp, "{:.2f}",
    lambda v: setattr(config, 'target_sine_amp', v)
)
add_mode_slider(
    "天上天下唯我独尊モード", ui,
    "Sine Frequency", 0.0, 2.0,
    config.target_sine_freq, "{:.2f}",
    lambda v: setattr(config, 'target_sine_freq', v)
)
add_mode_slider(
    "天上天下唯我独尊モード", ui,
    "Color Speed", 0.0, 2.0,
    config.color_speed, "{:.2f}",
    lambda v: setattr(config, 'color_speed', v)
)
ui.append_to_caption("<br>")
