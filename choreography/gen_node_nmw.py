#!/usr/bin/env python3
"""New Media Week (25 robots) layout -> node_nmw.csv + Max pan table.

Drawing frame (hangingPlan.pdf / PRODUKCJA p.47): 5 lines of 5.1 m, 1 m apart,
5 robots per line at 1 m pitch, odd lines shifted 0.5 m
(2026-09-28: 現物に合わせて位相を反転。観客から見て左端/真ん中/右端の列が手前、
その左右隣の列が 50 cm 奥)。 x = along the lines
(+x = right of the drawing), y = across (+y = top of the drawing). Centered.

FRONT = which side of the drawing the audience/operator looks from.
IDs are numbered in the viewer's frame: row 1 = nearest 5 robots, left -> right,
row 2 = next row back, ... so ID 1 = 手前左, ID 5 = 手前右, ID 25 = 奥右.

Outputs
  node_nmw.csv        id,x,y in the drawing frame (main.py / gen_robots_json.py)
  nmw_pan_table.txt   Max coll: "id, lr fb;"  lr 0=左..1=右, fb 0=手前..1=奥
Camera in main.py: cameraAngle must match FRONT (see FRONT_CAMERA_ANGLE).
"""
import math

FRONT = "right"   # "right" | "left" | "top" | "bottom"  (side of the drawing)

# unit vectors in the drawing frame: n = toward the viewer (near), l = viewer's left
FRONT_VECS = {
    "right":  ((1, 0), (0, -1)),
    "left":   ((-1, 0), (0, 1)),
    "bottom": ((0, -1), (1, 0)),
    "top":    ((0, 1), (-1, 0)),
}
FRONT_CAMERA_ANGLE = {"right": "0.0", "left": "math.pi", "bottom": "-math.pi/2", "top": "math.pi/2"}


def drawing_positions():
    pts = []
    xc = (0.3 + 4.8) / 2
    for li in range(5):                      # line 1 (top) .. line 5 (bottom)
        y = 2.0 - li
        x0 = 0.8 if li % 2 == 0 else 0.3
        for k in range(5):
            pts.append((round(x0 + k - xc, 2), round(y, 2), li + 1))
    return pts


def main():
    n, l = FRONT_VECS[FRONT]
    pts = drawing_positions()
    near = [p[0] * n[0] + p[1] * n[1] for p in pts]
    left = [p[0] * l[0] + p[1] * l[1] for p in pts]
    near_max, near_min = max(near), min(near)
    left_max, left_min = max(left), min(left)
    rows = []
    for p, nv, lv in zip(pts, near, left):
        row = int((near_max - nv) + 0.25)            # 0 = nearest row (0.5 m stagger folds in)
        fb = (near_max - nv) / (near_max - near_min)  # 0 手前 .. 1 奥
        lr = (left_max - lv) / (left_max - left_min)  # 0 左 .. 1 右
        rows.append((row, lr, fb, p))
    rows.sort(key=lambda r: (r[0], r[1]))

    with open("node_nmw.csv", "w") as f:
        f.write("id,x,y\n")
        for i, (row, lr, fb, p) in enumerate(rows, 1):
            f.write(f"{i},{p[0]:.2f},{p[1]:.2f}\n")
    with open("nmw_pan_table.txt", "w") as f:
        for i, (row, lr, fb, p) in enumerate(rows, 1):
            f.write(f"{i}, {lr:.3f} {fb:.3f};\n")

    print(f"FRONT = {FRONT} (viewer at the {FRONT} side of the drawing); main.py cameraAngle = {FRONT_CAMERA_ANGLE[FRONT]}")
    print("speakers: 1 手前左  2 手前右  3 奥左  4 奥右")
    print(" id  row  lr(0左..1右)  fb(0手前..1奥)  drawing x,y   line")
    for i, (row, lr, fb, p) in enumerate(rows, 1):
        print(f" {i:2d}   {row+1}    {lr:5.3f}         {fb:5.3f}        {p[0]:+.2f},{p[1]:+.2f}   L{p[2]}")


if __name__ == "__main__":
    main()
