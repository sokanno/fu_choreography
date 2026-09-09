#!/usr/bin/env python3
"""Generate rain/robots.json and territory/robots.json from a node csv.

    python3 gen_robots_json.py                 # choreography/node_nmw.csv (NMW 25台)
    python3 gen_robots_json.py node.csv        # 従来の29台配置

Output format: {"comment", "extent_m": {"x": [min,max], "y": [min,max]},
                "robots": [{"id", "x", "y", "world_x", "world_y"}, ...]}
(x, y) are normalized to [0,1]^2 over the extent; world_* are the csv meters.
The SC engines (rain_engine.scd / territory_engine.scd) pan each robot bilinearly
from (x, y): FL=(0,0) FR=(1,0) RL=(0,1) RR=(1,1).
"""
import csv
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
TARGETS = [os.path.join(ROOT, "rain", "robots.json"),
           os.path.join(ROOT, "territory", "robots.json")]


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else "node_nmw.csv"
    if not os.path.isabs(src):
        src = os.path.join(HERE, src)
    with open(src, newline="") as f:
        rows = [(int(r["id"]), float(r["x"]), float(r["y"])) for r in csv.DictReader(f)]
    xs = [r[1] for r in rows]
    ys = [r[2] for r in rows]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    robots = [{
        "id": rid,
        "x": round((x - x0) / (x1 - x0), 4),
        "y": round((y - y0) / (y1 - y0), 4),
        "world_x": x,
        "world_y": y,
    } for rid, x, y in rows]
    out = {
        "comment": (f"FU ceiling robots, generated from choreography/{os.path.basename(src)} "
                    "by gen_robots_json.py. (x,y) normalized to [0,1]^2; world_* are meters."),
        "extent_m": {"x": [x0, x1], "y": [y0, y1]},
        "robots": robots,
    }
    for t in TARGETS:
        with open(t, "w") as f:
            json.dump(out, f, indent=2)
        print(f"wrote {t}: {len(robots)} robots, extent x{out['extent_m']['x']} y{out['extent_m']['y']}")


if __name__ == "__main__":
    main()
