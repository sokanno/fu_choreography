"""OSC client for the FU rain engine (SuperCollider, rain_engine.scd).

Thin protocol wrapper — this is the only thing main.py needs to import to
drive the rain scene. Field math (rain cells, wind, anomaly scheduling)
stays on the caller's side per the spec.

Protocol (see rain_engine.scd header):
    /rain/phases    29 x float   (30 Hz, bundled)
    /rain/rates     29 x float   (30 Hz, bundled)
    /rain/heat      29 x int     (30 Hz, bundled)
    /rain/global    depth, material, wind_speed, wind_dir, master_gain_db
    /rain/materials 29 x float   per-robot material, -1 = follow global
    /rain/heatmode  mode (0=umbrella, 1=material), heat_material
    /rain/scene     0 = fade out, 1 = fade in
"""

import json
import os

from pythonosc import udp_client
from pythonosc.osc_bundle_builder import IMMEDIATELY, OscBundleBuilder
from pythonosc.osc_message_builder import OscMessageBuilder

_HERE = os.path.dirname(os.path.abspath(__file__))


def load_config(path=None):
    with open(path or os.path.join(_HERE, "config.json")) as f:
        return json.load(f)


def load_robots(path=None):
    """Returns list of dicts with id and normalized x, y in [0,1]^2."""
    with open(path or os.path.join(_HERE, "robots.json")) as f:
        return json.load(f)["robots"]


class RainClient:
    def __init__(self, host=None, port=None, config_path=None):
        cfg = load_config(config_path)["osc"]
        self.client = udp_client.SimpleUDPClient(
            host or cfg["sc_host"], port or cfg["sc_port"]
        )

    def _msg(self, address, args):
        b = OscMessageBuilder(address=address)
        for a in args:
            b.add_arg(a)
        return b.build()

    def send_frame(self, phases, rates, heat):
        """High-rate state (30 Hz), sent as one OSC bundle.

        phases: 29 floats [0, 2pi) / rates: 29 floats drops/sec / heat: 29 ints 0/1
        """
        bundle = OscBundleBuilder(IMMEDIATELY)
        bundle.add_content(self._msg("/rain/phases", [float(p) for p in phases]))
        bundle.add_content(self._msg("/rain/rates", [float(r) for r in rates]))
        bundle.add_content(self._msg("/rain/heat", [int(h) for h in heat]))
        self.client.send(bundle.build())

    def send_global(self, depth, material, wind_speed, wind_dir, master_gain_db=0.0):
        self.client.send_message(
            "/rain/global",
            [float(depth), float(material), float(wind_speed),
             float(wind_dir), float(master_gain_db)],
        )

    def send_materials(self, materials):
        """Per-robot material override (29 floats, -1 = follow global)."""
        self.client.send_message("/rain/materials", [float(m) for m in materials])

    def set_heat_mode(self, mode, heat_material=3.0):
        """mode: 0 = umbrella (rain stops above visitor), 1 = material switch."""
        self.client.send_message("/rain/heatmode", [int(mode), float(heat_material)])

    def scene(self, on):
        self.client.send_message("/rain/scene", [1 if on else 0])
