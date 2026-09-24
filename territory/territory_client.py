"""OSC client for the FU territory engine (SuperCollider, territory_engine.scd).

Thin protocol wrapper — this is the only thing main.py needs to import to
drive the territory scene's sound. The simulation itself (opinions, gaze,
fatigue, axis shifts) stays on the caller's side.

Protocol (see territory_engine.scd header):
    /terr/op      29 x float  unwrapped opinion angle rad   (20 Hz, bundled)
    /terr/conv    29 x float  conviction 0..1               (20 Hz, bundled)
    /terr/fat     29 x float  fatigue 0..1                  (20 Hz, bundled)
    /terr/ht      29 x float  height 0..1 (1 = descended)   (20 Hz, bundled)
    /terr/tilt    29 x float  pitch tilt -1..1 (+ = down)   (20 Hz, bundled)
    /terr/om      29 x float  |yaw velocity| rad/s          (20 Hz, bundled)
    /terr/global  R, C, camps, mean_op_angle, master_gain_db  (5 Hz)
    /terr/tick    idx (0-based), vel 0..1, pitch-class int
    /terr/shift   1 = auto, 0 = manual
    /terr/scene   0 = fade out, 1 = fade in
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
    """Returns list of dicts with id, normalized x, y and world_x, world_y (m)."""
    with open(path or os.path.join(_HERE, "robots.json")) as f:
        return json.load(f)["robots"]


class TerritoryClient:
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

    def send_state(self, op, conv, fat, ht, tilt, om, face=None):
        """High-rate state (20 Hz), one OSC bundle. All args: N floats.
        face: 0..1 facing-the-audience factor (voice directivity)."""
        bundle = OscBundleBuilder(IMMEDIATELY)
        bundle.add_content(self._msg("/terr/op", [float(v) for v in op]))
        bundle.add_content(self._msg("/terr/conv", [float(v) for v in conv]))
        bundle.add_content(self._msg("/terr/fat", [float(v) for v in fat]))
        bundle.add_content(self._msg("/terr/ht", [float(v) for v in ht]))
        bundle.add_content(self._msg("/terr/tilt", [float(v) for v in tilt]))
        bundle.add_content(self._msg("/terr/om", [float(v) for v in om]))
        if face is not None:
            bundle.add_content(self._msg("/terr/face", [float(v) for v in face]))
        self.client.send(bundle.build())

    def send_global(self, r, c, camps, mean_op, master_gain_db=0.0):
        self.client.send_message(
            "/terr/global",
            [float(r), float(c), float(camps), float(mean_op), float(master_gain_db)],
        )

    def say(self, idx, dur, amp, op, conv, fat, ht, tilt):
        """One utterance: robot idx takes the floor for dur seconds."""
        self.client.send_message(
            "/terr/say",
            [int(idx), float(dur), float(amp), float(op),
             float(conv), float(fat), float(ht), float(tilt)],
        )

    def tick(self, idx, vel, pc):
        """One knock: robot idx (0-based) crossed a half-turn at velocity vel."""
        self.client.send_message("/terr/tick", [int(idx), float(vel), int(pc)])

    def shift(self, auto):
        self.client.send_message("/terr/shift", [1 if auto else 0])

    def melt(self):
        """Front-line melt gesture (soft hiss wash, no boom)."""
        self.client.send_message("/terr/melt", [1])

    def split(self, phase):
        """Schism gesture: phase 0 = fissure begins (creak), 1 = secession (crack)."""
        self.client.send_message("/terr/split", [int(phase)])

    def scene(self, on):
        self.client.send_message("/terr/scene", [1 if on else 0])
