"""Per-process vehicle identity and ground registration; no motion commands."""
import copy
import json
import os
from pathlib import Path

FLEET = json.loads(Path('/config/fleet.json').read_text())
INDEX = int(os.environ.get('RACER_VEHICLE_INDEX', '0'))
VEHICLE = FLEET['vehicles'][INDEX]
NS = VEHICLE['namespace']
PREFIX = '/racer_multi/' + NS
CONFIG = Path('/config') / NS
OUT = Path(os.environ.get('RACER_OUTPUT', '/result'))

def translation(anchor, surveyed_ground):
    return [float(b)-float(a) for a,b in zip(anchor, surveyed_ground)]

def to_local_message(message, offset):
    """Translate all control points; preserve knots, absolute time and yaw."""
    out=copy.deepcopy(message)
    for p in out.pos_pts:
        p.x-=offset[0];p.y-=offset[1];p.z-=offset[2]
    return out
