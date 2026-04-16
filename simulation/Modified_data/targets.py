"""
Modified_data/targets.py
=========================
Ground target initialisation for the multi-target imaging simulation.

Defines lat / lon / alt for each of the five GroundTargetSimObject
instances (target1 … target5) and registers them with the vehicle
so their ECI positions are forwarded to the FSW bus every tick.

Targeting algorithm parameters are also set here so every RUN
scenario shares the same defaults; override them in the individual
input.py files if needed.

Usage (in any input.py):
    exec(open("Modified_data/targets.py", "r").read())
"""

import math

# ---------------------------------------------------------------------------
# Target definitions  –  (name, lat_deg, lon_deg, alt_m)
# ---------------------------------------------------------------------------
_target_defs = [
    ("Eastern USA",         30.0,  -70.0,  0.0),
    ("Cape Town, S. Africa",-33.9,   18.4,  0.0),
    ("Tokyo, Japan",         35.7,  139.7,  0.0),
    ("London, UK",           51.5,   -0.1,  0.0),
    ("Sao Paulo, Brazil",   -23.5,  -46.6,  0.0),
]

_sim_targets = [target1, target2, target3, target4, target5]

for _i, (_name, _lat, _lon, _alt) in enumerate(_target_defs):
    _t = _sim_targets[_i]
    _t.lat = math.radians(_lat)
    _t.lon = math.radians(_lon)
    _t.alt = _alt

# ---------------------------------------------------------------------------
# Register targets with the vehicle so their ECI positions are fed into
# the FSW SIM-to-FSW bus each dynamics tick.
# ---------------------------------------------------------------------------
vehicle.add_target(target1)
vehicle.add_target(target2)
vehicle.add_target(target3)
vehicle.add_target(target4)
vehicle.add_target(target5)

# ---------------------------------------------------------------------------
# Targeting algorithm defaults (tunable per-run after this exec())
# ---------------------------------------------------------------------------
vehicle.fsw.targeting.image_dwell_time_s = 1.0  # s – track time to count as imaged
vehicle.fsw.targeting.min_elevation_deg  =  5.0  # deg – min el to consider target visible
