#!/usr/bin/env python3
"""
Constants shared across the trajectory_driver package.
"""

import os
import json

# Maximum number of command slots in the Arduino's onboard motion buffer.
ARDUINO_BUF_MAX = 10

# Serial baud rate used to talk to the Arduino motor controller.
SERIAL_BAUD = 115200

# Step-per-degree scale factors for each joint.
#   Index 0-2: Shoulder pan/lift and elbow joints
#   Index 3-5: Wrist joints
SCALES = (
    341.3333, 341.3333, 341.3333,
    11.333,   11.333,   11.333,
)

# Joint names in controller order (must match the URDF).
JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

def _get_serial_port():
    config_file = os.path.join(os.path.dirname(__file__), 'serial_config.json')
    if os.path.exists(config_file):
        try:
            with open(config_file, 'r') as f:
                data = json.load(f)
                return data.get('serial_port', '/dev/ttyUSB0')
        except Exception:
            pass
    return '/dev/ttyUSB0'

# Default serial device path for the Arduino controller.
SERIAL_PORT = _get_serial_port()
