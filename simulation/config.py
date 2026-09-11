"""
config.py - Python mirror of Config.h and servo.h constants.

All values in this file must stay in sync with:
  include/Config.h
  include/servo.h (SERVO_CENTER_OFFSET)

Physical/camera constants marked ASSUMED are estimates for simulation
purposes and are NOT from the hardware source files.
"""

import os
import re

# ---------------------------------------------------------------------------
# Dynamically parse include/Config.h so simulation is always in sync!
# ---------------------------------------------------------------------------

def _parse_config_h():
    config_path = os.path.join(os.path.dirname(__file__), "..", "include", "Config.h")
    parsed = {}
    if os.path.exists(config_path):
        with open(config_path, "r") as f:
            for line in f:
                # Match `#define NAME VALUE` (handles floats with 'f', integers, etc.)
                match = re.match(r'^\s*#define\s+([A-Za-z0-9_]+)\s+([0-9\.\-]+)f?', line)
                if match:
                    name = match.group(1)
                    val_str = match.group(2)
                    # Convert to float or int
                    if '.' in val_str:
                        parsed[name] = float(val_str)
                    else:
                        parsed[name] = int(val_str)
    return parsed

_c_config = _parse_config_h()

# Steering PWM proportional factors (used for servo mapping in hardware)
STEERING_P_RIGHT: int = _c_config.get("STEERING_P_RIGHT", 50)
STEERING_P_LEFT: int = _c_config.get("STEERING_P_LEFT", 50)

# Physical steering limits (logical -100..+100 maps to these degrees)
STEERING_LIMIT_RIGHT: int = _c_config.get("STEERING_LIMIT_RIGHT", 80)
STEERING_LIMIT_LEFT: int = _c_config.get("STEERING_LIMIT_LEFT", -80)

# Servo mechanical center offset (maps to SERVO_CENTER_OFFSET in servo.h)
STEERING_OFFSET: int = 13  # degrees; also used as SERVO_CENTER_OFFSET

# Wheel speed duty cycles (0-100 scale, as sent to HbridgeSpeed)
SPEED_RIGHT: int = _c_config.get("SPEED_RIGHT", 90)
SPEED_LEFT: int = _c_config.get("SPEED_LEFT", 90)

# Electronic differential: fraction of inner-wheel speed reduction per % steer
DIFFERENTIAL_FACTOR: float = _c_config.get("DIFFERENTIAL_FACTOR", 0.3)

# Lookahead blending factor between bottom (close) and top (far) of detected line
LOOKAHEAD_FACTOR: float = _c_config.get("LOOKAHEAD_FACTOR", 0.5)

# Heading anticipation weight added to the error signal
HEADING_FACTOR: float = _c_config.get("HEADING_FACTOR", 0.0)

# PD steering controller gains
STEER_KP: float = _c_config.get("STEER_KP", 5.0)
STEER_KP_Q: float = _c_config.get("STEER_KP_Q", 0.25)
STEER_KD: float = _c_config.get("STEER_KD", 0.8)

# Exponential smoothing on the steering output
STEERING_ALPHA: float = _c_config.get("STEERING_ALPHA", 0.1)

# Minimum Y coordinate (in Pixy2 pixels) that a vector's bottom point must reach
MIN_BOT_Y: float = _c_config.get("MIN_BOT_Y", 10.0)

# Minimum vertical span in pixels
MIN_DY: float = _c_config.get("MIN_DY", 6.0)

# Minimum steering scale applied when a line is at maximum distance (MIN_BOT_Y).
MIN_STEER_SCALE: float = _c_config.get("MIN_STEER_SCALE", 0.4)

# ---------------------------------------------------------------------------
# From source/main.c - hard-coded constants in the control loop
# ---------------------------------------------------------------------------

# Maximum vectors the controller allocates
MAX_VECTORS: int = 10

# Pixy2 image dimensions (pixels)
PIXY_W: int = 79
PIXY_H: int = 51

# Image centre X used by the controller
IMAGE_CENTER_X: float = 39.0

# Approximate track width in pixels used when only one boundary is visible
TRACK_WIDTH_PX: float = 45.0

# Console print decimation (every Nth frame)
PRINT_EVERY_N: int = 30

# ---------------------------------------------------------------------------
# Physical / camera constants - ASSUMED / ESTIMATED for simulation
# ---------------------------------------------------------------------------

# Car wheelbase (front-to-rear axle distance) - ASSUMED
WHEELBASE: float = 0.18  # metres

# Maximum forward speed at 100% duty cycle - ASSUMED
MAX_SPEED_MS: float = 1.5  # metres per second

# Camera mounting height above ground - ASSUMED
CAMERA_HEIGHT: float = 0.18  # metres

# Camera tilt angle below horizontal - ASSUMED
CAMERA_ANGLE_DEG: float = 15.0  # degrees

# Pixy2 camera horizontal field of view - from Pixy2 datasheet
CAMERA_FOV_H_DEG: float = 75.0  # degrees

# Pixy2 camera vertical field of view - from Pixy2 datasheet
CAMERA_FOV_V_DEG: float = 47.0  # degrees

# NXP Cup standard track half-width - ASSUMED (full width 0.55 m)
TRACK_WIDTH: float = 0.55  # metres

# ---------------------------------------------------------------------------
# Simulation-only parameters (no C equivalent)
# ---------------------------------------------------------------------------

# Number of camera frames of processing delay to simulate.
# On the real MCXN947 board:
#   - Pixy2 I2C frame acquisition: ~16 ms (one full camera frame at ~60 fps)
#   - MCU PD controller + HbridgeSpeed update: ~1-2 ms
#   Total effective delay: ~1-2 camera frames.
# Set to 0 to disable latency simulation (ideal, zero-delay system).
PROCESSING_DELAY_FRAMES: int = 2
