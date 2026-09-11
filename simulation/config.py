"""
config.py - Python mirror of Config.h and servo.h constants.

All values in this file must stay in sync with:
  include/Config.h
  include/servo.h (SERVO_CENTER_OFFSET)

Physical/camera constants marked ASSUMED are estimates for simulation
purposes and are NOT from the hardware source files.
"""

# ---------------------------------------------------------------------------
# From include/Config.h - keep in sync!
# ---------------------------------------------------------------------------

# Steering PWM proportional factors (used for servo mapping in hardware)
STEERING_P_RIGHT: int = 50
STEERING_P_LEFT: int = 50

# Physical steering limits (logical -100..+100 maps to these degrees)
STEERING_LIMIT_RIGHT: int = 80   # degrees, positive = right
STEERING_LIMIT_LEFT: int = -80   # degrees, negative = left

# Servo mechanical center offset (maps to SERVO_CENTER_OFFSET in servo.h)
STEERING_OFFSET: int = 13  # degrees; also used as SERVO_CENTER_OFFSET

# Wheel speed duty cycles (0-100 scale, as sent to HbridgeSpeed)
SPEED_RIGHT: int = 90
SPEED_LEFT: int = 90

# Electronic differential: fraction of inner-wheel speed reduction per % steer
# 0.0 = no diff, 1.0 = stops inner wheel completely on max steer
DIFFERENTIAL_FACTOR: float = 0.3

# Lookahead blending factor between bottom (close) and top (far) of detected line
# 0.0 = react to closest point only (turns LATE)
# 1.0 = react to furthest point only (turns EARLY)
LOOKAHEAD_FACTOR: float = 0.2

# Heading anticipation weight added to the error signal
# 0.0 = only lateral position error (CTE) - recommended
# >0  = also weight line angle (can cause early turns due to perspective)
HEADING_FACTOR: float = 0.0

# PD steering controller gains
STEER_KP: float = 4.0     # proportional gain
STEER_KP_Q: float = 0.15  # quadratic gain (extra aggression at large errors)
STEER_KD: float = 0.5     # derivative gain

# Exponential smoothing on the steering output
# 0.0 = instant response, 0.9 = very sluggish
STEERING_ALPHA: float = 0.5

# Minimum Y coordinate (in Pixy2 pixels) that a vector's bottom point must
# reach before it is considered for steering.
# Pixy2: Y=0 = top of image (far), Y=51 = bottom (close to car).
# Higher value -> car reacts only to very close lines (turns LATER).
# Lower value  -> car reacts to distant lines (turns EARLIER).
MIN_BOT_Y: float = 30.0

# Minimum steering scale applied when a line is at maximum distance (MIN_BOT_Y).
# 0.0 = no steering for distant curves
# 1.0 = full steering regardless of distance (disables proximity scaling)
MIN_STEER_SCALE: float = 0.2

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

# Minimum Y-span of a vector for it to pass the "not too horizontal" filter
MIN_DY: float = 8.0

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
