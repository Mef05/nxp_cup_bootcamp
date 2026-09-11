# This file is a Python translation of source/main.c - keep in sync!
"""
controller.py - NXP Cup control loop, line-by-line translation of main.c.

The NXPController.update() method mirrors the control loop body in main.c
exactly:
  - Vector classification (left/right by bot_x vs IMAGE_CENTER_X)
  - MIN_DY horizontal-vector filter
  - MIN_BOT_Y proximity filter
  - center_bot / center_top with TRACK_WIDTH_PX one-side fallback
  - LOOKAHEAD_FACTOR blending
  - proximity scaling with MIN_STEER_SCALE
  - error = steer_scale * (WEIGHT_CTE * cte + WEIGHT_HEADING * heading)
  - PD: steer_cmd = STEER_KP*error + STEER_KP_Q*error*|error| + KD*(error-last_error)
  - Clamp to [-100, 100]
  - STEERING_ALPHA exponential smoothing
  - frames_lost logic: coast (x0.5 speed) for 1-5 frames, reverse at -85 for >5
  - Electronic differential: DIFFERENTIAL_FACTOR reduces inner-wheel speed

Public API:
    NXPController.update(vectors) -> (steer_output, speed_L, speed_R, debug_dict)
    NXPController.reset()
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

from config import (
    DIFFERENTIAL_FACTOR,
    HEADING_FACTOR,
    IMAGE_CENTER_X,
    LOOKAHEAD_FACTOR,
    MIN_BOT_Y,
    MIN_DY,
    MIN_STEER_SCALE,
    SPEED_LEFT,
    SPEED_RIGHT,
    STEER_KD,
    STEER_KP,
    STEER_KP_Q,
    STEERING_ALPHA,
    TRACK_WIDTH_PX,
)

# Pixy2 image height (hard-coded in main.c as `const float IMAGE_H = 51.0f`)
_IMAGE_H: float = 51.0

# Weights on CTE and heading components (from main.c variable declarations)
_WEIGHT_CTE: float = 1.0           # `const float WEIGHT_CTE = 1.0f`
_WEIGHT_HEADING: float = HEADING_FACTOR  # `const float WEIGHT_HEADING = HEADING_FACTOR`


class NXPController:
    """
    Python mirror of the main.c control loop body.

    State variables that persist across frames (matching main.c):
        last_error          : float   = 0.0
        frames_lost         : int     = 0
        current_steer       : float   = 0.0
        line_detected_once  : bool    = False
        frame_count         : int     = 0
    """

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        # main.c variable initialisations (lines 41-45)
        self.last_error: float = 0.0
        self.last_valid_error: float = 0.0
        self.frames_lost: int = 0
        self.current_steer: float = 0.0
        self.line_detected_once: bool = False
        self.frame_count: int = 0

    # ------------------------------------------------------------------
    # Main update - mirrors the while(1) loop body in main.c exactly
    # ------------------------------------------------------------------

    def update(
        self,
        vectors: List[Tuple[int, int, int, int]],
    ) -> Tuple[float, int, int, Dict]:
        """
        Process one camera frame.

        Args:
            vectors : list of (x0, y0, x1, y1) raw Pixy2 vectors.
                      Matches the `uint16_t vectors[MAX_VECTORS * 4]` array
                      in main.c, packed as x0=i*4+0, y0=i*4+1, x1=i*4+2, y1=i*4+3.

        Returns:
            steer_output : float in [-100, 100] - final smoothed steer command
            speed_L      : int   - left motor speed (same sign convention as HbridgeSpeed)
            speed_R      : int   - right motor speed
            debug        : dict  - diagnostic values for logging / plotting
        """
        # ------------------------------------------------------------------
        # frame_count++ (main.c line 59)
        # ------------------------------------------------------------------
        self.frame_count += 1
        num_vectors = len(vectors)

        # ------------------------------------------------------------------
        # Error accumulator reset each frame (main.c line 72)
        # ------------------------------------------------------------------
        error: float = 0.0

        # Debug payload (populated below)
        debug: Dict = {
            "have_left": False,
            "have_right": False,
            "center_bot": IMAGE_CENTER_X,
            "center_top": IMAGE_CENTER_X,
            "center_bot_y": MIN_BOT_Y,
            "cte": 0.0,
            "heading": 0.0,
            "proximity": 0.0,
            "steer_scale": MIN_STEER_SCALE,
            "error": 0.0,
            "steer_cmd": 0.0,
            "current_steer": 0.0,
            "speed_L": 0,
            "speed_R": 0,
            "frames_lost": self.frames_lost,
        }

        # ------------------------------------------------------------------
        # if (num_vectors > 0) { ... }   (main.c line 74)
        # ------------------------------------------------------------------
        if num_vectors > 0:
            self.line_detected_once = True  # main.c line 75

            # --------------------------------------------------------------
            # Vector classification - left / right   (main.c lines 87-142)
            # --------------------------------------------------------------
            have_left: bool = False
            have_right: bool = False
            left_bot: float = 0.0
            left_top: float = 0.0
            left_bot_y: float = 0.0
            right_bot: float = 0.0
            right_top: float = 0.0
            right_bot_y: float = 0.0
            best_left_dist: float = 1000.0
            best_right_dist: float = 1000.0

            for i in range(num_vectors):
                # main.c lines 94-97: unpack vector
                vx0 = float(vectors[i][0])
                vy0 = float(vectors[i][1])
                vx1 = float(vectors[i][2])
                vy1 = float(vectors[i][3])

                # main.c lines 100-104: MIN_DY filter (ignore near-horizontal)
                dy = vy1 - vy0
                abs_dy = abs(dy)

                # 1. Filter small vertical span (main.c line 103)
                if abs_dy < MIN_DY:
                    continue

                # main.c lines 108-115: determine bottom (high Y) and top point
                if vy0 > vy1:
                    bot_x = vx0; bot_y = vy0; top_x = vx1
                else:
                    bot_x = vx1; bot_y = vy1; top_x = vx0

                # main.c lines 120-121: MIN_BOT_Y filter
                if bot_y < MIN_BOT_Y:
                    continue

                # main.c lines 123-141: classify left / right
                if bot_x < IMAGE_CENTER_X:
                    dist = IMAGE_CENTER_X - bot_x
                    if dist < best_left_dist:
                        best_left_dist = dist
                        left_bot = bot_x
                        left_bot_y = bot_y
                        left_top = top_x
                        have_left = True
                else:
                    dist = bot_x - IMAGE_CENTER_X
                    if dist < best_right_dist:
                        best_right_dist = dist
                        right_bot = bot_x
                        right_bot_y = bot_y
                        right_top = top_x
                        have_right = True

            # --------------------------------------------------------------
            # center_bot / center_top calculation  (main.c lines 144-169)
            # --------------------------------------------------------------
            center_bot: float
            center_top: float
            center_bot_y: float

            if have_left and have_right:
                # Both boundaries visible
                center_bot = (left_bot + right_bot) * 0.5
                center_top = (left_top + right_top) * 0.5
                center_bot_y = (left_bot_y + right_bot_y) * 0.5
                self.frames_lost = 0
            elif have_left:
                # Only left boundary - estimate centre using TRACK_WIDTH_PX
                center_bot = left_bot + (TRACK_WIDTH_PX * 0.5)
                center_top = left_top + (TRACK_WIDTH_PX * 0.5)
                center_bot_y = left_bot_y
                self.frames_lost = 0
            elif have_right:
                # Only right boundary - estimate centre using TRACK_WIDTH_PX
                center_bot = right_bot - (TRACK_WIDTH_PX * 0.5)
                center_top = right_top - (TRACK_WIDTH_PX * 0.5)
                center_bot_y = right_bot_y
                self.frames_lost = 0
            else:
                # Vectors present but none passed filters
                self.frames_lost += 1
                center_bot = IMAGE_CENTER_X   # dummy (main.c line 166)
                center_top = IMAGE_CENTER_X   # dummy
                center_bot_y = MIN_BOT_Y      # dummy

            # --------------------------------------------------------------
            # Error calculation  (main.c lines 171-198)
            # --------------------------------------------------------------
            if self.frames_lost == 0:
                # Lookahead interpolation  (main.c line 173)
                target_x = (
                    center_bot * (1.0 - LOOKAHEAD_FACTOR)
                    + center_top * LOOKAHEAD_FACTOR
                )

                cte = target_x - IMAGE_CENTER_X          # main.c line 175
                heading = center_top - center_bot        # main.c line 176

                # Proximity scaling  (main.c lines 183-187)
                proximity = (center_bot_y - MIN_BOT_Y) / (_IMAGE_H - MIN_BOT_Y)
                if proximity < 0.0:
                    proximity = 0.0
                if proximity > 1.0:
                    proximity = 1.0
                steer_scale = MIN_STEER_SCALE + (1.0 - MIN_STEER_SCALE) * proximity

                # Combined error  (main.c line 189)
                # Combined error  (main.c line 189)
                error = steer_scale * ((_WEIGHT_CTE * cte) + (_WEIGHT_HEADING * heading))
                self.last_valid_error = error

                # Populate debug
                debug["have_left"] = have_left
                debug["have_right"] = have_right
                debug["center_bot"] = center_bot
                debug["center_top"] = center_top
                debug["center_bot_y"] = center_bot_y
                debug["cte"] = cte
                debug["heading"] = heading
                debug["proximity"] = proximity
                debug["steer_scale"] = steer_scale
                debug["error"] = error
            else:
                error = self.last_valid_error

        else:
            # num_vectors == 0  (main.c line 200)
            self.frames_lost += 1
            error = self.last_valid_error

        # ------------------------------------------------------------------
        # PD controller  (main.c lines 207-209)
        # ------------------------------------------------------------------
        abs_error = -error if error < 0.0 else error
        steer_cmd: float = (
            (STEER_KP * error)
            + (STEER_KP_Q * error * abs_error)
            + (STEER_KD * (error - self.last_error))
        )
        self.last_error = error  # main.c line 209

        # ------------------------------------------------------------------
        # Clamp  (main.c lines 211-214)
        # ------------------------------------------------------------------
        if steer_cmd > 100.0:
            steer_cmd = 100.0
        if steer_cmd < -100.0:
            steer_cmd = -100.0

        # ------------------------------------------------------------------
        # Exponential smoothing  (main.c lines 217-218)
        # ------------------------------------------------------------------
        self.current_steer = (
            STEERING_ALPHA * self.current_steer
            + (1.0 - STEERING_ALPHA) * steer_cmd
        )

        debug["steer_cmd"] = steer_cmd
        debug["current_steer"] = self.current_steer
        debug["frames_lost"] = self.frames_lost

        # ------------------------------------------------------------------
        # Speed + failsafe  (main.c lines 221-259)
        # ------------------------------------------------------------------
        steer_output = self.current_steer

        if not self.line_detected_once:
            # main.c lines 221-227: wait for first line detection
            speed_L = 0
            speed_R = 0
            steer_output = 0.0
            self.current_steer = 0.0

        elif self.frames_lost > 5:
            # main.c lines 228-231: reverse at -85 to search for the line
            speed_L = -85
            speed_R = -85

        elif self.frames_lost > 0:
            # main.c lines 232-238: coast at 50% speed, keep last steer
            speed_L = int(float(config.SPEED_MAX) * 0.5)
            speed_R = int(float(config.SPEED_MAX) * 0.5)

        else:
            # main.c lines 239-258: normal driving with dynamic speed and diff
            abs_steer = abs(self.current_steer)
            brake_factor = abs_steer / config.BRAKE_STEER_THRESHOLD
            if brake_factor > 1.0:
                brake_factor = 1.0
            
            base_speed = config.SPEED_MAX - int((config.SPEED_MAX - config.SPEED_MIN) * brake_factor)
            speed_L = base_speed
            speed_R = base_speed

            # Electronic differential  (main.c lines 247-253)
            if self.current_steer > 0.0:
                # Turning right: reduce right (inner) wheel speed
                diff_factor = 1.0 - (self.current_steer * 0.01 * DIFFERENTIAL_FACTOR)
                speed_R = int(float(base_speed) * diff_factor)
            elif self.current_steer < 0.0:
                # Turning left: reduce left (inner) wheel speed
                diff_factor = 1.0 - (-self.current_steer * 0.01 * DIFFERENTIAL_FACTOR)
                speed_L = int(float(base_speed) * diff_factor)

        debug["speed_L"] = speed_L
        debug["speed_R"] = speed_R

        return steer_output, speed_L, speed_R, debug
