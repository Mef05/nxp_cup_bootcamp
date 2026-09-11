# NXP Cup Race Car Simulation

A Python simulation that mirrors the C control logic from `source/main.c` and `include/Config.h`.

---

## Installation

```bash
pip install -r requirements.txt
```

---

## Running

```bash
# From the simulation/ directory:
python simulate.py

# Or from the project root:
python simulation/simulate.py
```

### Keyboard controls

| Key | Action |
|-----|--------|
| `SPACE` | Pause / Resume |
| `R` | Reset car to start |
| `+` / `=` | Increase speed multiplier |
| `-` | Decrease speed multiplier |
| `Q` / `Esc` | Quit |

### Non-interactive / headless test

```bash
python simulate.py --test 300    # run 300 frames without opening a window
```

---

## How to change parameters

All tuning parameters live in **`config.py`**, which is a direct mirror of `include/Config.h`.

| Python constant | C macro | Effect |
|-----------------|---------|--------|
| `STEER_KP` | `STEER_KP` | Proportional steering gain |
| `STEER_KP_Q` | `STEER_KP_Q` | Quadratic steering gain (aggression) |
| `STEER_KD` | `STEER_KD` | Derivative gain (dampening) |
| `LOOKAHEAD_FACTOR` | `LOOKAHEAD_FACTOR` | 0=react to close line, 1=react to far |
| `STEERING_ALPHA` | `STEERING_ALPHA` | Smoothing (0=instant, 0.9=sluggish) |
| `MIN_BOT_Y` | `MIN_BOT_Y` | Minimum Y to accept a vector |
| `MIN_STEER_SCALE` | `MIN_STEER_SCALE` | Steer scale at maximum distance |
| `DIFFERENTIAL_FACTOR` | `DIFFERENTIAL_FACTOR` | Electronic differential strength |

**After editing `config.py`, just re-run `simulate.py` - no recompilation needed.**

---

## File overview

| File | Mirrors C file | Description |
|------|---------------|-------------|
| `config.py` | `include/Config.h` | All constants |
| `track.py` | - | Track geometry (straight + curve segments) |
| `bicycle_model.py` | `source/servo.c` + `source/hbridge.c` | Kinematic car model |
| `camera.py` | `source/pixy.c` (hypothetical) | Perspective-projected Pixy2 simulation |
| `controller.py` | `source/main.c` | **Exact** C control loop translation |
| `simulate.py` | - | Main runner + matplotlib visualisation |

---

## Known limitations vs real hardware

1. **Track geometry is synthetic.** The track is a hand-defined sequence of
   straights and curves. The real track will differ.

2. **Camera projection is simplified.** The Pixy2's actual CNN line detection
   does complex subpixel fitting. The simulation uses a simple endpoint
   projection and adds Gaussian noise only. Occlusion and lighting effects
   are not modelled.

3. **Bicycle model ignores dynamics.** Slip angle, tire friction, suspension,
   and acceleration lag are not modelled. The car responds instantly to
   steering and speed changes.

4. **No PID for speed.** The real car runs open-loop duty cycles. The
   simulation converts duty cycle to m/s linearly via `MAX_SPEED_MS`
   (ASSUMED constant).

5. **Physical constants are estimated.** `WHEELBASE`, `MAX_SPEED_MS`,
   `CAMERA_HEIGHT`, `CAMERA_ANGLE_DEG` are all marked ASSUMED in
   `config.py`. Measure your car and update these for accurate simulation.

6. **No I2C errors.** The `pixy_status != kStatus_Success` path in `main.c`
   is not simulated (the simulated camera always succeeds).

7. **Electronic differential uses average speed for bicycle model.** The
   real car has different L/R wheel speeds on curves; the bicycle model
   only uses the mean of `(speed_L + speed_R) / 2` for forward speed.
