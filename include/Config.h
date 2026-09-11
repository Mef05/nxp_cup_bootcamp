#ifndef CONFIG_H
#define CONFIG_H

// Steering control coefficients
#define STEERING_P_RIGHT 50U
#define STEERING_P_LEFT 50U

// Physical steering limits
#define STEERING_LIMIT_RIGHT 70
#define STEERING_LIMIT_LEFT -70

// Steering angle offset
#define STEERING_OFFSET 13

// Dynamic Speed Control
// Max speed used on straights
#define SPEED_MAX 100
// Min speed used in sharp curves
#define SPEED_MIN 75
// The steering command magnitude (0-100) at which speed drops to SPEED_MIN.
// Lower value = brakes earlier and harder before curves.
#define BRAKE_STEER_THRESHOLD 35.0f

// Electronic differential: fraction of inner-wheel speed reduction per % steer
// 0.0 = no diff, 1.0 = stops inner wheel completely on max steer
#define DIFFERENTIAL_FACTOR 0.05f

// Lookahead blending factor between bottom (close) and top (far) of detected
// line 0.0 = react to closest point only (turns LATE) 1.0 = react to furthest
// point only (turns EARLY)
#define LOOKAHEAD_FACTOR 0.60f

// Heading anticipation weight added to the error signal
// 0.0 = only lateral position error (CTE) - recommended
#define HEADING_FACTOR 0.0f

// Steering PD Controller Tuning
#define STEER_KP 3.5f
// Quadratic gain: adds extra steering proportional to CTE^2
// 0.0f = pure linear (no extra mid-curve aggression)
// 0.1f = moderate (recommended starting point)
// 0.3f = very aggressive mid-curve
#define STEER_KP_Q 0.10f
#define STEER_KD 0.7f

// Steering Smoothing (0.0f = no smoothing/instant, 0.9f = very slow/smooth)
#define STEERING_ALPHA 0.1f

// Minimum Y coordinate (in Pixy2 pixels) that a vector's bottom point must
// Pixy2: Y=0 = top of image (far), Y=51 = bottom (close to car).
// Higher value -> car reacts only to very close lines (turns LATER).
// Lower value  -> car reacts to distant lines (turns EARLIER).
#define MIN_BOT_Y 10.0f

// Minimum vertical span (dy) in pixels for a vector to be accepted.
// Filters out noise and horizontal crossing markers.
#define MIN_DY 6.0f

// Minimum steering scale applied when a curve is at maximum distance
// (MIN_BOT_Y) 0.0f = No steering at all for far curves 1.0f = Full steering
// regardless of distance
#define MIN_STEER_SCALE 0.4f

#endif
