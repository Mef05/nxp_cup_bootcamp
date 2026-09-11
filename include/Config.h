#ifndef CONFIG_H
#define CONFIG_H

// Steering control coefficients
#define STEERING_P_RIGHT 50U
#define STEERING_P_LEFT 50U

// Physical steering limits
#define STEERING_LIMIT_RIGHT 80
#define STEERING_LIMIT_LEFT -80

// Steering angle offset
#define STEERING_OFFSET 13

// Wheel speeds
#define SPEED_RIGHT 90
#define SPEED_LEFT 90

// Electronic Differential Factor
// 0.0f = No differential, 1.0f = Strong differential (stops inner wheel on tight turns)
#define DIFFERENTIAL_FACTOR 0.3f

// Lookahead factor for line tracking
// 0.0f = Only// Lookahead blending factor between bottom (close) and top (far) of detected line
// 0.0 = react to closest point only (turns LATE)
// 1.0 = react to furthest point only (turns EARLY)
#define LOOKAHEAD_FACTOR 0.5f

// Heading anticipation weight added to the error signal
// 0.0 = only lateral position error (CTE) - recommended
#define HEADING_FACTOR 0.0f

// Steering PD Controller Tuning
#define STEER_KP 5.0f
// Quadratic gain: adds extra steering proportional to CTE^2
// 0.0f = pure linear (no extra mid-curve aggression)
// 0.1f = moderate (recommended starting point)
// 0.3f = very aggressive mid-curve
#define STEER_KP_Q 0.25f
#define STEER_KD 0.8f

// Steering Smoothing (0.0f = no smoothing/instant, 0.9f = very slow/smooth)
#define STEERING_ALPHA 0.1f

// Minimum Y coordinate (in Pixy2 pixels) that a vector's bottom point must
// reach before it is considered for steering.
// Pixy2: Y=0 = top of image (far), Y=51 = bottom (close to car).
// Higher value -> car reacts only to very close lines (turns LATER).
// Lower value  -> car reacts to distant lines (turns EARLIER).
#define MIN_BOT_Y 10.0f

// Minimum steering scale applied when a curve is at maximum distance (MIN_BOT_Y)
// 0.0f = No steering at all for far curves
// 1.0f = Full steering regardless of distance (disables proximity scaling)
#define MIN_STEER_SCALE 0.2f

#endif
