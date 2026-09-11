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
#define SPEED_RIGHT 100
#define SPEED_LEFT 100

// Electronic Differential Factor
// 0.0f = No differential, 1.0f = Strong differential (stops inner wheel on
// tight turns)
#define DIFFERENTIAL_FACTOR 0.3f

// Lookahead factor for line tracking
// 0.0f = Only look at the bottom of the line (close to car) -> Turns LATE
// 1.0f = Only look at the top of the line (far away/future) -> Turns EARLY
#define LOOKAHEAD_FACTOR 0.2f

// Heading anticipation factor
// 0.0f = Car only cares about its position on the track
// 1.0f = Car reacts strongly to the angle of the line ahead (turns early)
#define HEADING_FACTOR 1.0f

// Steering PD Controller Tuning
#define STEER_KP 8.0f
#define STEER_KD 0.5f

// Steering Smoothing (0.0f = no smoothing/instant, 0.9f = very slow/smooth)
#define STEERING_ALPHA 0.4f

// Minimum Y of a vector's bottom point to be considered for steering
// Pixy2 image is 79x51px. Y=0=top(far), Y=51=bottom(close to car).
// Higher value = car only reacts to lines very close to it (turns LATER)
// Lower value = car reacts to lines further away (turns EARLIER)
// Start with 20 and adjust. If turning too early, INCREASE this value.
#define MIN_BOT_Y 30.0f

#endif
