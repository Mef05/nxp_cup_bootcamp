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
#define SPEED_RIGHT 85
#define SPEED_LEFT 85

// Electronic Differential Factor
// 0.0f = No differential, 1.0f = Strong differential (stops inner wheel on tight turns)
#define DIFFERENTIAL_FACTOR 0.3f

// Lookahead factor for line tracking
// 0.0f = Only look at the bottom of the line (close to car) -> Turns LATE
// 1.0f = Only look at the top of the line (far away/future) -> Turns EARLY
#define LOOKAHEAD_FACTOR 0.2f

// Heading anticipation factor
// 0.0f = Car only cares about its position on the track
// 1.0f = Car reacts strongly to the angle of the line ahead (turns early)
#define HEADING_FACTOR 1.0f

#endif
