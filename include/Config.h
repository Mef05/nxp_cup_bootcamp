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
#define DIFFERENTIAL_FACTOR 0.5f

// Lookahead factor for line tracking
// 0.0f = Only look at the bottom of the line (close to car)
// 1.0f = Only look at the top of the line (far away/future)
#define LOOKAHEAD_FACTOR 0.6f

#endif
