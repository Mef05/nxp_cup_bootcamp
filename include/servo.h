#ifndef STEER_H_
#define STEER_H_

#ifdef __cplusplus
extern "C" {
#endif

// Offset-ul real la care roțile sunt perfect drepte. Ajustează valoarea asta dacă se dereglează mecanic.
#include "Config.h"
#define SERVO_CENTER_OFFSET ((double)STEERING_OFFSET)

void Steer(double angle);
void TestServo();

#ifdef __cplusplus
}
#endif

#endif
