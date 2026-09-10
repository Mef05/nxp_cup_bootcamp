#ifndef HBRIDGE_H
#define HBRIDGE_H

#include "fsl_ctimer.h"
#include "fsl_gpio.h"

typedef struct {
    CTIMER_Type   *pwmPeripheral;
    ctimer_match_t periodChannel;
    ctimer_match_t pwm1Channel;
    ctimer_match_t pwm2Channel;
    GPIO_Type     *motor1DirPort;
    uint32_t       motor1DirPin;
    GPIO_Type     *motor2DirPort;
    uint32_t       motor2DirPin;
} Hbridge;

extern Hbridge g_hbridge;

void HbridgeInit(Hbridge *h,
                 CTIMER_Type *pwmPeriph,
                 ctimer_match_t periodCh,
                 ctimer_match_t pwm1Ch,
                 ctimer_match_t pwm2Ch,
                 GPIO_Type *m1DirPort, uint32_t m1DirPin,
                 GPIO_Type *m2DirPort, uint32_t m2DirPin);


void HbridgeSpeed(Hbridge *h, int16_t speed1, int16_t speed2);


void HbridgeBrake(Hbridge *h);

#ifdef __cplusplus
}
#endif

#endif
