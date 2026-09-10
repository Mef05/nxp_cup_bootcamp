#include "hbridge.h"
#include "fsl_ctimer.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "fsl_debug_console.h"

Hbridge g_hbridge;
static uint32_t s_srcClockHz;

void HbridgeInit(Hbridge *h,
		 	 	 CTIMER_Type *pwmPeriph,
                 ctimer_match_t periodCh,
                 ctimer_match_t pwm1Ch,
                 ctimer_match_t pwm2Ch,
                 GPIO_Type *m1DirPort, uint32_t m1DirPin,
                 GPIO_Type *m2DirPort, uint32_t m2DirPin)
{
    ctimer_config_t config;

    g_hbridge = *h;
    h->periodChannel    = periodCh;
    h->pwm1Channel      = pwm1Ch;
    h->pwm2Channel      = pwm2Ch;
    h->motor1DirPort    = m1DirPort;
    h->motor1DirPin     = m1DirPin;
    h->motor2DirPort    = m2DirPort;
    h->motor2DirPin     = m2DirPin;
    h->pwmPeripheral    = pwmPeriph;
}

void HbridgeSpeed(Hbridge *h, int16_t speed1, int16_t speed2)
{
        // Clamp speed values to valid range [-100, 100]
    if (speed1 > 100) speed1 = 100;
    if (speed1 < -100) speed1 = -100;
    if (speed2 > 100) speed2 = 100;
    if (speed2 < -100) speed2 = -100;

    // Calculate duty cycles (absolute values)
    uint8_t duty1 = (uint8_t)((speed1 >= 0) ? speed1 : 100 + speed1);
    uint8_t duty2 = (uint8_t)((speed2 >= 0) ? speed2 : 100 + speed2);

    // Set motor 1 direction: 1 for reverse, 0 for forward
    GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, (speed1 < 0) ? 1U : 0U);    
    // Set motor 2 direction: 1 for reverse, 0 for forward
    GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, (speed2 < 0) ? 1U : 0U);

    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral,h->periodChannel , h->pwm1Channel, duty1);
    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral,h->periodChannel , h->pwm2Channel, duty2);
}

void HbridgeBrake(Hbridge *h)
{
    GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 1U);
    GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 1U);

    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral,h->periodChannel , h->pwm1Channel, 100U);
    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral,h->periodChannel , h->pwm2Channel, 100U);
}
