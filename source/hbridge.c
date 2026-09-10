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
    h->periodChannel    = periodCh;
    h->pwm1Channel      = pwm1Ch;
    h->pwm2Channel      = pwm2Ch;
    h->motor1DirPort    = m1DirPort;
    h->motor1DirPin     = m1DirPin;
    h->motor2DirPort    = m2DirPort;
    h->motor2DirPin     = m2DirPin;
    h->pwmPeripheral    = pwmPeriph;

    g_hbridge = *h;
}

void HbridgeSpeed(Hbridge *h, int16_t speed1, int16_t speed2)
{
    if (speed1 >  100) speed1 =  100;
    if (speed1 < -100) speed1 = -100;
    if (speed2 >  100) speed2 =  100;
    if (speed2 < -100) speed2 = -100;

    /*
     * NXP SDK CTIMER_UpdatePwmDutycycle inverts the duty cycle:
     * It keeps the pin HIGH for (100 - duty)% of the time.
     * 
     * H-Bridge Logic:
     * If DIR=0, motor sees voltage when PWM is HIGH. 
     *    -> True Power = (100 - duty). So duty = 100 - Power.
     * If DIR=1, motor sees voltage when PWM is LOW.
     *    -> True Power = duty. So duty = Power.
     */

    /* 
     * Dupa modificarea fizica a firelor de catre utilizator:
     * Ambele motoare merg INAINTE (Forward) cand DIR=1.
     * Deci speed > 0 -> DIR=1, speed < 0 -> DIR=0.
     */
    uint8_t dir1 = (speed1 >= 0) ? 1U : 0U;
    uint8_t p1   = (uint8_t)((speed1 >= 0) ? speed1 : -speed1);
    // Forward (dir=1): Power = LOW time -> duty = 100 - p
    // Reverse (dir=0): Power = HIGH time -> duty = p
    uint8_t duty1 = (dir1 == 1U) ? (100U - p1) : p1;

    uint8_t dir2 = (speed2 >= 0) ? 1U : 0U;
    uint8_t p2   = (uint8_t)((speed2 >= 0) ? speed2 : -speed2);
    uint8_t duty2 = (dir2 == 1U) ? (100U - p2) : p2;

    GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, dir1);
    GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, dir2);

    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel, h->pwm1Channel, duty1);
    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel, h->pwm2Channel, duty2);
}

void HbridgeBrake(Hbridge *h)
{
    /* Brake = Both terminals LOW. DIR=0, PWM=LOW. 
       To get PWM LOW 100% of the time, SDK needs duty=100. */
    GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 0U);
    GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 0U);

    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel, h->pwm1Channel, 100U);
    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel, h->pwm2Channel, 100U);
}
