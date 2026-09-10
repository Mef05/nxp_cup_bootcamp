#include "servo.h"
#include "fsl_ctimer.h"
#include "fsl_debug_console.h"
#include "peripherals.h"

void Steer(double angle) {
    // 0. Inversam directia fizica (deoarece o curba la dreapta ducea rotile in
    // stanga) angle = -angle;

    // 1. Clamp `angle` to the logical range [-100.0, 100.0]
    if (angle > 100.0)
        angle = 100.0;
    if (angle < -100.0)
        angle = -100.0;

    // Compensate for hardware offset (mechanical center is at SERVO_CENTER_OFFSET)
    // We map logical [-100, 0, 100] to hardware [-100, SERVO_CENTER_OFFSET, 100]
    double hw_angle;
    if (angle >= 0.0) {
        hw_angle = SERVO_CENTER_OFFSET + (angle * (100.0 - SERVO_CENTER_OFFSET) / 100.0);
    } else {
        hw_angle = SERVO_CENTER_OFFSET + (angle * (100.0 + SERVO_CENTER_OFFSET) / 100.0);
    }

    // 2. Map the hardware angle to a duty cycle between 5.0% and 10.0%
    double duty = 5.0 + (hw_angle + 100.0) * 5.0 / 200.0;

    // 3. Read the PWM period from the period channel match register
    uint32_t periodTicks = CTIMER2_PERIPHERAL->MR[CTIMER2_PWM_PERIOD_CH];

    // 4. Convert the duty cycle into pulse ticks; the match value is
    //    where the pulse starts, so use (100.0 - duty)
    uint32_t pulseTicks = periodTicks * (100.0 - duty) / 100.0;

    // 5. Write the result to the servo channel match register
    CTIMER2_PERIPHERAL->MR[2] = pulseTicks;
    // PRINTF("Steer logic: %lf, hw_angle: %lf\n", angle, hw_angle);
}

void TestServo() {
    volatile int Delay;
    volatile int SteerStrength;
    while (1) {
        for (SteerStrength = -100; SteerStrength <= 100; SteerStrength++) {
            Delay = 200000;
            while (Delay) {
                Delay--;
            }
            PRINTF("Steer: %d\n", SteerStrength);
            Steer(SteerStrength);
        }
    }
}
