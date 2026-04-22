#include "servo.h"
#include "Timer.h"
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"

#define SERVO_PERIOD_COUNTS   320000U

#define SERVO_MATCH_0_DEG     311500U
#define SERVO_MATCH_180_DEG   284500U

static uint16_t g_current_angle = 90;
static uint32_t g_current_match = 297500U;

static uint32_t servo_compute_match(uint16_t degrees)
{
    uint32_t span;
    uint32_t match;

    if (degrees > 180)
    {
        degrees = 180;
    }

    span = SERVO_MATCH_0_DEG - SERVO_MATCH_180_DEG;
    match = SERVO_MATCH_0_DEG - ((span * degrees) / 180U);

    return match;
}

void servo_init(void)
{
    uint32_t period_load;
    uint32_t center_match;

    // Enable clock to Port B and Timer1
    SYSCTL_RCGCGPIO_R |= 0x02;
    SYSCTL_RCGCTIMER_R |= 0x02;

    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {}
    while ((SYSCTL_PRTIMER_R & 0x02) == 0) {}

    // Configure PB5 for T1CCP1
    GPIO_PORTB_DEN_R |= 0x20;
    GPIO_PORTB_DIR_R |= 0x20;
    GPIO_PORTB_AFSEL_R |= 0x20;
    GPIO_PORTB_PCTL_R &= ~0x00F00000;
    GPIO_PORTB_PCTL_R |=  0x00700000;

    // Disable Timer1B during setup
    TIMER1_CTL_R &= ~0x0100;

    // 16-bit timer configuration
    TIMER1_CFG_R = 0x04;

    // Timer B periodic PWM mode
    TIMER1_TBMR_R = 0x0A;

    // Non-inverted PWM
    TIMER1_CTL_R &= ~0x4000;

    // 20 ms period
    period_load = SERVO_PERIOD_COUNTS - 1U;
    TIMER1_TBILR_R = period_load & 0xFFFF;
    TIMER1_TBPR_R   = (period_load >> 16) & 0xFF;

    // Start at 90 degrees
    center_match = servo_compute_match(90);
    TIMER1_TBMATCHR_R = center_match & 0xFFFF;
    TIMER1_TBPMR_R    = (center_match >> 16) & 0xFF;

    g_current_angle = 90;
    g_current_match = center_match;

    // Enable Timer1B
    TIMER1_CTL_R |= 0x0100;

    timer_waitMillis(300);
}

void servo_move(uint16_t degrees)
{
    uint32_t match;
    uint16_t diff;
    uint16_t wait_ms;

    if (degrees > 180)
    {
        degrees = 180;
    }

    match = servo_compute_match(degrees);

    TIMER1_TBMATCHR_R = match & 0xFFFF;
    TIMER1_TBPMR_R    = (match >> 16) & 0xFF;

    if (degrees > g_current_angle)
    {
        diff = degrees - g_current_angle;
    }
    else
    {
        diff = g_current_angle - degrees;
    }

    wait_ms = 20 + (diff * 4);
    timer_waitMillis(wait_ms);

    g_current_angle = degrees;
    g_current_match = match;
}

uint16_t servo_get_angle(void)
{
    return g_current_angle;
}

uint32_t servo_get_match_value(void)
{
    return g_current_match;
}
