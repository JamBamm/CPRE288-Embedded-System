/*
#include "Timer.h"
#include "lcd.h"
#include "button.h"
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"

#define PERIOD 320000

uint32_t match = 295000;
uint8_t mode = 1;         // 0 = 0°, 1 = 90°, 2 = 180°

void servo_write_raw(uint32_t match_val)
{
    TIMER1_TBMATCHR_R = match_val & 0xFFFF;
    TIMER1_TBPMR_R    = (match_val >> 16) & 0xFF;
}

void servo_init_calibration(void)
{
    SYSCTL_RCGCGPIO_R |= 0x02;
    SYSCTL_RCGCTIMER_R |= 0x02;

    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {}
    while ((SYSCTL_PRTIMER_R & 0x02) == 0) {}

    GPIO_PORTB_DEN_R |= 0x20;
    GPIO_PORTB_DIR_R |= 0x20;
    GPIO_PORTB_AFSEL_R |= 0x20;
    GPIO_PORTB_PCTL_R &= ~0x00F00000;
    GPIO_PORTB_PCTL_R |=  0x00700000;

    TIMER1_CTL_R &= ~0x0100;
    TIMER1_CFG_R = 0x04;
    TIMER1_TBMR_R = 0x0A;
    TIMER1_CTL_R &= ~0x4000;

    TIMER1_TBILR_R = (PERIOD - 1) & 0xFFFF;
    TIMER1_TBPR_R  = ((PERIOD - 1) >> 16) & 0xFF;

    servo_write_raw(match);

    TIMER1_CTL_R |= 0x0100;
}

int main(void)
{
    uint8_t btn;

    timer_init();
    lcd_init();
    button_init();
    servo_init_calibration();

    lcd_printf("Calibrate Servo");
    timer_waitMillis(200);

    while (1)
    {
        btn = button_getButton();

        if (btn == 1)  // decrease
        {
            match -= 500;
        }
        else if (btn == 2)  // increase
        {
            match += 500;
        }
        else if (btn == 3)  // switch mode
        {
            mode = (mode + 1) % 3;

            if (mode == 0) match = 311500;  // test 0°
            if (mode == 1) match = 297500;  // test 90°
            if (mode == 2) match = 284500;  // test 180°
        }

        servo_write_raw(match);

        lcd_clear();

        if (mode == 0)
            lcd_printf("Mode:0deg\nM:%u", match);
        else if (mode == 1)
            lcd_printf("Mode:90deg\nM:%u", match);
        else
            lcd_printf("Mode:180deg\nM:%u", match);

        timer_waitMillis(150);
    }
}
*/
#include "Timer.h"
#include "lcd.h"
#include "button.h"
#include "servo.h"
#include <stdint.h>

typedef enum
{
    CW = 0,
    CCW = 1
} direction_t;

static void wait_for_release(void)
{
    while (button_getButton() != 0)
    {
        timer_waitMillis(20);
    }
}

static void display_status(uint16_t angle, direction_t dir)
{
    lcd_clear();
    lcd_printf("Ang:%3d M:%6u\nDir:%s",
               angle,
               servo_get_match_value(),
               (dir == CCW) ? "CCW" : "CW");
}

int main(void)
{
    uint8_t button;
    uint16_t angle = 90;
    direction_t dir = CCW;

    timer_init();
    lcd_init();
    button_init();
    servo_init();

    // Part 1:
    lcd_clear();
    lcd_printf("Servo Demo");
    timer_waitMillis(500);

    servo_move(90);
    timer_waitMillis(700);

    servo_move(30);
    lcd_clear();
    lcd_printf("30 deg");
    timer_waitMillis(700);

    servo_move(150);
    lcd_clear();
    lcd_printf("150 deg");
    timer_waitMillis(700);

    servo_move(90);
    timer_waitMillis(700);

    // Part 2 + Part 3:
    angle = 90;
    dir = CCW;
    servo_move(angle);
    display_status(angle, dir);

    while (1)
    {
        button = button_getButton();

        if (button == 0)
        {
            timer_waitMillis(25);
            continue;
        }

        switch (button)
        {
            case 1:

                if (dir == CCW)
                {
                    if (angle < 180) angle += 1;
                }
                else
                {
                    if (angle > 0) angle -= 1;
                }
                servo_move(angle);
                break;

            case 2:

                if (dir == CCW)
                {
                    if (angle <= 175) angle += 5;
                    else angle = 180;
                }
                else
                {
                    if (angle >= 5) angle -= 5;
                    else angle = 0;
                }
                servo_move(angle);
                break;

            case 3:

                if (dir == CCW)
                {
                    dir = CW;
                }
                else
                {
                    dir = CCW;
                }
                break;

            case 4:

                if (dir == CW)
                {
                    angle = 5;
                }
                else
                {
                    angle = 175;
                }
                servo_move(angle);
                break;

            default:
                break;
        }

        display_status(angle, dir);
        wait_for_release();
        timer_waitMillis(80);
    }
}

