#include "servo.h"
#include "Timer.h"
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "button.h"
#include "lcd.h"

#define SERVO_PERIOD_COUNTS   320000U


static uint32_t g_match_0_deg = 311500U;
static uint32_t g_match_180_deg = 284500U;
//bot 18
//static uint32_t g_match_0_deg = 312800U;
//static uint32_t g_match_180_deg = 285950U;

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

    // distance between using variables
    span = g_match_0_deg - g_match_180_deg;
    match = g_match_0_deg - ((span * degrees) / 180U);

    return match;
}

void servo_init(void)
{
    uint32_t period_load;
    uint32_t center_match;

    // enable clock to Port B and Timer1
    SYSCTL_RCGCGPIO_R |= 0x02;
    SYSCTL_RCGCTIMER_R |= 0x02;

    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {}
    while ((SYSCTL_PRTIMER_R & 0x02) == 0) {}

    // configure PB5 for T1CCP1
    GPIO_PORTB_DEN_R |= 0x20;
    GPIO_PORTB_DIR_R |= 0x20;
    GPIO_PORTB_AFSEL_R |= 0x20;
    GPIO_PORTB_PCTL_R &= ~0x00F00000;
    GPIO_PORTB_PCTL_R |=  0x00700000;

    // disable Timer1B during setup
    TIMER1_CTL_R &= ~0x0100;

    // 16-bit timer configuration
    TIMER1_CFG_R = 0x04;

    // timer B periodic PWM mode
    TIMER1_TBMR_R = 0x0A;

    // non-inverted PWM
    TIMER1_CTL_R &= ~0x4000;

    // 20 ms period
    period_load = SERVO_PERIOD_COUNTS - 1U;
    TIMER1_TBILR_R = period_load & 0xFFFF;
    TIMER1_TBPR_R   = (period_load >> 16) & 0xFF;

    // start at 90 degrees
    center_match = servo_compute_match(90);
    TIMER1_TBMATCHR_R = center_match & 0xFFFF;
    TIMER1_TBPMR_R    = (center_match >> 16) & 0xFF;

    g_current_angle = 90;
    g_current_match = center_match;

    // enable Timer1B
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
	
	//find match
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
	
	//wait might change idk
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

void servo_calibrate(void) 
{
	//90 degree
    uint32_t current_match = (g_match_0_deg + g_match_180_deg) / 2; 
    int state = 0; 
    uint8_t button;
    
    int step_size = 50;         
    bool big_steps = false;     

    TIMER1_TBMATCHR_R = current_match & 0xFFFF;
    TIMER1_TBPMR_R    = (current_match >> 16) & 0xFF;

    while (state < 2) 
    {
        button = button_getButton();

		//go left
        if (button == 2) 
        {
            current_match += step_size; 
            TIMER1_TBMATCHR_R = current_match & 0xFFFF;
            TIMER1_TBPMR_R    = (current_match >> 16) & 0xFF;
            timer_waitMillis(100);
        } 
		//go right
        else if (button == 1) 
        {
            current_match -= step_size;
            TIMER1_TBMATCHR_R = current_match & 0xFFFF;
            TIMER1_TBPMR_R    = (current_match >> 16) & 0xFF;
            timer_waitMillis(100); 
        } 
		//toggle size of movement
        else if (button == 3)
        {
            big_steps = !big_steps; 
            step_size = big_steps ? 250 : 50; 
            timer_waitMillis(300); 
        }
		//confirm and save
        else if (button == 4) 
        {
            if (state == 0) 
            {
                g_match_0_deg = current_match;
				//now do 180
                state = 1; 
                timer_waitMillis(500); 
            } 
            else if (state == 1) 
            {
                g_match_180_deg = current_match;
				//exit
                state = 2; 
            }
        }

		//display match values 
        if (state == 0) 
        {
            lcd_printf("Set 0 Deg Ref\nMatch: %u\nStep: %d (B3)", current_match, step_size);
        } 
        else if (state == 1) 
        {
            lcd_printf("Set 180 Deg Ref\nMatch: %u\nStep: %d (B3)", current_match, step_size);
        }

        timer_waitMillis(50); 
    }

    
    lcd_printf("Calibrated!\n0:%u 180:%u\nPress B4 to Exit", g_match_0_deg, g_match_180_deg);
    
	//trap until pressed again
    while (button_getButton() != 4) 
    {
        timer_waitMillis(50);
    }
	//and released
    while (button_getButton() == 4) 
    {
        timer_waitMillis(50);
    }
    
    servo_move(90); 
    
    lcd_printf("Ready"); 
}

