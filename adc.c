// /*
//  * adc.c
//  *
//  *  Created on: Mar 25, 2026
//  *      Author: jbiced3
//  */
// #include <inc/tm4c123gh6pm.h>
// #include <stdint.h>

// void adc_init (void){

//     SYSCTL_RCGCADC_R |= 0x01;
//     //clock gpio
//     SYSCTL_RCGCGPIO_R |= 0x02;
//     // synchronize
//     while((SYSCTL_PRGPIO_R & 0x02) == 0) {}
//     while((SYSCTL_PRADC_R & 0x01) == 0) {}
//     // port B PB4
//     //set input
//     GPIO_PORTB_DIR_R &= ~0x10;
//     //enable alt
//     GPIO_PORTB_AFSEL_R |= 0x10;
//     //enable analog
//     GPIO_PORTB_DEN_R &= ~0x10;
//     GPIO_PORTB_AMSEL_R |= 0x10;
//     //set sampling
//     ADC0_PC_R &= ~0x0F;
//     ADC0_PC_R |= 0x01;
//     // set prioity
//     ADC0_SSPRI_R = 0x0123;
//     // disable before config
//     ADC0_ACTSS_R &= ~0x08;

//     //mask
//     ADC0_IM_R &= ~0x0F;

//     //Sets the emo3 15-12
//     ADC0_EMUX_R &= ~0x0D000000;
//     ADC0_EMUX_R |= 0x01000000;

//     //sets channe;
//     ADC0_SSMUX3_R &= ~0x0F;
//     ADC0_SSMUX3_R += 10;

//     //Sets RIS and END 0
//     ADC0_SSCTL3_R = 0x06;

//     // enable after config
//     ADC0_ACTSS_R |= 0x0008;

// }


// uint16_t adc_read (void){
//     uint16_t result;

//     //software pssi set in PC_R
//     ADC0_PSSI_R = 0x08;

//     //polling
//     while ((ADC0_RIS_R & 0x08) == 0){}

//     result = ADC0_SSFIFO3_R & 0xFFF;

//     //clear intturpt
//     ADC0_ISC_R |= 0x08;

//     return result;

// }


#include "adc.h"
#include "inc/tm4c123gh6pm.h"

typedef struct {
    uint16_t adc_value;
    int distance_cm;
} ir_calibration_t;


static const ir_calibration_t ir_table[] = {
    {3000, 10},
    {2780, 12},
    {2560, 14},
    {2350, 16},
    {2160, 18},
    {1980, 20},
    {1841, 21},
    {1681, 23},
    {1550, 26},
    {1430, 28},
    {1320, 30},
    {1220, 32},
    {1130, 34},
    {1050, 36},
    {980,  38},
    {920,  40},
    {870,  42},
    {820,  44},
    {780,  46},
    {740,  48},
    {700,  50}
};

static const int ir_table_size = sizeof(ir_table) / sizeof(ir_table[0]);

void adc_init(void)
{
    // Enable clocks for Port B and ADC0 
    SYSCTL_RCGCGPIO_R |= 0x02;
    SYSCTL_RCGCADC_R  |= 0x01;

    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {}
    while ((SYSCTL_PRADC_R  & 0x01) == 0) {}

    // Configure PB4 as AIN10
    GPIO_PORTB_DIR_R   &= ~0x10;
    GPIO_PORTB_AFSEL_R |=  0x10;
    GPIO_PORTB_DEN_R   &= ~0x10;
    GPIO_PORTB_AMSEL_R |=  0x10;

    // Disable SS3 during setup
    ADC0_ACTSS_R &= ~0x08;

    // Processor trigger for SS3
    ADC0_EMUX_R &= ~0xF000;

    // Use AIN10 
    ADC0_SSMUX3_R = 10;

    // Single sample, end of sequence, set raw interrupt status
    ADC0_SSCTL3_R = 0x06;

    // Polling, not ADC interrupts
    ADC0_IM_R &= ~0x08;

    // Default clock config
    ADC0_CC_R = 0x0;

    // Re-enable SS3 
    ADC0_ACTSS_R |= 0x08;
}

uint16_t adc_read(void)
{
    uint16_t result;

    ADC0_PSSI_R = 0x08;                 // Start SS3 conversion
    while ((ADC0_RIS_R & 0x08) == 0) {} // Wait for completion 

    result = ADC0_SSFIFO3_R & 0x0FFF;   // 12-bit result
    ADC0_ISC_R = 0x08;                  // Clear flag

    return result;
}

uint16_t adc_read_avg(void)
{
    uint32_t sum = 0;
    int i;

    for (i = 0; i < 16; i++)
    {
        sum += adc_read();
    }

    return (uint16_t)(sum / 16);
}

int ir_distance_from_adc(uint16_t adc_val)
{
    int i;

    // detects the closest calibrated value(upper bound)
    if (adc_val >= ir_table[0].adc_value)
    {
        return ir_table[0].distance_cm;
    }

    //  detects the farthest calibrated value(lower bound)
    if (adc_val <= ir_table[ir_table_size - 1].adc_value)
    {
        return ir_table[ir_table_size - 1].distance_cm;
    }


    // To return the corresponding calibrated distance value
    for (i = 0; i < ir_table_size - 1; i++)
    {
        uint16_t adc_high = ir_table[i].adc_value;
        uint16_t adc_low  = ir_table[i + 1].adc_value;

        if (adc_val <= adc_high && adc_val >= adc_low)
        {
            int dist_high = ir_table[i].distance_cm;
            int dist_low  = ir_table[i + 1].distance_cm;

            return dist_high + ((int)(adc_high - adc_val) * (dist_low - dist_high)) /
                               (int)(adc_high - adc_low);
        }
    }

    return -1;

}
