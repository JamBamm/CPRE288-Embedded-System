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
    // enable clocks for Port B and ADC0 
    SYSCTL_RCGCGPIO_R |= 0x02;
    SYSCTL_RCGCADC_R  |= 0x01;

    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {}
    while ((SYSCTL_PRADC_R  & 0x01) == 0) {}

    // configure PB4 as AIN10
    GPIO_PORTB_DIR_R   &= ~0x10;
    GPIO_PORTB_AFSEL_R |=  0x10;
    GPIO_PORTB_DEN_R   &= ~0x10;
    GPIO_PORTB_AMSEL_R |=  0x10;

    // disable SS3 during setup
    ADC0_ACTSS_R &= ~0x08;

    // processor trigger for SS3
    ADC0_EMUX_R &= ~0xF000;

    // use AIN10 
    ADC0_SSMUX3_R = 10;

    // single sample, end of sequence, set raw interrupt status
    ADC0_SSCTL3_R = 0x06;

    // polling, not ADC interrupts
    ADC0_IM_R &= ~0x08;

    // default clock config
    ADC0_CC_R = 0x0;

    // re-enable SS3 
    ADC0_ACTSS_R |= 0x08;
}

uint16_t adc_read(void)
{
    uint16_t result;

	// start SS3 conversion
    ADC0_PSSI_R = 0x08;             
	// wait for completion     
    while ((ADC0_RIS_R & 0x08) == 0) {} 

	// 12-bit result
    result = ADC0_SSFIFO3_R & 0x0FFF;   
	// clear flag
    ADC0_ISC_R = 0x08;                  

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


    // to return the corresponding calibrated distance value
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

void calibrate_ir_with_ping(void) {
    uart_sendStr("\r\n--- IR SENSOR AUTO-CALIBRATION ---\r\n");
    uart_sendStr("Place a flat object in front of the sensor.\r\n");
    uart_sendStr("Press B1 to record a data point.\r\n");
    uart_sendStr("Press B2 to finish and print table.\r\n");
    uart_sendStr("Press B4 to exit.\r\n");

    int ping_dists[30];
    uint16_t ir_raws[30];
    int count = 0;
    
    while(1) {
        uint8_t btn = button_getButton();
        
        //print
        float current_ping = ping_getDistance();
        uint16_t current_ir = adc_read_avg();
        lcd_printf("IR:%u P:%.1f\nPts:%d B1=Rec B2=End", current_ir, current_ping, count);
        
        if (btn == 1) { 
           //record the point
            if (count < 30) {
                ping_dists[count] = (int)(current_ping);
                ir_raws[count] = current_ir;
                char msg[60];
                sprintf(msg, "Recorded! PING: %d cm | IR RAW: %u\r\n", ping_dists[count], ir_raws[count]);
                uart_sendStr(msg);
                count++;
            } else {
                uart_sendStr("Max points reached (30).\r\n");
            }
            ///trap
            while(button_getButton() == 1) { timer_waitMillis(20); }
        } 
        else if (btn == 2) { 
            //print table
            uart_sendStr("\r\n======================================\r\n");
            uart_sendStr(" COPY AND PASTE THIS INTO adc.c:\r\n");
            uart_sendStr("======================================\r\n");
            uart_sendStr("static const ir_calibration_t ir_table[] = {\r\n");
            for (int i = 0; i < count; i++) {
                char msg[50];
                if (i == count - 1) {
                    sprintf(msg, "    {%u, %d}\r\n", ir_raws[i], ping_dists[i]);
                } else {
                    sprintf(msg, "    {%u, %d},\r\n", ir_raws[i], ping_dists[i]);
                }
                uart_sendStr(msg);
            }
            uart_sendStr("};\r\n======================================\r\n");
            while(button_getButton() == 2) { timer_waitMillis(20); }
        }
        else if (btn == 4) { 
      
            uart_sendStr("Exiting Calibration...\r\n");
            while(button_getButton() == 4) { timer_waitMillis(20); }
            break;
        }
        
        timer_waitMillis(100); 
    }
    
    lcd_printf("Diagnostic Mode\nReady.");
}