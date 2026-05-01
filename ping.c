/**
 * Driver for ping sensor
 * @file ping.c
 * @author Jamison Bice 
 */

#include "ping.h"
#include "Timer.h"
#include "driverlib/interrupt.h"

// Global shared variables
// Use extern declarations in the header file

volatile uint32_t g_start_time = 0;
volatile uint32_t g_end_time = 0;
volatile enum{LOW, HIGH, DONE} g_state = LOW;
volatile uint32_t pingDiff = 0;
//User Timer overflow flag on Timer 3
volatile uint8_t timerOverflow = 0;

void ping_init (void){

  // YOUR CODE HERE
  
   // enable clocks for Port B and Timer 3
    SYSCTL_RCGCGPIO_R |= 0x02;      
    SYSCTL_RCGCTIMER_R |= 0x08;  
    //wait and make sure
    while((SYSCTL_PRGPIO_R & 0x02) == 0) {}; 

    // config as digital input
    GPIO_PORTB_DEN_R |= 0x08;
    GPIO_PORTB_DIR_R &= ~0x08;
    //map to T3CCP1
    GPIO_PORTB_PCTL_R &= ~0xF000;
    GPIO_PORTB_PCTL_R |= 0x7000;

    //start alt
    GPIO_PORTB_AFSEL_R |= 0x08;
    // disable to configure for setup
    TIMER3_CTL_R &= ~0x0100;
    //16 bit
    TIMER3_CFG_R = 0x04;
	
	//set edges for both
	TIMER3_CTL_R &= ~0x0C00;
    TIMER3_CTL_R |= 0x0C00;
    
    // config as capture edge and count down
    TIMER3_TBMR_R = 0x07;           // 0x07 = 0b00111

    //both dedge
    TIMER3_TBPR_R = 0xFF;
    //prescaler
    TIMER3_TBILR_R = 0xFFFF;
    
    //set intterurpt
    TIMER3_ICR_R = 0x0400;
    TIMER3_IMR_R |= 0x0400;
	
    //enble inttruptes
	NVIC_PRI9_R = (NVIC_PRI9_R & 0xFFFFFF1F) | 0x00000020; 
    NVIC_EN1_R |= 0x10;

    IntRegister(INT_TIMER3B, TIMER3B_Handler);

    IntMasterEnable();

    // configure and enable the timer
    // set for both edges
    TIMER3_CTL_R |= 0x0100;
}

void ping_trigger (void){
    g_state = LOW;
    // disable timer and disable timer interrupt
    TIMER3_CTL_R &= ~0x0100;
    TIMER3_IMR_R &= ~0x0400;
    // disable alternate function (disconnect timer from port pin)
    GPIO_PORTB_AFSEL_R &= ~0x08;

    // YOUR CODE HERE FOR PING TRIGGER/START PULSE
    //set output
    GPIO_PORTB_DIR_R |= 0x08;

    GPIO_PORTB_DATA_R &= ~0x08;
    timer_waitMicros(5);
    GPIO_PORTB_DATA_R |= 0x08;
    timer_waitMicros(5);
    GPIO_PORTB_DATA_R &= ~0x08;


    // clear an interrupt that may have been erroneously triggered
	TIMER3_ICR_R = 0x0400;
    // re-enable alternate function, timer interrupt, and timer
    GPIO_PORTB_AFSEL_R |= 0x08;

    TIMER3_IMR_R |= 0x0400;
    TIMER3_CTL_R |= 0x0100;

}

void TIMER3B_Handler(void){

  // YOUR CODE HERE
  // As needed, go back to review your interrupt handler code for the UART lab.
  // What are the first lines of code in the ISR? Regardless of the device, interrupt handling
  // includes checking the source of the interrupt and clearing the interrupt status bit.
  // Checking the source: test the MIS bit in the MIS register (is the ISR executing
  // because the input capture event happened and interrupts were enabled for that event?
  // Clearing the interrupt: set the ICR bit (so that same event doesn't trigger another interrupt)
  // The rest of the code in the ISR depends on actions needed when the event happens.

    if (TIMER3_MIS_R & 0x0400)
        {
            TIMER3_ICR_R = 0x0400;

            if (g_state == LOW){
                g_start_time = TIMER3_TBR_R & 0x00FFFFFF;
                g_state = HIGH;
            }else if ( g_state == HIGH){
                g_end_time = TIMER3_TBR_R & 0x00FFFFFF;
                g_state = DONE;
            }


        }





}

float ping_getDistance (void){

    // YOUR CODE HERE
	ping_trigger();
    
    // wait until the ISR 
    while (g_state != DONE) {}

    //if (TIMER3_RIS_R & 0x0100) {
    //    timerOverflow++;
    //    TIMER3_ICR_R = 0x0100;
    //}

    if(g_start_time < g_end_time){
        timerOverflow++;
        pingDiff = (g_end_time - g_start_time ) & 0x00FFFFFF;
    }else {
        // calculate pulse width.
        // and works out perfectly across the 24-bit boundary.
        pingDiff = (g_start_time - g_end_time) & 0x00FFFFFF;
    }



    // convert clock cycles to time in seconds
    float time_seconds = pingDiff / 16000000.0;
    
    // convert time to distance sound = 343 m/s = 34300 cm/s
    // divide by 2 there and back
    float distance_cm = (time_seconds * 34300.0) / 2.0;

    return distance_cm;
}
