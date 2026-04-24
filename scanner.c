#include "scanner.h"
#include "servo.h"
#include "adc.h"
#include "ping_template.h"
#include "uart-interrupt.h"
#include "timer.h" // For wait functions
#include <stdio.h>

void perform_sweep(void) {
    char uart_msg[50];
    
    // Sweep from 0 to 180 degrees (usually in increments of 2 or 5)
    for (int angle = 0; angle <= 180; angle += 2) {
        servo_move(angle);
        
        // CRITICAL: Wait for the mechanical servo to reach the position
        timer_waitMillis(30); 
        
        // Read sensors
        float ir_adc = adc_read_avg();
        float ping_dist = ping_getDistance();
        
        // Format the data as a string (e.g., "ANGLE,IR,PING\n")
        sprintf(uart_msg, "%d,%.2f,%.2f\n", angle, ir_adc, ping_dist);
        
        // Send string to Python GUI
        uart_sendStr(uart_msg); 
    }
    
    // Optionally return servo to 90 degrees (center) when done
    servo_move(90);
}