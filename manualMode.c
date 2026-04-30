/* 
 Manual.c
 final project
 */

#include "movement.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "manualMode.h"
#include "movement.h"
#include "uart-interrupt.h"
#include "lcd.h"
#include "scanner.h"    // Needed for scanning
#include "servo.h"      // Needed for calibration
#include <stdbool.h>
#include <ctype.h>

//static int manual = 0;
//static char command_byte;

// read int (mm or deg)
int manual_read_number(void)
{
    char buf[10];
    int i = 0;
    char c;

    uart_sendStr(" Enter Value (mm/deg) and press ENTER: ");

    while (1)
    {
        c = uart_receive(); // Requires blocking UART receive here

        // Handle Enter key (carriage return)
        if ('\r' == c)
        {
            buf[i] = '\0'; // Null terminate
            break;
        }
        // Handle Backspace
        else if ('\b' == c || 127 == c)
        {
            if (i > 0)
            {
                i--;
                uart_sendStr("\b \b"); // Erase character from terminal visually
            }
        }
        // Only accept numbers and negative signs, prevent buffer overflow
        else if (i < 9 && (isdigit(c) || '-' == c))
        {
            buf[i] = c;
            i++;
            uart_sendChar(c); // Echo character back to user
        }
    }

    uart_sendStr("\r\n");
    return atoi(buf);
}

void manual_handle_command(char cmd, oi_t *sensor_data)
{
    int val = 0;
    char msg[100];
    int calib_angle;

    switch (cmd)
    {
    // --- Small Increment Execution Commands (WASD) ---
    case 'w':
        uart_sendStr("Stepping Forward 10mm\r\n");
        move_forward(sensor_data, 10, 100);
        break;

    case 's':
        // NO BACKWARDS MOVEMENT CONSTRAINT ENFORCED
        //uart_sendStr("STOPPED (Reverse Disabled for Safety)\r\n");
        //oi_setWheels(0, 0); 
        move_backward(sensor_data, 10, 100);
        break;

    case 'a':
        uart_sendStr("Turning Left 5 degrees\r\n");
        turn_left(sensor_data, 5, 50);
        break;

    case 'd':
        uart_sendStr("Turning Right 5 degrees\r\n");
        turn_right(sensor_data, 5, 50);
        break;

    case ' ': // Spacebar
        uart_sendStr("Emergency Stop\r\n");
        oi_setWheels(0, 0);
        break;

        // --- Argument-Based Execution Commands ---
    case 'f':
        oi_setWheels(0, 0); // Stop before waiting for input
        val = manual_read_number();
        sprintf(msg, "Moving Forward %d mm...\r\n", val);
        uart_sendStr(msg);
        move_forward(sensor_data, val, 100);
        break;

    case 't':
        oi_setWheels(0, 0); // Stop before waiting for input
        val = manual_read_number();
        sprintf(msg, "Turning %d degrees...\r\n", val);
        if (val > 0)
        {
            turn_left(sensor_data, val, 50);
        }
        else
        {
            turn_right(sensor_data, abs(val), 50);
        }
        break;

        // --- Special Modes (Scan and Calibrate) ---
    case 'p':
    case 'g':
        uart_sendStr("Initiating 180-Degree Radar Sweep...\r\n");
        oi_setWheels(0, 0); // Ensure stopped before scanning
        DetectedObject my_objects[10];

        perform_advanced_sweep(my_objects, 10);
        uart_sendStr("Sweep Complete.\r\n");
        break;

    case 'c':

        //call the calibration servo method

        servo_calibrate();

        break;

        // --- Ignore carriage returns and standard empty inputs ---
    case '\r':
    case '\n':
        break;

    default:
        uart_sendStr(
                "Unknown Command. Use w/a/d, ' ' to stop, f to move, t to turn, p to scan, c to calibrate.\r\n");
        break;
    }
}

/* void loop(void) {
 char c = uart_receive();

 if (c == 'M') {
 manual = 1;
 uart_sendStr("MAN\r\n");
 return;
 }
 if (c == 'A') {
 manual = 0;
 uart_sendStr("AUTO\r\n");
 return;
 }

 command_byte = c;

 if (manual) handleManualMode();
 else autonomous();
 }
 */
