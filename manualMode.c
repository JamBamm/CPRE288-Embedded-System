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
#include "scanner.h"    
#include "servo.h"      
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
		//uses blocking
        c = uart_receive(); 

		//handle enter
        if ('\r' == c)
        {
            buf[i] = '\0'; 
            break;
        }
        // handle backspace
        else if ('\b' == c || 127 == c)
        {
            if (i > 0)
            {
                i--;
                uart_sendStr("\b \b"); 
            }
        }
		//only accept numbers
        else if (i < 9 && (isdigit(c) || '-' == c))
        {
            buf[i] = c;
            i++;
            uart_sendChar(c); 
        }
    }

    uart_sendStr("\r\n");
    return atoi(buf);
}

void manual_handle_command(char cmd, oi_t *sensor_data)
{
    int val = 0;
    char msg[100];

    switch (cmd)
    {
    case 'w':
        uart_sendStr("Stepping Forward 10mm\r\n");
        int hazard_w = move_forward(sensor_data, 10, 100);
		if (hazard_w != 0) {
            uart_sendStr("\r\n[CRITICAL] Hardware Safety Stop! Please reverse.\r\n");
        }
        break;

    case 's':
        //uart_sendStr("STOPPED (Reverse Disabled for Safety)\r\n");
        //oi_setWheels(0, 0); 
        move_backward(sensor_data, 10, 100);
        break;

    case 'q':
        uart_sendStr("Turning Left 5 degrees\r\n");
        turn_left(sensor_data, 5, 50);
        break;

    case 'e':
        uart_sendStr("Turning Right 5 degrees\r\n");
        turn_right(sensor_data, 5, 50);
        break;

    case ' ': 
        uart_sendStr("Emergency Stop\r\n");
        oi_setWheels(0, 0);
        break;

	//argument based
    case 'f':
        oi_setWheels(0, 0); 
        val = manual_read_number();
        sprintf(msg, "Moving Forward %d mm...\r\n", val);
        uart_sendStr(msg);
        int hazard_f = move_forward(sensor_data, val, 100);
        if (hazard_f != 0) {
            uart_sendStr("\r\n[CRITICAL] Hardware Safety Stop! Please reverse.\r\n");
        }
        break;

    case 't':
        oi_setWheels(0, 0);
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

    
    case 'p':
			//maybe parking
		break;
    case 'g':
        uart_sendStr("Initiating 180-Degree Radar Sweep...\r\n");
        oi_setWheels(0, 0); 
        DetectedObject my_objects[10];

        perform_advanced_sweep(my_objects, 10);
        uart_sendStr("Sweep Complete.\r\n");
        break;

    case 'c':

        servo_calibrate();

        break;

	//ignore carragie returns
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
