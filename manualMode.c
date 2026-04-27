/* 
Manual.c
final project
*/


#include "movement.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>


static int manual = 0;
static char command_byte;

// read int (mm or deg)
int read_number() {
    char buf[10];
    int i = 0;
    char c;

    uart_sendStr("val: ");

    while (1) {
        c = uart_receive();

        if (c == '\r') {
            buf[i] = '\0';
            break;
        }

        if (i < 9) {
            buf[i++] = c;
            uart_sendChar(c);
        }
    }

    uart_sendStr("\r\n");
    return atoi(buf);
}

void handleManualMode(void) {
    int v;

    switch (command_byte) {
        case 'f':
            v = read_number();
            move_forward(sensor_data, v);
            break;

        case 'b':
            v = read_number();
            move_backward(sensor_data, v);
            break;

        case 'l':
            v = read_number();
            turn_angle(sensor_data, -v);
            break;

        case 'r':
            v = read_number();
            turn_angle(sensor_data, v);
            break;

        default:
            uart_sendStr("bad\r\n");
            break;
    }
}

void loop(void) {
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
