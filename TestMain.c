#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <inc/tm4c123gh6pm.h>

#include "Timer.h"
#include "lcd.h"
#include "button.h"
#include "open_interface.h"
#include "uart-interrupt.h"
#include "adc.h"
#include "ping.h"
#include "servo.h"
#include "scanner.h"
#include "movement.h"

extern volatile char g_command_byte;
extern volatile bool g_command_ready;

volatile float current_x = 0.0;
volatile float current_y = 0.0;
volatile int current_heading = 90;
void update_odometry(oi_t *sensor_data);
void send_telemetry(const char* message);
void send_gui_telemetry(oi_t *sensor_data);
int check_hazards(oi_t *sensor_data);

#define PI 3.14159265

//Guess
#define IR_JUMP_THRESHOLD 200
#define MIN_OBJECT_WIDTH_DEG 6
#define ROBOT_WIDTH 34

void print_menu(void) {
    uart_sendStr("\r\n=== CYBOT DIAGNOSTIC MENU ===\r\n");
    uart_sendStr("1: Test ADC (IR Raw & Calculated cm)\r\n");
    uart_sendStr("2: Test PING (Ultrasonic cm)\r\n");
    uart_sendStr("3: Test Servo (0 -> 90 -> 180)\r\n");
    uart_sendStr("4: Test Advanced Scanner Sweep\r\n");
    uart_sendStr("5: Test Movement (Drive 10cm Fwd/Back)\r\n");
    uart_sendStr("6: Test Bump Sensors\r\n");
    uart_sendStr("7: Test Cliff/Drop-off Sensors\r\n");
    uart_sendStr("8: Test Boundary (White Tape) Sensors\r\n");
    uart_sendStr("9: Halt / Stop All Motors\r\n");
    uart_sendStr("Select a test (1-9): ");
}

int main(void) {
    timer_init();
    lcd_init();
    button_init();
    uart_interrupt_init();
    adc_init();
    ping_init();
    servo_init();
    
    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);

    lcd_printf("Diagnostic Mode\nReady.");
    send_telemetry("Warehouse Telemetry Link Established.\r\n");

    print_menu();

    char tx_buffer[80];
    bool continuous_test = false;
    char current_test = '0';
    oi_setWheels(0, 0);

    while (1) {
        
        if (g_command_ready) {
            char cmd = g_command_byte;
            g_command_ready = false; 

            if (cmd == ' ' || cmd == 'x' || cmd == '9') {
                continuous_test = false;
                current_test = '0';
                oi_setWheels(0, 0);
                uart_sendStr("\r\n[HALTED] Tests stopped. Motors killed.\r\n");
                print_menu();
            } 
            else if (cmd >= '1' && cmd <= '8') {
                continuous_test = true;
                current_test = cmd;
                sprintf(tx_buffer, "\r\n--- Starting Test %c ---\r\n", current_test);
                uart_sendStr(tx_buffer);
            }
        }

        if (continuous_test) {
            
            if (current_test >= '5' && current_test <= '8') {
                oi_update(sensor_data);
            }

            switch (current_test) {
                
                case '1': // Test ADC 
                {
                    uint16_t raw_ir = adc_read_avg();
                    int dist_cm = ir_distance_from_adc(raw_ir);
                    sprintf(tx_buffer, "IR Raw: %d | Calc Dist: %d cm\r\n", raw_ir, dist_cm);
                    uart_sendStr(tx_buffer);
                    timer_waitMillis(250); 
                    break;
                }
                case '2': // Test PING
                {
                    float ping_dist = ping_getDistance();
                    sprintf(tx_buffer, "PING Dist: %.2f cm\r\n", ping_dist);
                    uart_sendStr(tx_buffer);
                    timer_waitMillis(250);
                    break;
                }
                case '3': // Test Servo
                    uart_sendStr("Moving to 0...\r\n");
                    servo_move(0);
                    timer_waitMillis(500);
                    
                    uart_sendStr("Moving to 90...\r\n");
                    servo_move(90);
                    timer_waitMillis(500);
                    
                    uart_sendStr("Moving to 180...\r\n");
                    servo_move(180);
                    timer_waitMillis(500);
                    
                    continuous_test = false; 
                    print_menu();
                    break;

                case '4': // Test Sweep
                {
                    uart_sendStr("Initiating Full Sweep...\r\n");
                    DetectedObject my_objects[10];
                    perform_advanced_sweep(my_objects, 10);
                    
                    uart_sendStr("Sweep Complete. Returning to 90.\r\n");
                    servo_move(90);
                    continuous_test = false; 
                    print_menu();
                    break;
                }
                case '5': // Test Movement
                    uart_sendStr("Driving Forward 100mm...\r\n");
                    move_forward(sensor_data, 100, 100);
                    timer_waitMillis(500);
                    
                    uart_sendStr("Driving Backward 100mm...\r\n");
                    move_backward(sensor_data, 100, 100);
                    
                    continuous_test = false; 
                    print_menu();
                    break;

                case '6': // Test Bump Sensors
                    sprintf(tx_buffer, "BUMPS -> Left: %d | Right: %d\r\n", sensor_data->bumpLeft, sensor_data->bumpRight);
                    uart_sendStr(tx_buffer);
                    
                    if (sensor_data->bumpLeft || sensor_data->bumpRight) {
                        lcd_printf("BUMP DETECTED!");
                    } else {
                        lcd_printf("Path clear.");
                    }
                    timer_waitMillis(200);
                    break;

                case '7': // Test Cliff Sensors (Holes)
                    sprintf(tx_buffer, "CLIFF -> L: %d | FL: %d | FR: %d | R: %d\r\n", 
                            sensor_data->cliffLeft, sensor_data->cliffFrontLeft, 
                            sensor_data->cliffFrontRight, sensor_data->cliffRight);
                    uart_sendStr(tx_buffer);
                    timer_waitMillis(250);
                    break;

                case '8': // Test Boundary Sensors (White Tape Signals)
                    // prints the raw analog values so you can see where your 2600 threshold hits
                    sprintf(tx_buffer, "TAPE (Raw) -> L: %d | FL: %d | FR: %d | R: %d\r\n", 
                            sensor_data->cliffLeftSignal, sensor_data->cliffFrontLeftSignal, 
                            sensor_data->cliffFrontRightSignal, sensor_data->cliffRightSignal);
                    uart_sendStr(tx_buffer);
                    timer_waitMillis(250);
                    break;

                default:
                    continuous_test = false;
                    break;
            }
        }
    }
    
    oi_free(sensor_data);
    return 0;
}

void update_odometry(oi_t *sensor_data) {
    current_heading += sensor_data->angle;
    while(current_heading >= 360) current_heading -= 360;
    while(current_heading < 0) current_heading += 360;

    float distance_cm = sensor_data->distance / 10.0;

    //convert from polar
    if (distance_cm != 0) {
        float heading_rad = current_heading * (PI / 180.0);
        current_x += distance_cm * cos(heading_rad);
        current_y += distance_cm * sin(heading_rad);
    }
}

void send_gui_telemetry(oi_t *sensor_data) {
    char gui_msg[120];

    // format: TEL:X,Y,Heading,L_Cliff,R_Cliff,FL_Cliff,FR_Cliff,LBump,RBump
    sprintf(gui_msg, "TEL:%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d\r\n",
            current_x, current_y, current_heading,
            sensor_data->cliffLeftSignal, sensor_data->cliffRightSignal,
            sensor_data->cliffFrontLeftSignal, sensor_data->cliffFrontRightSignal,
            sensor_data->bumpLeft, sensor_data->bumpRight);

    uart_sendStr(gui_msg);
}

void send_telemetry(const char* message) {

    //possibly imrpove to add the abilty to send multiple chars or a full file of text
    uart_sendStr(message);
}

int check_hazards(oi_t *sensor_data) {
    if (1 == sensor_data->bumpLeft) return 1;
    if (1 == sensor_data->bumpRight) return 2;

    //drop-off / hole detection (Codes 3-4)
    if (sensor_data->cliffLeftSignal <= 200 || sensor_data->cliffFrontLeftSignal <= 200) {
        return 3;
    }
    if (sensor_data->cliffRightSignal <= 200 || sensor_data->cliffFrontRightSignal <= 200) {
        return 4;
    }

    // boundary / white tape detection (Codes 5-6)
    //commented out cause the floor is all white on non test field
    //if (sensor_data->cliffLeftSignal >= 2600 || sensor_data->cliffFrontLeftSignal >= 2600) {
    //    return 5; // Left Boundary
    //}
    //if (sensor_data->cliffRightSignal >= 2600 || sensor_data->cliffFrontRightSignal >= 2600) {
    //    return 6; // Right Boundary
    //}

    return 0;
}
