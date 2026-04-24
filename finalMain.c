#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <inc/tm4c123gh6pm.h>

#include "Timer.h"            // For wait functions (timer_waitMillis, etc.)
#include "lcd.h"              // Local LCD debugging
#include "button.h"           // Onboard push buttons
#include "open_interface.h"   // iRobot Create 2 Base (bumpers, wheels, cliffs)
#include "uart-interrupt.h"   // Non-blocking UART communication
#include "adc.h"              // IR Sensor driver
#include "ping_template.h"    // Ultrasonic sensor driver
#include "servo.h"            // Servo PWM driver
#include "movement.h"         // Hazard-safe movement logic
#include "scanner.h"          // Sensor fusion wrapper (IR + Ping + Servo)

extern volatile char command_received;
extern volatile int command_flag;

//Auto and Manual
typedef enum {
    MODE_MANUAL,
    MODE_AUTO
} SystemMode;

typedef enum {
    AUTO_STATE_SCANNING,
    AUTO_STATE_DRIVING,
    AUTO_STATE_AVOIDING,
    AUTO_STATE_EMERGENCY,
    AUTO_STATE_DONE
} AutoState;

// Global state trackers
SystemMode current_mode = MODE_MANUAL;
AutoState current_auto_state = AUTO_STATE_SCANNING;
// Add these to the top of main.c
volatile float current_x = 0.0;
volatile float current_y = 0.0;
volatile int current_heading = 90;

#define PI 3.14159265
//Guess
#define IR_JUMP_THRESHOLD 200
#define MIN_OBJECT_WIDTH_DEG 6

typedef struct {
    uint8_t id;
    uint8_t s_angle;
    uint8_t e_angle;
    uint16_t dist_cm;
    uint8_t l_width_cm;
    int16_t x;
    int16_t y;
    int16_t heading;
    //maybe some others data usage shouldn't matter too much if we output to a file
} detectedObj;

oi_t *sensor_data;
char tx_buffer[100];

// --- Helper Function Prototypes ---
void update_odometry(oi_t *sensor_data);
void send_telemetry(const char* message);
int check_hazards(oi_t *sensor_data);

int main(void) {
    // 1. Initialize all hardware modules
    timer_init();
    lcd_init();
    uart_interrupt_init();
    adc_init();
    ping_init();
    servo_init();
    
    // 2. Initialize Roomba Base
    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);


    
    lcd_printf("System Ready.\nWaiting for GUI...");
	uart_sendStr("CYBOT INITIALIZED. WAITING FOR COMMANDS.\n");

    //main loop
    while (1) {

        oi_update(sensor_data);
        update_odometry(sensor_data);


        if (command_received != '\0') {
            
            // Global overrides
            if (command_received == 'm') {
                current_mode = MODE_MANUAL;
                lcd_printf("Mode: MANUAL\nReady.");
                uart_sendStr("STATUS: Switched to MANUAL mode.\n");
            } 
            else if (command_received == 'g') { // 'g' for "Go Auto"
                current_mode = MODE_AUTO;
                current_auto_state = AUTO_STATE_SCANNING; // Reset auto state
                lcd_printf("Mode: AUTO\nInitiating...");
                uart_sendStr("STATUS: Switched to AUTO mode.\n");
            }
            else if (command_received == 'x') { // Emergency Stop
                oi_setWheels(0, 0);
                current_mode = MODE_MANUAL;
                lcd_printf("EMERGENCY STOP");
                uart_sendStr("STATUS: EMERGENCY STOP. Mode reverted to Manual.\n");
            }
		else if (current_mode == MODE_MANUAL) {
            switch (command_received) {
                case 'c': //calibration state
                    lcd_printf("CALIBRATING...");
                    uart_sendStr("STATUS: Running calibration routine...\n");
                    
                    // servo_calibrate(); 
                    
                   //possible future stuff
                    
                    uart_sendStr("STATUS: Calibration Complete.\n");
                    lcd_printf("Calibrated.\nReady.");
                    break;

                case 'w': // Forward
                    lcd_printf("Moving Forward");
                    uart_sendStr("STATUS: Moving Forward\n");
                    //move_forward_safe(sensor_data, 10);
                    break;
                    
                case 's': // Reverse
                    lcd_printf("Reversing");
                    uart_sendStr("STATUS: Reversing\n");
                    // move_backward_safe(sensor_data, 10);
                    break;
                    
                case 'a': // Turn Left
                    lcd_printf("Turning Left");
                    uart_sendStr("STATUS: Turning Left\n");
                    // turn_left(sensor_data, 5);
                    break;
                    
                case 'd': // Turn Right
                    lcd_printf("Turning Right");
                    uart_sendStr("STATUS: Turning Right\n");
                    // turn_right(sensor_data, 5);
                    break;
                    
                case 'p': // Perform full sensor sweep
                    lcd_printf("Scanning...");
                    uart_sendStr("STATUS: Initiating Sweep\n");
                    perform_sweep(); 
                    break;
                    

                default:
                    uart_sendStr("ERROR: Unknown command received.\n");
                    break;
            }
			}
            
            command_received = '\0'; 
			lcd_printf("Ready.");
        }
		
		if (current_mode == MODE_AUTO) {
            
            switch (current_auto_state) {
                
                case AUTO_STATE_SCANNING:
                    lcd_printf("AUTO: Scanning...");
                    // Perform sweep
                    // Analyze data array to find the target gap/object
                    // Calculate turn angle to face target
                    // turn_right(sensor_data, calculated_angle);
                    
                    // Transition to driving
                    current_auto_state = AUTO_STATE_DRIVING;
                    break;
                    
                case AUTO_STATE_DRIVING:
                    lcd_printf("AUTO: Driving...");
                    
                    // Call a modified move function that returns an int status
                    // 0 = arrived safely, 1 = bump, 2 = cliff, 3 = boundary
                    int drive_status = move_forward_safe_auto(sensor_data, target_distance);
                    
                    if (drive_status == 0) {
						// Moved 20cm safely! 
						
						// CHECK WIN CONDITION: Did we cross the 2-meter finish line?
						if (current_y >= 2000) {
							current_auto_state = AUTO_STATE_DONE;
						} else {
							current_auto_state = AUTO_STATE_SCANNING; 
						}
                    } else {
                        current_auto_state = AUTO_STATE_AVOIDING;
                    }
                    break;

                case AUTO_STATE_EMERGENCY:
                    lcd_printf("HAZARD DETECTED!\nHalting.");

                    // Alert PC GUI
                    char tx_buffer[50];
                    sprintf(tx_buffer, "HAZARD:%d,%d,%d,%d\r\n",
                        sensor_data->bumpLeft, sensor_data->bumpRight,
                        sensor_data->cliffLeft, sensor_data->cliffRight);
                    send_telemetry(tx_buffer);

                    // Back up slightly safely
                    move_backward(sensor_data, 100); // Back up 10cm
                    current_auto_state = AUTO_STATE_AVOIDING;
                    break;

                case AUTO_STATE_AVOIDING:
                    lcd_printf("AUTO: Avoiding Hazard!");
                    // Back up slightly
                    // Determine which way to turn based on the hazard
                    // Navigate around
                    
                    // Once clear, go back to scanning to re-orient
                    current_auto_state = AUTO_STATE_SCANNING;
                    break;
                    
                case AUTO_STATE_DONE:
                    lcd_printf("AUTO: Mission Complete");
                    uart_sendStr("STATUS: Auto Mission Complete.\n");
                    current_mode = MODE_MANUAL; // Revert to manual control
                    break;
            }
        }
    }
	
    oi_free(sensor_data);
    return 0;
}


// --- Odometry Implementation ---
void update_odometry(oi_t *sensor_data) {
    // 1. Get relative angle from chassis (Replace with IMU later if implemented)
    current_heading += sensor_data->angle;

    // Keep heading bound between 0 and 359 degrees
    while(current_heading >= 360) current_heading -= 360;
    while(current_heading < 0) current_heading += 360;

    // 2. Calculate distance traveled since last update
    float distance = sensor_data->distance; // in mm

    if (distance != 0) {
        // Convert heading to radians for trig math
        float heading_rad = current_heading * (PI / 180.0);

        // Update X and Y coordinates on the Cartesian plane
        current_x += distance * cos(heading_rad);
        current_y += distance * sin(heading_rad);
    }
}

// --- UART Telemetry & Processing ---
void send_telemetry(const char* message) {
    int i = 0;
    while(message[i] != '\0') {
        uart_sendChar(message[i]); // Assuming uart_sendChar is in uart-interrupt.c
        i++;
    }
}
// --- Hazard Checking ---
int check_hazards(oi_t *sensor_data) {
    // Returns 1 if a bump or cliff is detected, 0 if safe
    if (sensor_data->bumpLeft || sensor_data->bumpRight ||
        sensor_data->cliffLeft || sensor_data->cliffRight ||
        sensor_data->cliffFrontLeft || sensor_data->cliffFrontRight ||
        sensor_data->wheelDropLeft || sensor_data->wheelDropRight) {
        return 1;
    }
    return 0;
}