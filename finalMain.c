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
#include "ping.h"    // Ultrasonic sensor driver
#include "servo.h"            // Servo PWM driver
#include "movement.h"         // Hazard-safe movement logic
#include "scanner.h"          // Sensor fusion wrapper (IR + Ping + Servo)
#include "manualMode.h"

extern volatile char g_command_byte;
extern volatile int g_command_ready;

typedef enum {
    STATE_IDLE,
    STATE_MANUAL_TELEOP,
    STATE_AUTO_SCANNING,
    STATE_AUTO_DRIVING,
    STATE_AUTO_AVOID,
    STATE_AUTO_PARKING,      // New Parking State added!
    STATE_HAZARD_RECOVERY
} RobotState;

typedef struct {
    float x;
    float y;
} VirtualWall;

VirtualWall my_virtual_walls[20];
int num_virtual_walls = 0;

static RobotState current_state = STATE_IDLE;
static RobotState previous_state = STATE_IDLE; 
volatile int g_hazard_flag = 0; 


#define PI 3.14159265
//Guess
#define IR_JUMP_THRESHOLD 200
#define MIN_OBJECT_WIDTH_DEG 6
#define ROBOT_WIDTH 34

oi_t *sensor_data;
char tx_buffer[100];
volatile float current_x = 0.0;
volatile float current_y = 0.0;
volatile int current_heading = 90;


// ==============================================================================
// Helper Function Prototypes
// ==============================================================================
void update_odometry(oi_t *sensor_data);
void send_telemetry(const char* message);
void send_gui_telemetry(oi_t *sensor_data);
int check_hazards(oi_t *sensor_data);

int main(void) {
    // 1. Initialize all hardware modules
    timer_init();
    lcd_init();
	button_init();
    uart_interrupt_init();
	adc_init();
	ping_init();
	servo_init();
    
    // 2. Initialize Roomba Base
    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);

    oi_setWheels(0, 0);

    
    lcd_printf("Logistics Bot\nStatus: IDLE");
    send_telemetry("Warehouse Telemetry Link Established.\r\n");
	send_telemetry("Commands: [m] Manual | [a] Auto | [x] E-STOP\r\n> ");
	float parking_drive_distance = 0.0;

    //main loop
    while (1) {



        oi_update(sensor_data);
        update_odometry(sensor_data);
        send_gui_telemetry(sensor_data);
		
        if(!(current_state == STATE_IDLE)){

		// Check hardware for physical or terrain hazards
        int current_hazard = check_hazards(sensor_data);
        if (current_hazard != 0 && g_hazard_flag == 0) {
            oi_setWheels(0, 0); // IMMEDIATE STOP
            g_hazard_flag = current_hazard;
        }

        // Preempt FSM if hazard detected
        if (g_hazard_flag != 0 && current_state != STATE_HAZARD_RECOVERY) {
            previous_state = current_state; 
            current_state = STATE_HAZARD_RECOVERY;
        }
        }
		// ----------------------------------------------------------------------
        // B. ASYNCHRONOUS UART COMMAND PARSING (E-STOP & MODE SWITCHES)
        // ----------------------------------------------------------------------
        if (g_command_ready) { 
		// EMERGENCY STOP (Spacebar or 'x')
            if (g_command_byte == ' ' || g_command_byte == 'x') {
                oi_setWheels(0, 0); // IMMEDIATE HALT
                current_state = STATE_MANUAL_TELEOP; // Kick to manual
                g_hazard_flag = 0; // Clear hazard flags so we don't get stuck
                lcd_printf("EMERGENCY STOP\nMode: MANUAL");
                send_telemetry("\r\n[EMERGENCY STOP ACTIVATED] Reverting to Manual.\r\n> ");
                g_command_ready = false; 
            }else if ('r' == g_command_byte) {
				oi_setWheels(0, 0);
				current_state = STATE_IDLE;
				g_hazard_flag = 0;
				
				// 1. Clear internal memory
				num_virtual_walls = 0;
				
				// 2. Reset Odometry to starting point
				current_x = 0.0;
				current_y = 0.0;
				current_heading = 90;
				
				// 3. Send a special packet to tell the GUI to clear its visual map
				send_telemetry("CMD:RESET_MAP\r\n");
				
				send_telemetry("\r\n[SYSTEM RESET] Map, Odometry, and Walls Cleared. Standing by in IDLE.\r\n> ");
				lcd_printf("Status: IDLE\nMap Cleared");
				
				g_command_ready = false;
			}
			
            else if ('m' == g_command_byte) {
                current_state = STATE_MANUAL_TELEOP;
                oi_setWheels(0, 0);
                send_telemetry("\r\n[MODE: MANUAL TELEOP]\r\n> ");
                lcd_printf("Mode: MANUAL");
                g_command_ready = false; // Consume command
            } 
            else if ('a' == g_command_byte) {
                current_state = STATE_AUTO_SCANNING;
                g_hazard_flag = 0; 
                send_telemetry("\r\n[MODE: AUTONOMOUS LOGISTICS]\r\n> ");
                lcd_printf("Mode: AUTO");
                g_command_ready = false; 
            }
            // Add custom trigger for testing the parking state
            else if ('p' == g_command_byte && current_state != STATE_MANUAL_TELEOP) {
                current_state = STATE_AUTO_PARKING;
                g_command_ready = false;
            }
        }
		
        int num_objs_found;
        int move_status;

		// ----------------------------------------------------------------------
        // C. FINITE STATE MACHINE (FSM)
        // ----------------------------------------------------------------------
        switch (current_state) {
            
            case STATE_IDLE:
                // Awaiting mode selection
                break;

            case STATE_MANUAL_TELEOP:
                if (g_command_ready) {
                    manual_handle_command(g_command_byte, sensor_data);
                    g_command_ready = false; // Consume command
                }
                break;

            case STATE_AUTO_SCANNING:
                oi_setWheels(0, 0);
				send_telemetry("Scanning for aisle path...\r\n");
				
				// 1. Sweep and fill the array
                DetectedObject my_objects[10];
                num_objs_found = perform_advanced_sweep(my_objects, 10);
				
				inject_virtual_walls(my_objects, &num_objs_found);

                // 2. [Optional] Find and print the smallest object if you need it for GUI/logs
                DetectedObject smallest = find_smallest_object(my_objects, num_objs_found);

				// 2. CHECK FOR PARKING ZONE FIRST (Highest Priority)
				DetectedGap target_gap = check_for_parking_zone(my_objects, num_objs_found);
				
				if (target_gap.id != -1) {
					send_telemetry("Destination Zone Detected! Initiating Parking Sequence...\r\n");
					int turn_angle = target_gap.c_angle - 90;
                    if (turn_angle > 0) { turn_left(sensor_data, turn_angle, 100); } 
                    else if (turn_angle < 0) { turn_right(sensor_data, abs(turn_angle), 100); }
                    
                    // Save the calculated distance to the zone
                    parking_drive_distance = target_gap.dist_cm;
					
					current_state = STATE_AUTO_PARKING; 
				} 
				else {
					// 3. No parking zone found. Proceed with normal warehouse navigation.
					DetectedGap my_gaps[10];
					int num_gaps_found = calculate_all_gaps(my_objects, num_objs_found, my_gaps, 10);
					
					// Find a gap wide enough for the 35cm bot + safety margin
					target_gap = find_best_driveable_gap(my_gaps, num_gaps_found, ROBOT_WIDTH);
					
					if (target_gap.id != -1) {
						int turn_angle = target_gap.c_angle - 90;
						if (turn_angle > 0) { turn_left(sensor_data, turn_angle, 100); } 
						else if (turn_angle < 0) { turn_right(sensor_data, abs(turn_angle), 100); }
						
						current_state = STATE_AUTO_DRIVING;
					} else {
						send_telemetry("BLOCKED: No safe gaps. Reversing out...\r\n");
						move_backward(sensor_data, 100, 100);
						turn_left(sensor_data, 90, 100); 
					}
				}
				break;
				//Maybe we add a global distance variable that this could intellgiently set
				//we could then use in the driving state
				//This is the most important part and we might need some intelligent helper functions that can determine the best route

            case STATE_AUTO_DRIVING:
                // Move forward using your newly updated safe movement wrapper!
                // It will block here until it reaches 250mm OR hits a hazard/interrupt.
                move_status = move_forward(sensor_data, 250, 150); // Speed = 150
                
                if (move_status == 0) {
                    // Successfully drove the segment, re-scan to adjust path
                    current_state = STATE_AUTO_SCANNING;
                } else if (move_status > 0) {
                    // Safe movement returned a hazard code, pass it to global recovery
                    g_hazard_flag = move_status; 
                }
                // (If move_status == -1, user triggered an e-stop, loop repeats and handles it above)
                break;

            case STATE_HAZARD_RECOVERY:
                oi_setWheels(0, 0);
				
				lcd_printf("HAZARD DETECTED!\nHalting.");

				// Alert PC GUI
				char tx_buffer[50];
				sprintf(tx_buffer, "HAZARD:%d,%d,%d,%d\r\n",
					sensor_data->bumpLeft, sensor_data->bumpRight,
					sensor_data->cliffLeft, sensor_data->cliffRight);
				send_telemetry(tx_buffer);
				
				if (g_hazard_flag >= 5 && num_virtual_walls < 20) {
					// Tape/Hole is roughly 15cm directly in front of the robot's center
					float heading_rad = current_heading * (PI / 180.0);
					my_virtual_walls[num_virtual_walls].x = current_x + (15.0 * cos(heading_rad));
					my_virtual_walls[num_virtual_walls].y = current_y + (15.0 * sin(heading_rad));
					num_virtual_walls++;
					send_telemetry("Virtual Wall Recorded.\r\n");
				}				

                if (1 == g_hazard_flag) { // Left Bump
                    send_telemetry("HAZARD: Left Bump! Turning right...\r\n");
                    move_backward(sensor_data, 20, 20);
                    turn_right(sensor_data, 90, 100);
                    current_state = STATE_AUTO_AVOID;
                } 
                else if (2 == g_hazard_flag) { // Right Bump
                    send_telemetry("HAZARD: Right Bump! Turning left...\r\n");
                    move_backward(sensor_data, 20, 20);
                    turn_left(sensor_data, 90, 100);
                    current_state = STATE_AUTO_AVOID;
                } 
                else if (3 == g_hazard_flag) { // Left Drop-off
                    send_telemetry("CRITICAL: Left Drop-off! Reversing and turning right.\r\n");
                    move_backward(sensor_data, 30, 20);
                    turn_right(sensor_data, 120, 100); 
                    current_state = STATE_AUTO_SCANNING;
                    g_hazard_flag = 0; // Clear flag after recovery actions

                }
                else if (4 == g_hazard_flag) { // Right Drop-off
                    send_telemetry("CRITICAL: Right Drop-off! Reversing and turning left.\r\n");
                    move_backward(sensor_data, 30, 20);
                    turn_left(sensor_data, 120, 100); 
                    current_state = STATE_AUTO_SCANNING;
                    g_hazard_flag = 0; // Clear flag after recovery actions

                }
                else if (5 == g_hazard_flag) { // Left Boundary
                    send_telemetry("WARNING: Left Safe Zone Boundary! Turning right.\r\n");
                    move_backward(sensor_data, 40, 30); 
                    turn_right(sensor_data, 120, 100);  
                    current_state = STATE_AUTO_SCANNING;
                    g_hazard_flag = 0; // Clear flag after recovery actions

                }
                else if (6 == g_hazard_flag) { // Right Boundary
                    send_telemetry("WARNING: Right Safe Zone Boundary! Turning left.\r\n");
                    move_backward(sensor_data, 40, 30); 
                    turn_left(sensor_data, 120, 100);  
                    current_state = STATE_AUTO_SCANNING;
                    g_hazard_flag = 0; // Clear flag after recovery actions

                }
                
                break;

            case STATE_AUTO_AVOID:
                send_telemetry("Executing Avoidance Maneuver...\r\n");
                //Should probably make this smarter and maybe use previous scan data or scan itself
				//to smartely avoid and not get stuck
                // Drive forward along the face of the object (Speed = 150)
                move_forward(sensor_data, 250, 150); 
                
                // Turn back to parallel original path
                if (1 == g_hazard_flag) {
                    turn_left(sensor_data, 90, 100); // Speed = 100
                } else {
                    turn_right(sensor_data, 90, 100); // Speed = 100
                }
                
                send_telemetry("Obstacle cleared. Resuming path.\r\n");
                current_state = STATE_AUTO_DRIVING;
                g_hazard_flag = 0; // Clear flag after recovery actions
                break;
                
            case STATE_AUTO_PARKING:
                // Parking Logistics logic (Navigating the 4 pillars)
                send_telemetry("Commencing Precision Parking Sequence...\r\n");
                
                // Drive the distance calculated by the scanner at a slow, precise speed
                if (parking_drive_distance > 0) {
                    move_forward(sensor_data, parking_drive_distance, 50); // Speed = 50
                }
                
                // Ensure the robot is fully stopped
                oi_setWheels(0, 0); 
                
                // Print the exact phrase required by the project constraints
                lcd_printf("Delivery Complete.");
                send_telemetry("Destination Reached! Delivery Complete.\r\n");
                
                // Mission Accomplished, return to idle
                current_state = STATE_IDLE;
                
                // 1. Utilize IR and PING to verify width between pillars
                // 2. Drive slowly into zone
                // move_forward(sensor_data, 150, 50); // Speed = 50 for precision
                
                send_telemetry("Parking Complete! Payload ready for drop-off.\r\n");
                current_state = STATE_IDLE; // Mission Accomplished
                break;

            default:
                current_state = STATE_IDLE;
                break;
        }
        
        timer_waitMillis(10); // Loop stability
    }
    
    oi_free(sensor_data);
    return 0;
}


void update_odometry(oi_t *sensor_data) {
    // Update Heading (Using wheel encoders until IMU is ready)
    current_heading += sensor_data->angle;

    while(current_heading >= 360) current_heading -= 360;
    while(current_heading < 0) current_heading += 360;

	float distance_cm = sensor_data->distance / 10.0;

    // Update X/Y coordinates using Cartesian math
    if (distance != 0) {
        float heading_rad = current_heading * (PI / 180.0);
        current_x += distance * cos(heading_rad);
        current_y += distance * sin(heading_rad);
    }
}

void send_gui_telemetry(oi_t *sensor_data) {
    char gui_msg[120];
    // The target GUI tracks analog cliff signals, not just binary flags.
    // Format: TEL:X,Y,Heading,L_Cliff,R_Cliff,FL_Cliff,FR_Cliff,LBump,RBump
    sprintf(gui_msg, "TEL:%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d\r\n",
            current_x, current_y, current_heading,
            sensor_data->cliffLeftSignal, sensor_data->cliffRightSignal,
            sensor_data->cliffFrontLeftSignal, sensor_data->cliffFrontRightSignal,
            sensor_data->bumpLeft, sensor_data->bumpRight);

    uart_sendStr(gui_msg);
}

void send_telemetry(const char* message) {
    // Wrapper to easily send strings, useful if you ever change UART ports
	//Possibly imrpove to add the abilty to send multiple chars or a full file of text 
    uart_sendStr(message);
}

int check_hazards(oi_t *sensor_data) {
    // 1. Bump Sensors (Codes 1-2)
    if (1 == sensor_data->bumpLeft) return 1;
    if (1 == sensor_data->bumpRight) return 2;
    
    // 2. Drop-off / Hole Detection (Codes 3-4)
    if (sensor_data->cliffLeftSignal <= 200 || sensor_data->cliffFrontLeftSignal <= 200) {
        return 3; // Left Drop-off
    }
    if (sensor_data->cliffRightSignal <= 200 || sensor_data->cliffFrontRightSignal <= 200) {
        return 4; // Right Drop-off
    }
    
    // 3. Boundary / White Tape Detection (Codes 5-6)
    //Commented out cause the floor is all white on non test field
    //if (sensor_data->cliffLeftSignal >= 2600 || sensor_data->cliffFrontLeftSignal >= 2600) {
    //    return 5; // Left Boundary
    //}
    //if (sensor_data->cliffRightSignal >= 2600 || sensor_data->cliffFrontRightSignal >= 2600) {
    //    return 6; // Right Boundary
    //}
    
    return 0; // Safe
}

void inject_virtual_walls(DetectedObject objects[], int *num_objs) {
    int i;
    for (i = 0; i < num_virtual_walls; i++) {
        // 1. Calculate distance from robot to this saved wall
        float dx = my_virtual_walls[i].x - current_x;
        float dy = my_virtual_walls[i].y - current_y;
        float dist = sqrt((dx * dx) + (dy * dy));
        
        // 2. Calculate the absolute angle to the wall
        float abs_angle_rad = atan2(dy, dx);
        int abs_angle_deg = abs_angle_rad * (180.0 / PI);
        if (abs_angle_deg < 0) abs_angle_deg += 360;
        
        // 3. Convert absolute angle to robot-relative angle
        int relative_angle = abs_angle_deg - current_heading + 90;
        
        // Normalize to 0-360
        while (relative_angle < 0) relative_angle += 360;
        while (relative_angle >= 360) relative_angle -= 360;
        
        // 4. If the wall is in front of us (0-180 deg) and close by (< 80cm)
        if (relative_angle >= 0 && relative_angle <= 180 && dist < 80.0 && *num_objs < 10) {
            
            // Inject a 30cm wide "Ghost Object" to block the gap!
            objects[*num_objs].id = 900 + i; // Special ID for ghost walls
            objects[*num_objs].dist_cm = dist;
            objects[*num_objs].c_angle = relative_angle;
            
            // Make the ghost object artificially wide so the robot won't try to squeeze past it
            objects[*num_objs].l_width_cm = 30.0; 
            objects[*num_objs].s_angle = relative_angle - 15;
            if(objects[*num_objs].s_angle < 0) objects[*num_objs].s_angle = 0;
            objects[*num_objs].e_angle = relative_angle + 15;
            if(objects[*num_objs].e_angle > 180) objects[*num_objs].e_angle = 180;
            
            (*num_objs)++;
            
            send_telemetry("Injected Virtual Wall into scan data!\r\n");
        }
    }
}


