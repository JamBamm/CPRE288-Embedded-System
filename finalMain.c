#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <inc/tm4c123gh6pm.h>

#include "Timer.h"           
#include "lcd.h"              
#include "button.h"           
#include "open_interface.h"   
#include "uart-interrupt.h"   
#include "adc.h"              
#include "ping.h"    
#include "servo.h"            
#include "movement.h"         
#include "scanner.h"          
#include "manualMode.h"
#include "avoidance.h"
#include "IMU/theimu.h"


extern volatile char g_command_byte;
extern volatile bool g_command_ready;

typedef enum {
    STATE_IDLE,
    STATE_MANUAL_TELEOP,
    STATE_AUTO_SCANNING,
    STATE_AUTO_DRIVING,
    // STATE_AUTO_AVOID,
    STATE_AUTO_PARKING,     
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
volatile float target_drive_distance = 0.0;

volatile bool use_imu = false;


void update_odometry(oi_t *sensor_data);
void send_telemetry(const char* message);
void send_gui_telemetry(oi_t *sensor_data);
int check_hazards(oi_t *sensor_data);

int main(void) {
    //init
    timer_init();
    lcd_init();
	button_init();
    uart_interrupt_init();
	adc_init();
	ping_init();
	servo_init();
	imu_init();
    timer_waitMillis(100);

	
    if (imu_connected()) {
        use_imu = true;
        imu_writeReg(IMU_OPR_MODE, NDOF);
        imu_setDefaultUnits();
        send_telemetry("SYSTEM: IMU Detected. Using absolute heading.\r\n");
    } else {
        use_imu = false;
        send_telemetry("SYSTEM: IMU NOT DETECTED. Falling back to wheel encoders.\r\n");
    }
    
    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);

    oi_setWheels(0, 0);

    int telemetry_counter = 0;
	
    lcd_printf("Logistics Bot\nStatus: IDLE");
    send_telemetry("Warehouse Telemetry Link Established.\r\n");
	send_telemetry("Commands: [m] Manual | [a] Auto | [x] E-STOP\r\n> ");
	float parking_drive_distance = 0.0;

    //main loop
    while (1) {



        oi_update(sensor_data);
        update_odometry(sensor_data);
		
		telemetry_counter++;
        if (telemetry_counter >= 20) {
            send_gui_telemetry(sensor_data);
            telemetry_counter = 0;
        }
		
        if(!(current_state == STATE_IDLE)){

		//check hazrads
        int current_hazard = check_hazards(sensor_data);
        if (current_hazard != 0 && g_hazard_flag == 0) {
            oi_setWheels(0, 0); 
            g_hazard_flag = current_hazard;
        }

        // go to hazard state
        if (g_hazard_flag != 0 && current_state != STATE_HAZARD_RECOVERY && current_state != STATE_MANUAL_TELEOP) {
            previous_state = current_state; 
            current_state = STATE_HAZARD_RECOVERY;
        }
        }
		
		//state commands 
        if (g_command_ready) { 
		
			//stop used for auto
            if (g_command_byte == ' ' || g_command_byte == 'x') {
                oi_setWheels(0, 0); 
                current_state = STATE_MANUAL_TELEOP;
                g_hazard_flag = 0; 
                lcd_printf("EMERGENCY STOP\nMode: MANUAL");
                send_telemetry("\r\n[EMERGENCY STOP ACTIVATED] Reverting to Manual.\r\n> ");
                g_command_ready = false; 
				
			//idle reset
            }else if ('r' == g_command_byte) {
				oi_setWheels(0, 0);
				current_state = STATE_IDLE;
				g_hazard_flag = 0;
				
				num_virtual_walls = 0;
				
				current_x = 0.0;
				current_y = 0.0;
				current_heading = 90;
				
				send_telemetry("CMD:RESET_MAP\r\n");
				
				send_telemetry("\r\n[SYSTEM RESET] Map, Odometry, and Walls Cleared. Standing by in IDLE.\r\n> ");
				lcd_printf("Status: IDLE\nMap Cleared");
				
				g_command_ready = false;
			}
			//manual
            else if ('m' == g_command_byte) {
                current_state = STATE_MANUAL_TELEOP;
                oi_setWheels(0, 0);
                send_telemetry("\r\n[MODE: MANUAL TELEOP]\r\n> ");
                lcd_printf("Mode: MANUAL");
                g_command_ready = false; 
            } 
			//auto
            else if ('a' == g_command_byte) {
                current_state = STATE_AUTO_SCANNING;
                g_hazard_flag = 0; 
                send_telemetry("\r\n[MODE: AUTONOMOUS LOGISTICS]\r\n> ");
                lcd_printf("Mode: AUTO");
                g_command_ready = false; 
            }
			//test parking state
            else if ('p' == g_command_byte && current_state != STATE_MANUAL_TELEOP) {
                current_state = STATE_AUTO_PARKING;
                g_command_ready = false;
            }
			else if (current_state == STATE_IDLE) {
                g_command_ready = false;
            }
        }
		
        int num_objs_found;
        int move_status;

		//state machine for auto mainly
        switch (current_state) {
            
            case STATE_IDLE:
                break;

            case STATE_MANUAL_TELEOP:
                if (g_command_ready) {
                    manual_handle_command(g_command_byte, sensor_data);
                    g_command_ready = false; 
                }
                break;

            case STATE_AUTO_SCANNING:
                oi_setWheels(0, 0);
				send_telemetry("Scanning for aisle path...\r\n");
				
				//scan
                DetectedObject my_objects[10];
                num_objs_found = perform_advanced_sweep(my_objects, 10);
				
				//if bump or cliff inject as object
				inject_virtual_walls(my_objects, &num_objs_found);

                DetectedObject smallest = find_smallest_object(my_objects, num_objs_found);

				//check for parking zone
				DetectedGap target_gap = check_for_parking_zone(my_objects, num_objs_found);
				
				//parking found
				if (target_gap.id == 100) { 
                    // 2+ PILLARS FOUND
                    send_telemetry("Destination Zone Detected! Initiating Parking Sequence...\r\n");
                    int turn_angle = target_gap.c_angle - 90;
                    if (turn_angle > 0) { turn_left(sensor_data, turn_angle, 100); } 
                    else if (turn_angle < 0) { turn_right(sensor_data, abs(turn_angle), 100); }
                    
                    if (target_gap.dist_cm > 60.0) {
                        target_drive_distance = (target_gap.dist_cm * 10.0) - 400.0; 
                        current_state = STATE_AUTO_DRIVING;
                    } else {
                        parking_drive_distance = target_gap.dist_cm * 10.0;
                        current_state = STATE_AUTO_PARKING; 
                    }
                } 
                else if (target_gap.id == 99) { 
                    // ONLY 1 PILLAR FOUND
                    send_telemetry("1 Pillar found. Approaching to rescan...\r\n");
                    int turn_angle = target_gap.e_angle - 90; 
                    if (turn_angle > 0) { turn_left(sensor_data, turn_angle, 100); } 
                    else if (turn_angle < 0) { turn_right(sensor_data, abs(turn_angle), 100); }
                    
                    target_drive_distance = target_gap.dist_cm * 10.0; 
                    if (target_drive_distance < 50.0) target_drive_distance = 50.0;
                    
                    current_state = STATE_AUTO_DRIVING; 
                }
                else {
					DetectedGap my_gaps[10];
					int num_gaps_found = calculate_all_gaps(my_objects, num_objs_found, my_gaps, 10);
					
					//find gap
					target_gap = find_best_driveable_gap(my_gaps, num_gaps_found, ROBOT_WIDTH);
					
					if (target_gap.id != -1) {
                        int turn_angle = target_gap.c_angle - 90;
                        if (turn_angle > 0) { turn_left(sensor_data, turn_angle, 100); } 
                        else if (turn_angle < 0) { turn_right(sensor_data, abs(turn_angle), 100); }
                        
                        target_drive_distance = (target_gap.dist_cm * 10.0) - 150.0;
                        
                        if (target_drive_distance < 50.0) target_drive_distance = 50.0; // Minimum step
                        
                        current_state = STATE_AUTO_DRIVING;
                    } else {
                        send_telemetry("BLOCKED: No safe gaps. Turning around...\r\n");
                        turn_left(sensor_data, 180, 100); 
                    }
				}
				break;
				//Maybe we add a global distance variable that this could intellgiently set
				//we could then use in the driving state
				//This is the most important part and we might need some intelligent helper functions that can determine the best route

            case STATE_AUTO_DRIVING:
				
				if (target_drive_distance > 0) {
                   float step = (target_drive_distance < 50.0) ? target_drive_distance : 50.0;
                    
                   move_status = move_forward(sensor_data, step, 100);
                    
                    if (move_status == 0) {
                        target_drive_distance -= 50.0;
                    } else if (move_status > 0) {
                        g_hazard_flag = move_status; 
                        previous_state = current_state;
                        current_state = STATE_HAZARD_RECOVERY;
                        target_drive_distance = 0.0; 
                    } else if (move_status == -1) {
                        current_state = STATE_MANUAL_TELEOP;
                        target_drive_distance = 0.0; 
                    }
                } else {
                    current_state = STATE_AUTO_SCANNING;
                }
                break;

            case STATE_HAZARD_RECOVERY:
                oi_setWheels(0, 0);
				
				lcd_printf("HAZARD DETECTED!\nHalting.");
				//gui update
				char tx_buffer[50];
				sprintf(tx_buffer, "HAZARD:%d,%d,%d,%d\r\n",
					sensor_data->bumpLeft, sensor_data->bumpRight,
					sensor_data->cliffLeft, sensor_data->cliffRight);
				send_telemetry(tx_buffer);
				
				//if boundary set as virtual wall
				if (g_hazard_flag >= 1 && num_virtual_walls < 20) {
                    int wall_angle = current_heading;
                    if (g_hazard_flag == 1 || g_hazard_flag == 3 || g_hazard_flag == 5) {
                        wall_angle += 30;
                    } else if (g_hazard_flag == 2 || g_hazard_flag == 4 || g_hazard_flag == 6) {
                        wall_angle -= 30; 
                    }
                    
                    float heading_rad = wall_angle * (PI / 180.0);
                    my_virtual_walls[num_virtual_walls].x = current_x + (17.0 * cos(heading_rad));
                    my_virtual_walls[num_virtual_walls].y = current_y + (17.0 * sin(heading_rad));
                    num_virtual_walls++;
                    send_telemetry("Virtual Wall Recorded.\r\n");
                }
				//need another virtual wall type thing
				//for the other hazrds that allows the scanner to still scan over it 
				//but blocks it in the map as something you can't drive over
				//so it will only every bump or go over a cliff once
				
				int avoid_result = avoid_hazard_super_safe(sensor_data, g_hazard_flag);
				
				if (avoid_result == AVOID_RESULT_DESTINATION_FOUND) {
                    current_state = STATE_AUTO_PARKING;
                } else if (avoid_result == AVOID_RESULT_SAFE_CONTINUE) {
                    current_state = STATE_AUTO_SCANNING; 
                } else {
                    current_state = STATE_MANUAL_TELEOP; 
                }
                
                g_hazard_flag = 0; 
                break;
				
                /* if (1 == g_hazard_flag) { // left Bump
                    send_telemetry("HAZARD: Left Bump! Turning right...\r\n");
                    move_backward(sensor_data, 20, 20);
                    turn_right(sensor_data, 90, 100);
                    current_state = STATE_AUTO_AVOID;
                } 
                else if (2 == g_hazard_flag) { // right Bump
                    send_telemetry("HAZARD: Right Bump! Turning left...\r\n");
                    move_backward(sensor_data, 20, 20);
                    turn_left(sensor_data, 90, 100);
                    current_state = STATE_AUTO_AVOID;
                } 
                else if (3 == g_hazard_flag) { // left Drop-off
                    send_telemetry("CRITICAL: Left Drop-off! Reversing and turning right.\r\n");
                    move_backward(sensor_data, 10, 20);
                    turn_right(sensor_data, 120, 100); 
                    current_state = STATE_AUTO_SCANNING;
                    g_hazard_flag = 0; 

                }
                else if (4 == g_hazard_flag) { // right Drop-off
                    send_telemetry("CRITICAL: Right Drop-off! Reversing and turning left.\r\n");
                    move_backward(sensor_data, 10, 20);
                    turn_left(sensor_data, 120, 100); 
                    current_state = STATE_AUTO_SCANNING;
                    g_hazard_flag = 0; 

                }
                else if (5 == g_hazard_flag) { // left Boundary
                    send_telemetry("WARNING: Left Safe Zone Boundary! Turning right.\r\n");
                    move_backward(sensor_data, 10, 30); 
                    turn_right(sensor_data, 120, 100);  
                    current_state = STATE_AUTO_SCANNING;
                    g_hazard_flag = 0; 

                }
                else if (6 == g_hazard_flag) { // Right Boundary
                    send_telemetry("WARNING: Right Safe Zone Boundary! Turning left.\r\n");
                    move_backward(sensor_data, 10, 30); 
                    turn_left(sensor_data, 120, 100);  
                    current_state = STATE_AUTO_SCANNING;
                    g_hazard_flag = 0; 

                }
                
                break; */

            /* case STATE_AUTO_AVOID:
                send_telemetry("Executing Avoidance Maneuver...\r\n");
                //Should probably make this smarter and maybe use previous scan data or scan itself
				//to smartely avoid and not get stuck
				
                // should already be turned away so turn forward
                move_forward(sensor_data, 250, 150); 
                
                //turn back 
                if (1 == g_hazard_flag) {
                    turn_left(sensor_data, 90, 100); 
                } else {
                    turn_right(sensor_data, 90, 100);
                }
                
                send_telemetry("Obstacle cleared. Resuming path.\r\n");
                current_state = STATE_AUTO_DRIVING;
                g_hazard_flag = 0; 
                break; */
                
            case STATE_AUTO_PARKING:
				send_telemetry("\r\n--- PRECISION PARKING SCAN ---\r\n");
				oi_setWheels(0, 0); 
				
				DetectedObject park_objects[10];
				int park_objs_found = perform_advanced_sweep(park_objects, 10);
				DetectedGap gate = check_for_parking_zone(park_objects, park_objs_found);

				if (gate.id == 100) { 
					// We see the front two pillars
					int turn_angle = gate.c_angle - 90;
					
					// Turn to perfectly face the center of the gate
					if (turn_angle > 2) { turn_left(sensor_data, turn_angle, 75); } 
					else if (turn_angle < -2) { turn_right(sensor_data, abs(turn_angle), 75); }

					if (gate.dist_cm > 25.0) {
						// We are still far away. Step halfway there to correct for drift.
						float step_mm = (gate.dist_cm * 10.0) * 0.5; // Drive 50% of the distance
						send_telemetry("Approaching gate...\r\n");
						move_forward(sensor_data, step_mm, 75); 
						// The loop will repeat STATE_AUTO_PARKING and scan again
						
					} else {
						// We are exactly at the threshold of the front pillars!
						// Drive the final ~30cm to park inside the square
						send_telemetry("Gate threshold reached. Entering Destination Zone...\r\n");
						move_forward(sensor_data, 300.0, 50); // 300mm = 30cm
						oi_setWheels(0, 0); 
						
						lcd_printf("Delivery Complete.");
						send_telemetry("\r\n[DESTINATION REACHED] Parking Complete.\r\n> ");
						current_state = STATE_IDLE; 
					}
					
				} else if (gate.id == 99) {
					// We only see 1 pillar, or a messy diagonal. Turn towards it and nudge closer.
					int turn_angle = gate.e_angle - 90;
					if (turn_angle > 5) { turn_left(sensor_data, turn_angle, 75); } 
					else if (turn_angle < -5) { turn_right(sensor_data, abs(turn_angle), 75); }
					
					move_forward(sensor_data, 100.0, 75); // Nudge 10cm forward to get a better viewing angle
					
				} else {
					// Lost the pillars entirely
					send_telemetry("\r\n[ERROR] Lost Destination Zone. Reverting to scanning.\r\n");
					current_state = STATE_AUTO_SCANNING;
				}
				break;
            default:
                current_state = STATE_IDLE;
                break;
        }
        
        timer_waitMillis(10); //might be enough
    }
    
    oi_free(sensor_data);
    return 0;
}


void update_odometry(oi_t *sensor_data) {
	
	if (use_imu) {
        mag_t* mag = imu_getMag();
        float raw_heading = mag->heading;
        free(mag); 
        
        current_heading = (int)raw_heading;
    } else {
        current_heading += sensor_data->angle;
    }

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

void inject_virtual_walls(DetectedObject objects[], int *num_objs) {
    int i;
    for (i = 0; i < num_virtual_walls; i++) {
		
		//find distance from
        float dx = my_virtual_walls[i].x - current_x;
        float dy = my_virtual_walls[i].y - current_y;
        float dist = sqrt((dx * dx) + (dy * dy));
        
		//find the angle
        float abs_angle_rad = atan2(dy, dx);
        int abs_angle_deg = abs_angle_rad * (180.0 / PI);
        if (abs_angle_deg < 0) abs_angle_deg += 360;
        
		//find the angle to robot
        int relative_angle = abs_angle_deg - current_heading + 90;
        
        while (relative_angle < 0) relative_angle += 360;
        while (relative_angle >= 360) relative_angle -= 360;
        
		//if its in front of us 
        if (relative_angle >= 0 && relative_angle <= 180 && dist < 80.0 && *num_objs < 10) {
            
			//special id
            objects[*num_objs].id = 900 + i; 
            objects[*num_objs].dist_cm = dist;
            objects[*num_objs].c_angle = relative_angle;
            
			//make it wide
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


