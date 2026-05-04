#include "scanner.h"
#include "servo.h"
#include "adc.h"
#include "ping.h"
#include "uart-interrupt.h"
#include <stdio.h>
#include <math.h>
#include "timer.h"

#define IR_THRESHOLD_CM 80.0    // must be closer than this to be an object
#define IR_JUMP_THRESHOLD 200   // adjust this based on what works
#define MIN_OBJECT_WIDTH_DEG 5  // filter out tiny noise spikes
#define MAX_PILLAR_WIDTH_CM 12.0 // 3 inches is 7.62cm. 12cm gives a safe buffer for sensor noise.
#define PI 3.14159265

extern volatile char g_command_byte;
extern volatile bool g_command_ready;

/**
 * scans and detects objects
 * Returns int of object count of found object
 */
int perform_advanced_sweep(DetectedObject objects[], int max_objects) {
    char gui_msg[75];
    int object_count = 0;
    int in_object = 0;
    
    uint16_t ir_history[181] = {0};
    uint8_t lookback = 4;
    int angle;

    float min_ping_in_obj = 999.0;
    int min_ping_angle = 0;

    for (angle = 0; angle <= 180; angle += 2) {
        
        //interrupt
        if (g_command_ready) {
            if (g_command_byte == ' ' || g_command_byte == 'x' || g_command_byte == 'm') {
                uart_sendStr("\r\n[WARNING] Scan Aborted by User Interrupt!\r\n");
                servo_move(90);       
                return object_count;  
            }
        }
        
        servo_move(angle);
        
        uint16_t ir_raw = adc_read_avg();
        float ping_dist = ping_getDistance();
        float ir_dist = ir_distance_from_adc(ir_raw); 
        timer_waitMillis(30);

        
        sprintf(gui_msg, "RAW:%d,%.2f,%.2f,%d\n", angle, ir_dist, ping_dist, ir_raw);
        uart_sendStr(gui_msg);
        
        ir_history[angle] = ir_raw;
        uint16_t past_ir = (angle >= lookback) ? ir_history[angle - lookback] : ir_history[0];
        
        if (!in_object) {
            if (((int)ir_raw - (int)past_ir) > IR_JUMP_THRESHOLD) {
                in_object = 1;
                
                min_ping_in_obj = ping_dist;
                min_ping_angle = angle;
                
                if (object_count < max_objects) {
                    objects[object_count].s_angle = angle - (lookback / 2);
                    if (objects[object_count].s_angle < 0) objects[object_count].s_angle = 0;
                }
            }
        } 
        else {
            if (ping_dist < min_ping_in_obj) {
                min_ping_in_obj = ping_dist;
                min_ping_angle = angle;
            }

            if (((int)past_ir - (int)ir_raw) > IR_JUMP_THRESHOLD ) {
                in_object = 0;
                
                if (object_count < max_objects) {
                    objects[object_count].e_angle = angle - (lookback / 2);
                    int radial_width = objects[object_count].e_angle - objects[object_count].s_angle;
                    
                    if (radial_width >= MIN_OBJECT_WIDTH_DEG) {
                        
                        objects[object_count].c_angle = min_ping_angle; 
                        objects[object_count].dist_cm = min_ping_in_obj;
                        
                        objects[object_count].l_width_cm = 2.0 * PI * min_ping_in_obj * ((float)radial_width / 360.0);
                        objects[object_count].id = object_count + 1;
                        
                        sprintf(gui_msg, "OBJ:%d,%d,%d,%d,%.2f,%.2f\n", 
                            objects[object_count].id, objects[object_count].s_angle, 
                            objects[object_count].e_angle, objects[object_count].c_angle, 
                            objects[object_count].dist_cm, objects[object_count].l_width_cm);
                        uart_sendStr(gui_msg);
                        
                        object_count++;
                    }
                }
            }
        }
    } // End of 180-degree loop

    // --- 180 DEGREE EDGE CASE (Object cut off by end of sweep) ---
    if (in_object && object_count < max_objects) {
        objects[object_count].e_angle = 180;
        int radial_width = 180 - objects[object_count].s_angle;
        
        // Filter out tiny noise spikes
        if (radial_width >= MIN_OBJECT_WIDTH_DEG) {
            
            // Use the closest Ping distance recorded before the sweep ended
            objects[object_count].c_angle = min_ping_angle; 
            objects[object_count].dist_cm = min_ping_in_obj;
            
            // Calculate physical width using the minimum distance
            objects[object_count].l_width_cm = 2.0 * PI * min_ping_in_obj * ((float)radial_width / 360.0);
            objects[object_count].id = object_count + 1;
            
            sprintf(gui_msg, "OBJ:%d,%d,%d,%d,%.2f,%.2f\n", 
                objects[object_count].id, 
                objects[object_count].s_angle, 
                objects[object_count].e_angle, 
                objects[object_count].c_angle, 
                objects[object_count].dist_cm, 
                objects[object_count].l_width_cm);
            uart_sendStr(gui_msg);
            
            object_count++;
        }
    }

    // Return the sensor to straight-ahead when finished
    servo_move(90);
    return object_count;
}

/**
 * finds the smallest object in an array of them
 * Returns smallest object
 */
DetectedObject find_smallest_object(DetectedObject objects[], int num_objects) {
    DetectedObject smallest_obj;
    smallest_obj.id = -1; 
    smallest_obj.l_width_cm = 9999.0; 

    if (num_objects == 0) {
        return smallest_obj;
    }

	//iterate and check for smallest
    int i;
    for (i = 0; i < num_objects; i++) {
        if (objects[i].l_width_cm < smallest_obj.l_width_cm) {
            smallest_obj = objects[i];
        }
    }

    if (smallest_obj.id != -1) {
        char gui_msg[100];
        sprintf(gui_msg, "\r\n[SMALLEST FOUND] -> Obj %d (Width: %.2f cm at %d deg)\r\n\r\n", 
                smallest_obj.id, smallest_obj.l_width_cm, smallest_obj.c_angle);
        uart_sendStr(gui_msg);
    }

    return smallest_obj;
}

/**
 * calculates all physical gaps between detected objects
 * returns the number of gaps found.
 */
int calculate_all_gaps(DetectedObject objects[], int num_objects, DetectedGap gaps[], int max_gaps) {
    int gap_count = 0;
    char uart_msg[100];
    
    uart_sendStr("\r\n--- CALCULATING GAPS ---\r\n");

    // no objects its one big gap
    if (num_objects == 0) {
        gaps[0].id = 1;
        gaps[0].s_angle = 0;
        gaps[0].e_angle = 180;
        gaps[0].c_angle = 90;
        gaps[0].dist_cm = 200.0; 
        gaps[0].l_width_cm = 999.0; 
        uart_sendStr("No objects detected. Full 180-degree path is clear.\r\n");
        return 1;
    }

    // gap between 0 and object
    if (objects[0].s_angle > 5 && gap_count < max_gaps) {
        gaps[gap_count].id = gap_count + 1;
        gaps[gap_count].s_angle = 0;
        gaps[gap_count].e_angle = objects[0].s_angle;
        gaps[gap_count].c_angle = gaps[gap_count].e_angle / 2;
        gaps[gap_count].dist_cm = objects[0].dist_cm; 
        
		//find witdh
        int angular_width = gaps[gap_count].e_angle - gaps[gap_count].s_angle;
        gaps[gap_count].l_width_cm = gaps[gap_count].dist_cm * (angular_width * (PI / 180.0));
        
        gap_count++;
    }

    // gap between objects
    int i;
    for (i = 0; i < num_objects - 1; i++) {
        if (gap_count >= max_gaps) break;
        
        gaps[gap_count].id = gap_count + 1;
        gaps[gap_count].s_angle = objects[i].e_angle;
        gaps[gap_count].e_angle = objects[i + 1].s_angle;
        gaps[gap_count].c_angle = gaps[gap_count].s_angle + ((gaps[gap_count].e_angle - gaps[gap_count].s_angle) / 2);
        
		//closer of two object for safe distance
        gaps[gap_count].dist_cm = (objects[i].dist_cm < objects[i+1].dist_cm) ? objects[i].dist_cm : objects[i+1].dist_cm;

        // find unkown side with geometry law of cosines
        // c^2 = a^2 + b^2 - 2ab*cos(C)
        float a = objects[i].dist_cm;
        float b = objects[i+1].dist_cm;
        float angle_rad = (gaps[gap_count].e_angle - gaps[gap_count].s_angle) * (PI / 180.0);
        gaps[gap_count].l_width_cm = sqrt((a*a) + (b*b) - (2*a*b*cos(angle_rad)));

        gap_count++;
    }

    // last object and 180 degrees
    if (objects[num_objects - 1].e_angle < 175 && gap_count < max_gaps) {
        gaps[gap_count].id = gap_count + 1;
        gaps[gap_count].s_angle = objects[num_objects - 1].e_angle;
        gaps[gap_count].e_angle = 180;
        gaps[gap_count].c_angle = gaps[gap_count].s_angle + ((180 - gaps[gap_count].s_angle) / 2);
        gaps[gap_count].dist_cm = objects[num_objects - 1].dist_cm;
        
        int angular_width = 180 - gaps[gap_count].s_angle;
        gaps[gap_count].l_width_cm = gaps[gap_count].dist_cm * (angular_width * (PI / 180.0));
        
        gap_count++;
    }

	//send all gaps
    int j;
    for (j = 0; j < gap_count; j++) {
        sprintf(uart_msg, "=> Gap %d: %d to %d deg (Center: %d) | Width: %.2f cm | Safely drive: %.2f cm\r\n", 
                gaps[j].id, gaps[j].s_angle, gaps[j].e_angle, gaps[j].c_angle, gaps[j].l_width_cm, gaps[j].dist_cm);
        uart_sendStr(uart_msg);
    }

    return gap_count;
}

/**
 * finds closest gap to the robots heading and wide enough
 */
DetectedGap find_best_driveable_gap(DetectedGap gaps[], int num_gaps, int robot_width_cm) {
    DetectedGap best_gap;
    best_gap.id = -1; 
    
	//how far off 90 degrees
    int smallest_deviation = 999; 

    int i;
    for (i = 0; i < num_gaps; i++) {
        
        if (gaps[i].l_width_cm > robot_width_cm) {
            
            int deviation_from_center = abs(gaps[i].c_angle - 90);
            
            if (deviation_from_center < smallest_deviation) {
                smallest_deviation = deviation_from_center;
                best_gap = gaps[i];
            }
        }
    }

    if (best_gap.id != -1) {
        char msg[80];
        sprintf(msg, "\r\n[PATH SELECTED] -> Gap %d at %d deg. Width: %.2f cm\r\n\r\n", best_gap.id, best_gap.c_angle, best_gap.l_width_cm);
        uart_sendStr(msg);
    } else {
        uart_sendStr("\r\n[WARNING] No gap wide enough for robot detected!\r\n\r\n");
    }

    return best_gap;
}



/**
 * checks for thin pillars
 * retunrs a gap representing the path 
 */
DetectedGap check_for_parking_zone(DetectedObject objects[], int num_objects) {
    int pillar_indices[10];
    int pillar_count = 0;
    DetectedGap parking_target;
    parking_target.id = -1; 
    char uart_msg[120];

 
    int i;
    for ( i = 0; i < num_objects; i++) {
        if (objects[i].l_width_cm > 0.0 && objects[i].l_width_cm <= 12.0) {
            pillar_indices[pillar_count] = i;
            pillar_count++;
        }
    }

    if (pillar_count == 0) {
        return parking_target; 
    } 
    else if (pillar_count == 1) {
        int p_idx = pillar_indices[0];
        parking_target.id = 99; 
        parking_target.e_angle = objects[p_idx].c_angle;
        
        float approach_dist = objects[p_idx].dist_cm - 25.0;
        parking_target.dist_cm = (approach_dist > 5.0) ? approach_dist : 5.0;
        
        sprintf(uart_msg, "\r\n[PARKING] 1 Pillar found at %d deg. Need closer look.\r\n", parking_target.e_angle);
        uart_sendStr(uart_msg);
        return parking_target;
    }

    int best_p1 = -1, best_p2 = -1;
    float best_alignment = 999.0; 
    float best_gap_width = 0.0;

    int iu;
    int j;
    for ( iu = 0; iu < pillar_count - 1; iu++) {
        for ( j = iu + 1; j < pillar_count; j++) {
            int p1 = pillar_indices[iu];
            int p2 = pillar_indices[j];

            float a = objects[p1].dist_cm;
            float b = objects[p2].dist_cm;
            float angle_rad = abs(objects[p1].c_angle - objects[p2].c_angle) * (PI / 180.0);
            
            float gap_width = sqrt((a*a) + (b*b) - (2*a*b*cos(angle_rad)));
            
            float alignment_diff = fabs(a - b);

            if (gap_width > 35.0 && gap_width < 75.0 && alignment_diff < 35.0) {
                
                if (alignment_diff < best_alignment) {
                    best_alignment = alignment_diff;
                    best_gap_width = gap_width;
                    best_p1 = p1;
                    best_p2 = p2;
                }
            }
        }
    }

    if (best_p1 != -1 && best_p2 != -1) {
        parking_target.id = 100; 
        parking_target.s_angle = objects[best_p1].c_angle;
        parking_target.e_angle = objects[best_p2].c_angle;
        
        parking_target.c_angle = (parking_target.s_angle + parking_target.e_angle) / 2;

        parking_target.dist_cm = (objects[best_p1].dist_cm + objects[best_p2].dist_cm) / 2.0;
        parking_target.l_width_cm = best_gap_width;

        sprintf(uart_msg, "\r\n[PARKING] Gate Found! Width: %.1fcm, Dist: %.1fcm, Aim: %d deg\r\n", 
                parking_target.l_width_cm, parking_target.dist_cm, parking_target.c_angle);
        uart_sendStr(uart_msg);
    } else {
        parking_target.id = 99; 
        parking_target.e_angle = objects[pillar_indices[0]].c_angle;
        parking_target.dist_cm = objects[pillar_indices[0]].dist_cm - 20.0;
        uart_sendStr("\r\n[PARKING] Pillars seen, but no valid gate. Re-aligning...\r\n");
    }

    return parking_target;
}


/* void perform_sweep(void) {
    char uart_msg[50];
    
    // sweep from 0 to 180 degrees (usually in increments of 2 or 5)
    for (int angle = 0; angle <= 180; angle += 2) {
        servo_move(angle);
        
        timer_waitMillis(30); 
        
        float ir_adc = adc_read_avg();
        float ping_dist = ping_getDistance();
        
        sprintf(uart_msg, "%d,%.2f,%.2f\n", angle, ir_adc, ping_dist);
        
        uart_sendStr(uart_msg); 
    }
    
    servo_move(90);
} */
