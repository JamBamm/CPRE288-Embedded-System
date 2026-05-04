#include <stdio.h>
#include <stdlib.h>
#include "avoidance.h"
#include "movement.h"
#include "scanner.h"
#include "uart-interrupt.h"
#include "lcd.h"
#include "Timer.h"

static int last_bad_angle = -1;
static int last_turn_direction = 1; /* 1 = right, -1 = left */
#define ROBOT_WIDTH 34


int turn_to_scan_angle(oi_t *sensor_data, int scan_angle)
{
    int delta = scan_angle - 90; 

    if (delta > 5) {
        last_turn_direction = 1;
        return turn_left(sensor_data, delta, 100); 
    }
    else if (delta < -5) {
        last_turn_direction = -1;
        return turn_right(sensor_data, -delta, 100);
    }
    
    return 0; 
}

int avoid_hazard_super_safe(oi_t *sensor_data, int hazard_code)
{
    char msg[100];
    int turn_status;
    int move_status;

    oi_setWheels(0, 0); 

    sprintf(msg, "EMERGENCY AVOIDANCE INITIATED. Code: %d\r\n", hazard_code);
    uart_sendStr(msg);
    lcd_printf("AVOIDING HAZARD\nCode: %d", hazard_code);
    timer_waitMillis(150);

    if (hazard_code == -1) {
        uart_sendStr("AVOIDANCE ABORTED BY OPERATOR\r\n");
        return AVOID_RESULT_STOPPED;
    }

    if (hazard_code >= 1 && hazard_code <= 6) {
        uart_sendStr("AVOIDANCE: Micro-backup to clear sensors...\r\n");
        move_backward(sensor_data, 15, 30); 
    }

    uart_sendStr("AVOIDANCE: Turn 180, then scan before moving\r\n");
    if (hazard_code == 2 || hazard_code == 4 || hazard_code == 6) {
        turn_status = turn_left(sensor_data, 180, 100);
    } else {
        turn_status = turn_right(sensor_data, 180, 100);
    }

    if (turn_status != 0) {
        uart_sendStr("AVOIDANCE: Interrupted during 180 turn, stopping\r\n");
        return AVOID_RESULT_STOPPED;
    }

    uart_sendStr("AVOIDANCE: Scanning escape routes...\r\n");
    DetectedObject my_objects[10];
    int num_objs = perform_advanced_sweep(my_objects, 10);

    DetectedGap dest = check_for_parking_zone(my_objects, num_objs);
    if (dest.id != -1) {
        uart_sendStr("AVOIDANCE: Destination found during escape!\r\n");
        lcd_printf("Dest Found!");
        
        int turn_angle = dest.c_angle - 90;
        if (turn_angle > 0) turn_left(sensor_data, turn_angle, 100); 
        else if (turn_angle < 0) turn_right(sensor_data, abs(turn_angle), 100);
        
        return AVOID_RESULT_DESTINATION_FOUND;
    }

    DetectedGap my_gaps[10];
    int num_gaps = calculate_all_gaps(my_objects, num_objs, my_gaps, 10);
    DetectedGap best_gap = find_best_driveable_gap(my_gaps, num_gaps, ROBOT_WIDTH);
    
    if (best_gap.id == -1) {
        uart_sendStr("AVOIDANCE: No safe sector found. Turning 90 degrees to re-scan.\r\n");
        turn_left(sensor_data, 90, 100);

        num_objs = perform_advanced_sweep(my_objects, 10);
        num_gaps = calculate_all_gaps(my_objects, num_objs, my_gaps, 10);
        best_gap = find_best_driveable_gap(my_gaps, num_gaps, ROBOT_WIDTH);

        if (best_gap.id == -1) {
            uart_sendStr("AVOIDANCE: Still trapped. Robot stopped for operator assistance.\r\n");
            lcd_printf("No Safe Path");
            return AVOID_RESULT_STOPPED;
        }
    }

    sprintf(msg, "AVOIDANCE: Choosing escape gap %d at %d deg\r\n", best_gap.id, best_gap.c_angle);
    uart_sendStr(msg);
    
    int turn_angle = best_gap.c_angle - 90;
    if (turn_angle > 0) turn_left(sensor_data, turn_angle, 100); 
    else if (turn_angle < 0) turn_right(sensor_data, abs(turn_angle), 100);

    float escape_dist = (best_gap.dist_cm * 10.0) - 150.0;
    if (escape_dist > 250.0) escape_dist = 250.0; 
    if (escape_dist < 50.0) escape_dist = 50.0;

    move_status = move_forward(sensor_data, escape_dist, 100); 
    
    if (move_status != 0) {
        uart_sendStr("AVOIDANCE: Hazard hit during escape drive! Handing to Manual.\r\n");
        return AVOID_RESULT_STOPPED; 
	}

    uart_sendStr("AVOIDANCE: Recovery complete, returning to normal operation.\r\n");
    lcd_printf("Avoided Safely");
    
    return AVOID_RESULT_SAFE_CONTINUE;
}
