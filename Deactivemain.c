/**
 * lab6-interrupt_template.c
 *
 * Template file for CprE 288 Lab 6
 *
 * @author Diane Rover, 2/15/2020
 *
 */

 /*
#include "Timer.h"
#include "lcd.h"
#include "cyBot_Scan.h"  // For scan sensors
#include "uart-interrupt.h"
#include "stdbool.h"
#include "stdio.h"
#include "driverlib/interrupt.h"
#include "movement.h"
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "adc.h"


// Uncomment or add any include directives that are needed
// #include "open_interface.h"

// #include "movement.h"
 //#include "button.h"


//#warning "Possible unimplemented functions"

extern volatile char command_byteGo;
extern volatile char command_byteStop;


extern volatile int command_flag;

#define PI 3.14159265
//Guess
#define IR_JUMP_THRESHOLD 200

typedef struct {
    int8_t id;
    int16_t s_angle;
    int16_t e_angle;
    int16_t dist_cm;
    int16_t l_width_cm;
} detectedObj;

detectedObj scan_and_find_smallest() {
    cyBOT_Scan_t scan_data;
    char buffer[100];

    detectedObj smallest_obj;
    smallest_obj.id = -1;
    smallest_obj.l_width_cm = 1000;

    int object_start = -1;
    int object_count = 0;
    int i, j;
    //int last = 0;
	
    //check last
	int ir_history[181] = {0}; 
	int lookback = 4;
	
    uart_sendStr("\r\n--- SCANNING  ---\r\n");

    sprintf(buffer, "\r\nDegrees\tDistance\tIR avg (cm)\r\n");
    uart_sendStr(buffer);

    for (i = 0; i <= 180; i += 2) {

        //check for stop
        if (command_flag == 2) {
            uart_sendStr("\r\nScan Interrupted by User\r\n");
            return smallest_obj;
        }

        //average ir
        int ir_sum = 0;
        for (j = 0; j < 3; j++) {
            cyBOT_Scan(i, &scan_data);
            ir_sum += scan_data.IR_raw_val;
            //if (scan_data.sound_dist > 200){
             //   ir_sum = 1;
            //}
        }
        int ir_avg = ir_sum / 3;
		ir_history[i] = ir_avg;
		
		if (i >= lookback){
		    int past_ir = ir_history[i - lookback];
		}else{
		    int past_ir = int past_ir;
		}

		int past_ir = (i >= lookback) ? ir_history[i - lookback] : ir_history[0];
		
        sprintf(buffer, "%d\t%.2f\t%d\r\n", i, scan_data.sound_dist, ir_avg);
        uart_sendStr(buffer);

        //If no object
		if (object_start == -1) {
		    //Start object found edge
            if ((ir_avg - past_ir) > IR_JUMP_THRESHOLD) {
                object_start = i - (lookback / 2); 
            }
        }
        else {
            //Found other edge
            if ((past_ir - ir_avg) > IR_JUMP_THRESHOLD) {
                int object_end = i - (lookback / 2);
                int radial_width = object_end - object_start;

                //Check for no noise
                if (radial_width >= 6) {
                    object_count++;

                    int midpoint = object_start + (radial_width / 2);
                    cyBOT_Scan(midpoint, &scan_data);
                    double distance = scan_data.sound_dist;
                    cyBOT_Scan(midpoint -1, &scan_data);

                    //Angle formula from the prelab
                    double linear_width = 2.0 * PI * distance * ((double)radial_width / 360.0);

                    sprintf(buffer, "Obj %d: Start: %d, End: %d, Dist: %.2f cm, Width: %.2f cm\r\n",
                            object_count, object_start, object_end, distance, linear_width);
                    uart_sendStr(buffer);

                    if (linear_width < smallest_obj.l_width_cm) {
                        smallest_obj.id = object_count;
                        smallest_obj.s_angle = object_start;
                        smallest_obj.e_angle = object_end;
                        smallest_obj.dist_cm = distance;
                        smallest_obj.l_width_cm = linear_width;
                    }
                }
                object_start = -1;
            }
        }
    }
    
    //Edge ceck for 180 kinda uselss
    if (object_start != -1) {
        int radial_width = 180 - object_start;
        if (radial_width >= 6) {
             object_count++;
             int midpoint = object_start + (radial_width / 2);
             cyBOT_Scan(midpoint, &scan_data);
             double distance = scan_data.sound_dist;
             double linear_width = 2.0 * PI * distance * ((double)radial_width / 360.0);
             
             if (linear_width < smallest_obj.l_width_cm) {
                 smallest_obj.id = object_count;
                 smallest_obj.s_angle = object_start;
                 smallest_obj.e_angle = 180;
                 smallest_obj.dist_cm = distance;
                 smallest_obj.l_width_cm = linear_width;
             }
        }
    }

    return smallest_obj;
}


int main(void) {
    timer_init();
    lcd_init();
    uart_interrupt_init();
    cyBOT_init_Scan(0b0111);
    //cyBOT_Scan_t scan_data;
    //cyBOT_SERVO_cal();


    //init
    right_calibration_value = 248500;
    left_calibration_value = 1256500;

    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);

    char buffer[100];

    detectedObj target;

    command_byteGo = 'g';
    command_byteStop = 's';

    uart_sendStr("\r\nSystem Ready. Send 'g' to start mission, 's' to stop.\r\n> ");

    while(1) {

        //check for stop
        if (command_flag == 2) {
            oi_setWheels(0,0);
            uart_sendStr("\r\nMission Aborted.\r\n> ");
            command_flag = 0;
        }

        //check for run
        if (command_flag == 1) {
            command_flag = 0;
            bool mission_running = true;

            while (mission_running) {
                lcd_clear();
                lcd_printf("Scanning...");

                target = scan_and_find_smallest();

                //chrck for stop
                if (command_flag == 2) {
                    oi_setWheels(0,0);
                    uart_sendStr("\r\nMission Aborted.\r\n> ");
                    command_flag = 0;
                    mission_running = false;
                    continue;
                }

                //print target drive to it
                if (target.id != -1) {
                    sprintf(buffer, "Smallest Object ID: %d\r\n", target.id);
                    uart_sendStr(buffer);

                    sprintf(buffer, "Width: %d degrees\r\n", target.l_width_cm);
                    uart_sendStr(buffer);

                    sprintf(buffer, "Start: %d\tEnd: %d\r\n", target.s_angle, target.e_angle);
                    uart_sendStr(buffer);

                    sprintf(buffer, "Distance: %d cm\r\n", target.dist_cm);
                    uart_sendStr(buffer);

                    //If close enough mission complete
                    if (target.dist_cm <= 20){
                        uart_sendStr("\r\nTarget Reached.\r\n> ");
                       lcd_clear();
                       lcd_printf("Mission Complete");
                       mission_running = false;
                    }

                    int midpoint = target.s_angle + ((target.e_angle - target.s_angle) / 2);
                    cyBOT_Scan(midpoint, NULL);

                    //uart_sendStr(buffer);
                    lcd_clear();
                    lcd_printf("Target Acquired  Smallest: %d deg", midpoint);

                    sprintf(buffer, "Target Acquired  Smallest: %d deg\r\n", midpoint);
                    uart_sendStr(buffer);

                    if (midpoint < 90) {
                        turn_right(sensor_data, (90 - midpoint));
                    } else if (midpoint > 90) {
                        turn_left(sensor_data, (midpoint - 90));
                    }

                    double dist_to_drive_mm = (target.dist_cm * 10.0) - 130.0;

                   if (dist_to_drive_mm > 0) {
                        sprintf(buffer, "Moving %.2f mm closer...\r\n", dist_to_drive_mm);
                        uart_sendStr(buffer);

                        move_forward(sensor_data, dist_to_drive_mm);
                    }else{
                        uart_sendStr("\r\nTarget Reached.\r\n> ");
                      lcd_clear();
                      lcd_printf("Mission Complete");
                      mission_running = false;
                    }

                    /*
                    if (dist_to_drive_mm <= 0) {
                        uart_sendStr("\r\nTarget Reached.\r\n> ");
                        lcd_clear();
                        lcd_printf("Mission Complete");
                        mission_running = false;
                    } else {
                        float drive_chunk = 800;
                        if(dist_to_drive_mm > 800){
                            drive_chunk = 800;
                        }else{
                            drive_chunk = dist_to_drive_mm;
                        }

                        sprintf(buffer, "Moving %.2f mm closer...\r\n", drive_chunk);
                        uart_sendStr(buffer);

                        move_forward(sensor_data, drive_chunk);


                    }

                    

                } else {
                    uart_sendStr("\r\nNo valid objects found. Awaiting next command.\r\n> ");
                    lcd_clear();
                    lcd_printf("No Objects");
                    mission_running = false;
                }
            }
        }
    }

    oi_free(sensor_data);
    return 0;
}
*/
/*
    char uart_msg[50];
    char buf [21];
    int count = 0;


       // OPTIONAL
       //assign a value to command_byte if you want to know whether that ASCII code is received
       //note that command_byte is global shared variable read by the ISR
       //for example, try using a tab character as a command from PuTTY
		uart_sendStr("Type something; max 20 chars before we enter for you \n");
        uart_sendStr("> ");
		uart_sendStr("Interrupt Mode Active. Send 'g' to start, 's' to stop.\r\n> ");
		
		command_byteGo = 'g';
		command_byteStop = 's';
		bool is_scanning = false;


        while (1)
        {

            char c = uart_receive();

            if (c=='\r')
            {
                uart_sendStr("\r\n");

                buf[count] = '\0';
                lcd_clear();
                lcd_printf("%s", buf);

                uart_sendStr("You typed: ");
                uart_sendStr(buf);
                uart_sendStr("\n> ");

                count = 0;
                continue;
            }

           // uart_sendChar(c);

            buf[count++] = c;

            lcd_clear();
            lcd_printf("%s", buf);


            //lcd_printf("Char: %c", c);
           // lcd_moveto(1, 0);
            //lcd_printf("Count: %d", count);


			//1 is checked for Go 
			if (command_flag == 1) {
					is_scanning = true;
					lcd_printf("Scan Status: Running");
					uart_sendStr("\r\n--- SCAN REPORT ---\r\nDegrees\tDistance(cm)\r\n");
				command_flag = 0;
				
			}
            //buf[count++] = c;

            //lcd_clear();

            if (is_scanning) {
				int i;
				
				for (i = 0; i <= 180; i += 2) {
					//check for Stop
					if (command_flag == 2) {
						is_scanning = false;
						command_flag = 0;
                    
						lcd_printf("Scan Interrupted");
						uart_sendStr("\r\nScan Interrupted by User\r\n> ");
						break; 
					}

					
				//copied scan lab 3
				cyBOT_Scan(i, &scan_data);
				sprintf(uart_msg, "%d\t%.2f\r\n", i, scan_data.sound_dist);
                uart_sendStr(uart_msg);
				}
				//lcd_printf("Scan Status: Scanning");
				//timer_waitMillis(1000);
				//lcd_printf("Scan Status: Scanning 2222222");
				//timer_waitMillis(100);
				
				//no interupt
                if (is_scanning){
                    is_scanning = false;
                    lcd_printf("Scan Complete");
                    uart_sendStr("Scan Complete.\r\n> ");
                }

			}


            if (count == 20)
            {
                buf[count] = '\0';
                lcd_clear();
                lcd_printf("%s", buf);

                uart_sendStr("\r\nYou entered: ");
                uart_sendStr(buf);
                uart_sendStr("\n> ");

                count = 0;
            }



        }

  }
    */
        // YOUR CODE HERE
                    //first, try leaving this loop empty and see what happens
                    //then add code for your application
                    // OPTIONAL
                    //test and reset command_flag if your ISR is updating it
                    //for example, if the flag is 1, do something, like send a message to PuTTY or LCD, or stop a sensor scan, etc.
                    //be sure to reset command_flag so you don't keep responding to an old flag
//  while(1)
//  {
//      bool input2 =  false;
//      char input = uart_receive_nonblocking();
//      if(input == 'g'){
//          input2 = true ;
//          lcd_printf("Scan Status: Running");
//      }
//
//      else if(input == 's'){
//          input2 = false;
//          lcd_printf("Scan Status: Interrupted");
//      }
//
//      else if (input2){
//          lcd_printf("Scan Status: Scanning");
//          timer_waitMillis(1000);
//            lcd_printf("Scan Status: Scanning 2222222");
//
//      }
//  }



//Part-2:
/*
#include <stdio.h>
#include <stdint.h>
#include "lcd.h"
#include "timer.h"
#include "adc.h"

int main(void)
{
    uint16_t adc_avg;
    int distance_cm;

    timer_init();
    lcd_init();
    adc_init();

    lcd_printf("Part 2 Ready");
    timer_waitMillis(1000);

    while (1)
    {
        // Get averaged ADC value (16 samples)
        adc_avg = adc_read_avg();

        // Convert ADC to distance using your calibration
        distance_cm = ir_distance_from_adc(adc_avg);

        // Display both values on LCD 
        lcd_clear();
        lcd_printf("ADC:%4u\nDist:%2d cm", adc_avg, distance_cm);

        // Small delay for readability
        timer_waitMillis(5000);
    }

    return 0;
}
*/







//Part-3
#include "Timer.h"
#include "lcd.h"
#include "cyBot_Scan.h"
#include "uart-interrupt.h"
#include "driverlib/interrupt.h"
#include "movement.h"
#include "adc.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

extern volatile char command_byteGo;
extern volatile char command_byteStop;
extern volatile int command_flag;

#define PI 3.14159265


#define IR_JUMP_THRESHOLD 200
#define MIN_OBJECT_WIDTH_DEG 6


typedef struct {
    int8_t id;
    int16_t s_angle;
    int16_t e_angle;
    int16_t dist_cm;
    int16_t l_width_cm;
} detectedObj;

// Move servo, wait, and return averaged RAW ADC value
uint16_t ir_raw_scan_at_angle(int angle_deg)
{
    servo_move(angle_deg);
    timer_waitMillis(200);
    return adc_read_avg();
}

detectedObj scan_and_find_smallest(void)
{
    cyBOT_Scan_t scan_data;
    char buffer[120];

    detectedObj smallest_obj;
    smallest_obj.id = -1;
    smallest_obj.s_angle = -1;
    smallest_obj.e_angle = -1;
    smallest_obj.dist_cm = 0;
    smallest_obj.l_width_cm = 1000;

    int object_start = -1;
    int object_count = 0;
    int i;

    uint16_t ir_history[181] = {0};
    int lookback = 4;

    uart_sendStr("\r\n--- SCANNING ---\r\n");
    uart_sendStr("Deg\tADC Raw\tIR(cm)\tPING(cm)\r\n");

    for (i = 0; i <= 180; i += 2)
    {
        uint16_t ir_raw;
        int ir_cm;
        uint16_t past_ir;

        if (command_flag == 2)
        {
            uart_sendStr("\r\nScan Interrupted by User\r\n");
            return smallest_obj;
        }

        ir_raw = ir_raw_scan_at_angle(i);
        ir_cm = ir_distance_from_adc(ir_raw);

        // Keep cyBOT scan for PING reading 
        cyBOT_Scan(i, &scan_data);

        ir_history[i] = ir_raw;
        past_ir = (i >= lookback) ? ir_history[i - lookback] : ir_history[0];

        sprintf(buffer, "%d\t%u\t%d\t%.2f\r\n", i, ir_raw, ir_cm, scan_data.sound_dist);
        uart_sendStr(buffer);

        lcd_clear();
        lcd_printf("A:%3d\nADC:%4u", i, ir_raw);

        // Detect leading edge
        if (object_start == -1)
        {
            if ((int)ir_raw - (int)past_ir > IR_JUMP_THRESHOLD)
            {
                object_start = i - (lookback / 2);
                if (object_start < 0)
                {
                    object_start = 0;
                }
            }
        }
        else
        {
            // Detect trailing edge 
            if ((int)past_ir - (int)ir_raw > IR_JUMP_THRESHOLD)
            {
                int object_end = i - (lookback / 2);
                int radial_width = object_end - object_start;

                if (radial_width >= MIN_OBJECT_WIDTH_DEG)
                {
                    int midpoint = object_start + (radial_width / 2);
                    double distance;
                    double linear_width;


                    // Use PING at midpoint for distance/width calculation

                    timer_waitMillis(200);
                    servo_move(midpoint);
                    cyBOT_Scan(midpoint, &scan_data);

                    distance = scan_data.sound_dist;
                    linear_width = 2.0 * PI * distance * ((double)radial_width / 360.0);

                    object_count++;

                    sprintf(buffer,
                            "Obj %d: Start:%d End:%d Dist:%.2f cm Width:%.2f cm\r\n",
                            object_count, object_start, object_end, distance, linear_width);
                    uart_sendStr(buffer);

                    if (linear_width < smallest_obj.l_width_cm)
                    {
                        smallest_obj.id = object_count;
                        smallest_obj.s_angle = object_start;
                        smallest_obj.e_angle = object_end;
                        smallest_obj.dist_cm = (int16_t)distance;
                        smallest_obj.l_width_cm = (int16_t)linear_width;
                    }
                }

                object_start = -1;
            }
        }
    }

    // Handle object still open at 180 degrees
    if (object_start != -1)
    {
        int object_end = 180;
        int radial_width = object_end - object_start;

        if (radial_width >= MIN_OBJECT_WIDTH_DEG)
        {
            int midpoint = object_start + (radial_width / 2);
            double distance;
            double linear_width;


            timer_waitMillis(200);
            servo_move(midpoint);
            cyBOT_Scan(midpoint, &scan_data);

            distance = scan_data.sound_dist;
            linear_width = 2.0 * PI * distance * ((double)radial_width / 360.0);

            object_count++;

            sprintf(buffer,
                    "Obj %d: Start:%d End:%d Dist:%.2f cm Width:%.2f cm\r\n",
                    object_count, object_start, object_end, distance, linear_width);
            uart_sendStr(buffer);

            if (linear_width < smallest_obj.l_width_cm)
            {
                smallest_obj.id = object_count;
                smallest_obj.s_angle = object_start;
                smallest_obj.e_angle = object_end;
                smallest_obj.dist_cm = (int16_t)distance;
                smallest_obj.l_width_cm = (int16_t)linear_width;
            }
        }
    }

    return smallest_obj;
}

int main(void)
{
    char buffer[120];
    detectedObj target;

    timer_init();
    lcd_init();
    uart_interrupt_init();
    cyBOT_init_Scan(0b0111);
    adc_init();

    right_calibration_value = 248500;
    left_calibration_value = 1256500;

    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);

    command_byteGo = 'g';
    command_byteStop = 's';

    uart_sendStr("\r\nSystem Ready. Send 'g' to start mission, 's' to stop.\r\n> ");

    while (1)
    {
        if (command_flag == 2)
        {
            oi_setWheels(0, 0);
            uart_sendStr("\r\nMission Aborted.\r\n> ");
            command_flag = 0;
        }

        if (command_flag == 1)
        {
            bool mission_running = true;
            command_flag = 0;

            while (mission_running)
            {
                int midpoint;
                double dist_to_drive_mm;

                lcd_clear();
                lcd_printf("Scanning...");

                target = scan_and_find_smallest();

                if (command_flag == 2)
                {
                    oi_setWheels(0, 0);
                    uart_sendStr("\r\nMission Aborted.\r\n> ");
                    command_flag = 0;
                    mission_running = false;
                    continue;
                }

                if (target.id == -1)
                {
                    uart_sendStr("\r\nNo valid objects found.\r\n> ");
                    lcd_clear();
                    lcd_printf("No Objects");
                    mission_running = false;
                    continue;
                }

                sprintf(buffer, "Smallest Object ID: %d\r\n", target.id);
                uart_sendStr(buffer);

                sprintf(buffer, "Width: %d cm\r\n", target.l_width_cm);
                uart_sendStr(buffer);

                sprintf(buffer, "Start: %d\tEnd: %d\r\n", target.s_angle, target.e_angle);
                uart_sendStr(buffer);

                sprintf(buffer, "Distance: %d cm\r\n", target.dist_cm);
                uart_sendStr(buffer);

                if (target.dist_cm <= 20)
                {
                    uart_sendStr("\r\nTarget Reached.\r\n> ");
                    lcd_clear();
                    lcd_printf("Mission Complete");
                    mission_running = false;
                    continue;
                }

                midpoint = target.s_angle + ((target.e_angle - target.s_angle) / 2);

                lcd_clear();
                lcd_printf("Target:%d deg", midpoint);

                sprintf(buffer, "Target midpoint: %d deg\r\n", midpoint);
                uart_sendStr(buffer);

                if (midpoint < 90)
                {
                    turn_right(sensor_data, (90 - midpoint) * 0.8);
                }
                else if (midpoint > 90)
                {
                    turn_left(sensor_data, (midpoint - 90)  * 0.8);
                }

                dist_to_drive_mm = (target.dist_cm * 10.0) - 130.0;

                if (dist_to_drive_mm > 0)

                {
                    sprintf(buffer, "Moving %.2f mm closer...\r\n", dist_to_drive_mm);
                    uart_sendStr(buffer);
                    move_forward(sensor_data, dist_to_drive_mm);
                }
                else
                {
                    uart_sendStr("\r\nTarget Reached.\r\n> ");
                    lcd_clear();
                    lcd_printf("Mission Complete");
                    mission_running = false;
                }
            }
        }
    }

    oi_free(sensor_data);
    return 0;
}

