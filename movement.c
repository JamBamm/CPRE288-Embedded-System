#include "open_interface.h"
#include "Timer.h"
#include "movement.h"
#include "uart-interrupt.h"
#include <math.h>

extern volatile char g_command_byte;
extern volatile bool g_command_ready;

extern void update_odometry(oi_t *sensor_data);
extern void send_gui_telemetry(oi_t *sensor_data);

extern volatile int command_flag;

//exact same hazar method as the one in main
int check_hazards(oi_t *sensor_data) {
    if (1 == sensor_data->bumpLeft) return 1;
    if (1 == sensor_data->bumpRight) return 2;
    
    
    if (sensor_data->cliffLeftSignal <= 200 || sensor_data->cliffFrontLeftSignal <= 200) {
        return 3; 
    }
    if (sensor_data->cliffRightSignal <= 200 || sensor_data->cliffFrontRightSignal <= 200) {
        return 4; 
    }
    
    // boundary / white tape detection (Codes 5-6)
    // Comment this block out when testing on your white tile floor!
    /*
    if (sensor_data->cliffLeftSignal >= 2600 || sensor_data->cliffFrontLeftSignal >= 2600) {
        return 5; // Left Boundary
    }
    if (sensor_data->cliffRightSignal >= 2600 || sensor_data->cliffFrontRightSignal >= 2600) {
        return 6; // Right Boundary
    }
    */
    
    return 0; // Safe
}




int move_forward(oi_t *self, double distance_mm, int speed) {
    double sum = 0;
    
    // absolute value
    speed = abs(speed);
    oi_setWheels(speed, speed);

    while (sum < distance_mm) {
        oi_update(self);
		
		update_odometry(self);
        send_gui_telemetry(self);
		
        sum += self->distance;
		
		//chekc hazard
        int hazard = check_hazards(self);
        if (hazard != 0) {
            oi_setWheels(0, 0); 
            return hazard; 
        }

		//check intterupt
        if (g_command_ready) {
            if (g_command_byte == ' ' || g_command_byte == 'm') {
                oi_setWheels(0, 0);
                g_command_ready = false; 
                return -1;
            }
        }
        
		//might loop to fast otherwise idk for sure 
        timer_waitMillis(50); 
    }

    oi_setWheels(0, 0);
    return 0; 
}


int move_backward(oi_t *self, double distance_mm, int speed) {
    double sum = 0;
    
    speed = abs(speed);
    oi_setWheels(-speed, -speed);

    while (sum < distance_mm) {
        oi_update(self);
		
		update_odometry(self);
        send_gui_telemetry(self);
        // use abs for distance
        sum += abs(self->distance); 

        if (g_command_ready) {
            if (g_command_byte == ' ' || g_command_byte == 'm') {
                oi_setWheels(0, 0);
                g_command_ready = false; 
                return -1;
            }
        }
        
        timer_waitMillis(50); //up for debate 
    }

    oi_setWheels(0, 0);
    return 0;
}

int turn_right(oi_t *self, double degrees, int speed) {
    double sum = 0;
    
    speed = abs(speed);
    oi_setWheels(-speed, speed); 

    while (sum < degrees) {
        oi_update(self);
		
		update_odometry(self);
        send_gui_telemetry(self);
        
        //need to use IMU in future
        // use abs for cw
        sum += abs(self->angle); 

        if (g_command_ready) {
            if (g_command_byte == ' ' || g_command_byte == 'm') {
                oi_setWheels(0, 0);
                g_command_ready = false;
                return -1;
            }
        }
        timer_waitMillis(50); //up for debate 
    }

    oi_setWheels(0, 0);
    return 0;
}


int turn_left(oi_t *self, double degrees, int speed) {
    double sum = 0;
    
    speed = abs(speed);
    oi_setWheels(speed, -speed); 

    while (sum < degrees) {
        oi_update(self);
		
		update_odometry(self);
        send_gui_telemetry(self);
        
        //need to use IMU in future
        sum += self->angle; 

        if (g_command_ready) {
            if (g_command_byte == ' ' || g_command_byte == 'm') {
                oi_setWheels(0, 0);
                g_command_ready = false;
                return -1;
            }
        }
        timer_waitMillis(50);
    }

    oi_setWheels(0, 0);
    return 0;
}
