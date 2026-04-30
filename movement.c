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

int check_hazards(oi_t *sensor_data) {
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



// ==============================================================================
// Safe Forward Movement
// ==============================================================================
int move_forward(oi_t *self, double distance_mm, int speed) {
    double sum = 0;
    
    // Ensure speed is positive for forward movement
    speed = abs(speed);
    oi_setWheels(speed, speed);

    while (sum < distance_mm) {
        oi_update(self);
		
		update_odometry(self);
        send_gui_telemetry(self);
		
        sum += self->distance;

        // 1. Hardware Hazard Checks
        int hazard = check_hazards(self);
        if (hazard != 0) {
            oi_setWheels(0, 0); 
            return hazard; 
        }

        // 2. Software Interrupt Checks (Emergency Stop from Base Station)
        if (g_command_ready) {
            // If user presses spacebar or 'm' for manual mode, abort movement!
            if (g_command_byte == ' ' || g_command_byte == 'm') {
                oi_setWheels(0, 0);
                g_command_ready = false; // Consume the command
                return -1;
            }
        }
        
        timer_waitMillis(50); // Loop stability
    }

    oi_setWheels(0, 0);
    return 0; // Success
}

// ==============================================================================
// Safe Backward Movement
// ==============================================================================
int move_backward(oi_t *self, double distance_mm, int speed) {
    double sum = 0;
    
    // Ensure speed is positive, then negate it for wheels
    speed = abs(speed);
    oi_setWheels(-speed, -speed);

    while (sum < distance_mm) {
        oi_update(self);
		
		update_odometry(self);
        send_gui_telemetry(self);
        // Distance is negative when moving backwards, use absolute value for logic
        sum += abs(self->distance); 

        // 2. Software Interrupt Checks
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

// ==============================================================================
// Safe Right Turn (Clockwise)
// ==============================================================================
int turn_right(oi_t *self, double degrees, int speed) {
    double sum = 0;
    
    speed = abs(speed);
    oi_setWheels(-speed, speed); 

    while (sum < degrees) {
        oi_update(self);
		
		update_odometry(self);
        send_gui_telemetry(self);
        
        // Future Upgrade: Replace this with reading from the IMU heading!
        // Angle is negative for CW turns in open_interface, use absolute value
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

// ==============================================================================
// Safe Left Turn (Counter-Clockwise)
// ==============================================================================
int turn_left(oi_t *self, double degrees, int speed) {
    double sum = 0;
    
    speed = abs(speed);
    oi_setWheels(speed, -speed); 

    while (sum < degrees) {
        oi_update(self);
		
		update_odometry(self);
        send_gui_telemetry(self);
        
        // Future Upgrade: Replace this with reading from the IMU heading!
        // Angle is positive for CCW turns
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
