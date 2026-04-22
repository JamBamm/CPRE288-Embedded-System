#include "lcd.h"
#include "open_interface.h"
#include "Timer.h"
#include "movement.h"
#include "driverlib/interrupt.h"
#include "uart-interrupt.h"
#include <math.h>

extern volatile char command_byteGo;
extern volatile char command_byteStop;


extern volatile int command_flag;

static void drive_forward_no_bump(oi_t *self, double distance_mm, int speed)
{
    double sum = 0;
    oi_setWheels(speed, speed);

    while (sum < distance_mm) {
        oi_update(self);
        sum += self->distance;
        if (command_flag == 2) {
            oi_setWheels(0, 0);
            break;
        }
    }

    oi_setWheels(0, 0);
}

double move_forward(oi_t *self, double distance_mm)
{
    double forward_sum = 0;
    const double backup_mm = 150.0;
    const double lateral_mm = 250.0;
    const double turn_deg  = 90.0;

    oi_setWheels(100, 100);

    while (forward_sum < distance_mm) {
        oi_update(self);
        forward_sum += self->distance;
        if (command_flag == 2) {
                    oi_setWheels(0, 0);
                    break;
                }


        if (self->bumpLeft || self->bumpRight) {
            
            oi_setWheels(0, 0);

            
            move_backward(self, backup_mm);

           
            oi_update(self);

           
            if (self->bumpRight && !self->bumpLeft) {
                turn_left(self, turn_deg);     
            } else if (self->bumpLeft && !self->bumpRight) {
                turn_right(self, turn_deg);    
            } else {
                
                turn_right(self, turn_deg);
            }

            drive_forward_no_bump(self, lateral_mm, 100);

            if (self->bumpRight && !self->bumpLeft) {
                turn_right(self, turn_deg);
            } else if (self->bumpLeft && !self->bumpRight) {
                turn_left(self, turn_deg);
            } else {
                
                turn_left(self, turn_deg);
            }
            
            drive_forward_no_bump(self, backup_mm, 100);

            oi_setWheels(100, 100);
        }
    }

    oi_setWheels(0, 0);
    return forward_sum;
}

//change direction then move forward
double move_backward(oi_t *self, double distance_mm)
{
    double sum = 0;
    oi_setWheels(-100, -100);

    while (sum < distance_mm) {
        oi_update(self);
        sum -= self->distance; 
        if (command_flag == 2) {
            oi_setWheels(0, 0);
            break;
        }
    }

    oi_setWheels(0, 0);
    return sum;
}

double turn_right(oi_t *self, double degrees)
{
    double sum = 0;
    oi_setWheels(-100, 100);

    while (sum < degrees) {
        oi_update(self);
        sum -= self->angle;
        if (command_flag == 2) {
            oi_setWheels(0, 0);
            break;
        }
    }

    oi_setWheels(0, 0);
    return sum;
}

double turn_left(oi_t *self, double degrees)
{
    double sum = 0;
    oi_setWheels(100, -100);

    while (sum < degrees) {
        oi_update(self);
        sum += self->angle; 
        if (command_flag == 2) {
            oi_setWheels(0, 0);
            break;
        }
    }

    oi_setWheels(0, 0);
    return sum;
}

// Returns 0 if safe, or an error code (1 for bump, 2 for cliff, etc.) if a hazard is hit
int check_hazards(oi_t *sensor_data) {
    oi_update(sensor_data);
    
    // Check for physical bumps
    if (sensor_data->bumpLeft || sensor_data->bumpRight) {
        return 1; 
    }
    
    // Check for cliffs (dropping off a ledge)
    if (sensor_data->cliffLeft || sensor_data->cliffRight || 
        sensor_data->cliffFrontLeft || sensor_data->cliffFrontRight) {
        return 2; 
    }
    
    // Check for boundary lines (white tape)
    if (sensor_data->cliffLeftSignal > 2700 || sensor_data->cliffRightSignal > 2700) { 
        // Note: You must calibrate the > 2700 threshold based on your specific floor/tape!
        return 3;
    }
    
    return 0; // All clear!
}

void move_forward_safe(oi_t *sensor_data, int millimeters) {
    int sum = 0;
    oi_setWheels(150, 150); // Move forward at 150 mm/s
    
    while (sum < millimeters) {
        oi_update(sensor_data);
        sum += sensor_data->distance;
        
        int hazard = check_hazards(sensor_data);
        if (hazard != 0) {
            oi_setWheels(0, 0); // STOP IMMEDIATELY
            
            // Optional: Automatically back up 5cm when you hit something
            // move_backward(sensor_data, 50); 
            
            // Alert Python GUI about the hazard
            uart_sendStr("HAZARD DETECTED\n"); 
            return; // Exit the function early
        }
    }
    
    oi_setWheels(0, 0); // Stop when destination reached safely
}
// Returns 0 if it reached the distance, or the hazard code if it stopped early
int move_forward_safe_auto(oi_t *sensor_data, int millimeters) {
    int sum = 0;
    oi_setWheels(150, 150); 
    
    while (sum < millimeters) {
        // BREAKOUT CLAUSE: If the user hit the emergency stop button in GUI
        if (command_received == 'x' || command_received == 'm') {
            oi_setWheels(0, 0);
            return -1; // Abort code
        }

        oi_update(sensor_data);
        sum += sensor_data->distance;
        
        int hazard = check_hazards(sensor_data);
        if (hazard != 0) {
            oi_setWheels(0, 0);
            // Automatically back up away from the hazard
            // move_backward(sensor_data, 50); 
            return hazard; // Let the state machine handle the avoidance logic
        }
    }
    
    oi_setWheels(0, 0);
    return 0; // Success
}
