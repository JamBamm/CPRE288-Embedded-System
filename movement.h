#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "open_interface.h"

// All movement functions now return an integer status code:
//  0 = Success (Reached target distance/angle)
//  1 = Left Bump Hazard
//  2 = Right Bump Hazard
//  3 = Cliff/Boundary Hazard
// -1 = Interrupted by User (UART spacebar/mode switch)

int move_forward(oi_t *self, double distance_mm, int speed);
int move_backward(oi_t *self, double distance_mm, int speed);
int turn_right(oi_t *self, double degrees, int speed);
int turn_left(oi_t *self, double degrees, int speed);

int check_hazards(oi_t *sensor_data);

#endif /* MOVEMENT_H_ */
