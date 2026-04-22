#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "open_interface.h" 


double move_forward(oi_t *self, double distance_mm);
double move_backward(oi_t *self, double distance_mm);
double turn_right(oi_t *self, double degrees);
double turn_left(oi_t *self, double degrees);


#endif /* MOVEMENT_H_ */
