#ifndef SERVO_H_
#define SERVO_H_

#include <stdint.h>

void servo_init(void);
void servo_move(uint16_t degrees);

uint16_t servo_get_angle(void);
uint32_t servo_get_match_value(void);
void servo_calibrate(void);


#endif /* SERVO_H_ */
