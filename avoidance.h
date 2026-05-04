#ifndef AVOIDANCE_H_
#define AVOIDANCE_H_

#include "open_interface.h"

#define AVOID_RESULT_SAFE_CONTINUE      0
#define AVOID_RESULT_DESTINATION_FOUND  1
#define AVOID_RESULT_STOPPED           -1

int avoid_hazard_super_safe(oi_t *sensor_data, int hazard_code);
int turn_to_scan_angle(oi_t *sensor_data, int scan_angle);

#endif /* AVOIDANCE_H_ */
