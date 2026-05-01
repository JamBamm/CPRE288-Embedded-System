#ifndef SCANNER_H_
#define SCANNER_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int16_t id;
    uint8_t s_angle;
    uint8_t e_angle;
	uint8_t c_angle;
    float dist_cm;
    float l_width_cm;
} DetectedObject;

typedef struct {
    int16_t id;
    uint8_t s_angle;
    uint8_t e_angle;
	uint8_t c_angle;
    float dist_cm;
    float l_width_cm;
} DetectedGap;

int perform_advanced_sweep(DetectedObject objects[], int max_objects);
DetectedObject find_smallest_object(DetectedObject objects[], int num_objects);

int calculate_all_gaps(DetectedObject objects[], int num_objects, DetectedGap gaps[], int max_gaps);
DetectedGap find_best_driveable_gap(DetectedGap gaps[], int num_gaps, int robot_width_cm);

DetectedGap check_for_parking_zone(DetectedObject objects[], int num_objects);

#endif
