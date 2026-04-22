#ifndef SCANNER_H_
#define SCANNER_H_

#include <stdint.h>

// A struct to hold a single data point
typedef struct {
    uint16_t angle;
    float ir_distance;
    float ping_distance;
} ScanData;

// Function to perform a full 180-degree sweep and transmit data
void perform_sweep(void);

#endif