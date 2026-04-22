#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>

void adc_init(void);
uint16_t adc_read(void);
uint16_t adc_read_avg(void);
int ir_distance_from_adc(uint16_t adc_val);

#endif /* ADC_H_ */
