#ifndef ADC_H
#define ADC_H
#include <avr/io.h>
#include <avr/interrupt.h>
void start_adc();
void adc_init();
void next_input_pin();
#endif