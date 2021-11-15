#ifndef ADC_H
#define ADC_H
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
void start_adc();
void adc_init();
void next_input_pin();
uint8_t convert_voltage(double ADCVoltage);
#endif