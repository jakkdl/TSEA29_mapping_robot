#ifndef ADC_H
#define ADC_H
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdbool.h>
extern bool g_IRDone;
void StartAdc();
void AdcInit();
void NextInputPin();
uint16_t ConvertVoltage(double ADCVoltage);
#endif
