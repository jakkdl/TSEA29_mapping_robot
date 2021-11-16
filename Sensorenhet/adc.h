#ifndef ADC_H
#define ADC_H
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdbool.h>
bool g_IRDone;
void StartAdc();
void AdcInit();
void NextInputPin();
uint8_t ConvertVoltage(double ADCVoltage);
#endif