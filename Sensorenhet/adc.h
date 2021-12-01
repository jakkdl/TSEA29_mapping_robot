#ifndef ADC_H
#define ADC_H
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <stdbool.h>
extern bool g_IRDone;
void StartAdc();
void AdcInit();
void MeasureIR();
void NextInputPin();
uint16_t ConvertVoltage(double ADCVoltage);
#endif