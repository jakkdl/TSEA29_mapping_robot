#ifndef ADC_H
#define ADC_H
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <stdbool.h>
extern bool g_IRDone;
void AdcInit();
void ADCRead(uint8_t);
void MeasureIR();
void NextInputPin();
uint16_t ConvertVoltage(double ADCVoltage);
#endif