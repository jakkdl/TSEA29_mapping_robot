#ifndef GYRO_H
#define GYRO_H
#define F_CPU 16000000UL
#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
extern uint16_t g_Vref;
extern int32_t g_gyroFault;
extern bool g_startup;
int16_t MLXGyroVal();
void MeasureMLX();
void GyroInit(void);
#endif