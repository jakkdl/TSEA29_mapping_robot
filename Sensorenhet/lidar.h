#ifndef LIDAR_H
#define LIDAR_H
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define TIME_BETWEEN_SEND 100 // time in ms
uint16_t MeasureLidarFront();
uint16_t MeasureLidarBack();
void ExtInterruptInit();
void TimerInit();
void MsTimerInit();
#endif