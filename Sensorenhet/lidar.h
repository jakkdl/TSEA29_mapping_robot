#ifndef LIDAR_H
#define LIDAR_H
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
void MeasureLidar();
void ExtInterruptInit();
void TimerInit();
void TimerStop();
void MsTimerInit();
#endif