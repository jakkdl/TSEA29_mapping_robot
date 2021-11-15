#ifndef LIDAR_H
#define LIDAR_H
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
void measure_lidar();
void external_interrupt_init();
void timer_init();
void timer_stop();
#endif