#ifndef LIDAR_H
#define LIDAR_H
#include <avr/io.h>
#include <avr/interrupt.h>
void measure_lidar();
void external_interrupt_init();
void timer_init();
void timer_stop();
#endif