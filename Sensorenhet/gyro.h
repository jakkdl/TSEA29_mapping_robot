#ifndef GYRO_H
#define GYRO_H
#define F_CPU 16000000UL
#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
uint8_t g_angle;
int16_t MLXGyroVal();
void StartMLX();
#endif