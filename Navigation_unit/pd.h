#ifndef PD_CONTROLLER_H
#define PD_CONTROLLER_H
#include <stdint.h>

typedef struct {
	/* internal memory */
	int16_t PrevCTE;

	/* controller output */
	int16_t out;

} PDcontroller;

void PDcontroller_Update();
void PDcontroller_NewGoal();
#endif
