#include <stdint.h>
#ifndef PD_CONTROLLER_H
#define PD_CONTROLLER_H

typedef struct {

	/* Pd-constants */
	int8_t Kp;
	int8_t Kd;

	/* internal memory */
	int16_t PrevCTE;

	/* controller output */
	int16_t out;

} PDcontroller;

void PDcontroller_Init( PDcontroller *pd );
void PDcontroller_Update(  PDcontroller *pd, int16_t referencePosX, int16_t referencePosY, int16_t targetPosX, int16_t targetPosY, int16_t currentPosX, int16_t currentPosY );

#endif
