
#ifndef PD_CONTROLLER_H
#define PD_CONTROLLER_H
#include <stdint.h>

typedef struct {

	/* Pd-constants */
	uint8_t Kp;
	uint8_t Kd;

	/* internal memory */
	int16_t PrevCTE;

	/* controller output */
	int16_t out;

} PDcontroller;

void PDcontroller_Init( PDcontroller *pd );
void PDcontroller_Update( PDcontroller *pd, uint16_t referencePosX, uint16_t referencePosY, uint16_t targetPosX, uint16_t targetPosY, uint16_t currentPosX, uint16_t currentPosY );
void PDcontroller_Rest();
int16_t PDcontroller_Out();

#endif
