
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
void PDcontroller_Rest();
int16_t PDcontroller_Out();
void PDcontroller_Set_RefNode( uint16_t posX, uint16_t posY );
#endif
