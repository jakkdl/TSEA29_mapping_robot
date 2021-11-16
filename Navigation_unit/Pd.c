#include "Pd.h"
#include "navigation_unit.h"
#include <stdint.h>

void PDController_Init( PDcontroller *pd ) {
	/* Clear controller internal memory */
	pd->Kd = 0; //set this to a good value at a later time
	pd->Kp = 0; //set this to a good value at a later time
	pd->PrevCTE = 0;
	pd->out = 0.;
}

void PDcontroller_Update( PDcontroller *pd, uint16_t referencePosX, uint16_t referencePosY, uint16_t targetPosX, uint16_t targetPosY, uint16_t currentPosX, uint16_t currentPosY ){
	
	/* calculate the dot product between reference node one and target node with respect to current pos */
	uint16_t deltaX = targetPosX - referencePosX;
	uint16_t deltaY = targetPosY - referencePosY;
	
	uint16_t RX = currentPosX - targetPosX;
	uint16_t RY = currentPosY - targetPosY;
	
	/* cross track error for current iteration */
	uint16_t CTE = ( RY*deltaX - RX*deltaY ) / ( deltaX*deltaX + deltaY*deltaY );
	
	/*
	* int16_t u = ( RX*deltaX - RY*deltaY ) / ( deltaX*deltaX + deltaY*deltaY );
	*/
	
	/*
    * e(t) = r(t) - u(t) Error signal (this part needed?)
    */
	
    /*
    * Kp*error proptional part of pd controller
    */
    uint16_t proptional = pd->Kp * CTE; 

    /*
    * Kp*derivative error is the dervitave part of the pd controller
    */
    uint16_t derivative = pd->Kd * ( CTE - pd->PrevCTE );

    /*
    * U(out) = proptional part + derivative part
    */
    pd->out = proptional + derivative;
    pd->PrevCTE = CTE;

} 

void PDcontroller_Rest( PDcontroller *pd ){
	/* rest the current cross track error*/
	pd->PrevCTE = 0;
}

int16_t PDcontroller_Out( PDcontroller *pd ){
	return pd->out;
}

