
#include <stdint.h>
#include "Pd.h"

void PDController_Init( PDcontroller *pd ) {

	/* Clear controller internal memory */
	pd->Kd = 0; //set this to a good value at a later time
	pd->Kp = 0; //set this to a good value at a later time
	pd->PrevCTE = 0;
	pd->out = 0.;

}



void PDcontroller_Update(  PDcontroller *pd, int16_t referencePosX, int16_t referencePosY, int16_t targetPosX, int16_t targetPosY, int16_t currentPosX, int16_t currentPosY ){
	
	/* calculate the dot product between reference node one and target node with respect to current pos */
	int16_t deltaX = targetPosX - referencePosX;
	int16_t deltaY = targetPosY - referencePosY;
	
	int16_t RX = currentPosX - targetPosX;
	int16_t RY = currentPosY - targetPosY;
	
	/* cross track error for current iteration */
	int16_t CTE = ( RY*deltaX - RX*deltaY ) / ( deltaX*deltaX + deltaY*deltaY );
	
	/*
	* int16_t u = ( RX*deltaX - RY*deltaY ) / ( deltaX*deltaX + deltaY*deltaY );
	*/
	
	/*
    * e(t) = r(t) - u(t) Error signal (this part needed?)
    */
	
    /*
    * Kp*error proptional part of pd controller
    */
    int16_t proptional = pd->Kp * CTE;

    /*
    * Kp*derivative error is the dervitave part of the pd controller
    */
    int16_t derivative = pd->Kd * ( CTE - pd->PrevCTE );

    /*
    * U(out) = proptional part + derivative part
    */
    pd->out = proptional + derivative;
    pd->PrevCTE = CTE;

} 

