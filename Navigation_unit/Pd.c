#include "Pd.h"
#include "navigation_unit.h"
#include <stdint.h>
PDcontroller pd;
uint16_t referencePosX;
uint16_t referencePosY;

void PDController_Init() {
	/* Clear controller internal memory */
	pd.PrevCTE = 0;
	pd.out = 0.;
}

void PDcontroller_Update(){
	
	/* calculate the dot product between reference node one and target node with respect to current pos */
	uint16_t deltaX = navigationGoalX - referencePosX;
	uint16_t deltaY = navigationGoalY - referencePosY;
	
	uint16_t RX = currentPosX - navigationGoalX;
	uint16_t RY = currentPosY - navigationGoalY; 
	
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
    uint16_t proptional = pdkp * CTE; 

    /*
    * Kp*derivative error is the dervitave part of the pd controller
    */
    uint16_t derivative = pdkd * ( CTE - pd.PrevCTE );

    /*
    * U(out) = proptional part + derivative part
    */
    pd.out = proptional + derivative;
    pd.PrevCTE = CTE;

} 

void PDcontroller_Rest(){
	/* rest the current cross track error*/
	pd.PrevCTE = 0;
}

int16_t PDcontroller_Out(){
	return pd.out;
}

void PDcontroller_Set_RefNode( uint16_t posX, uint16_t posY ){
	referencePosX = posX;
	referencePosY = posY;
}

