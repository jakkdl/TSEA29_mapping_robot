#include <stdint.h>
#include <stdlib.h>

#include "pd.h"
#include "navigation_unit.h"

void PDcontroller_Rest();
void PDcontroller_Set_RefNode();

PDcontroller pd;
uint16_t referencePosX;
uint16_t referencePosY;

void turnToHeading()
{
    // TODO PD-regulate this

    // insert math here
    // vänster
    if (g_currentHeading - g_navigationGoalHeading > FULL_TURN/2)
    {
        g_wheelDirectionLeft = DIR_BACKWARD;
        g_wheelDirectionRight = DIR_FORWARD;
    }
    else
    {
        g_wheelDirectionLeft = DIR_FORWARD;
        g_wheelDirectionRight = DIR_BACKWARD;
    }

    g_wheelSpeedLeft = 255;
    g_wheelSpeedRight = 255;

}
void PDcontroller_Update()
{
    // calculate what heading we should go in
    if (abs(g_currentHeading - g_navigationGoalHeading) > FULL_TURN/128)
    {
        //turn on the spot
        return;
    }

    /* calculate the dot product between reference node one and target node with respect to current pos */
    int16_t deltaX = g_navigationGoalX - referencePosX;
    int16_t deltaY = g_navigationGoalY - referencePosY;

    int16_t RX = g_currentPosX - g_navigationGoalX;
    int16_t RY = g_currentPosY - g_navigationGoalY;

    /* cross track error for current iteration */
    int16_t CTE = ( RY*deltaX - RX*deltaY ) / ( deltaX*deltaX + deltaY*deltaY );

    /*
     * e(t) = r(t) - u(t) Error signal (this part needed?)
     */

    /*
     * Kp*error proptional part of pd controller
     */
    int16_t proportional = g_pdKp * CTE;

    /*
     * Kp*derivative error is the dervitave part of the pd controller
     */
    int16_t derivative = g_pdKd * ( CTE - pd.PrevCTE );

    /*
     * U(out) = proportional part + derivative part
     */
    int16_t out = proportional + derivative;
    int16_t PrevCTE = CTE;

    // TODO Set g_wheelSpeedLeft & right
	if (out < 0)
	{
		g_wheelSpeedRight = INT8_MAX;
		g_wheelSpeedLeft = INT8_MAX+out;
	}
	else
	{
		g_wheelSpeedLeft = INT8_MAX;
		g_wheelSpeedRight= INT8_MAX-out;
	}
}

void PDcontroller_NewGoal()
{
    PDcontroller_Rest();
    PDcontroller_Set_RefNode();
}

void PDcontroller_Rest(){
    /* rest the current cross track error*/
    pd.PrevCTE = 0;
}

int16_t PDcontroller_Out(){
    return pd.out;
}

void PDcontroller_Set_RefNode(){
    referencePosX = g_currentPosX;
    referencePosY = g_currentPosY;
}

