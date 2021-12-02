#include <stdint.h>
#include <stdlib.h>

#include "pd.h"
#include "navigation_unit.h"
#include "../AVR_testing/test.h"

void PDcontroller_Rest();
void PDcontroller_Set_RefNode();

PDcontroller pd;
uint16_t referencePosX;
uint16_t referencePosY;

void turnToHeading()
{
    // TODO PD-regulate this

    // insert math here
    if (g_currentHeading == g_navigationGoalHeading)
    {
        return;
    }
    // vänster
    else if (g_currentHeading - g_navigationGoalHeading > FULL_TURN/2)
    {
        g_wheelDirectionLeft = DIR_BACKWARD;
        g_wheelDirectionRight = DIR_FORWARD;
    }
    else
    {
        g_wheelDirectionLeft = DIR_FORWARD;
        g_wheelDirectionRight = DIR_BACKWARD;
    }

    // TODO determine speed according to remaining left to turn
    g_wheelSpeedLeft = 255;
    g_wheelSpeedRight = 255;

}

#if __TEST__
Test_test(Test, turnToHeading)
{
    enum Direction oldDirLeft = g_wheelDirectionLeft;
    enum Direction oldDirRight = g_wheelDirectionRight;
    uint8_t oldSpeedLeft = g_wheelSpeedLeft;
    uint8_t oldSpeedRight = g_wheelSpeedRight;
    uint16_t oldGoalheading = g_navigationGoalHeading;
    uint16_t oldheading = g_currentHeading;

    // no turn
    g_currentHeading = FULL_TURN/2;
    g_navigationGoalHeading = g_currentHeading;
    turnToHeading();
    Test_assertEquals(g_wheelDirectionLeft, oldDirLeft);
    Test_assertEquals(g_wheelDirectionRight, oldDirRight);

    // Exactly turn around (default right turn)
    g_currentHeading = 0;
    g_navigationGoalHeading = FULL_TURN/2;
    turnToHeading();
    Test_assertEquals(g_wheelDirectionLeft, DIR_FORWARD);
    Test_assertEquals(g_wheelDirectionRight, DIR_BACKWARD);

    g_currentHeading = FULL_TURN/2;
    g_navigationGoalHeading = 0;
    turnToHeading();
    Test_assertEquals(g_wheelDirectionLeft, DIR_FORWARD);
    Test_assertEquals(g_wheelDirectionRight, DIR_BACKWARD);

    // left turn
    g_currentHeading = FULL_TURN/2;
    g_navigationGoalHeading = g_currentHeading + FULL_TURN/4;
    turnToHeading();
    Test_assertEquals(g_wheelDirectionLeft, DIR_BACKWARD);
    Test_assertEquals(g_wheelDirectionRight, DIR_FORWARD);

    // right turn
    g_currentHeading = FULL_TURN/2;
    g_navigationGoalHeading = g_currentHeading - FULL_TURN/4;
    turnToHeading();
    Test_assertEquals(g_wheelDirectionLeft, DIR_FORWARD);
    Test_assertEquals(g_wheelDirectionRight, DIR_BACKWARD);

    // left turn across 0
    g_currentHeading = FULL_TURN/8*7;
    g_navigationGoalHeading = g_currentHeading + FULL_TURN/4;
    turnToHeading();
    Test_assertEquals(g_wheelDirectionLeft, DIR_BACKWARD);
    Test_assertEquals(g_wheelDirectionRight, DIR_FORWARD);

    // right turn across 0
    g_currentHeading = FULL_TURN/8;
    g_navigationGoalHeading = g_currentHeading - FULL_TURN/4;
    turnToHeading();
    Test_assertEquals(g_wheelDirectionLeft, DIR_FORWARD);
    Test_assertEquals(g_wheelDirectionRight, DIR_BACKWARD);


    g_wheelDirectionLeft = oldDirLeft;
    g_wheelDirectionRight = oldDirRight;
    g_wheelSpeedLeft = oldSpeedLeft;
    g_wheelSpeedRight = oldSpeedRight;
    g_navigationGoalHeading = oldGoalheading;
    g_currentHeading = oldheading;
}

#endif
void PDcontroller_Update()
{
    // calculate what heading we should go in
    if (abs(g_currentHeading - g_navigationGoalHeading) > FULL_TURN/128)
    {
        //turn on the spot
        return;
    }

    /* calculate the dot product between reference node one and target node with respect to current pos */
    uint16_t deltaX = g_navigationGoalX - referencePosX;
    uint16_t deltaY = g_navigationGoalY - referencePosY;

    uint16_t RX = g_currentPosX - g_navigationGoalX;
    uint16_t RY = g_currentPosY - g_navigationGoalY;

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
    uint16_t proptional = g_pdKp * CTE;

    /*
     * Kp*derivative error is the dervitave part of the pd controller
     */
    uint16_t derivative = g_pdKd * ( CTE - pd.PrevCTE );

    /*
     * U(out) = proptional part + derivative part
     */
    pd.out = proptional + derivative;
    pd.PrevCTE = CTE;

    // TODO Set g_wheelSpeedLeft & right

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

