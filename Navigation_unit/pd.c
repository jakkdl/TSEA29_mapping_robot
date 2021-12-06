#include <stdint.h>
#include <stdlib.h>

#include "pd.h"
#include "navigation_unit.h"
#include "../AVR_testing/test.h"

void PDcontroller_Reset();
void PDcontroller_Set_RefNode();

#define MAX_SPEED 0x40
#define TURN_SENSITIVITY FULL_TURN / 32

struct PDcontroller {
    /* internal memory */
    double PrevCTE;

    /* controller output */
    int16_t out;

} pd;
uint16_t g_referencePosX;
uint16_t g_referencePosY;

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
    g_wheelSpeedLeft = MAX_SPEED;
    g_wheelSpeedRight = MAX_SPEED;

}

void PDcontroller_Update(void)
{
    int16_t temp = abs((int16_t) g_currentHeading - g_navigationGoalHeading);
    // calculate what heading we should go in
    if (pd.PrevCTE == 0)
    {
        if (temp > 2048)
        {
            /*((int16_t) g_currentHeading - g_navigationGoalHeading > TURN_SENSITIVITY ||
              (int16_t) g_navigationGoalHeading - g_currentHeading > TURN_SENSITIVITY))*/

            //turn on the spot
            turnToHeading();
            return;
        }
    }


    /* calculate the dot product between reference node one and target node with respect to current pos */
    int32_t deltaX = g_navigationGoalX - g_referencePosX;
    int32_t deltaY = g_navigationGoalY - g_referencePosY;

    int32_t RX = g_currentPosX - g_navigationGoalX;
    int32_t RY = g_currentPosY - g_navigationGoalY;

    /* cross track error for current iteration */
    double CTE = (double)( RY*deltaX - RX*deltaY ) / ( deltaX*deltaX + deltaY*deltaY );

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
    pd.PrevCTE = CTE;

    if (out < 0)
    {
        g_wheelSpeedRight = MAX_SPEED;
        g_wheelSpeedLeft = MAX_SPEED+out;
    }
    else
    {
        g_wheelSpeedLeft = MAX_SPEED;
        g_wheelSpeedRight= MAX_SPEED-out;
    }
    //g_wheelSpeedLeft = MAX_SPEED;
    //g_wheelSpeedRight = MAX_SPEED;
}

void PDcontroller_NewGoal(void)
{
    PDcontroller_Reset();
    PDcontroller_Set_RefNode();
}

void PDcontroller_Reset(){
    /* reset the current cross track error*/
    pd.PrevCTE = 0;
}

int16_t PDcontroller_Out(){
    return pd.out;
}

void PDcontroller_Set_RefNode(){
    g_referencePosX = g_currentPosX;
    g_referencePosY = g_currentPosY;
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

Test_test(Test, PDcontroller_Update)
{
    enum Direction oldDirLeft = g_wheelDirectionLeft;
    enum Direction oldDirRight = g_wheelDirectionRight;
    uint8_t oldSpeedLeft = g_wheelSpeedLeft;
    uint8_t oldSpeedRight = g_wheelSpeedRight;
    uint16_t oldGoalheading = g_navigationGoalHeading;
    uint16_t oldheading = g_currentHeading;
    uint16_t oldPosX = g_currentPosX;
    uint16_t oldPosY = g_currentPosY;
    uint16_t oldGoalX = g_navigationGoalX;
    uint16_t oldGoalY = g_navigationGoalY;
    uint8_t oldPdKd = g_pdKd;
    uint8_t oldPdKp = g_pdKp;


    g_currentPosX = GridToMm(10);
    g_currentPosY = GridToMm(10);
    g_navigationGoalX = GridToMm(11);
    g_navigationGoalY = GridToMm(10);
    g_currentHeading = 0;
    g_navigationGoalHeading = 0;
    g_pdKd = 50;
    g_pdKp = 50;
    PDcontroller_NewGoal();

    PDcontroller_Update();
    Test_assertEquals(g_wheelSpeedLeft, MAX_SPEED);
    Test_assertEquals(g_wheelSpeedRight, MAX_SPEED);

    g_currentPosX += 100;
    g_currentPosY += 50;

    PDcontroller_Update();
    Test_assertEquals(g_wheelSpeedLeft, MAX_SPEED);
    Test_assertEquals(g_wheelSpeedRight, MAX_SPEED-12);

    g_currentPosX += 100;
    g_currentPosY += 50;

    PDcontroller_Update();
    Test_assertEquals(g_wheelSpeedLeft, MAX_SPEED);
    Test_assertEquals(g_wheelSpeedRight, MAX_SPEED-18);

    g_currentPosX += 100;
    g_currentPosY -= 50;

    PDcontroller_Update();
    Test_assertEquals(g_wheelSpeedLeft, MAX_SPEED);
    Test_assertEquals(g_wheelSpeedRight, MAX_SPEED);

    g_currentPosX += 100;
    g_currentPosY -= 50;

    PDcontroller_Update();
    Test_assertEquals(g_wheelSpeedLeft, MAX_SPEED-6);
    Test_assertEquals(g_wheelSpeedRight, MAX_SPEED);

    g_wheelDirectionLeft = oldDirLeft;
    g_wheelDirectionRight = oldDirRight;
    g_wheelSpeedLeft = oldSpeedLeft;
    g_wheelSpeedRight = oldSpeedRight;
    g_navigationGoalHeading = oldGoalheading;
    g_currentHeading = oldheading;
    g_currentPosX = oldPosX;
    g_currentPosY = oldPosY;
    g_navigationGoalX = oldGoalX;
    g_navigationGoalY = oldGoalY;
    g_pdKd = oldPdKd;
    g_pdKp = oldPdKp;
}

#endif
