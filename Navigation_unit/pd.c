#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "pd.h"
#include "navigation_unit.h"
#include "../AVR_testing/test.h"

void PDcontroller_Reset();
void PDcontroller_Set_RefNode();
bool passedGoal();

#define MIN_SPEED 0x40

#define MAX_SPEED 0x80

#define TURN_SENSITIVITY (FULL_TURN/128)

// robot will stop when the middle of the robot is within this many mm of
// the middle of the target square along both the x and y axis
//TODO: drive with small sensitivity along the intended driving line
//but if we drive far to the side of the target, stop and don't continue driving
#define POS_SENSITIVITY 100

/* internal memory */
double g_PrevCTE;

uint16_t g_referencePosX;
uint16_t g_referencePosY;

void turnToHeading()
{
    // Extra: if the robot's position is shifting,
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
    // mostly needed if increasing update rate doesn't fix oscillating behavior
	if(abs(g_currentHeading-g_navigationGoalHeading)<TURN_SENSITIVITY*2)
	{
		g_wheelSpeedLeft = g_wheelSpeedRight = MAX_SPEED/2;
	}
	else
	{
		g_wheelSpeedLeft = MAX_SPEED;
		g_wheelSpeedRight = MAX_SPEED;
	}
}



// is called repeatedly to move towards the navigationGoal
// returns true when it is close enough, false otherwise
// is responsible for setting wheelSpeed and wheelDirection
bool PDcontroller_Update(void)
{
    send_debug_2(g_navigationGoalX, g_navigationGoalY, 0xFE);
    send_debug_2(g_referencePosX, g_referencePosY, 0xFD);
    send_debug(g_navigationGoalHeading, 0xFC);

    int16_t temp = abs((int16_t) g_currentHeading - g_navigationGoalHeading);

    // Extra: reverse to a square if that's easier, i.e. if temp > FULL_TURN/4

    // calculate what heading we should go in
    // this currently only triggers at the beginning (prevCTE=0), and relies
    // on the pd-controller steering correctly afterwards and not requiring
    // turns on the spot
    if (g_PrevCTE == 0.0 && temp >= TURN_SENSITIVITY)
    {
        //turn on the spot
        turnToHeading();
        return false;
    }


    /* calculate the dot product between reference node one and target node with respect to current pos */
    int32_t RX = ((int32_t) g_currentPosX) - g_navigationGoalX;
    int32_t RY = ((int32_t) g_currentPosY) - g_navigationGoalY;

    if ((abs(RX) < POS_SENSITIVITY && abs(RY) < POS_SENSITIVITY) || passedGoal())
    {
        g_wheelSpeedLeft = 0;
        g_wheelSpeedRight = 0;
        return true;
    }

    int32_t deltaX = ((int32_t) g_navigationGoalX) - g_referencePosX;
    int32_t deltaY = ((int32_t) g_navigationGoalY) - g_referencePosY;

    /* cross track error for current iteration */
    double CTE = ((double)( RY*deltaX - RX*deltaY )) / ( deltaX*deltaX + deltaY*deltaY );
    /*send_debug((int16_t)CTE, 42);
      send_debug((int16_t)RX & 0xFFFF, 43);
      send_debug((int16_t)((RX>>8)& 0xFFFF) , 43);
      send_debug((int16_t)RY& 0xFFFF, 44);
      send_debug((int16_t)(RY>>8)& 0xFFFF, 44);
      send_debug((int16_t)deltaX& 0xFFFF, 45);
      send_debug((int16_t)(deltaX>>8)& 0xFFFF, 45);
      send_debug((int16_t)deltaY& 0xFFFF, 46);
      send_debug((int16_t)(deltaY>>8)& 0xFFFF, 46);
	  send_debug((uint16_t)TURN_SENSITIVITY, 47);
	  send_debug((int16_t)temp, 48);*/
    //i have no idea what this is but works but pointer magic?
    //create a 2 part long with the CTE as reference then point to is in 2 different byte


    /*
     * e(t) = r(t) - u(t) Error signal (this part needed?)
     */

    /*
     * Kp*error proptional part of pd controller
     */
    int16_t proportional = round(CTE * g_pdKp);


    /*
     * Kp*derivative error is the derivative part of the pd controller
     */
    int16_t derivative = round(( CTE - g_PrevCTE )*g_pdKd);

    //send_debug_2(proportional, derivative, 0xFF);

    /*
     * U(out) = proportional part + derivative part
     */
    int16_t out = proportional + derivative;
    g_PrevCTE = CTE;

    g_wheelDirectionRight = DIR_FORWARD;
    g_wheelDirectionLeft = DIR_FORWARD;

    //TODO: this one shouldn't really happen much anymore, but
    //should be handled in better ways
    if (abs(out) > MAX_SPEED)
    {
        send_debug(0, 0);
        g_wheelSpeedLeft = 0;
        g_wheelSpeedRight = 0;
        out = MAX_SPEED;
        return true;
    }
    if (out < 0)
    {
        // out is negative
        g_wheelSpeedRight = MAX_SPEED;
        g_wheelSpeedLeft = MAX_SPEED+out;
    }
    else
    {
        // out is positive
        g_wheelSpeedRight = MAX_SPEED-out;
        g_wheelSpeedLeft = MAX_SPEED;
    }

    return false;
}


void PDcontroller_NewGoal(void)
{
    PDcontroller_Reset();
    PDcontroller_Set_RefNode();
}

void PDcontroller_Reset(){
    /* reset the current cross track error*/
    g_PrevCTE = 0;
}

void PDcontroller_Set_RefNode(){
    g_referencePosX = g_currentPosX;
    g_referencePosY = g_currentPosY;
}

bool passedGoal()
{
	uint8_t dir = 0;
	if (!(g_currentHeading > 57344))
	{
		dir = round((double)g_currentHeading/16384);
	}
	bool passed = false;
	switch(dir)
	{
		case 0:
		if (g_currentPosX > g_navigationGoalX + POS_SENSITIVITY)
		{
			passed = true;
		}
		break;
		case 1:
		if (g_currentPosY > g_navigationGoalY + POS_SENSITIVITY)
		{
			passed = true;
		}
		break;
		case 2:
		if (g_currentPosX < g_navigationGoalX - POS_SENSITIVITY)
		{
			passed = true;
		}
		break;
		case 3:
		if (g_currentPosY < g_navigationGoalY - POS_SENSITIVITY)
		{
			passed = true;
		}
		break;
		default:
		break;
	}
	return passed;
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

    // this one's failing for now

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
    Test_assertEquals(g_wheelSpeedRight, MAX_SPEED-19);

    g_currentPosX += 100;
    g_currentPosY -= 50;

    // POS_SENSITIVITY so high it stops by here
    PDcontroller_Update();
      Test_assertEquals(g_wheelSpeedLeft, MAX_SPEED);
      Test_assertEquals(g_wheelSpeedRight, MAX_SPEED);

      g_currentPosX += 100;
      g_currentPosY -= 50;

      /*PDcontroller_Update();
      Test_assertEquals(g_wheelSpeedLeft, MAX_SPEED-6);
      Test_assertEquals(g_wheelSpeedRight, MAX_SPEED);*/

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

Test_test(Test, arrived_at_goal_turn)
{
    enum Direction oldDirLeft = g_wheelDirectionLeft;
    enum Direction oldDirRight = g_wheelDirectionRight;
    uint8_t oldSpeedLeft = g_wheelSpeedLeft;
    uint8_t oldSpeedRight = g_wheelSpeedRight;
    bool                oldGoalSet               = g_navigationGoalSet;
    uint16_t            oldNavigationGoalHeading = g_navigationGoalHeading;
    uint16_t            oldCurrentHeading        = g_currentHeading;
    uint16_t            oldNavigationGoalX       = g_navigationGoalX;
    uint16_t            oldNavigationGoalY       = g_navigationGoalY;

    g_navigationGoalSet = true;
    g_navigationGoalX = g_currentPosX;
    g_navigationGoalY = g_currentPosY;

    g_navigationGoalHeading = FULL_TURN / 2;
    g_currentHeading        = FULL_TURN / 2 + TURN_SENSITIVITY - 1;
    PDcontroller_NewGoal();
    Test_assertTrue(PDcontroller_Update());

    g_currentHeading = FULL_TURN / 2 - TURN_SENSITIVITY + 1;
    PDcontroller_NewGoal();
    Test_assertTrue(PDcontroller_Update());

    g_navigationGoalHeading = 0;
    g_currentHeading        = TURN_SENSITIVITY - 1;
    PDcontroller_NewGoal();
    Test_assertTrue(PDcontroller_Update());

    g_navigationGoalHeading = 0;
    g_currentHeading        = 1 - TURN_SENSITIVITY;
    PDcontroller_NewGoal();
    Test_assertTrue(PDcontroller_Update());

    // test falsity
    g_navigationGoalHeading = FULL_TURN / 2;
    g_currentHeading        = FULL_TURN / 2 + TURN_SENSITIVITY;
    PDcontroller_NewGoal();
    Test_assertTrue(!PDcontroller_Update());

    g_currentHeading = FULL_TURN / 2 - TURN_SENSITIVITY;
    PDcontroller_NewGoal();
    Test_assertTrue(!PDcontroller_Update());

    g_navigationGoalHeading = 0;
    g_currentHeading        = TURN_SENSITIVITY;
    PDcontroller_NewGoal();
    Test_assertTrue(!PDcontroller_Update());

    g_navigationGoalHeading = 0;
    g_currentHeading        = -TURN_SENSITIVITY;
    PDcontroller_NewGoal();
    Test_assertTrue(!PDcontroller_Update());

    // restore old values
    g_wheelDirectionLeft = oldDirLeft;
    g_wheelDirectionRight = oldDirRight;
    g_wheelSpeedLeft = oldSpeedLeft;
    g_wheelSpeedRight = oldSpeedRight;
    g_navigationGoalSet    = oldGoalSet;
    g_navigationGoalHeading = oldNavigationGoalHeading;
    g_currentHeading        = oldCurrentHeading;
    g_navigationGoalX = oldNavigationGoalX;
    g_navigationGoalY = oldNavigationGoalY;
}

Test_test(Test, arrived_at_goal_pos)
{
    bool                oldGoalSet         = g_navigationGoalSet;
    uint16_t            oldNavigationGoalX = g_navigationGoalX;
    uint16_t            oldNavigationGoalY = g_navigationGoalY;
    uint16_t            oldNavigationGoalHeading = g_navigationGoalHeading;
    uint16_t            oldCurrentPosX     = g_currentPosX;
    uint16_t            oldCurrentPosY     = g_currentPosY;
    uint8_t oldSpeedLeft = g_wheelSpeedLeft;
    uint8_t oldSpeedRight = g_wheelSpeedRight;

    g_navigationGoalSet = true;
    g_navigationGoalHeading = g_currentHeading;

    g_currentPosX     = GridToMm(27);
    g_currentPosY     = GridToMm(5);
    g_navigationGoalX = g_currentPosX + POS_SENSITIVITY - 1;
    g_navigationGoalY = g_currentPosY + POS_SENSITIVITY - 1;
    Test_assertTrue(PDcontroller_Update());

    g_navigationGoalX = g_currentPosX + POS_SENSITIVITY - 1;
    g_navigationGoalY = g_currentPosY + POS_SENSITIVITY;
    Test_assertTrue(!PDcontroller_Update());

    g_navigationGoalX = g_currentPosX + POS_SENSITIVITY;
    g_navigationGoalY = g_currentPosY + POS_SENSITIVITY - 1;
    Test_assertTrue(!PDcontroller_Update());

    g_navigationGoalX = g_currentPosX + POS_SENSITIVITY;
    g_navigationGoalY = g_currentPosY + POS_SENSITIVITY;
    Test_assertTrue(!PDcontroller_Update());

    // check that we didn't accidentally modify navgoaltype
    Test_assertEquals(g_navigationGoalSet, true);

    g_navigationGoalSet  = oldGoalSet;
    g_navigationGoalX    = oldNavigationGoalX;
    g_navigationGoalY    = oldNavigationGoalY;
    g_navigationGoalHeading = oldNavigationGoalHeading;
    g_currentPosX        = oldCurrentPosX;
    g_currentPosY        = oldCurrentPosY;
    g_wheelSpeedLeft = oldSpeedLeft;
    g_wheelSpeedRight = oldSpeedRight;
}
#endif
