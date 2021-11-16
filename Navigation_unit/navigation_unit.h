// Global data variables and constants used across the whole navigation unit

#ifndef NAVIGATION_UNIT_NAVIGATION_UNIT_H_
#define NAVIGATION_UNIT_NAVIGATION_UNIT_H_
#include <stdbool.h>

#include "../AVR_common/robot.h"

// TODO make these Enums
enum NavigationMode
{
    MANUAL,
    AUTONOMOUS
};

enum Direction
{
    DIR_FORWARD,
    DIR_BACKWARD
};

#define FULL_TURN 65536 // 1 << 16

// This broke with {}'s, unsure why /john
#define GridToMm(coord) (coord) * 250 + 125
#define MmToGrid(coord) (coord) / 250

/* GLOBAL VARIABLES */

// navigation mode manual/auto
extern enum NavigationMode g_navigationMode; // = MANUAL;

// PD-constants
extern uint8_t g_pdKd; // = 0;
extern uint8_t g_pdKp; // = 0;

// Current heading, specified as a fraction of the maximal value (FULL_TURN) for
// the equivalent fraction around a full turn. So e.g. 1/3 of FULL_TURN is 1/3
// turn to the left.
// right = 0
// up   = FULL_TURN/4   = 0b01 << 14 = 16384 = pow(2, 14)
// left = FULL_TURN/2   = 0b10 << 14 = 32768 = pow(2, 15)
// down = FULL_TURN*3/4 = 0b11 << 14 = 49152 pow(2, 14) + pow(2, 15)
// Don't yet know how the initial value of the heading is specified for the
// competition
extern uint16_t g_currentHeading; // = FULL_TURN/4;

// Current position in millimetre, relative to bottom left
// We assume we start in the middle (square 24) in the X direction.
extern uint16_t g_currentPosX; // = grid_to_mm(24);
extern uint16_t g_currentPosY; // = 0;

// We can either use bool + unsigned
extern enum Direction g_wheelDirectionLeft;    // = dir_forward;
extern enum Direction g_wheelDirectionRight;   // = dir_forward;
extern uint8_t        g_wheelSpeedLeft;  // = 0;
extern uint8_t        g_wheelSpeedRight; // = 0;

// or a signed value.
// int8_t WHEEL_SPEED_LEFT = 0;
// int8_t WHEEL_SPEED_RIGHT = 0;
// depending on what's easiest when translating to PWM values for the robot
// and/or what the navigation algorithm wanna use.

// Current navigation goal
extern bool                g_navigationGoalSet;     // = false;
extern uint16_t            g_navigationGoalX;       // = 24;
extern uint16_t            g_navigationGoalY;       // = 0;
extern uint16_t            g_navigationGoalHeading; // = 0;

// Map
extern uint8_t g_navigationMap[49][25];

#endif // NAVIGATION_UNIT_NAVIGATION_UNIT_H_
