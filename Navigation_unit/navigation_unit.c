#include "navigation_unit.h"

/* GLOBAL VARIABLES */

// navigation mode manual/auto
enum NavigationMode navigationMode = manual;

// PD-constants
uint8_t pdkd = 0;
uint8_t pdkp = 0;

// Current heading, specified as a fraction of the maximal value (FULL_TURN) for
// the equivalent fraction around a full turn. So e.g. 1/3 of FULL_TURN is 1/3
// turn to the left.
// right = 0
// up   = FULL_TURN/4   = 0b01 << 14 = 16384 = pow(2, 14)
// left = FULL_TURN/2   = 0b10 << 14 = 32768 = pow(2, 15)
// down = FULL_TURN*3/4 = 0b11 << 14 = 49152 pow(2, 14) + pow(2, 15)
// Don't yet know how the initial value of the heading is specified for the
// competition
uint16_t currentHeading = FULL_TURN / 4;

// Current position in millimetre, relative to bottom left
// We assume we start in the middle (square 24) in the X direction.
uint16_t currentPosX = grid_to_mm(24);
uint16_t currentPosY = 0;

// We can either use bool + unsigned
enum Direction wheelDirLeft = dir_forward;
enum Direction wheelDirRight = dir_forward;
uint8_t wheelSpeedLeft = 0;
uint8_t wheelSpeedRight = 0;

// or a signed value.
// int8_t WHEEL_SPEED_LEFT = 0;
// int8_t WHEEL_SPEED_RIGHT = 0;
// depending on what's easiest when translating to PWM values for the robot
// and/or what the navigation algorithm wanna use.

// Current navigation goal
enum NavigationGoal navigationGoalType = none;
uint16_t navigationGoalX = 24;
uint16_t navigationGoalY = 0;
uint16_t navigationGoalHeading = 0;

// Map
uint8_t navigationMap[49][25];
