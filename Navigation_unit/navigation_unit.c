#include "navigation_unit.h"

/* GLOBAL VARIABLES */

// struct with sensor data
// two instances

// navigation mode manual/auto
enum NavigationMode g_navigationMode = MANUAL;

// PD-constants
uint8_t g_pdKd = 0;
uint8_t g_pdKp = 0;

// Current heading, specified as a fraction of the maximal value (FULL_TURN) for
// the equivalent fraction around a full turn. So e.g. 1/3 of FULL_TURN is 1/3
// turn to the left.
// right = 0
// up   = FULL_TURN/4   = 0b01 << 14 = 16384 = pow(2, 14)
// left = FULL_TURN/2   = 0b10 << 14 = 32768 = pow(2, 15)
// down = FULL_TURN*3/4 = 0b11 << 14 = 49152 pow(2, 14) + pow(2, 15)
// Don't yet know how the initial value of the heading is specified for the
// competition
uint16_t g_currentHeading = FULL_TURN / 4;

// Current position in millimetre, relative to bottom left
// We assume we start in the middle (square 24) in the X direction.
uint16_t g_currentPosX = GridToMm(24);
uint16_t g_currentPosY = 0;

// We can either use bool + unsigned
enum Direction g_wheelDirLeft    = DIR_FORWARD;
enum Direction g_wheelDirRight   = DIR_FORWARD;
uint8_t        g_wheelSpeedLeft  = 0;
uint8_t        g_wheelSpeedRight = 0;

// or a signed value.
// int8_t WHEEL_SPEED_LEFT = 0;
// int8_t WHEEL_SPEED_RIGHT = 0;
// depending on what's easiest when translating to PWM values for the robot
// and/or what the navigation algorithm wanna use.

// Current navigation goal
bool                g_navigationSet         = false;
uint16_t            g_navigationGoalX       = 24;
uint16_t            g_navigationGoalY       = 0;
uint16_t            g_navigationGoalHeading = 0;

// Map
uint8_t g_navigationMap[49][25];
