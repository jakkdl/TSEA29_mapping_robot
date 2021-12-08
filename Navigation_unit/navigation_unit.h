// Global data variables and constants used across the whole navigation unit

#ifndef NAVIGATION_UNIT_NAVIGATION_UNIT_H_
#define NAVIGATION_UNIT_NAVIGATION_UNIT_H_
#include <stdbool.h>


#include "../AVR_common/robot.h"
#include "../AVR_common/uart.h"

#define ComUnitSend(x) Uart_Send_0(x)
#define COM_UNIT_INTERFACE 0
#define SENSOR_UNIT_INTERFACE 1

// Send a debug value to the display-unit, type is used to differentiate
// between different types of values being sent for debugging
void send_debug(uint16_t value, int8_t type);
void send_debug_2(uint16_t value_1, uint16_t value_2, int8_t type)

enum NavigationMode
{
    MANUAL,
    AUTONOMOUS
};

enum Direction
{
	DIR_BACKWARD,
    DIR_FORWARD
};

#define FULL_TURN 65536 // 1 << 16

// This broke with {}'s, unsure why /john
#define GridToMm(coord) (coord) * 400 + 200
#define MmToGrid(coord) (coord) / 400

/* GLOBAL VARIABLES */

// navigation mode manual/auto
extern enum NavigationMode g_navigationMode; // = MANUAL;

// PD-constants
// default values for global variables are set in c-file
extern volatile uint8_t g_pdKd;
extern volatile uint8_t g_pdKp;

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

// Current position in millimeter, relative to bottom left
// We assume we start in the middle (square 24) in the X direction.
extern volatile uint16_t g_currentPosX; // = grid_to_mm(24);
extern volatile uint16_t g_currentPosY; // = 0;

// We can either use bool + unsigned
extern enum Direction g_wheelDirectionLeft;  // = dir_forward;
extern enum Direction g_wheelDirectionRight; // = dir_forward;
extern uint8_t        g_wheelSpeedLeft;      // = 0;
extern uint8_t        g_wheelSpeedRight;     // = 0;

// or a signed value.
// int8_t WHEEL_SPEED_LEFT = 0;
// int8_t WHEEL_SPEED_RIGHT = 0;
// depending on what's easiest when translating to PWM values for the robot
// and/or what the navigation algorithm wanna use.

// Current navigation goal
extern volatile bool g_navigationGoalSet;     // = false;
extern uint16_t g_navigationGoalX;       // = 24;
extern uint16_t g_navigationGoalY;       // = 0;
extern uint16_t g_navigationGoalHeading; // = 0;

// Map
#define MAP_X_MAX 49
#define MAP_Y_MAX 25
extern int8_t g_navigationMap[MAP_X_MAX][MAP_Y_MAX];
// out-of-bonds is treated as always wall
#define IsWall(x, y) (x >= MAP_X_MAX || y >= MAP_Y_MAX || (g_navigationMap[(x)][(y)] & 0x80))
#define IsUnknown(x, y) (x < MAP_X_MAX && y < MAP_Y_MAX && (g_navigationMap[(x)][(y)] == 0))
#define IsEmpty(x, y) (x < MAP_X_MAX && y < MAP_Y_MAX && (g_navigationMap[(x)][(y)] > 0))

#define MakeWall(x, y)                                                         \
    if (g_navigationMap[(x)][(y)] != INT8_MIN)                                 \
    g_navigationMap[(x)][(y)] -= 1
#define MakeEmpty(x, y)                                                        \
    if (g_navigationMap[(x)][(y)] != INT8_MAX)                                 \
    g_navigationMap[(x)][(y)] += 1
#define MakeUnknown(x, y) g_navigationMap[(x)][(y)] = 0

#endif // NAVIGATION_UNIT_NAVIGATION_UNIT_H_
