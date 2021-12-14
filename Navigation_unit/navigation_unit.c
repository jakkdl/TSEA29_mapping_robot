#include "navigation_unit.h"
#include "../AVR_common/sensors.h"

/* GLOBAL VARIABLES */

// struct with sensor data
// two instances

// navigation mode manual/auto
enum NavigationMode g_navigationMode = MANUAL;

// PD-constants
volatile uint8_t g_pdKd = 50;
volatile uint8_t g_pdKp = 100;

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

// Current position in millimeter, relative to bottom left
// We assume we start in the middle (square 24) in the X direction.
uint16_t volatile g_currentPosX = GridToMm(24);
uint16_t volatile g_currentPosY = GridToMm(0);

// We can either use bool + unsigned
enum Direction g_wheelDirectionLeft  = DIR_FORWARD;
enum Direction g_wheelDirectionRight = DIR_FORWARD;
uint8_t        g_wheelSpeedLeft      = 0;
uint8_t        g_wheelSpeedRight     = 0;

// or a signed value.
// int8_t WHEEL_SPEED_LEFT = 0;
// int8_t WHEEL_SPEED_RIGHT = 0;
// depending on what's easiest when translating to PWM values for the robot
// and/or what the navigation algorithm wanna use.

// Current navigation goal
volatile bool       g_navigationGoalSet     = false;
uint16_t            g_navigationGoalX       = 24;
uint16_t            g_navigationGoalY       = 0;
uint16_t            g_navigationGoalHeading = 0;

// Map
int8_t g_navigationMap[MAP_X_MAX][MAP_Y_MAX];

void send_debug(uint16_t value, int8_t type)
{
    static struct data_packet data;
    data.address = ADR_DEBUG;
    data.byte_count = 3;
    data.bytes[0] = type;
    data.bytes[1] = Uint16ToByte0(value);
    data.bytes[2] = Uint16ToByte1(value);
    Uart_Send_0(&data);
}

void send_debug_2(uint16_t value_1, uint16_t value_2, int8_t type)
{
    static struct data_packet data;
    data.address = ADR_DEBUG;
    data.byte_count = 5;
    data.bytes[0] = type;
    data.bytes[1] = Uint16ToByte0(value_1);
    data.bytes[2] = Uint16ToByte1(value_1);
    data.bytes[3] = Uint16ToByte0(value_2);
    data.bytes[4] = Uint16ToByte1(value_2);
    Uart_Send_0(&data);
}