// Global data variables and constants used across the whole navigation unit

#ifndef NAVIGATION_UNIT_H
#define NAVIGATION_UNIT_H
#include <stdbool.h>

#include "robot.h"

//TODO make these Enums
#define NAVIGATION_MODE_MANUAL 0
#define NAVIGATION_MODE_AUTONOMOUS 1

#define DIR_FORWARD 0
#define DIR_BACKWARD 1

// Unsure how to represent this, currently broke it into
// no goal set
#define NAVIGATION_GOAL_NONE 0
// We only want to change our direction to NAVIGATION_GOAL_HEADING
// ignore NAVIGATION_GOAL_X & Y
#define NAVIGATION_GOAL_TURN 1
// We want to move to NAVIGATION_GOAL_X & Y, and might need to turn.
// ignore NAVIGATION_GOAL_HEADING
#define NAVIGATION_GOAL_MOVE 2

#define FULL_TURN 65536 // 1 << 16

#define grid_to_mm(coord) { (coord)*250 + 125 }

struct Com_packet
{
    uint8_t adress;
    uint8_t packet_count;
    uint8_t data_packets[7];
};


uint8_t handle_command(uint8_t id);
uint8_t resend(uint8_t adress);
uint8_t set_pd_kd(uint8_t kd);
uint8_t set_pd_kp(uint8_t kp);
uint8_t command_stop();
uint8_t command_start();
uint8_t command_set_target_square(uint8_t id);


/* GLOBAL VARIABLES */

// navigation mode manual/auto
uint8_t navigationMode = NAVIGATION_MODE_MANUAL;

// PD-constants
uint8_t pdkd = 0;
uint8_t pdkp = 0;

// Current heading, specified as a fraction of the maximal value (FULL_TURN) for the
// equivalent fraction around a full turn. So e.g. 1/3 of FULL_TURN is 1/3 turn to the left.
// right = 0
// up   = FULL_TURN/4   = 0b01 << 14 = 16384 = pow(2, 14)
// left = FULL_TURN/2   = 0b10 << 14 = 32768 = pow(2, 15)
// down = FULL_TURN*3/4 = 0b11 << 14 = 49152 pow(2, 14) + pow(2, 15)
// Don't yet know how the initial value of the heading is specified for the competition
uint16_t currentHeading = FULL_TURN/4;

// Current position in millimetre, relative to bottom left
// We assume we start in the middle (square 24) in the X direction.
uint16_t currentPosX = grid_to_mm(24);
uint16_t currentPosY = 0;

// We can either use bool + unsigned
bool wheelDirLeft = DIR_FORWARD;
bool wheelDirRight = DIR_FORWARD;
uint8_t wheelSpeedLeft = 0;
uint8_t wheelSpeedRight = 0;

// or a signed value.
//int8_t WHEEL_SPEED_LEFT = 0;
//int8_t WHEEL_SPEED_RIGHT = 0;
// depending on what's easiest when translating to PWM values for the robot
// and/or what the navigation algorithm wanna use.

// Current navigation goal
uint8_t navigationGoalType = NAVIGATION_GOAL_NONE;
uint8_t navigationGoalX = 24;
uint8_t navigationGoalY = 0;
uint8_t navigationGoalHeading = 0;


// Map
uint8_t navigationMap[49][25];

#endif
