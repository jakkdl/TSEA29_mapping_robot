#ifndef NAVIGATION_UNIT_H
#define NAVIGATION_UNIT_H
#include <stdbool.h>

#include "robot.h"

#define NAVIGATION_MODE_MANUAL 0
#define NAVIGATION_MODE_AUTONOMOUS 1

#define DIR_FORWARD 0
#define DIR_BACKWARD 1


#define FULL_TURN 65536 // 1 << 16


struct Com_packet
{
    short adress;
    short packet_count;
    short data_packets[7];
};


short handle_command(short id);
short resend(short adress);
short set_pd_kd(short kd);
short set_pd_kp(short kp);
short command_stop();
short command_start();
short command_set_target_square(short id);

short NAVIGATION_MODE = NAVIGATION_MODE_MANUAL;
short PD_KD = 0;
short PD_KP = 0;

// Don't yet know how this is specified for the competition
int CURRENT_HEADING = FULL_TURN/4;
short CURRENT_POS_X = 24;
short CURRENT_POS_Y = 0;

// We can either use bool + unsigned
bool WHEEL_DIR_LEFT = DIR_FORWARD;
bool WHEEL_DIR_RIGHT = DIR_FORWARD;
unsigned short WHEEL_SPEED_LEFT = 0;
unsigned short WHEEL_SPEED_RIGHT = 0;

// or a signed value.
//short WHEEL_SPEED_LEFT = 0;
//short WHEEL_SPEED_RIGHT = 0;
// depending on what's easiest when translating to PWM values for the robot

// Unsure how to represent this, currently broke it into
// no goal set
#define NAVIGATION_GOAL_NONE 0
// We only want to change our direction to NAVIGATION_GOAL_HEADING
// ignore NAVIGATION_GOAL_X & Y
#define NAVIGATION_GOAL_TURN 1
// We want to move to NAVIGATION_GOAL_X & Y, and might need to turn.
// ignore NAVIGATION_GOAL_HEADING
#define NAVIGATION_GOAL_MOVE 2

short NAVIGATION_GOAL_TYPE = NAVIGATION_GOAL_NONE;
short NAVIGATION_GOAL_X = 24;
short NAVIGATION_GOAL_Y = 0;
short NAVIGATION_GOAL_HEADING = 0;


#endif
