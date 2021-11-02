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

uint8_t NAVIGATION_MODE = NAVIGATION_MODE_MANUAL;
uint8_t PD_KD = 0;
uint8_t PD_KP = 0;

// Don't yet know how this is specified for the competition
uint16_t CURRENT_HEADING = FULL_TURN/4;
uint8_t CURRENT_POS_X = 24;
uint8_t CURRENT_POS_Y = 0;

// We can either use bool + unsigned
bool WHEEL_DIR_LEFT = DIR_FORWARD;
bool WHEEL_DIR_RIGHT = DIR_FORWARD;
uint8_t WHEEL_SPEED_LEFT = 0;
uint8_t WHEEL_SPEED_RIGHT = 0;

// or a signed value.
//int8_t WHEEL_SPEED_LEFT = 0;
//int8_t WHEEL_SPEED_RIGHT = 0;
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

uint8_t NAVIGATION_GOAL_TYPE = NAVIGATION_GOAL_NONE;
uint8_t NAVIGATION_GOAL_X = 24;
uint8_t NAVIGATION_GOAL_Y = 0;
uint8_t NAVIGATION_GOAL_HEADING = 0;


#endif
