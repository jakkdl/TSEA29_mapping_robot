#include <stdint.h>
#include "robot.h"

const uint8_t ADR_DATA_PACKETS[] = {
    2, // LIDAR_FORWARD
    2, // LIDAR_BACKWARD
    2, // IR_LEFTFRONT
    2, // IR_LEFTBACK
    2, // IR_RIGHTFRONT
    2, // IR_RIGHTBACK
    2, // ODOMETER **DESIGN SPEC INCORRECT**
    2, // GYRO
    4, // POSITION
    2, // HEADER
    3, // MAP_UPDATE
    1, // COMMAND
    7, // DEBUG
    1, // PD_KP
    1, // PD_KD
    1  // PARITY_ERROR
};
