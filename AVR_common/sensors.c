#include <stdint.h>
//TODO measure
const int8_t LASER_POSITION_X[] =
{
    100, //lidar_forward;
    -100, //lidar_backward;
    50, //ir_leftfront;
    -50, //ir_leftback;
    50, //ir_rightfront;
    -50, //ir_rightback;
};
const int8_t LASER_POSITION_Y[] =
{
    0, //lidar_forward;
    0, //lidar_backward;
    50, //ir_leftfront;
    50, //ir_leftback;
    -50, //ir_rightfront;
    -50, //ir_rightback;
};
const uint8_t LASER_DIRECTION[] =
{
    0, //lidar_forward;
    2, //lidar_backward;
    1, //ir_leftfront;
    1, //ir_leftback;
    3, //ir_rightfront;
    3, //ir_rightback;
};
