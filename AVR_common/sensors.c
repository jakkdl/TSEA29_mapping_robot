#include <stdint.h>

const int8_t LASER_POSITION_X[] =
{
    120, //lidar_forward;
    -120, //lidar_backward;
    80, //ir_leftfront;
    -85, //ir_leftback;
    80, //ir_rightfront;
    -85, //ir_rightback;
};
const int8_t LASER_POSITION_Y[] =
{
    0, //lidar_forward;
    0, //lidar_backward;
    70, //ir_leftfront;
    70, //ir_leftback;
    -70, //ir_rightfront;
    -70, //ir_rightback;
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
