// TSEA29, initial author JL
// rotation math
//
#include "../AVR_common/robot.h"
#include "../AVR_common/sensors.h"
#include "../AVR_common/uart.h"
#include "../AVR_testing/test.h"
#include "navigation_unit.h"
#include <math.h>
#include <stdlib.h>

// if we hit right at the corner of a wall, we don't know which
// coordinate it corresponds to. So we throw out all values too close
// to the corners.
#define CORNER_SENSITIVITY 30

// map update throws out an update if a wall is too far from where it can be
#define MAX_ERROR 50

#define AXLE_WIDTH 200          // must be measured
#define MID_TO_WHEEL_CENTER 141 // must be measured

#define USE_ODO_FOR_HEADING 1
#define M_TAU (2 * M_PI)
#define min(x, y) x < y ? x : y
double g_cosHeading;
double g_sinHeading;

const double COS_QUARTERS[] = { 1.0, 0, -1.0, 0 };
const double SIN_QUARTERS[] = { 0, 1, 0, -1 };

int8_t laser_positive_x(uint16_t x, uint16_t y, uint8_t end_x_coord, double delta_y);
int8_t laser_negative_x(uint16_t x, uint16_t y, uint8_t end_x_coord, double delta_y);
int8_t laser_positive_y(uint16_t x, uint16_t y, uint8_t end_y_coord, double delta_x);
int8_t laser_negative_y(uint16_t x, uint16_t y, uint8_t end_y_coord, double delta_x);

int8_t laser_loop(uint8_t  max_steps,
                  uint8_t  a_0,
                  uint16_t b_0,
                  int8_t   delta_a,
                  double   delta_b,
                  int8_t (*f)(uint8_t, uint8_t));
int8_t mark_empty(uint8_t x, uint8_t y);
int8_t mark_empty_rev(uint8_t y, uint8_t x);

int8_t draw_laser_line(int8_t  x,
                       int8_t  y,
                       uint8_t  sensor_direction,
                       uint16_t distance);

void send_map_update(uint8_t x, uint8_t y, int8_t value);
// using the trigonometric addition formulas
double laser_cos(uint8_t direction)
{
    return g_cosHeading * COS_QUARTERS[direction] +
           g_sinHeading * SIN_QUARTERS[direction];
}
double laser_sin(uint8_t direction)
{
    return g_sinHeading * COS_QUARTERS[direction] +
           g_cosHeading * SIN_QUARTERS[direction];
}

double heading_to_radian(uint16_t heading)
{
    // return ((double) (heading * 2)) / ((double) FULL_TURN) * M_PI;
    return (double)heading / FULL_TURN * M_TAU;
}

uint16_t radian_to_heading(double angle)
{
    return round(angle * FULL_TURN / M_TAU);
}

// calculate new heading, using only the odometers
int16_t odo_heading_change(struct sensor_data* data)
{
    int16_t arc_length;
    int16_t res;
    if (g_wheelDirectionLeft != g_wheelDirectionRight)
    {
        // from the formula of circle sector
        // see image rotate_on_the_spot_heading_update.jpg
        arc_length = (data->odometer_right + data->odometer_left) / 2;
        res = radian_to_heading((double) arc_length / MID_TO_WHEEL_CENTER);
    }
    else
    {
        arc_length = data->odometer_right - data->odometer_left;
        res = radian_to_heading((double) arc_length / AXLE_WIDTH);
    }

    if (g_wheelDirectionRight == DIR_FORWARD)
    {
        return res;
    }
    return -res;
}

int8_t calculate_heading_and_position(struct sensor_data* data)
{
    // if needed we can check what direction we intended to turn in
    // by checking wheelDirection for turning on the spot, and comparing ordos
    // if driving straight, but looks like the gyro gives a signed value
    // with positive clockwise
    double   headingAvg;
    int16_t  distance;
    int16_t heading_change;

    // overflow handles modulo for us

#if USE_ODO_FOR_HEADING
    heading_change = odo_heading_change(data);
#else
    heading_change = data->gyro;
#endif
    // only update position if driving straight
    if (g_wheelDirectionLeft == g_wheelDirectionRight &&
        (data->odometer_left > 0 || data->odometer_right > 0))
    {
        // if we turned while driving, we don't know when along the line
        // we changed direction. So we take half the gyro change and assume
        // a straight line.
        headingAvg = heading_to_radian(g_currentHeading + heading_change / 2);
        //headingAvg = heading_to_radian(g_currentHeading + heading_change);

        if (data->odometer_left == data->odometer_right)
        {
            distance = data->odometer_left;
        }
        else
        {
            // this can also be calibrated with wheel speed
            // crude estimate
            //distance = (data->odometer_left + data->odometer_right) / 2;

            // estimated w/ circle sector + pythagoras
            if (data->odometer_left < data->odometer_right)
            {
                distance = (double) M_SQRT2 * (data->odometer_left * AXLE_WIDTH
                    / (data->odometer_right - data->odometer_left)
                    + AXLE_WIDTH / 2);
            }
            else
            {
                distance = (double) M_SQRT2 * (data->odometer_right * AXLE_WIDTH
                    / (data->odometer_left - data->odometer_right)
                    + AXLE_WIDTH / 2);
            }
            //distance = 1.414214 * ((double) min(data->odometer_left, data->odometer_right)
             //       / heading_change + AXLE_WIDTH/2);
        }

        if (g_wheelDirectionLeft == DIR_BACKWARD)
        {
            distance = -distance;
        }

        g_currentPosX += round(cos(headingAvg) * distance);
        g_currentPosY += round(sin(headingAvg) * distance);
    }
    g_currentHeading += heading_change;

    // TODO use lidar & IR to calibrate heading and position
    return 0;
}

int8_t update_map(struct sensor_data* data)
{
    // cache cos & sin
    double headingRad = heading_to_radian(g_currentHeading);

    g_cosHeading = cos(headingRad);
    g_sinHeading = sin(headingRad);

    draw_laser_line(LaserPositionX(data, data->lidar_forward),
                    LaserPositionY(data, data->lidar_forward),
                    LaserDirection(data, data->lidar_forward),
                    data->lidar_forward);
    // lidar backward
    // 4*ir

    // We can optionally also check if the robot is currently standing on top of
    // a potential wall (or several), and mark those as empty.
    // This can be more or less ambitious, taking into account the width of the
    // robot and stuff.
    return 0;
}

int8_t draw_laser_line(int8_t  laser_x,
                       int8_t  laser_y,
                       uint8_t  sensor_direction,
                       uint16_t distance)
{
    // see laser_intersecting_pot_walls.jpg
    // where x_r,y_r is pos_x,pos_y
    // x_s, y_s is laser_x, laser_y
    // d is distance
    // x_n , y_n is end_x, end_y

    uint8_t end_x_coord;
    uint8_t end_y_coord;
    double cos = laser_cos(sensor_direction);
    double sin = laser_sin(sensor_direction);
    double tan = 0.0;
    double cot = 0.0;
    if (cos != 0.0 && sin != 0.0)
    {
        tan = fabs(sin / cos);
        cot = fabs(cos / sin);
    }

    uint16_t start_x = g_currentPosX + g_cosHeading * laser_x + laser_cos(1) * laser_y;
    uint16_t start_y = g_currentPosY + g_sinHeading * laser_y + laser_sin(1) * laser_x;

    // Calculate X, Y for endpoint when laser hits wall -> end_x, end_y
    // given the distance.
    uint16_t end_x = start_x + cos * distance;
    uint16_t end_y = start_y + sin * distance;

    if (end_x > UINT16_MAX - MAX_ERROR)
    {
        end_x_coord = 0;
    }
    else
    {
        end_x_coord = round((double) end_x / 400);
    }
    if (end_y > UINT16_MAX - MAX_ERROR)
    {
        end_y_coord = 0;
    }
    else
    {
        end_y_coord = round((double) end_y / 400);
    }

    if (end_x_coord > MAP_X_MAX || end_y_coord > MAP_Y_MAX)
    {
        return -1;
    }

    uint16_t x_dif = abs((int16_t)(end_x - end_x_coord * 400));
    uint16_t y_dif = abs((int16_t)(end_y - end_y_coord * 400));

    if (x_dif > MAX_ERROR && y_dif > MAX_ERROR)
    {
        return -1;
    }

    // in which quadrant is the laser pointing
    uint8_t quadrant_heading =
      ((g_currentHeading >> 14) + sensor_direction) & 3;

    // Depending on direction we're hitting different walls of the cell, and
    // that gives different coordinates for the cell
    if (x_dif < y_dif)
    {
        // we're hitting the left or right side of the wall
        // to get the proper y coordinate we should round it down
        end_y_coord = end_y / 400;

        // bit magic to check if we're in 2nd or 3rd quadrant
        // if so we're hitting the right side, and should subtract one
        if (quadrant_heading == 1 || quadrant_heading == 2)
        {
            end_x_coord -= 1;
        }
    }
    else
    {
        // top or bottom
        end_x_coord = end_x / 400;

        // if in the 3rd or 4th quadrant, we're hitting the top
        // and should subtract one from the y coordinate.
        if (quadrant_heading & 0x2)
        {
            end_y_coord -= 1;
        }
    }

    if (end_x_coord < MAP_X_MAX && end_y_coord < MAP_Y_MAX &&
            (x_dif > CORNER_SENSITIVITY || y_dif > CORNER_SENSITIVITY))
    {
        // mark as wall
        g_navigationMap[end_x_coord][end_y_coord] -= 1;

        // if the cell state changed, send update to com-unit
        if (g_navigationMap[end_x_coord][end_y_coord] == -1)
        {
            send_map_update(end_x_coord, end_y_coord, WALL);
        }
        else if (g_navigationMap[end_x_coord][end_y_coord] == 0)
        {
            send_map_update(end_x_coord, end_y_coord, UNKNOWN);
        }
    }

    // TODO: throw out values that are incorrect due to the wall being
    // too close (the measured voltage from IR, and therefore the distance, will
    // be misleading).



    /* Calculate which cells the laser passed trough before hitting the wall */
    // A line drawn across a square grid at an angle will intersect with points
    // at (X_0, Y_0), (X_0+1, Y_1), (X_0+2, Y_2) and (X_0, Y_0), (X_1, Y_0+1),
    // (X_2, Y_0+2) (where these will yield the same set of points for 45
    // degrees / tau/8 radians)

    switch (quadrant_heading)
    {
        case 0:
            if (laser_positive_x(start_x, start_y, end_x_coord, tan) ||
                    laser_positive_y(start_x, start_y, end_y_coord, cot))
            {
                return -1;
            }
            break;
        case 1:
            if (laser_negative_x(start_x, start_y, end_x_coord, tan) ||
                    laser_positive_y(start_x, start_y, end_y_coord, -cot))
            {
                return -1;
            }
            break;
        case 2:
            if (laser_negative_x(start_x, start_y, end_x_coord, -tan) ||
                    laser_negative_y(start_x, start_y, end_y_coord, -cot))
            {
                return -1;
            }
            break;
        case 3:
            if (laser_positive_x(start_x, start_y, end_x_coord, -tan) ||
                    laser_negative_y(start_x, start_y, end_y_coord, cot))
            {
                return -1;
            }
            break;
        default:
            return -1;
    }
    return 0;
}

int8_t laser_positive_x(uint16_t x, uint16_t y, uint8_t end_x_coord, double delta_y)
{
    uint8_t  x_0       = x % 400 ? x / 400 + 1 : x / 400; // ceil
    uint16_t y_0       = y + delta_y * (x_0 - (double) x / 400);
    uint8_t  max_steps = end_x_coord - x_0;
    if (max_steps > MAP_X_MAX)
    {
        return 0;
    }
    return laser_loop(max_steps, x_0, y_0, +1.0, delta_y, mark_empty);
}

int8_t laser_negative_x(uint16_t x, uint16_t y, uint8_t end_x_coord, double delta_y)
{
    uint8_t  x_0       = x / 400 - 1; // integer division == floor
    uint16_t y_0       = y + delta_y * ((double) x / 400 - x_0 + 1);
    uint8_t  max_steps = x_0 - end_x_coord;
    if (max_steps > MAP_X_MAX)
    {
        return 0;
    }
    return laser_loop(max_steps, x_0, y_0, -1.0, delta_y, mark_empty);
}

int8_t laser_positive_y(uint16_t x, uint16_t y, uint8_t end_y_coord, double delta_x)
{
    uint8_t  y_0       = y % 400 ? y / 400 + 1 : y / 400; // ceil
    uint16_t x_0       = x + delta_x * (y_0 - (double) y / 400);
    uint8_t  max_steps = end_y_coord - y_0;
    if (max_steps > MAP_Y_MAX)
    {
        return 0;
    }
    return laser_loop(max_steps, y_0, x_0, +1.0, delta_x, mark_empty_rev);
}

int8_t laser_negative_y(uint16_t x, uint16_t y, uint8_t end_y_coord, double delta_x)
{
    uint8_t  y_0       = y / 400 - 1; // integer division == floor
    uint16_t x_0       = x + delta_x * ((double) y / 400 - y_0 + 1);
    uint8_t  max_steps = y_0 - end_y_coord;
    if (max_steps > MAP_Y_MAX)
    {
        return 0;
    }
    return laser_loop(max_steps, y_0, x_0, -1.0, delta_x, mark_empty_rev);
}

int8_t laser_loop(uint8_t  max_steps,
                  uint8_t  a,
                  uint16_t b_0,
                  int8_t   delta_a,
                  double   delta_b,
                  int8_t (*f)(uint8_t, uint8_t))
{
    int8_t res = 0;
    double raw_b;
    for (uint8_t steps = 0; steps < max_steps; ++steps)
    {
        raw_b = (b_0 + steps * delta_b * 400);
        if (abs(raw_b - round(raw_b / 400)*400) > CORNER_SENSITIVITY)
        {
            res += (*f)(a, floor(raw_b / 400));
        }
        a += delta_a;
    }
    return res;
}

int8_t mark_empty_rev(uint8_t y, uint8_t x)
{
    return mark_empty(x, y);
}

int8_t mark_empty(uint8_t x, uint8_t y)
{
    if (x >= MAP_X_MAX || y >= MAP_Y_MAX)
    {
        return -1;
    }
    if (g_navigationMap[x][y] == 0)
    {
        g_navigationMap[x][y] = 1;
        send_map_update(x, y, 1);
    }
    else if (g_navigationMap[x][y] == -1)
    {
        g_navigationMap[x][y] = 0;
        send_map_update(x, y, 0);
    }
    else if (g_navigationMap[x][y] != INT8_MAX)
    {
        g_navigationMap[x][y] += 1;
    }
    else
    {
        return -1;
    }
    return 0;
}

#define COM_UNIT_INTERFACE 1
void send_map_update(uint8_t x, uint8_t y, int8_t value)
{
    struct data_packet packet;
    packet.address = MAP_UPDATE;
    packet.byte_count = 3;
    packet.bytes[0] = x;
    packet.bytes[1] = y;
    packet.bytes[2] = value;

    DATA_Transmit(COM_UNIT_INTERFACE, &packet);
}

#if __TEST__

Test_test(Test, heading_to_radian)
{
    Test_assertFloatEquals(heading_to_radian(0), 0.0);
    Test_assertFloatEquals(heading_to_radian(32768), M_PI);
    Test_assertFloatEquals(heading_to_radian(16384), M_PI / 2);
    Test_assertFloatEquals(heading_to_radian(-16384), 6 * M_PI / 4);
}

Test_test(Test, calc_heading_and_pos)
{
    uint16_t       oldCurrentPosX         = g_currentPosX;
    uint16_t       oldCurrentPosY         = g_currentPosY;
    uint16_t       oldHeading             = g_currentHeading;
    enum Direction oldWheelDirectionLeft  = g_wheelDirectionLeft;
    enum Direction oldWheelDirectionRight = g_wheelDirectionRight;

    struct sensor_data data;
    data.odometer_left  = 0;
    data.odometer_right = 0;
    data.gyro           = 0;

    g_currentHeading      = 0;
    g_currentPosX         = 0;
    g_currentPosY         = 0;
    g_wheelDirectionLeft  = DIR_FORWARD;
    g_wheelDirectionRight = DIR_FORWARD;

    Test_assertEquals(calculate_heading_and_position(&data), 0);
    Test_assertEquals(g_currentPosX, 0);
    Test_assertEquals(g_currentPosY, 0);
    Test_assertEquals(g_currentHeading, 0);
    Test_assertEquals(g_wheelDirectionLeft, DIR_FORWARD);
    Test_assertEquals(g_wheelDirectionRight, DIR_FORWARD);

#if !USE_ODO_FOR_HEADING
    // turn one quarter to the left
    data.gyro = 16384; // FULL_TURN / 4
#else
    // split into two eigth turns
    data.odometer_left = MID_TO_WHEEL_CENTER * M_TAU / 8; //157
    data.odometer_right = MID_TO_WHEEL_CENTER * M_TAU / 8;
    g_wheelDirectionLeft = DIR_BACKWARD;
    Test_assertEquals(calculate_heading_and_position(&data), 0);
    Test_assertEquals(g_currentPosX, 0);
    Test_assertEquals(g_currentPosY, 0);
    Test_assertEquals(g_currentHeading, 8137); // 0.3 degree error
    g_currentHeading = 8192;
#endif

    Test_assertEquals(calculate_heading_and_position(&data), 0);
    Test_assertEquals(g_currentPosX, 0);
    Test_assertEquals(g_currentPosY, 0);
#if !USE_ODO_FOR_HEADING
    Test_assertEquals(g_currentHeading, 16384);
#else
    Test_assertEquals(g_currentHeading, 16329);
#endif

#if !USE_ODO_FOR_HEADING
    // then one eight to the right
    data.gyro = -8192; // FULL_TURN / 8
#else
    // split into two eigth turns
    data.odometer_left = MID_TO_WHEEL_CENTER * M_TAU / 8; //157
    data.odometer_right = MID_TO_WHEEL_CENTER * M_TAU / 8;
    g_wheelDirectionLeft = DIR_FORWARD;
    g_wheelDirectionRight = DIR_BACKWARD;
#endif

    Test_assertEquals(calculate_heading_and_position(&data), 0);
    Test_assertEquals(g_currentPosX, 0);
    Test_assertEquals(g_currentPosY, 0);
    Test_assertEquals(g_currentHeading, 8192);

    // now forward 100mm
    data.gyro           = 0;
    data.odometer_left  = 100;
    data.odometer_right = 100;
    g_wheelDirectionLeft = DIR_FORWARD;
    g_wheelDirectionRight = DIR_FORWARD;
    Test_assertEquals(calculate_heading_and_position(&data), 0);
    Test_assertEquals(g_currentPosX, 71); //70.7
    Test_assertEquals(g_currentPosY, 71); //70.7
    Test_assertEquals(g_currentHeading, 8192);

#if !USE_ODO_FOR_HEADING
    data.gyro = -10691;
#endif
    data.odometer_left  = 255;
    data.odometer_right = 50;

    // robot will average out the heading, and is the same as a 90 degree turn
    // followed by movement, then another 90 degrees
    Test_assertEquals(calculate_heading_and_position(&data), 0);
    Test_assertEquals(g_currentPosX, 272); //273.6 in theory
    Test_assertEquals(g_currentPosY, 127); //127.7 in theory
    Test_assertEquals(g_currentHeading, 63037);

    // then reverse 100mm
    // we're now standing at 5/8s of a turn
    data.gyro             = 0;
    g_wheelDirectionLeft  = DIR_BACKWARD;
    g_wheelDirectionRight = DIR_BACKWARD;
    data.odometer_left = 100;
    data.odometer_right = 100;

    Test_assertEquals(calculate_heading_and_position(&data), 0);
    Test_assertEquals(g_currentPosX, 175); //174.856
    Test_assertEquals(g_currentPosY, 151); //150.73
    Test_assertEquals(g_currentHeading, 63037);

    g_currentPosX         = oldCurrentPosX;
    g_currentPosY         = oldCurrentPosY;
    g_currentHeading      = oldHeading;
    g_wheelDirectionLeft  = oldWheelDirectionLeft;
    g_wheelDirectionRight = oldWheelDirectionRight;
}

Test_test(Test, mark_empty)
{
    g_navigationMap[0][0] = -2;
    Test_assertEquals(mark_empty(0, 0), 0);
    Test_assertEquals(g_navigationMap[0][0], -1);

    Test_assertEquals(mark_empty(0, 0), 0);
    Test_assertEquals(g_navigationMap[0][0], 0);

    Test_assertEquals(mark_empty(0, 0), 0);
    Test_assertEquals(g_navigationMap[0][0], 1);

    Test_assertEquals(mark_empty(0, 0), 0);
    Test_assertEquals(g_navigationMap[0][0], 2);

    g_navigationMap[0][0] = INT8_MAX;
    Test_assertEquals(mark_empty(0, 0), -1);
    Test_assertEquals(g_navigationMap[0][0], INT8_MAX);

    g_navigationMap[0][0] = 0;
}

Test_test(Test, laser_loop_1)
{
    // 2 cells straight to the right
    Test_assertEquals(laser_loop(2, 1, 200, +1.0, 0, mark_empty), 0);

    Test_assertEquals(g_navigationMap[1][0], 1);
    Test_assertEquals(g_navigationMap[2][0], 1);
    g_navigationMap[1][0] = 0;
    g_navigationMap[2][0] = 0;
}

Test_test(Test, laser_loop_2)
{
    // 2 cells straight to the left
    Test_assertEquals(laser_loop(2, 2, 200, -1.0, 0, mark_empty), 0);

    Test_assertEquals(g_navigationMap[2][0], 1);
    Test_assertEquals(g_navigationMap[1][0], 1);
    g_navigationMap[1][0] = 0;
    g_navigationMap[2][0] = 0;
}

Test_test(Test, laser_loop_3)
{
    // 3 cells straight to the right into the wall
    Test_assertEquals(laser_loop(3, MAP_X_MAX-3, 200,
                +1.0, 0, mark_empty), 0);

    Test_assertEquals(g_navigationMap[MAP_X_MAX-1][0], 1);
    Test_assertEquals(g_navigationMap[MAP_X_MAX-2][0], 1);
    Test_assertEquals(g_navigationMap[MAP_X_MAX-3][0], 1);
    g_navigationMap[MAP_X_MAX-1][0] = 0;
    g_navigationMap[MAP_X_MAX-2][0] = 0;
    g_navigationMap[MAP_X_MAX-3][0] = 0;
}

Test_test(Test, laser_loop_4)
{
    // 4 cells straight to the left into the wall
    Test_assertEquals(laser_loop(4, 3, 200, -1.0, 0, mark_empty), 0);

    Test_assertEquals(g_navigationMap[3][0], 1);
    Test_assertEquals(g_navigationMap[2][0], 1);
    Test_assertEquals(g_navigationMap[1][0], 1);
    Test_assertEquals(g_navigationMap[0][0], 1);

    g_navigationMap[0][0] = 0;
    g_navigationMap[1][0] = 0;
    g_navigationMap[2][0] = 0;
    g_navigationMap[3][0] = 0;
}

Test_test(Test, laser_loop_5)
{
    // 1/8 turn laser, hitting corners so nothing should be updated
    Test_assertEquals(laser_loop(10, 1, 400, 1, 1.0, mark_empty), 0);
}

Test_test(Test, laser_loop_6)
{
    // 1/8 turn laser, starting in (200,0) and hitting the first x-wall
    // at (400, 200).
    Test_assertEquals(laser_loop(5, 1, 200, 1, 1.0, mark_empty), 0);

    Test_assertEquals(g_navigationMap[1][0], 1);
    Test_assertEquals(g_navigationMap[2][1], 1);
    Test_assertEquals(g_navigationMap[3][2], 1);
    Test_assertEquals(g_navigationMap[4][3], 1);
    Test_assertEquals(g_navigationMap[5][4], 1);

    g_navigationMap[1][0] = 0;
    g_navigationMap[2][1] = 0;
    g_navigationMap[3][2] = 0;
    g_navigationMap[4][3] = 0;
    g_navigationMap[5][4] = 0;
}

Test_test(Test, laser_loop_7)
{
    // 5/16 laser, starting in (,0) and hitting the first x-wall
    // at (400, 200).
    Test_assertEquals(laser_loop(4, 4, 200, -1, 2.4142135, mark_empty), 0);

    Test_assertEquals(g_navigationMap[4][0], 1);
    Test_assertEquals(g_navigationMap[3][2], 1);
    Test_assertEquals(g_navigationMap[2][5], 1);
    Test_assertEquals(g_navigationMap[1][7], 1);

    g_navigationMap[4][0] = 0;
    g_navigationMap[3][2] = 0;
    g_navigationMap[2][5] = 0;
    g_navigationMap[1][7] = 0;
}

Test_test(Test, laser_loop_8)
{
    // 11/16 laser, hitting the first wall at (1600,3400)
    Test_assertEquals(laser_loop(4, 4, 8*400+200, -1, -2.41421356, mark_empty), 0);

    Test_assertEquals(g_navigationMap[4][8], 1);
    Test_assertEquals(g_navigationMap[3][6], 1);
    Test_assertEquals(g_navigationMap[2][3], 1);
    Test_assertEquals(g_navigationMap[1][1], 1);

    g_navigationMap[4][8] = 0;
    g_navigationMap[3][6] = 0;
    g_navigationMap[2][3] = 0;
    g_navigationMap[1][1] = 0;
}

Test_test(Test, laser_loop_9)
{
    // 25/32 laser, hitting the first wall at (1600,3400)
    Test_assertEquals(laser_loop(4, 8, (MAP_Y_MAX-1)*400+200, 1,
                -5.02733949, mark_empty), 0);

    Test_assertEquals(g_navigationMap[8][24], 1);
    Test_assertEquals(g_navigationMap[9][19], 1);
    Test_assertEquals(g_navigationMap[10][14], 1);
    Test_assertEquals(g_navigationMap[11][9], 1);

    g_navigationMap[8][24] = 0;
    g_navigationMap[9][19] = 0;
    g_navigationMap[10][14] = 0;
    g_navigationMap[11][9] = 0;
}

Test_test(Test, laser_loop_10)
{
    Test_assertEquals(laser_loop(4, 3, 200, -1.0, 0, mark_empty_rev), 0);

    Test_assertEquals(g_navigationMap[0][3], 1);
    Test_assertEquals(g_navigationMap[0][2], 1);
    Test_assertEquals(g_navigationMap[0][1], 1);
    Test_assertEquals(g_navigationMap[0][0], 1);

    g_navigationMap[0][0] = 0;
    g_navigationMap[0][1] = 0;
    g_navigationMap[0][2] = 0;
    g_navigationMap[0][3] = 0;
}

Test_test(Test, laser_positive_x_1)
{
    // 2 cells straight to the right
    // there's a wall at (3, 0)
    Test_assertEquals(laser_positive_x(200, 200, 3, 0), 0);

    Test_assertEquals(g_navigationMap[1][0], 1);
    Test_assertEquals(g_navigationMap[2][0], 1);
    g_navigationMap[1][0] = 0;
    g_navigationMap[2][0] = 0;
}

Test_test(Test, laser_positive_x_2)
{
    // 2 cells straight to the right
    // there's a wall at (3, 0)
    for (uint16_t start_x = 199; start_x < 202; ++start_x)
    {
        for (uint16_t start_y = 199; start_y < 202; ++start_y)
        {
            Test_assertEquals(laser_positive_x(start_x, start_y, 3, 0), 0);

            Test_assertEquals(g_navigationMap[1][0], 1);
            Test_assertEquals(g_navigationMap[2][0], 1);
            g_navigationMap[1][0] = 0;
            g_navigationMap[2][0] = 0;
        }
    }
}

Test_test(Test, laser_negative_x_1)
{
    // 2 cells straight to the left from (1400, 200)
    Test_assertEquals(laser_negative_x(1400, 200, 0, 0), 0);

    Test_assertEquals(g_navigationMap[2][0], 1);
    Test_assertEquals(g_navigationMap[1][0], 1);
    g_navigationMap[1][0] = 0;
    g_navigationMap[2][0] = 0;
}

Test_test(Test, laser_negative_x_2)
{
    // 2 cells straight to the left from (1400, 200)
    for (uint16_t start_x = 1399; start_x < 1402; ++start_x)
    {
        for (uint16_t start_y = 199; start_y < 202; ++start_y)
        {
            Test_assertEquals(laser_negative_x(start_x, start_y, 0, 0), 0);

            Test_assertEquals(g_navigationMap[2][0], 1);
            Test_assertEquals(g_navigationMap[1][0], 1);
            g_navigationMap[1][0] = 0;
            g_navigationMap[2][0] = 0;
        }
    }
}

Test_test(Test, laser_positive_y_1)
{
    // 3 cells straight up into the wall, from (200, 8600)
    for (uint16_t start_x = 199; start_x < 202; ++start_x)
    {
        for (uint16_t start_y = 8599; start_y < 8602; ++start_y)
        {
            Test_assertEquals(laser_positive_y(start_x, start_y, 25, 0), 0);

            Test_assertEquals(g_navigationMap[0][MAP_Y_MAX-1], 1);
            Test_assertEquals(g_navigationMap[0][MAP_Y_MAX-2], 1);
            Test_assertEquals(g_navigationMap[0][MAP_Y_MAX-3], 1);

            g_navigationMap[0][MAP_Y_MAX-1] = 0;
            g_navigationMap[0][MAP_Y_MAX-2] = 0;
            g_navigationMap[0][MAP_Y_MAX-3] = 0;
        }
    }
}

Test_test(Test, laser_negative_y_1)
{
    // 4 cells straight down into the wall
    // same as laser_loop_10
    for (uint16_t start_x = 199; start_x < 202; ++start_x)
    {
        for (uint16_t start_y = 1799; start_y < 1802; ++start_y)
        {
            Test_assertEquals(laser_negative_y(start_x, start_y, -1, 0), 0);

            Test_assertEquals(g_navigationMap[0][3], 1);
            Test_assertEquals(g_navigationMap[0][2], 1);
            Test_assertEquals(g_navigationMap[0][1], 1);
            Test_assertEquals(g_navigationMap[0][0], 1);

            g_navigationMap[0][0] = 0;
            g_navigationMap[0][1] = 0;
            g_navigationMap[0][2] = 0;
            g_navigationMap[0][3] = 0;
        }
    }
}
Test_test(Test, draw_laser_line)
{
    uint16_t oldPosX = g_currentPosX;
    uint16_t oldPosY = g_currentPosY;
    uint16_t oldHeading = g_currentHeading;

    g_currentPosX = 500;
    g_currentPosY = 200;
    g_currentHeading = 0;

    double headingRad = heading_to_radian(g_currentHeading);
    g_cosHeading = cos(headingRad);
    g_sinHeading = sin(headingRad);

    // a laser positioned 100 ahead of the robot
    // pointing to the right
    // detects a wall at 1400mm distance
    Test_assertEquals(draw_laser_line(100, 0, 0, 1400), 0);

    Test_assertEquals(g_navigationMap[2][0], 1);
    Test_assertEquals(g_navigationMap[3][0], 1);
    Test_assertEquals(g_navigationMap[4][0], 1);
    Test_assertEquals(g_navigationMap[5][0], -1);

    g_navigationMap[2][0] = 0;
    g_navigationMap[3][0] = 0;
    g_navigationMap[4][0] = 0;
    g_navigationMap[5][0] = 0;

    // a laser positioned 100 behind the robot
    g_currentPosX = 1100;
    Test_assertEquals(draw_laser_line(-100, 0, 2, 1000), 0);

    Test_assertEquals(g_navigationMap[0][0], 1);
    Test_assertEquals(g_navigationMap[1][0], 1);

    g_navigationMap[0][0] = 0;
    g_navigationMap[1][0] = 0;






    g_currentPosX = oldPosX;
    g_currentPosY = oldPosY;
    g_currentHeading = oldHeading;

}

Test_test(Test, draw_laser_line_2)
{
    uint16_t oldPosX = g_currentPosX;
    uint16_t oldPosY = g_currentPosY;
    uint16_t oldHeading = g_currentHeading;

    g_currentHeading = 48392;

    double headingRad = heading_to_radian(g_currentHeading);
    g_cosHeading = cos(headingRad);
    g_sinHeading = sin(headingRad);

    g_currentPosX = GridToMm(24);
    g_currentPosY = GridToMm(12);

    Test_assertEquals(g_currentPosX, 9800);
    Test_assertEquals(g_currentPosY, 5000);

    Test_assertEquals(draw_laser_line(100, 0, 0, 5012), 0);

    Test_assertEquals(g_navigationMap[24][11], 1);
    Test_assertEquals(g_navigationMap[24][10], 1);
    Test_assertEquals(g_navigationMap[24][9], 1);
    Test_assertEquals(g_navigationMap[24][8], 1);
    Test_assertEquals(g_navigationMap[24][7], 1);
    Test_assertEquals(g_navigationMap[24][6], 1);

    Test_assertEquals(g_navigationMap[23][3], 1);
    Test_assertEquals(g_navigationMap[23][2], 1);
    Test_assertEquals(g_navigationMap[23][1], 1);
    Test_assertEquals(g_navigationMap[23][0], 1);

    g_navigationMap[24][11] = 0;
    g_navigationMap[24][10] = 0;
    g_navigationMap[24][9] = 0;
    g_navigationMap[24][8] = 0;
    g_navigationMap[24][7] = 0;
    g_navigationMap[24][6] = 0;
    g_navigationMap[23][3] = 0;
    g_navigationMap[23][2] = 0;
    g_navigationMap[23][1] = 0;
    g_navigationMap[23][0] = 0;

    Test_assertEquals(draw_laser_line(100, 0, 0, 4612), 0);

    Test_assertEquals(g_navigationMap[24][11], 1);
    Test_assertEquals(g_navigationMap[24][10], 1);
    Test_assertEquals(g_navigationMap[24][9], 1);
    Test_assertEquals(g_navigationMap[24][8], 1);
    Test_assertEquals(g_navigationMap[24][7], 1);
    Test_assertEquals(g_navigationMap[24][6], 1);

    Test_assertEquals(g_navigationMap[23][3], 1);
    Test_assertEquals(g_navigationMap[23][2], 1);
    Test_assertEquals(g_navigationMap[23][1], 1);
    Test_assertEquals(g_navigationMap[23][0], -1);

    g_navigationMap[24][11] = 0;
    g_navigationMap[24][10] = 0;
    g_navigationMap[24][9] = 0;
    g_navigationMap[24][8] = 0;
    g_navigationMap[24][7] = 0;
    g_navigationMap[24][6] = 0;
    g_navigationMap[23][3] = 0;
    g_navigationMap[23][2] = 0;
    g_navigationMap[23][1] = 0;
    g_navigationMap[23][0] = 0;

    g_currentPosX = oldPosX;
    g_currentPosY = oldPosY;
    g_currentHeading = oldHeading;
}

// test corner sensitivity
// test max error
#endif // __TEST__
