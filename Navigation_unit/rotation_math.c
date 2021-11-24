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

#define AXLE_WIDTH 200;          // must be measured
#define MID_TO_WHEEL_CENTER 141; // must be measured
#define WHEEL_CIRCUMFERENCE 65;
#define COGS 12; // should be checked

double g_cosHeading;
double g_sinHeading;

const double COS_QUARTERS[] = { 1.0, 0, -1.0, 0 };
const double SIN_QUARTERS[] = { 0, 1, 0, -1 };

void laser_positive_x(uint16_t x, uint16_t y, uint16_t end_x, double delta_y);
void laser_negative_x(uint16_t x, uint16_t y, uint16_t end_x, double delta_y);
void laser_positive_y(uint16_t x, uint16_t y, uint16_t end_y, double delta_x);
void laser_negative_y(uint16_t x, uint16_t y, uint16_t end_y, double delta_x);

void laser_loop_x(uint8_t  max_steps,
                  uint8_t  x_0,
                  uint16_t y_0,
                  int8_t   delta_x,
                  double   delta_y);
void laser_loop_y(uint8_t  max_steps,
                  uint16_t x_0,
                  uint8_t  y_0,
                  double   delta_x,
                  int8_t   delta_y);
void mark_empty(uint8_t x, uint8_t y);

int8_t draw_laser_line(uint8_t  x,
                       uint8_t  y,
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
    return (double)heading / FULL_TURN * 2 * M_PI;
}

int8_t calculate_heading_and_position(struct sensor_data* data)
{
    // if needed we can check what direction we intended to turn in
    // by checking wheelDirection for turning on the spot, and comparing ordos
    // if driving straight, but looks like the gyro gives a signed value
    // with positive clockwise
    uint16_t previousHeading = g_currentHeading;
    double   headingRad;
    int16_t  distance;

    // overflow handles modulo for us
    g_currentHeading += data->gyro;

    // only update position if driving straight
    if (g_wheelDirectionLeft == g_wheelDirectionRight &&
        (data->odometer_left > 0 || data->odometer_right > 0))
    {
        // if we turned while driving, we don't know when along the line
        // we changed direction. So we take half the gyro change and assume
        // a straight line.
        headingRad = heading_to_radian(previousHeading + data->gyro / 2);

        // this can also be calibrated with wheel speed
        distance = (data->odometer_left + data->odometer_right) / 2;
        if (g_wheelDirectionLeft == DIR_BACKWARD)
        {
            distance = -distance;
        }

        // We can likely use a look-up table here, but no use creating it
        // before we've tested the gyro / figured out the precision/values
        // out of it
        g_currentPosX += cos(headingRad) * distance;
        g_currentPosY += sin(headingRad) * distance;
    }

    // cache cos & sin
    headingRad = heading_to_radian(g_currentHeading);

    g_cosHeading = cos(headingRad);
    g_sinHeading = sin(headingRad);

    // TODO use lidar & IR to calibrate heading and position
    return 0;
}

int8_t update_map(struct sensor_data* data)
{
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

int8_t draw_laser_line(uint8_t  laser_x,
                       uint8_t  laser_y,
                       uint8_t  sensor_direction,
                       uint16_t distance)
{
    // see laser_intersecting_pot_walls.jpg
    // where x_r,y_r is pos_x,pos_y
    // x_s, y_s is laser_x, laser_y
    // d is distance
    // x_n , y_n is end_x, end_y

    double cos = laser_cos(sensor_direction);
    double sin = laser_cos(sensor_direction);

    uint16_t start_x = g_currentPosX + cos * laser_x;
    uint16_t start_y = g_currentPosY + sin * laser_y;

    // Calculate X, Y for endpoint when laser hits wall -> end_x, end_y
    // given the distance.
    uint16_t end_x = start_x + cos * distance;
    uint16_t end_y = start_y + sin * distance;

    uint8_t end_x_coord = round(end_x / 400);
    uint8_t end_y_coord = round(end_y / 400);

    uint16_t x_dif = abs((int16_t)(end_x - end_x_coord * 400));
    uint16_t y_dif = abs((int16_t)(end_x - end_x_coord * 400));

    if (x_dif < CORNER_SENSITIVITY && y_dif < CORNER_SENSITIVITY)
    {
        return -1;
    }

    if (x_dif > MAX_ERROR && y_dif > MAX_ERROR)
    {
        return -1;
    }

    // in which quadrant is the laser pointing
    uint8_t quadrant_heading =
      ((g_currentHeading >> 14) + sensor_direction) ^ 3;

    // Depending on direction we're hitting different walls of the cell, and
    // that gives different coordinates for the cell
    if (x_dif < y_dif)
    {
        // we're hitting the left or right side of the wall
        // to get the proper y coordinate we should round it down
        end_y_coord = floor(end_y / 400);

        // bit magic to check if we're in 2nd or 3rd quadrant
        // if so we're hitting the left side, and should subtract one
        if (quadrant_heading == 1 || quadrant_heading == 2)
        {
            end_x_coord -= 1;
        }
    }
    else
    {
        // top or bottom
        end_x_coord = floor(end_x / 400);

        // if in the 3rd or 4th quadrant, we're hitting the top
        // and should subtract one from the y coordinate.
        if (quadrant_heading & 0x2)
        {
            end_y_coord -= 1;
        }
    }

    if (end_x_coord < MAP_X_MAX && end_y_coord < MAP_Y_MAX)
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

    /*** Calculate which cells the laser passed trough before hitting the wall
     * ***/

    // A line drawn across a square grid at an angle will intersect with points
    // at (X_0, Y_0), (X_0+1, Y_1), (X_0+2, Y_2) and (X_0, Y_0), (X_1, Y_0+1),
    // (X_2, Y_0+2) (where these will yield the same set of points for 45
    // degrees / tau/8 radians) We handle these separately

    // calculate the first point where the laser crosses x%400 == 0
    // TODO rounding up/down, and +/- depends on heading
    //
    switch (quadrant_heading)
    {
        case 0:
            laser_positive_x(start_x, start_y, end_x, sin);
            laser_positive_y(start_x, start_y, end_y, cos);
            // y_0 = math.ceil(y / 400);
            // x_0 = x + cos * (y / 400 - y_0);
            // max_steps = round(end_y/400) - y_0;
            // loop(max_steps, x_0, y_0, 1, sin, &no_rounding, &floor);
            break;
        case 1:
            laser_negative_x(start_x, start_y, end_x, sin);
            laser_positive_y(start_x, start_y, end_y, -cos);
            break;
        case 2:
            laser_negative_x(start_x, start_y, end_x, -sin);
            laser_negative_y(start_x, start_y, end_y, -cos);
            // uint8_t x_0 = floor(x / 400);
            // uint8_t y_0 = y + sin * (x / 400 - x_0);
            break;
        case 3:
            laser_positive_x(start_x, start_y, end_x, -sin);
            laser_negative_y(start_x, start_y, end_y, cos);
            // uint8_t x_0 = ceil(x / 400);
            // uint8_t y_0 = y + sin * (x_0 - x / 400);
            break;
        default:
            return -1;
    }
    // loop(max_steps, x_0, y_0, delta_x, sin*delta_y, &no_rounding, &floor);
    /*
    //do the same for y%400 == 0
    y_0 = math.ceil(laser_y / 400);
    x_0 = laser_x + cos(heading) * (laser_y / 400) - y_0;
    // number of potential walls the laser crosses before hitting the end wall
    max_steps = end_y - y_0;

    for (int steps = 0; steps < max_steps; ++steps)
    {
        int y = y_0 + steps;
        int x = math.floor(x_0 + steps * sin(heading));
    */
    return 0;
}

void laser_positive_x(uint16_t x, uint16_t y, uint16_t end_x, double delta_y)
{
    uint8_t  x_0       = ceil(x / 400);
    uint16_t y_0       = y + delta_y * ((double)x_0 - (double)x / 400);
    uint8_t  max_steps = round(end_x / 400) - x_0;
    laser_loop_x(max_steps, x_0, y_0, +1.0, delta_y);
}

void laser_negative_x(uint16_t x, uint16_t y, uint16_t end_x, double delta_y)
{
    uint8_t  x_0       = floor(x / 400);
    uint16_t y_0       = y + delta_y * (x_0 - x / 400);
    uint8_t  max_steps = x_0 - round(end_x / 400);
    laser_loop_x(max_steps, x_0, y_0, -1.0, delta_y);
}

void laser_positive_y(uint16_t x, uint16_t y, uint16_t end_y, double delta_x)
{
    uint8_t  y_0       = ceil(y / 400);
    uint16_t x_0       = x + delta_x * (y / 400 - y_0);
    uint8_t  max_steps = round(end_y / 400) - y_0;
    laser_loop_y(max_steps, x_0, y_0, delta_x, +1.0);
}

void laser_negative_y(uint16_t x, uint16_t y, uint16_t end_y, double delta_x)
{
    uint8_t  y_0       = floor(y / 400);
    uint16_t x_0       = x + delta_x * (y / 400 - y_0);
    uint8_t  max_steps = y_0 - round(end_y / 400);
    laser_loop_y(max_steps, x_0, y_0, delta_x, -1.0);
}

void laser_loop_x(uint8_t  max_steps,
                  uint8_t  x_0,
                  uint16_t y_0,
                  int8_t   delta_x,
                  double   delta_y)
{
    uint8_t x = x_0;
    for (uint8_t steps = 0; steps < max_steps; ++steps)
    {
        x += delta_x;
        mark_empty(x, floor((y_0 + steps * delta_y * 400) / 400));
    }
}

void laser_loop_y(uint8_t  max_steps,
                  uint16_t x_0,
                  uint8_t  y_0,
                  double   delta_x,
                  int8_t   delta_y)
{
    uint8_t x;
    uint8_t y = y_0;
    for (uint8_t steps = 0; steps < max_steps; ++steps)
    {
        y += delta_y;
        mark_empty(x = floor((x_0 + steps * delta_x * 400) / 400), y);
    }
}

void mark_empty(uint8_t x, uint8_t y)
{
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

    // turn one quarter to the left
    data.gyro = 16384; // FULL_TURN / 4

    Test_assertEquals(calculate_heading_and_position(&data), 0);
    Test_assertEquals(g_currentPosX, 0);
    Test_assertEquals(g_currentPosY, 0);
    Test_assertEquals(g_currentHeading, 16384);

    // then one eight to the right
    data.gyro = -8192; // FULL_TURN / 8

    Test_assertEquals(calculate_heading_and_position(&data), 0);
    Test_assertEquals(g_currentPosX, 0);
    Test_assertEquals(g_currentPosY, 0);
    Test_assertEquals(g_currentHeading, 8192);

    // now forward 100mm
    data.gyro           = 0;
    data.odometer_left  = 100;
    data.odometer_right = 100;
    Test_assertEquals(calculate_heading_and_position(&data), 0);
    Test_assertEquals(g_currentPosX, 70);
    Test_assertEquals(g_currentPosY, 70);
    Test_assertEquals(g_currentHeading, 8192);

    // half a turn to the right
    // and forward 100mm
    data.gyro = -32768;

    // robot will average out the heading, and is the same as a 90 degree turn
    // followed by movement, then another 90 degrees
    Test_assertEquals(calculate_heading_and_position(&data), 0);
    Test_assertEquals(g_currentPosX, 140);
    Test_assertEquals(g_currentPosY, 0);
    Test_assertEquals(g_currentHeading, 40960); // FULL_TURN * 5 / 8

    // then reverse 100mm
    // we're now standing at 5/8s of a turn
    data.gyro             = 0;
    g_wheelDirectionLeft  = DIR_BACKWARD;
    g_wheelDirectionRight = DIR_BACKWARD;

    Test_assertEquals(calculate_heading_and_position(&data), 0);
    Test_assertEquals(g_currentPosX, 210);
    Test_assertEquals(g_currentPosY, 70);
    Test_assertEquals(g_currentHeading, 40960);

    g_currentPosX         = oldCurrentPosX;
    g_currentPosY         = oldCurrentPosY;
    g_currentHeading      = oldHeading;
    g_wheelDirectionLeft  = oldWheelDirectionLeft;
    g_wheelDirectionRight = oldWheelDirectionRight;
}

Test_test(Test, mark_empty)
{
    g_navigationMap[0][0] = -2;
    mark_empty(0, 0);
    Test_assertEquals(g_navigationMap[0][0], -1);

    mark_empty(0, 0);
    Test_assertEquals(g_navigationMap[0][0], 0);

    mark_empty(0, 0);
    Test_assertEquals(g_navigationMap[0][0], 1);

    mark_empty(0, 0);
    Test_assertEquals(g_navigationMap[0][0], 2);

    g_navigationMap[0][0] = INT8_MAX;
    mark_empty(0, 0);
    Test_assertEquals(g_navigationMap[0][0], INT8_MAX);

    g_navigationMap[0][0] = 0;
}

bool check_map(void)
{
    bool result = true;
    stdout      = &mystdout;
    for (uint8_t x = 0; x < MAP_X_MAX; ++x)
    {
        for (uint8_t y = 0; y < MAP_Y_MAX; ++y)
        {
            if (g_navigationMap[x][y] != 0)
            {
                printf("(%u, %u) = %d\n", x, y, g_navigationMap[x][y]);
                result = false;
            }
        }
    }
    return result;
}

Test_test(Test, laser_loop_x)
{
    Test_assertTrue(check_map());
    // 2 cells straight to the right
    laser_loop_x(2, 0, 0, +1.0, 0);

    Test_assertEquals(g_navigationMap[1][0], 1);
    Test_assertEquals(g_navigationMap[2][0], 1);
    g_navigationMap[1][0] = 0;
    g_navigationMap[2][0] = 0;
    Test_assertTrue(check_map());
}
#endif

/***** Code I'll delete soon probs ******/

/*** UNUSED: We will primarily use gyro for calculating new heading****/
// ### Formula for new heading, given how much we rotated on the spot ###
// old_heading [radians], measured clockwise from starting position.
// direction==1 for turning to the right
// direction==-1 for turning to the left
// cog_steps [mm] from the ordometer
/*int rotated_on_the_spot(float old_heading, int direction, int cog_steps)
{

    // from the formula of circle sector
    // see image rotate_on_the_spot_heading_update.jpg
    float new_heading = (old_heading + direction *
        (cog_steps * WHEEL_CIRCUMFERENCE / COGS) / MID_TO_WHEEL_CENTER);
    return new_heading;
}*/

/*** This is a meme/WIP, and will never actually be used.***/
// Formula for new heading and position, given that one wheel is stationary
/// and the other has moved, so we're rotating around the stationary wheel
//
//
// old_heading [radians]
// direction==1 if driving forward, -1 if reverse
// turning_wheel==1 for left wheel, -1 for right wheel
// old_pos_x, old_pos_y positions of the midpoint of the robot
/*void one_wheel_rotation(float old_heading,
        int direction,
        int turning_wheel,
        int old_pos_x,
        int old_pos_y,
        float &new_heading,
        int &new_pos_x,
        int &new_pos_y)
{
    // from the formula of circle sector
    // same math as rotated_on_the_spot, except it can turn in two directions
    // see single_wheel_turn_heading_update.jpg
    new_heading = (old_heading + direction * turning_wheel *
            (cog_steps * WHEEL_CIRCUMFERENCE / COGS) / MID_TO_WHEEL_CENTER);

    // DARK MAGIC
    // a bunch of trigonometry and circle sector stuff
    // see single_wheel_turn_position_update_SKETCH.jpg for a proof of concept
    if (direction == 1 && turning_wheel == 1)
    {
        float angle_change = new_heading - old_heading;
        float small_angle = (0.5 * math.tau - angle_change) / 2 - old_heading;
        float base = 2 * math.sin(angle_change/2) / MID_TO_WHEEL_CENTER;

        // needs +/- depending on quadrant
        new_pos_x = old_pos_x + math.cos(small_angle) / base;
        new_pos_y = old_pos_y + math.sin(small_angle) / base;
    }
    else
    {
        arghlabarg;
    }


}*/

// the number of ordometer steps that represent a full turn
// the robot is 200x200, so the radius is ~100
// int MAGIC_ORDO = 2 * math.pi * 100 / WHEEL_CIRCUMFERENCE / COGS;
// //approximate theoretical value

// Helper function for determining where a laser crosses a potential wall
// Takes ordometer steps for left & right wheelpair, and returns a delta_x to be
// used when drawing the imaginary "lines" of a laser as it crosses the floor.
// currently unused
/*
float delta_x(int left_ordo, int right_ordo) {
    int res = 1;
    int stuff;
    if (left_ordo > right_ordo)
    {
        stuff = (left_ordo - right_ordo) % MAGIC_ORDO;
        if (stuff < MAGIC_ORDO/2)
        {
            res = -1;
        }
    }
    else
    {
        stuff = (right_ordo - left_ordo) % MAGIC_ORDO;
        if (stuff > MAGIC_ORDO/2)
        {
            res = -1;
        }
    }

    // v = 2*math.pi* stuff / MAGIC_ORDO
    // d_x = sin(v)
    return MAGIC_TABLE[stuff];

}*/
