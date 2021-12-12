// TSEA29, initial author JL
// rotation math
//
#include <math.h>
#include <stdlib.h>
#include "navigation_unit.h"
#include "../AVR_common/robot.h"
#include "../AVR_common/sensors.h"
#include "../AVR_common/uart.h"
#include "../AVR_testing/test.h"

// if we hit right at the corner of a wall, we don't know which
// coordinate it corresponds to. So we throw out all values within this many mm
// of the corners.
#define CORNER_SENSITIVITY 30

// map update throws out an update if a wall is too far from where it can be
#define MAX_ERROR 50
#define LIDAR_MIN 150
#define IR_MIN 80

// Minimum and maximum values for the sensors
#define IR_MIN 80
#define IR_MAX 350
#define LIDAR_MIN 100

// If the gyro is broken, we can exclusively use the odometers for heading
// Long-term you maybe want to use a hybrid of them both though
#define USE_ODO_FOR_HEADING 0

// The odometer spins when turning on the spot, not turning far enough.
// Lowering this causes us to update less from the odometer, turning more.
#define SPIN_RATIO 0.708

// Measurements used to calculate position
#define AXLE_WIDTH 170
#define MID_TO_WHEEL_CENTER 110

#define M_TAU (2 * M_PI)
#define min(x, y) x < y ? x : y
double g_cosHeading = 0;
double g_sinHeading = 1;

const double COS_QUARTERS[] = { 1.0, 0, -1.0, 0 };
const double SIN_QUARTERS[] = { 0, 1, 0, -1 };
struct laser_data
{
    uint16_t startX;
    uint16_t startY;
    int16_t endX;
    int16_t endY;

    // which side of a cell are we hitting
    // 0 left, 1 bottom, 2 right, 3 top
    // -1 if invalid data
    int8_t collision_type;

    // in what quadrant is the laser pointing, 0-indexed
    int8_t quadrant;

    // end - (round(end/GRID_SIZE)*GRID_SIZE)
    int16_t offset;
    double cos;
    double sin;
};
struct laser_data g_laser_data[6];

int8_t laser_positive_x(uint16_t x, uint16_t y, uint8_t end_x_coord, double delta_y);
int8_t laser_negative_x(uint16_t x, uint16_t y, uint8_t end_x_coord, double delta_y);
int8_t laser_positive_y(uint16_t x, uint16_t y, uint8_t end_y_coord, double delta_x);
int8_t laser_negative_y(uint16_t x, uint16_t y, uint8_t end_y_coord, double delta_x);
bool adjust_position(struct sensor_data* sd);
bool calculate_laser_data(
        struct sensor_data* sd,
        struct laser_data* ld,
        uint8_t sensor_id);

int8_t laser_loop(uint8_t  max_steps,
        uint8_t  a_0,
        uint16_t b_0,
        int8_t   delta_a,
        double   delta_b,
        int8_t (*f)(uint8_t, uint8_t));
int8_t mark_empty(uint8_t x, uint8_t y);
int8_t mark_empty_rev(uint8_t y, uint8_t x);

int8_t draw_laser_line(struct laser_data* ld);

void send_position(void);
void send_heading(void);
void send_map_update(uint8_t x, uint8_t y, int8_t value);
// using the trigonometric addition formulas
double laser_cos(uint8_t direction)
{
    return g_cosHeading * COS_QUARTERS[direction] -
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

int16_t calculate_dif(int16_t pos)
{
    if (pos < 0)
    {
        return pos;
    }
    int16_t res = pos % GRID_SIZE;
    if (res > GRID_SIZE/2)
    {
        return res-GRID_SIZE;
    }
    return res;
}

void update_trig_cache(void)
{
    double headingRad = heading_to_radian(g_currentHeading);
    g_cosHeading = cos(headingRad);
    g_sinHeading = sin(headingRad);
}

// helper function to calculate the coordinates of a sensor on the robot
void calculate_sensor_position(int8_t x, int8_t y,
        uint16_t* start_x, uint16_t* start_y)
{
    *start_x = g_currentPosX + round(
            g_cosHeading * x + laser_cos(1) * y);
    *start_y = g_currentPosY + round(
            g_sinHeading * x + laser_sin(1) * y);
}

// calculate heading change, using only the odometers
int16_t odo_heading_change(struct sensor_data* data)
{
    int16_t arc_length;
    int16_t res;
    if (g_wheelDirectionLeft != g_wheelDirectionRight)
    {
        // from the formula of circle sector
        // see image rotate_on_the_spot_heading_update.jpg
        arc_length = (data->odometer_right + data->odometer_left) / 2;
        res = radian_to_heading((double) arc_length * SPIN_RATIO / MID_TO_WHEEL_CENTER);
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
    static struct data_packet packet;

    // overflow handles modulo for us

#if USE_ODO_FOR_HEADING
    heading_change = odo_heading_change(data);
#else
    heading_change = data->gyro;
#endif
    // ideally use all three of odometer, gyro and wheelspeed

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
            distance = (data->odometer_left + data->odometer_right) / 2;

            // estimated w/ circle sector + pythagoras
            /*if (data->odometer_left < data->odometer_right)
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
              }*/
            //distance = 1.414214 * ((double) min(data->odometer_left, data->odometer_right)
            //       / heading_change + AXLE_WIDTH/2);
        }

        if (g_wheelDirectionLeft == DIR_BACKWARD)
        {
            distance = -distance;
        }

        g_currentPosX += round(cos(headingAvg) * distance);
        g_currentPosY += round(sin(headingAvg) * distance);
		
        send_position();


    }
    if (heading_change)
    {
		if (data->odometer_left == 0 && data->odometer_right == 0)
		{
			return 0;
		}
        g_currentHeading += heading_change;
        packet.address = ADR_HEADING;
        packet.byte_count = 2;
        packet.bytes[0] = Uint16ToByte0(g_currentHeading);
        packet.bytes[1] = Uint16ToByte1(g_currentHeading);
        ComUnitSend(&packet);

        // cache cos & sin
        update_trig_cache();
    }
	adjust_position(data);
    // TODO use lidar & IR to calibrate heading and position
    return 0;
}

void send_position(void)
{
    struct data_packet packet;
    packet.address = POSITION;
    packet.byte_count = 4;
    packet.bytes[0] = Uint16ToByte0(g_currentPosX);
    packet.bytes[1] = Uint16ToByte1(g_currentPosX);
    packet.bytes[2] = Uint16ToByte0(g_currentPosY);
    packet.bytes[3] = Uint16ToByte1(g_currentPosY);
    ComUnitSend(&packet);
}
void send_heading(void)
{
    struct data_packet packet;
    packet.address = ADR_HEADING;
    packet.byte_count = 2;
    packet.bytes[0] = Uint16ToByte0(g_currentHeading);
    packet.bytes[1] = Uint16ToByte1(g_currentHeading);
    ComUnitSend(&packet);
}

/*void adjust_heading_and_position(struct sensor_data* data)
{
    update_trig_cache();
    //struct laser_data ld;
    for (int8_t i = 0; i < 6; ++i)
    {
        //calculate_laser_data(data, &ld, i);
    }
    //adjust_heading(data);
    //adjust_position(data);
}*/

double calculate_wanted_heading(struct laser_data* ld, uint16_t distance)
{
    double wanted_heading_rad;
    if (ld->collision_type & 0x1)
    {
        //y collision
        double ratio = ((double)ld->endY - ld->offset - g_currentPosY) / distance;
        if (abs(ratio) > 1)
        {
            return NAN;
        }
        wanted_heading_rad = asin(ratio);
        if ((ld->quadrant == 1 || ld->quadrant == 2) && abs(ratio)< 1)
        {
            // mirror across y axis
            wanted_heading_rad = M_PI - wanted_heading_rad;
        }
    }
    else
    {
        double ratio = ((double)ld->endX - ld->offset - g_currentPosX) / distance;
        if (abs(ratio) > 1)
        {
            return NAN;
        }
        wanted_heading_rad = acos(ratio);
        if (ld->quadrant & 0x2 && abs(ratio) < 1)
        {
            // mirror across x axis
            wanted_heading_rad = -wanted_heading_rad;
        }
    }
    return wanted_heading_rad;
}
//#include <stdio.h>
#define MAX_HEADING_DIFF FULL_TURN/64
int8_t adjust_heading(struct sensor_data* sd)
{
    int16_t sum = 0;
    uint16_t wanted_heading;
    int16_t diff;
    int8_t count = 0;
    struct laser_data ld;
    uint16_t distance;
    double wanted_heading_rad;
    for (int8_t i = 0; i < 6; ++i)
    {
        if (!calculate_laser_data(sd, &ld, i))
        {
            printf("%d\n", i);
            continue;
        }
        distance = ((uint16_t*)sd)[i];
        if (LASER_DIRECTION[i] & 0x1)
        {
            distance += abs(LASER_POSITION_Y[i]);
        }
        else
        {
            distance += abs(LASER_POSITION_X[i]);
        }
        wanted_heading_rad = calculate_wanted_heading(&ld, distance);

        wanted_heading = radian_to_heading(wanted_heading_rad) - LASER_DIRECTION[i]*FULL_TURN/4;
        diff = (int16_t)(wanted_heading - g_currentHeading);
        //printf("%d %f %u %u\n", i, wanted_heading_rad, wanted_heading, diff);
        if (abs(diff) < MAX_HEADING_DIFF)
        {
            sum += diff;
            ++count;
        }

    }
    if (count >= 1)
    {
        g_currentHeading += round((double)sum / count);
        update_trig_cache();
        send_heading();
        return true;
    }
    return false;
}

#if __TEST__
struct foo {
    uint16_t arr[6];
};
Test_test(Test, calculate_wanted_heading_y)
{
    uint16_t oldPosY = g_currentPosY;
    struct laser_data ld;
    double res;

    g_currentPosY = 4320;
    ld.endY = 5598;
    ld.offset = -2;
    ld.collision_type = 1;
    ld.quadrant = 1;
    res = calculate_wanted_heading(&ld, 1280);
    Test_assertEquals(radian_to_heading(res), FULL_TURN/4);

    g_currentPosY = 4080;
    ld.endY = 2802;
    ld.offset = 2;
    ld.collision_type = 3;
    ld.quadrant = 3;
    res = calculate_wanted_heading(&ld, 1280);
    Test_assertEquals(radian_to_heading(res), FULL_TURN/4*3);

    g_currentPosY = 0;
    ld.endY = 1000;
    ld.offset = 0;
    ld.collision_type = 1;
    ld.quadrant = 0;
    res = calculate_wanted_heading(&ld, 2613);
    Test_assertEquals(radian_to_heading(res), FULL_TURN/16);

    res = calculate_wanted_heading(&ld, 1082);
    //12288 in theory w/ no rounding on the distance
    Test_assertEquals(radian_to_heading(res), 12297);

    ld.quadrant = 1;
    res = calculate_wanted_heading(&ld, 1082);
    // 20480 in theory (FT/16 + FT/4)
    Test_assertEquals(radian_to_heading(res), 20471);
    res = calculate_wanted_heading(&ld, 2613);
    Test_assertEquals(radian_to_heading(res), FULL_TURN/2 - FULL_TURN/16);

    ld.collision_type = 3;
    g_currentPosY = 1000;
    ld.endY = 0;

    ld.quadrant = 2;
    res = calculate_wanted_heading(&ld, 1082);
    // theory: 45056 = FT*3/4 - FT/16
    Test_assertEquals(radian_to_heading(res), 45065);
    res = calculate_wanted_heading(&ld, 2613);
    Test_assertEquals(radian_to_heading(res), FULL_TURN/2 + FULL_TURN/16);

    ld.quadrant = 3;
    res = calculate_wanted_heading(&ld, 1082);
    // theory: 53248 FULL_TURN*3/4 + FULL_TURN/16
    Test_assertEquals(radian_to_heading(res), 53239);
    res = calculate_wanted_heading(&ld, 2613);
    Test_assertEquals(radian_to_heading(res), FULL_TURN - FULL_TURN/16);

    g_currentPosY = oldPosY;
}
Test_test(Test, calculate_wanted_heading_x)
{
    uint16_t oldPosX = g_currentPosX;
    struct laser_data ld;
    double res;

    g_currentPosX = 0;
    ld.endX = 1000;
    ld.offset = 0;
    ld.collision_type = 0;

    ld.quadrant = 0;

    res = calculate_wanted_heading(&ld, 1082);
    // theory: FULL_TURN/16 = 4096
    Test_assertEquals(radian_to_heading(res), 4087);
    res = calculate_wanted_heading(&ld, 2613);
    Test_assertEquals(radian_to_heading(res), FULL_TURN/4 - FULL_TURN/16);

    ld.quadrant = 3;
    res = calculate_wanted_heading(&ld, 1082);
    //theory: FULL_TURN - FULL_TURN/16 = 61440
    Test_assertEquals(radian_to_heading(res), 61449);
    res = calculate_wanted_heading(&ld, 2613);
    Test_assertEquals(radian_to_heading(res), FULL_TURN/4*3 + FULL_TURN/16);

    g_currentPosX = 1000;
    ld.endX = 0;
    ld.collision_type = 2;
    ld.quadrant = 1;

    res = calculate_wanted_heading(&ld, 1082);
    // theory: FULL_TURN/2 - FULL_TURN/16 = 28672
    Test_assertEquals(radian_to_heading(res), 28681);
    res = calculate_wanted_heading(&ld, 2613);
    Test_assertEquals(radian_to_heading(res), FULL_TURN/4 + FULL_TURN/16);

    ld.quadrant = 2;

    res = calculate_wanted_heading(&ld, 1082);
    // theory: FULL_TURN/2 + FULL_TURN/16 = 36864
    Test_assertEquals(radian_to_heading(res), 36855);
    res = calculate_wanted_heading(&ld, 2613);
    Test_assertEquals(radian_to_heading(res), FULL_TURN/4*3 - FULL_TURN/16);

    g_currentPosX = oldPosX;
}
Test_test(Test, adjust_heading)
{
    uint16_t oldHeading = g_currentHeading;
    uint16_t oldPosX = g_currentPosX;
    uint16_t oldPosY = g_currentPosY;

    g_currentHeading = FULL_TURN/4+FULL_TURN/128;
    g_currentPosX = GridToMm(24);
    g_currentPosY = GridToMm(10);
    update_trig_cache();

    struct foo sd = { .arr = {1280, 1280, 530, 530, 530, 530}};
    Test_assertEquals(adjust_heading((struct sensor_data*) &sd), true);
    Test_assertEquals(g_currentHeading, FULL_TURN/4);

    g_currentHeading = FULL_TURN/16+FULL_TURN/128;
    g_currentPosX = GridToMm(24);
    g_currentPosY = GridToMm(10);
    update_trig_cache();

    sd = (struct foo){ .arr = {529, 529, 452, 452, 452, 452}};
    Test_assertEquals(adjust_heading((struct sensor_data*) &sd), true);
    Test_assertEquals(g_currentHeading, FULL_TURN/16);

    g_currentHeading = oldHeading;
    g_currentPosX = oldPosX;
    g_currentPosY = oldPosY;
}
#endif //__test__

// the largest sensor difference that's taken into consideration
#define MAX_ADJUST 50

// the minimum number of sensors within MAX_ADJUST in order to update position
#define MIN_ADJUST_SENSORS 2

// assumes heading is correct, and calibrates the robots position
// according to the distance to the walls
bool adjust_position(struct sensor_data* sd)
{
    int16_t sumX = 0;
    int16_t sumY = 0;
    int8_t countX = 0;
    int8_t countY = 0;
    struct laser_data ld;

    for (int8_t i = 0; i < 6; ++i)
    {
        if (!calculate_laser_data(sd, &ld, i))
        {
            continue;
        }
        switch (ld.collision_type)
        {
            case 0:
                // left
            case 2:
                // right side
                // adjusting the X position
                ++countX;
                sumX += ld.offset;
                break;
            case 1:
                // bottom
            case 3:
                // top
                ++countY;
                sumY += ld.offset;
                break;
            default:
                // invalid data
                continue;
        }
    }
    // positive offset means the laser_end is to the right
    // of where it "should" be, so we should move the robot
    // to the left, i.e. subtract
    if (countX >= MIN_ADJUST_SENSORS)
    {
        g_currentPosX -= round((double)sumX / countX);
    }
    if (countY >= MIN_ADJUST_SENSORS)
    {
        g_currentPosY -= round((double)sumY / countY);
    }
    if (countX >= MIN_ADJUST_SENSORS
            || countY >= MIN_ADJUST_SENSORS)
    {
        send_position();
        return true;
    }
    return false;
}

int8_t update_map(struct sensor_data* sd)
{
    struct laser_data ld;
    for (int i=0; i < 6; ++i)
    {
        calculate_laser_data(sd, &ld, i);
        draw_laser_line(&ld);
    }

    // We can optionally also check if the robot is currently standing on top of
    // a potential wall (or several), and mark those as empty.
    // This can be more or less ambitious, taking into account the width of the
    // robot and stuff.
    return 0;
}

bool calculate_laser_data(
        struct sensor_data* sd,
        struct laser_data* ld,
        uint8_t sensor_id)
{
    int8_t laser_x = LASER_POSITION_X[sensor_id];
    int8_t laser_y = LASER_POSITION_Y[sensor_id];
    int8_t laser_dir = LASER_DIRECTION[sensor_id];
    uint16_t distance = *((uint16_t*)sd + sensor_id);

    if (distance == 0 || distance == UINT16_MAX)
    {
        ld->collision_type = -1;
        return false;
    }
    // lidar
    if (sensor_id < 2 && (distance < LIDAR_MIN || distance > 10000))
    {
        ld->collision_type = -1;
        return false;
    }
    // IR
    else if ((sensor_id & 0x6) && (distance < IR_MIN || distance > 800))
    {
        ld->collision_type = -1;
        return false;
    }

    // extra: calculate if the laser *should* hit a wall we're sure about
    // that it's not hitting, and if so throw out that value

    ld->cos = laser_cos(laser_dir);
    ld->sin = laser_sin(laser_dir);

    calculate_sensor_position(laser_x, laser_y,
            &(ld->startX), &(ld->startY));

    ld->endX = ld->startX + round(ld->cos * distance);
    ld->endY = ld->startY + round(ld->sin * distance);

    int16_t x_dif = calculate_dif(ld->endX);
    int16_t y_dif = calculate_dif(ld->endY);

    // in which quadrant is the laser pointing
    ld->quadrant = ((g_currentHeading >> 14) + laser_dir) & 3;

    if (abs(x_dif) < abs(y_dif))
    {
        if (abs(y_dif) < CORNER_SENSITIVITY || abs(x_dif) > MAX_ERROR)
        {
            ld->collision_type = ld->quadrant = -1;
            return false;
        }
        ld->offset = x_dif;
        if (ld->quadrant == 1 || ld->quadrant == 2)
        {
            // right
            ld->collision_type = 2;
        }
        else
        {
            // left
            ld->collision_type = 0;
        }
    }
    else
    {
        if (abs(x_dif) < CORNER_SENSITIVITY || abs(y_dif) > MAX_ERROR)
        {
            ld->collision_type = ld->quadrant = -1;
            return false;
        }
        //Extra: if y_dif < CORNER, set -1
        ld->offset = y_dif;
        // if in the 2nd or 3rd quadrant, we're hitting the top
        if (ld->quadrant & 0x2)
        {
            ld->collision_type = 3;
        }
        else
        {
            // bottom
            ld->collision_type = 1;
        }
    }
    return true;
}

// Main function for updating the map, sets a wall where the laser ends
// and marks squares leading up to it (not including the square the sensor is
// in) as empty space.
int8_t draw_laser_line(struct laser_data* ld)
{
    // see laser_intersecting_pot_walls.jpg
    // where x_r,y_r is pos_x,pos_y
    // x_s, y_s is laser_x, laser_y
    // d is distance
    // x_n , y_n is end_x, end_y

    if (ld->offset > MAX_ERROR || ld->collision_type == -1)
    {
        return -1;
    }
    uint8_t end_x_coord;
    uint8_t end_y_coord;
    uint16_t other_dif;

    // Depending on direction we're hitting different walls of the cell, and
    // that gives different coordinates for the cell
    switch(ld->collision_type)
    {
        case 0:
            //hitting left side
            end_x_coord = (ld->endX - ld->offset) / GRID_SIZE;
            end_y_coord = ld->endY / GRID_SIZE;
            other_dif = ld->endY % GRID_SIZE;
            break;
        case 1:
            // bottom
            end_x_coord = ld->endX / GRID_SIZE;
            end_y_coord = (ld->endY - ld->offset) / GRID_SIZE;
            other_dif = ld->endX % GRID_SIZE;
            break;
        case 2:
            // right
            end_x_coord = (ld->endX - ld->offset) / GRID_SIZE - 1;
            end_y_coord = ld->endY / GRID_SIZE;
            other_dif = ld->endY % GRID_SIZE;
            break;
        case 3:
            // top
            end_x_coord = ld->endX / GRID_SIZE;
            end_y_coord = (ld->endY - ld->offset) / GRID_SIZE - 1;
            other_dif = ld->endX % GRID_SIZE;
            break;
        default:
            return -1;
    }

    if (end_x_coord < MAP_X_MAX && end_y_coord < MAP_Y_MAX &&
            (abs(other_dif) > CORNER_SENSITIVITY || abs(ld->offset) > CORNER_SENSITIVITY))
    {
		if (g_navigationMap[end_x_coord][end_y_coord] > INT8_MIN)
		{
        // mark as wall
        g_navigationMap[end_x_coord][end_y_coord] -= 1;

        // if the cell state changed, send update to com-unit
        if (g_navigationMap[end_x_coord][end_y_coord] == -1)
        {
            send_map_update(end_x_coord, end_y_coord, -1);
        }
        else if (g_navigationMap[end_x_coord][end_y_coord] == 0)
        {
            send_map_update(end_x_coord, end_y_coord, 0);
        }
        else
        {
            send_map_update(end_x_coord, end_y_coord,
                    g_navigationMap[end_x_coord][end_y_coord]);
        }
		}
    }

    /* Calculate which cells the laser passed trough before hitting the wall */
    // A line drawn across a square grid at an angle will intersect with points
    // at (X_0, Y_0), (X_0+1, Y_1), (X_0+2, Y_2) and (X_0, Y_0), (X_1, Y_0+1),
    // (X_2, Y_0+2) (where these will yield the same set of points for 45
    // degrees / tau/8 radians)

    double tan = 0.0;
    double cot = 0.0;
    if (ld->cos != 0.0 && ld->sin != 0.0)
    {
        tan = fabs(ld->sin / ld->cos);
        cot = fabs(ld->cos / ld->sin);
    }

    switch (ld->quadrant)
    {
        case 0:
            if (laser_positive_x(ld->startX, ld->startY, end_x_coord, tan) ||
                    laser_positive_y(ld->startX, ld->startY, end_y_coord, cot))
            {
                return -1;
            }
            break;
        case 1:
            if (laser_negative_x(ld->startX, ld->startY, end_x_coord, tan) ||
                    laser_positive_y(ld->startX, ld->startY, end_y_coord, -cot))
            {
                return -1;
            }
            break;
        case 2:
            if (laser_negative_x(ld->startX, ld->startY, end_x_coord, -tan) ||
                    laser_negative_y(ld->startX, ld->startY, end_y_coord, -cot))
            {
                return -1;
            }
            break;
        case 3:
            if (laser_positive_x(ld->startX, ld->startY, end_x_coord, -tan) ||
                    laser_negative_y(ld->startX, ld->startY, end_y_coord, cot))
            {
                return -1;
            }
            break;
        default:
            return -1;
    }
    return 0;
}

// helper functions for draw_laser_line that calls laser_loop,
// used to mark empty squares.
int8_t laser_positive_x(uint16_t x, uint16_t y, uint8_t end_x_coord, double delta_y)
{
    uint8_t  x_0       = x % GRID_SIZE ? x / GRID_SIZE + 1 : x / GRID_SIZE; // ceil
    uint16_t y_0       = y + delta_y * (x_0 - (double) x / GRID_SIZE);
    uint8_t  max_steps = end_x_coord - x_0;
    if (max_steps > MAP_X_MAX)
    {
        return 0;
    }
    return laser_loop(max_steps, x_0, y_0, +1.0, delta_y, mark_empty);
}

int8_t laser_negative_x(uint16_t x, uint16_t y, uint8_t end_x_coord, double delta_y)
{
    uint8_t  x_0       = x / GRID_SIZE - 1; // integer division == floor
    uint16_t y_0       = y + delta_y * ((double) x / GRID_SIZE - x_0 + 1);
    uint8_t  max_steps = x_0 - end_x_coord;
    if (max_steps > MAP_X_MAX)
    {
        return 0;
    }
    return laser_loop(max_steps, x_0, y_0, -1.0, delta_y, mark_empty);
}

int8_t laser_positive_y(uint16_t x, uint16_t y, uint8_t end_y_coord, double delta_x)
{
    uint8_t  y_0       = y % GRID_SIZE ? y / GRID_SIZE + 1 : y / GRID_SIZE; // ceil
    uint16_t x_0       = x + delta_x * (y_0 - (double) y / GRID_SIZE);
    uint8_t  max_steps = end_y_coord - y_0;
    if (max_steps > MAP_Y_MAX)
    {
        return 0;
    }
    return laser_loop(max_steps, y_0, x_0, +1.0, delta_x, mark_empty_rev);
}

int8_t laser_negative_y(uint16_t x, uint16_t y, uint8_t end_y_coord, double delta_x)
{
    uint8_t  y_0       = y / GRID_SIZE - 1; // integer division == floor
    uint16_t x_0       = x + delta_x * ((double) y / GRID_SIZE - y_0 + 1);
    uint8_t  max_steps = y_0 - end_y_coord;
    if (max_steps > MAP_Y_MAX)
    {
        return 0;
    }
    return laser_loop(max_steps, y_0, x_0, -1.0, delta_x, mark_empty_rev);
}

// innermost loop for map updates, called for both x-aligned and y-aligned
// updates. Calls the function supplied by the caller, which is mark_empty or
// mark_empty_rev
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
        raw_b = (b_0 + steps * delta_b * GRID_SIZE);
        if (abs(raw_b - round(raw_b / GRID_SIZE)*GRID_SIZE) > CORNER_SENSITIVITY)
        {
            res += (*f)(a, floor(raw_b / GRID_SIZE));
        }
        a += delta_a;
    }
    return res;
}

// reverses parameters before calling mark_empty, to make laser_loop fully
// generalizable
int8_t mark_empty_rev(uint8_t y, uint8_t x)
{
    return mark_empty(x, y);
}

// marks a square as empty and calls send_map_update
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
        send_map_update(x, y, g_navigationMap[x][y]);
    }
    else
    {
        return -1;
    }
    return 0;
}

// marks a square as a wall and calls send_map_update
int8_t mark_walll(uint8_t x, uint8_t y)
{

}
// Sends a map update to the display unit
void send_map_update(uint8_t x, uint8_t y, int8_t value)
{
    struct data_packet packet;
    packet.address = MAP_UPDATE;
    packet.byte_count = 3;
    packet.bytes[0] = x;
    packet.bytes[1] = y;
    packet.bytes[2] = value;

    ComUnitSend(&packet);
}

#if __TEST__

Test_test(Test, calc_sensor_pos)
{
    uint16_t x, y;
    uint16_t oldPosX = g_currentPosX;
    uint16_t oldPosY = g_currentPosY;
    uint16_t oldHeading = g_currentHeading;

    g_currentPosX = 100;
    g_currentPosY = 90;
    g_currentHeading = 0;
    update_trig_cache();

    calculate_sensor_position(50, 45, &x, &y);
    Test_assertEquals(x, 150);
    Test_assertEquals(y, 135);

    calculate_sensor_position(-50, 45, &x, &y);
    Test_assertEquals(x, 50);
    Test_assertEquals(y, 135);

    calculate_sensor_position(50, -45, &x, &y);
    Test_assertEquals(x, 150);
    Test_assertEquals(y, 45);

    calculate_sensor_position(-50, -45, &x, &y);
    Test_assertEquals(x, 50);
    Test_assertEquals(y, 45);

    g_currentHeading = 16384;
    update_trig_cache();
    Test_assertEquals(laser_cos(1), -1);

    calculate_sensor_position(50, 45, &x, &y);
    Test_assertEquals(x, 55);
    Test_assertEquals(y, 140);

    calculate_sensor_position(-50, 45, &x, &y);
    Test_assertEquals(x, 55);
    Test_assertEquals(y, 40);

    calculate_sensor_position(50, -45, &x, &y);
    Test_assertEquals(x, 145);
    Test_assertEquals(y, 140);

    calculate_sensor_position(-50, -45, &x, &y);
    Test_assertEquals(x, 145);
    Test_assertEquals(y, 40);

    g_currentHeading = 32768;
    update_trig_cache();
    calculate_sensor_position(50, 45, &x, &y);
    Test_assertEquals(x, 50);
    Test_assertEquals(y, 45);

    g_currentHeading = 49152;
    update_trig_cache();
    calculate_sensor_position(50, 45, &x, &y);
    Test_assertEquals(x, 145);
    Test_assertEquals(y, 40);

    g_currentPosX = oldPosX;
    g_currentPosY = oldPosY;
    g_currentHeading = oldHeading;
}

Test_test(Test, heading_to_radian)
{
    Test_assertFloatEquals(heading_to_radian(0), 0.0);
    Test_assertFloatEquals(heading_to_radian(32768), M_PI);
    Test_assertFloatEquals(heading_to_radian(16384), M_PI / 2);
    Test_assertFloatEquals(heading_to_radian(-16384), 6 * M_PI / 4);
}

Test_test(Test, calc_heading_and_pos)
{
    stdout = &mystdout;
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
    data.odometer_left = round((double)MID_TO_WHEEL_CENTER * M_TAU / 8 /SPIN_RATIO); //157
    data.odometer_right = data.odometer_left;
    g_wheelDirectionLeft = DIR_BACKWARD;
    Test_assertEquals(calculate_heading_and_position(&data), 0);
    Test_assertEquals(g_currentPosX, 0);
    Test_assertEquals(g_currentPosY, 0);
    //Test_assertEquals(g_currentHeading, 8137); // 0.3 degree error
    //rounding error due to odometer data being crude and being rounded
    //after accounting for spin ratio
    Test_assertEquals(g_currentHeading, 8190);
    g_currentHeading = 8192;
#endif

    Test_assertEquals(calculate_heading_and_position(&data), 0);
    Test_assertEquals(g_currentPosX, 0);
    Test_assertEquals(g_currentPosY, 0);
#if !USE_ODO_FOR_HEADING
    Test_assertEquals(g_currentHeading, 16384);
#else
    //Test_assertEquals(g_currentHeading, 16329);
    Test_assertEquals(g_currentHeading, 16382);
#endif
    g_currentHeading = 16384;

#if !USE_ODO_FOR_HEADING
    // then one eight to the right
    data.gyro = -8192; // FULL_TURN / 8
#else
    // split into two eigth turns
    data.odometer_left = round((double)MID_TO_WHEEL_CENTER * M_TAU / 8 / SPIN_RATIO); //157
    data.odometer_right = data.odometer_left;
    g_wheelDirectionLeft = DIR_FORWARD;
    g_wheelDirectionRight = DIR_BACKWARD;
#endif

    Test_assertEquals(calculate_heading_and_position(&data), 0);
    Test_assertEquals(g_currentPosX, 0);
    Test_assertEquals(g_currentPosY, 0);
#if !USE_ODO_FOR_HEADING
    Test_assertEquals(g_currentHeading, 8192);
#else
    Test_assertEquals(g_currentHeading, 8194);
    g_currentHeading = 8192;
#endif

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
    data.gyro = -12578;
#endif
    data.odometer_left  = 255;
    data.odometer_right = 50;

    // robot will average out the heading, and is the same as a 90 degree turn
    // followed by movement, then another 90 degrees
    Test_assertEquals(calculate_heading_and_position(&data), 0);
    //Test_assertEquals(g_currentPosX, 272); //273.6 in theory

    Test_assertEquals(g_currentPosX, 220); //273.6 in theory

    //Test_assertEquals(g_currentPosY, 127); //127.7 in theory
    Test_assertEquals(g_currentPosY, 99); //127.7 in theory

    Test_assertEquals(g_currentHeading, 61150);

    // then reverse 100mm
    // we're now standing at 5/8s of a turn
    data.gyro             = 0;
    g_wheelDirectionLeft  = DIR_BACKWARD;
    g_wheelDirectionRight = DIR_BACKWARD;
    data.odometer_left = 100;
    data.odometer_right = 100;

    g_currentPosX = 272;
    g_currentPosY = 127;
    g_currentHeading = 63037;

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
    Test_assertEquals(laser_loop(10, 1, GRID_SIZE, 1, 1.0, mark_empty), 0);
}

Test_test(Test, laser_loop_6)
{
    // 1/8 turn laser, starting in (200,0) and hitting the first x-wall
    // at (GRID_SIZE, 200).
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
    // at (GRID_SIZE, 200).
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
    Test_assertEquals(laser_loop(4, 4, 8*GRID_SIZE+200, -1, -2.41421356, mark_empty), 0);

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
    Test_assertEquals(laser_loop(4, 8, (MAP_Y_MAX-1)*GRID_SIZE+200, 1,
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

    update_trig_cache();
    Test_assertEquals(g_cosHeading, 1);
    Test_assertEquals(g_sinHeading, 0);

    // a laser positioned 120 ahead of the robot
    // pointing to the right
    // detects a wall at 1400mm distance
    struct sensor_data sd;
    sd.lidar_forward = 1380;
    struct laser_data ld;
    calculate_laser_data(&sd, &ld, 0);
    Test_assertEquals(ld.startX, 620);
    Test_assertEquals(ld.startY, 200);
    Test_assertEquals(ld.endX, 2000);
    Test_assertEquals(ld.endY, 200);
    Test_assertEquals(ld.collision_type, 0);
    Test_assertEquals(ld.quadrant, 0);
    Test_assertEquals(ld.offset, 0);
    Test_assertEquals(ld.cos, 1);
    Test_assertEquals(ld.sin, 0);

    Test_assertEquals(draw_laser_line(&ld), 0);

    Test_assertEquals(g_navigationMap[2][0], 1);
    Test_assertEquals(g_navigationMap[3][0], 1);
    Test_assertEquals(g_navigationMap[4][0], 1);
    Test_assertEquals(g_navigationMap[5][0], -1);

    g_navigationMap[2][0] = 0;
    g_navigationMap[3][0] = 0;
    g_navigationMap[4][0] = 0;
    g_navigationMap[5][0] = 0;

    // a laser positioned 120 behind the robot
    g_currentPosX = 1120;
    sd.lidar_backward = 1000;
    calculate_laser_data(&sd, &ld, 1);
    Test_assertEquals(ld.collision_type, 2);
    Test_assertEquals(ld.quadrant, 2);
    Test_assertEquals(draw_laser_line(&ld), 0);

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

    update_trig_cache();

    g_currentPosX = GridToMm(24) - 20;
    g_currentPosY = GridToMm(12)+100;

    Test_assertEquals(g_currentPosX, 9780);
    Test_assertEquals(g_currentPosY, 5100);

    // posX = 9800
    // posY = 5100
    // distance = 5012
    // laser_x = 100
    // heading = 48392
    // cosHeading = -0.0728
    // sinHeading = -0.9973
    // start_x = 9793
    // start_y = 5000
    struct sensor_data sd;
    sd.lidar_forward = 5012;
    struct laser_data ld;
    calculate_laser_data(&sd, &ld, 0);

    Test_assertEquals(ld.collision_type, 3);
    Test_assertEquals(ld.quadrant, 2);

    Test_assertEquals(draw_laser_line(&ld), 0);
    //Test_assertEquals(draw_laser_line(120, 0, 0, 5012), 0);

    Test_assertEquals(g_navigationMap[24][11], 1);
    Test_assertEquals(g_navigationMap[24][10], 1);
    Test_assertEquals(g_navigationMap[24][9], 1);
    Test_assertEquals(g_navigationMap[24][8], 1);
    Test_assertEquals(g_navigationMap[24][7], 1);

    Test_assertEquals(g_navigationMap[23][4], 1);
    Test_assertEquals(g_navigationMap[23][3], 1);
    Test_assertEquals(g_navigationMap[23][2], 1);
    Test_assertEquals(g_navigationMap[23][1], 1);
    Test_assertEquals(g_navigationMap[23][0], 1);

    g_navigationMap[24][11] = 0;
    g_navigationMap[24][10] = 0;
    g_navigationMap[24][9] = 0;
    g_navigationMap[24][8] = 0;
    g_navigationMap[24][7] = 0;
    g_navigationMap[23][4] = 0;
    g_navigationMap[23][3] = 0;
    g_navigationMap[23][2] = 0;
    g_navigationMap[23][1] = 0;
    g_navigationMap[23][0] = 0;

    sd.lidar_forward = 4592;
    calculate_laser_data(&sd, &ld, 0);
    Test_assertEquals(ld.collision_type, 3);
    Test_assertEquals(ld.quadrant, 2);
    Test_assertEquals(draw_laser_line(&ld), 0);
    //Test_assertEquals(draw_laser_line(100, 0, 0, 4612), 0);

    Test_assertEquals(g_navigationMap[24][11], 1);
    Test_assertEquals(g_navigationMap[24][10], 1);
    Test_assertEquals(g_navigationMap[24][9], 1);
    Test_assertEquals(g_navigationMap[24][8], 1);
    Test_assertEquals(g_navigationMap[24][7], 1);

    Test_assertEquals(g_navigationMap[23][4], 1);
    Test_assertEquals(g_navigationMap[23][3], 1);
    Test_assertEquals(g_navigationMap[23][2], 1);
    Test_assertEquals(g_navigationMap[23][1], 1);
    Test_assertEquals(g_navigationMap[23][0], -1);

    g_navigationMap[24][11] = 0;
    g_navigationMap[24][10] = 0;
    g_navigationMap[24][9] = 0;
    g_navigationMap[24][8] = 0;
    g_navigationMap[24][7] = 0;
    g_navigationMap[23][4] = 0;
    g_navigationMap[23][3] = 0;
    g_navigationMap[23][2] = 0;
    g_navigationMap[23][1] = 0;
    g_navigationMap[23][0] = 0;

    g_currentPosX = oldPosX;
    g_currentPosY = oldPosY;
    g_currentHeading = oldHeading;
}

Test_test(Test, draw_laser_line_3)
{
    uint16_t oldPosX = g_currentPosX;
    uint16_t oldPosY = g_currentPosY;
    uint16_t oldHeading = g_currentHeading;

    g_currentPosX = GridToMm(24);
    g_currentPosY = GridToMm(0);
    g_currentHeading = FULL_TURN/4;

    update_trig_cache();

    // a laser positioned 120 ahead of the robot
    // detects a wall at 880 distance
    struct sensor_data sd;
    sd.lidar_forward = 880;
    struct laser_data ld;
    calculate_laser_data(&sd, &ld, 0);
    Test_assertEquals(ld.collision_type, 1);
    Test_assertEquals(ld.quadrant, 1);
    Test_assertEquals(draw_laser_line(&ld), 0);
    //Test_assertEquals(draw_laser_line(120, 0, 0, 880), 0);

    Test_assertEquals(g_navigationMap[24][1], 1);
    Test_assertEquals(g_navigationMap[24][2], 1);
    Test_assertEquals(g_navigationMap[24][3], -1);

    g_navigationMap[24][1] = 0;
    g_navigationMap[24][2] = 0;
    g_navigationMap[24][3] = 0;

    // a laser positioned 85 behind the robot and 70 to the side
    // i.e. ir_leftback detects a wall at 130
    sd.ir_leftback = 130;
    calculate_laser_data(&sd, &ld, 3);
    Test_assertEquals(ld.collision_type, 2);
    Test_assertEquals(ld.quadrant, 2);
    Test_assertEquals(draw_laser_line(&ld), 0);
    //Test_assertEquals(draw_laser_line(-85, 70, 1, 130), 0);
    // currentX = 9800
    // currentY = 200
    // currentHeading = 16384
    // cos = 0
    // sinHeading = 1
    // cosHeading+1 = -1
    // sinHeading-1 = 0
    // start_x = 9800 + 0*-85 + 1*70 = 9870
    // start_y = 200 + 1*-85 + 0*70 = 115
    // end_x = 9870

    Test_assertEquals(g_navigationMap[23][0], -1);

    g_navigationMap[23][0] = 0;

    // ir_rightfront detects a wall at 130
    sd.ir_rightfront = 130;
    calculate_laser_data(&sd, &ld, 4);
    // currentX = 9800
    // currentY = 200
    // currentHeading = 16384
    Test_assertEquals(g_cosHeading, 0);
    Test_assertEquals(g_sinHeading, 1);
    // cosHeading+1 = -1
    Test_assertEquals(laser_cos(1), -1);
    // sinHeading-1 = 0
    Test_assertEquals(laser_sin(1), 0);
    // start_x = 9800 + 0*80 + -1*-70 = 9870
    Test_assertEquals(ld.startX, 9870);
    // start_y = 200 + 1*80 + 0*70 = 115
    Test_assertEquals(ld.startY, 280);
    // end_x = 10000
    Test_assertEquals(ld.endX, 10000);
    Test_assertEquals(ld.endY, 280);
    Test_assertEquals(ld.collision_type, 0);
    Test_assertEquals(ld.quadrant, 0);

    Test_assertEquals(draw_laser_line(&ld), 0);

    Test_assertEquals(g_navigationMap[25][0], -1);

    g_navigationMap[25][0] = 0;



    g_currentPosX = oldPosX;
    g_currentPosY = oldPosY;
    g_currentHeading = oldHeading;
}


Test_test(Test, update_map)
{
    uint16_t oldPosX = g_currentPosX;
    uint16_t oldPosY = g_currentPosY;
    uint16_t oldHeading = g_currentHeading;

    g_currentPosX = GridToMm(24);
    g_currentPosY = GridToMm(10) + 200;
    g_currentHeading = FULL_TURN/4;

    struct sensor_data sd;
    sd.lidar_forward = 680;
    sd.lidar_backward = 680;
    sd.ir_leftfront = 130;
    sd.ir_leftback = 530;
    sd.ir_rightfront = 530;
    sd.ir_rightback = 130;

    Test_assertEquals(update_map(&sd), 0);
    Test_assertEquals(g_navigationMap[24][12], 1);
    Test_assertEquals(g_navigationMap[24][13], -1);

    Test_assertEquals(g_navigationMap[24][9], 1);
    Test_assertEquals(g_navigationMap[24][8], -1);

    Test_assertEquals(g_navigationMap[23][11], -1);

    Test_assertEquals(g_navigationMap[23][10], 1);
    Test_assertEquals(g_navigationMap[22][10], -1);

    Test_assertEquals(g_navigationMap[25][11], 1);
    Test_assertEquals(g_navigationMap[26][11], -1);

    Test_assertEquals(g_navigationMap[25][10], -1);

    g_navigationMap[24][12] = 0;
    g_navigationMap[24][13] = 0;

    g_navigationMap[24][9] = 0;
    g_navigationMap[24][8] = 0;

    g_navigationMap[23][11] = 0;

    g_navigationMap[23][10] = 0;
    g_navigationMap[22][10] = 0;

    g_navigationMap[25][11] = 0;
    g_navigationMap[26][11] = 0;

    g_navigationMap[25][10] = 0;

    g_currentPosX = oldPosX;
    g_currentPosY = oldPosY;
    g_currentHeading = oldHeading;
}

Test_test(Test, calculate_dif)
{
    Test_assertEquals(calculate_dif(3590), -10);
    Test_assertEquals(calculate_dif(4415), 15);
    Test_assertEquals(calculate_dif(-30), -30);
    Test_assertEquals(calculate_dif(-9999), -9999);
}
Test_test(Test, adjust_position)
{
    uint16_t oldPosX = g_currentPosX;
    uint16_t oldPosY = g_currentPosY;
    uint16_t oldHeading = g_currentHeading;

    g_currentPosX = GridToMm(24);
    g_currentPosY = GridToMm(10);
    g_currentHeading = FULL_TURN/4;
    update_trig_cache();
    struct foo sd = { .arr = {480, 480, 130, 130, 130, 130}};

    Test_assertEquals(adjust_position((struct sensor_data*) &sd), true);
    Test_assertEquals(g_currentPosX, GridToMm(24));
    Test_assertEquals(g_currentPosY, GridToMm(10));

    sd = (struct foo){ .arr = {470, 490, 140, 140, 120, 120}};
    Test_assertEquals(adjust_position((struct sensor_data*) &sd), true);
    Test_assertEquals(g_currentPosX, GridToMm(24)+10);
    Test_assertEquals(g_currentPosY, GridToMm(10));

    g_currentPosX = GridToMm(24);
    g_currentPosY = GridToMm(10);
    sd = (struct foo){ .arr = {470, 490, 130, 130, 120, 120}};
    Test_assertEquals(adjust_position((struct sensor_data*) &sd), true);
    Test_assertEquals(g_currentPosX, GridToMm(24)+5);
    Test_assertEquals(g_currentPosY, GridToMm(10));

    g_currentHeading = FULL_TURN/2;
    update_trig_cache();

    g_currentPosX = GridToMm(24)+8;
    g_currentPosY = GridToMm(10);
    sd = (struct foo){ .arr = {470, 490, 130, 130, 120, 120}};
    Test_assertEquals(adjust_position((struct sensor_data*) &sd), true);
    Test_assertEquals(g_currentPosX, GridToMm(24)+8);
    Test_assertEquals(g_currentPosY, GridToMm(10)+5);

    g_currentPosX = GridToMm(24)+8;
    g_currentPosY = GridToMm(10);
    sd = (struct foo){ .arr = {470, 490, 0, 0, 120, 120}};
    Test_assertEquals(adjust_position((struct sensor_data*) &sd), false);
    Test_assertEquals(g_currentPosX, GridToMm(24)+8);
    Test_assertEquals(g_currentPosY, GridToMm(10));

    g_currentPosX = GridToMm(24);
    g_currentPosY = GridToMm(10);
    sd = (struct foo){ .arr = {470, 490, 330, 30, 120, 120}};
    Test_assertEquals(adjust_position((struct sensor_data*) &sd), false);
    Test_assertEquals(g_currentPosX, GridToMm(24));
    Test_assertEquals(g_currentPosY, GridToMm(10));

    g_currentHeading = FULL_TURN/8;
    update_trig_cache();

    g_currentPosX = GridToMm(24)+10; //9810
    g_currentPosY = GridToMm(10);
    sd = (struct foo){ .arr = {728, 728, 293, 127, 133, 298}};
    Test_assertEquals(adjust_position((struct sensor_data*) &sd), true);

    Test_assertEquals(g_currentPosX, GridToMm(24)); //9800
    Test_assertEquals(g_currentPosY, GridToMm(10)); //4200


    g_currentPosX = oldPosX;
    g_currentPosY = oldPosY;
    g_currentHeading = oldHeading;
}
// test corner sensitivity
// test max error
#endif // __TEST__
