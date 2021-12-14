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

// Minimum and maximum values for the sensors
#define IR_MIN 80
#define IR_MAX 350
#define LIDAR_MIN 100
#define LIDAR_MAX 10000

// If the gyro is broken, we can exclusively use the odometers for heading
// Long-term you maybe want to use a hybrid of them both though
#define USE_ODO_FOR_HEADING 0

// The odometer spins when turning on the spot, not turning far enough.
// Lowering this causes us to update less from the odometer, turning more.
#define SPIN_RATIO 0.708

// Measurements used to calculate position
#define AXLE_WIDTH 170
#define MID_TO_WHEEL_CENTER 110


#define MAX_HEADING_DIFF FULL_TURN/32

// how much of the perceived angle change should be updated
// in one tick
#define ADJUST_RATIO 4

#define MIN_ADJUST_SENSORS_HEADING 3

#define M_TAU (2 * M_PI)
#define min(x, y) x < y ? x : y
double g_cosHeading = 0;
double g_sinHeading = 1;

const double COS_QUARTERS[] = { 1.0, 0, -1.0, 0 };
const double SIN_QUARTERS[] = { 0, 1, 0, -1 };
struct laser_data
{
    double startX;
    double startY;
    double endX;
    double endY;

    // which side of a cell are we hitting
    // 0 left, 1 bottom, 2 right, 3 top
    // -1 if invalid data
    int8_t collision_type;

    // in what quadrant is the laser pointing, 0-indexed
    int8_t quadrant;

    // end - (round(end/GRID_SIZE)*GRID_SIZE)
    double offset;
    double cos;
    double sin;

    // is the sensor operating within it's reliable limits
    // 0 = yes, 1 = too short, 2 = too far
    int8_t reliable;

    // are we too close to the corner, so the coordinates are unreliable?
    bool corner_error;
    // are we too far from where a wall should possibly be?
    bool offset_error;
};
struct laser_data g_laser_data[6];

int8_t laser_positive_x(uint16_t x, uint16_t y, uint16_t end_x, double delta_y);
int8_t laser_negative_x(uint16_t x, uint16_t y, uint16_t end_x, double delta_y);
int8_t laser_positive_y(uint16_t x, uint16_t y, uint16_t end_y, double delta_x);
int8_t laser_negative_y(uint16_t x, uint16_t y, uint16_t end_y, double delta_x);
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
bool mark_wall(uint8_t x, uint8_t y);

int8_t draw_laser_line(struct laser_data* ld);
int8_t adjust_heading(struct sensor_data* sd);

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

double calculate_dif(double pos)
{
    if (pos < 0)
    {
        return pos;
    }
    double res = fmod(pos, GRID_SIZE);
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
        double* start_x, double* start_y)
{
    *start_x = g_currentPosX + g_cosHeading * x + laser_cos(1) * y;
    *start_y = g_currentPosY + g_sinHeading * x + laser_sin(1) * y;
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
    return 0;
}

int8_t update_heading_and_position(struct sensor_data* data)
{
    calculate_heading_and_position(data);
    adjust_position(data);
    adjust_heading(data);
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

double calculate_wanted_heading(struct laser_data* ld, double distance)
{
    double wanted_heading_rad;
    if (ld->collision_type & 0x1)
    {
        //y collision
        double ratio = ((double)ld->endY - ld->offset - g_currentPosY) / distance;
        //printf("ratio: %f, endY: %f, offset: %f, pos: %u, distance: %f\n",
                //ratio, ld->endY, ld->offset, g_currentPosY, distance);
        if (ratio > 1)
        {
            //return NAN;
            //printf("+yNAN\n");
            wanted_heading_rad = M_PI/2;
        }
        else if (ratio < -1)
        {
            //printf("+yNAN\n");
            wanted_heading_rad = -M_PI/2;
        }
        else
        {
            wanted_heading_rad = asin(ratio);
        }
        if ((ld->quadrant == 1 || ld->quadrant == 2) && abs(ratio)< 1)
        {
            // mirror across y axis
            wanted_heading_rad = M_PI - wanted_heading_rad;
            //printf("mirror: %f ", wanted_heading_rad);
        }
    }
    else
    {
        double ratio = ((double)ld->endX - ld->offset - g_currentPosX) / distance;
        if (ratio > 1)
        {
            //printf("+xNAN\n");
            wanted_heading_rad = 0;
        }
        else if (ratio < -1)
        {
            //printf("-xNAN\n");
            wanted_heading_rad = M_PI;
        }
        else
        {
            wanted_heading_rad = acos(ratio);
        }
        if (ld->quadrant & 0x2 && abs(ratio) < 1)
        {
            // mirror across x axis
            wanted_heading_rad = -wanted_heading_rad;
        }
    }
    return wanted_heading_rad;
}

double cwh2(struct laser_data* ld)
{
    double wanted_heading_rad;
    if (ld->collision_type & 0x1)
    {
        double a = (double) ld->endX - g_currentPosX;
        double b = (double) ld->endY - g_currentPosY;
        double c = (double) ld->endY - ld->offset - g_currentPosY;
        //printf("%f %f %f %f %f\n", ld->endX, ld->endY, a, b, c);
        wanted_heading_rad = 2* atan2(sqrt(a*a+b*b-c*c)-a, b-c);
        return wanted_heading_rad;
    }
    return NAN;

}
#define SQUARE(x) ((x)*(x))
//#include <stdio.h>

int8_t adjust_heading_2(struct sensor_data* sd)
{
    struct laser_data ld;

    int8_t count = 0;
    double sum = 0.0;
    //double wanted_heading;
    double diff1;
    double distance;
    double s_distance;
    double wanted_heading_rad;
    double heading_shift;
    double curr_heading = heading_to_radian(g_currentHeading);
    for (int8_t i = 0; i < 6; ++i)
    {
        double diff = 9999;
        calculate_laser_data(sd, &ld, i);
        if (ld.reliable == 1 || ld.reliable == 2 || ld.corner_error)
        {
            continue;
        }
        //printf("%d ", i);
        s_distance = ((uint16_t*)sd)[i];
        if (LASER_DIRECTION[i] & 0x1)
        {
            //printf("dirY ");
            distance = sqrt((uint16_t)SQUARE(LASER_POSITION_X[i])+
                    SQUARE(abs(LASER_POSITION_Y[i]) + s_distance));
            //printf("dist: %f = (%d)**2+(%d + %f)**2\n",
                    //distance, LASER_POSITION_X[i],
                    //LASER_POSITION_Y[i], s_distance);
            heading_shift = acos((abs(LASER_POSITION_Y[i])+s_distance) / distance);
        }
        else
        {
            //printf("dirX ");
            //distance += abs(LASER_POSITION_X[i]);
            distance = sqrt((uint16_t)SQUARE(LASER_POSITION_Y[i])+
                    SQUARE(abs(LASER_POSITION_X[i]) + s_distance));
            heading_shift = acos((abs(LASER_POSITION_X[i])+s_distance) / distance);
        }
        /*printf("endX: %f ", ld.endX);
        printf("endY: %f ", ld.endY);
        printf("%d %d %f\t",
                ld.collision_type,
                ld.quadrant,
                ld.offset);
        printf("dist: %f\n", distance);*/
        //double aoeu = cwh2(&ld);
        //printf("cwh2: %f %u\n", aoeu, radian_to_heading(aoeu));
        wanted_heading_rad = calculate_wanted_heading(&ld, distance);
        double laser_dir = LASER_DIRECTION[i]*M_TAU/4;
        /*printf("whr: %.2f hs: %.2f whr+hs+dir: %.2f whr-hs+dir: %.2f whr+hs-dir: %.2f whr-hs-dir: %.2f\n",
                wanted_heading_rad,
                heading_shift,
                wanted_heading_rad + heading_shift + laser_dir,
                wanted_heading_rad - heading_shift + laser_dir,
                wanted_heading_rad + heading_shift - laser_dir,
                wanted_heading_rad - heading_shift - laser_dir
                );*/

        /*if (ld.quadrant == -1)
        {
            wanted_heading = wanted_heading_rad + LASER_DIRECTION[i]*M_TAU/4;
        }
        else
        {
            wanted_heading = wanted_heading_rad - LASER_DIRECTION[i]*M_TAU/4;
        }*/
        /*if ((LASER_POSITION_X[i] > 0) ^ (LASER_DIRECTION[i] > 2))
        {
            wanted_heading += heading_shift;
            //printf("whr+hs: %f, hs: %f  ", wanted_heading, heading_shift);
        }
        else
        {
            wanted_heading -= heading_shift;
            //printf("whr-hs: %f, hs: %f  ", wanted_heading, heading_shift);
        }*/
        // there's some logic to whethere you should add or subtract heading
        // shift, but I'm failing to figure it out atm
        if (heading_shift > 0)
        {
            for (int i=-1; i < 2; i+=2)
            {
                for (int j=-1; j < 2; j+=2)
                {
                    diff1 = fmod(wanted_heading_rad - curr_heading
                            + i * heading_shift
                            + j * laser_dir, M_TAU);
                    if (diff1 > M_TAU/2)
                    {
                        diff1 -= M_TAU;
                    }
                    else if (diff1 < -M_TAU/2)
                    {
                        diff1 += M_TAU;
                    }
                    //printf(" %f ", diff1);
                    if (fabs(diff1) < fabs(diff))
                    {
                        diff = diff1;
                    }
                }
            }
        }
        else
        {
            diff = fmod(wanted_heading_rad - curr_heading - laser_dir, M_TAU);
            if (diff > M_TAU/2)
            {
                diff -= M_TAU;
            }
            else if (diff < -M_TAU/2)
            {
                diff += M_TAU;
            }
        }

        //printf("\n%d %f %f\n", i, wanted_heading_rad, diff);
        if (abs(diff) < MAX_HEADING_DIFF)
        {
            sum += diff;
            ++count;
        }
    }
    //printf("\nsum: %f, count: %d, %u\n", sum, count, radian_to_heading(sum/count));
    if (count >= MIN_ADJUST_SENSORS_HEADING)
    {
        g_currentHeading += radian_to_heading(sum / count / ADJUST_RATIO);
        update_trig_cache();
        send_heading();
        return true;
    }
    return false;
}

// Angle of the slope of cos & sin in the four quadrants
// i.e. is the function increasing (1) or decreasing (0)
// which can be used to see if our heading is too small or too large
// when checking the offset of each laser
int8_t COS_DELTA[] = {+1, +1, -1, -1};
int8_t SIN_DELTA[] = {-1, +1, +1, -1};

int8_t adjust_dir(struct laser_data* ld)
{
    if (ld->offset == 0)
    {
        return 0;
    }
    if (ld->collision_type & 1)
    {
        // y hit
        if (ld->offset < 0)
        {
            return SIN_DELTA[ld->quadrant];
        }
        else
        {
            return -SIN_DELTA[ld->quadrant];
        }
    }
    else
    {
        if (ld->offset < 0)
        {
            return COS_DELTA[ld->quadrant];
        }
        else
        {
            return -COS_DELTA[ld->quadrant];
        }
    }
}

#if __TEST__
Test_test(Test, adjust_dir_x)
{
    struct laser_data ld;

    ld.collision_type = 0;

    ld.quadrant = 0;
    ld.offset = +1;
    Test_assertEquals(adjust_dir(&ld), 1);
    ld.offset = -1;
    Test_assertEquals(adjust_dir(&ld), -1);

    ld.quadrant = 3;
    ld.offset = +1;
    Test_assertEquals(adjust_dir(&ld), -1);
    ld.offset = -1;
    Test_assertEquals(adjust_dir(&ld), 1);

    ld.collision_type = 2;

    ld.quadrant = 1;
    ld.offset = +1;
    Test_assertEquals(adjust_dir(&ld), 1);
    ld.offset = -1;
    Test_assertEquals(adjust_dir(&ld), -1);

    ld.quadrant = 2;
    ld.offset = +1;
    Test_assertEquals(adjust_dir(&ld), -1);
    ld.offset = -1;
    Test_assertEquals(adjust_dir(&ld), 1);
}
Test_test(Test, adjust_dir_y)
{
    struct laser_data ld;

    ld.collision_type = 1;

    ld.quadrant = 0;
    ld.offset = +1;
    Test_assertEquals(adjust_dir(&ld), -1);
    ld.offset = -1;
    Test_assertEquals(adjust_dir(&ld), 1);

    ld.quadrant = 1;
    ld.offset = +1;
    Test_assertEquals(adjust_dir(&ld), 1);
    ld.offset = -1;
    Test_assertEquals(adjust_dir(&ld), -1);

    ld.collision_type = 3;

    ld.quadrant = 2;
    ld.offset = +1;
    Test_assertEquals(adjust_dir(&ld), 1);
    ld.offset = -1;
    Test_assertEquals(adjust_dir(&ld), -1);

    ld.quadrant = 3;

    ld.offset = +1;
    Test_assertEquals(adjust_dir(&ld), -1);
    ld.offset = -1;
    Test_assertEquals(adjust_dir(&ld), 1);
}
#endif

int8_t adjust_heading(struct sensor_data* sd)
{
    struct laser_data ld;

    //int8_t count = 0;
    int8_t sum = 0;
    //double distance;
    //double s_distance;
    //double wanted_heading_rad;
    //double heading_shift;
    //double curr_heading = heading_to_radian(g_currentHeading);
    for (int8_t i = 0; i < 6; ++i)
    {
        //double diff = 9999;
        calculate_laser_data(sd, &ld, i);
        // if the laser fails to detect a wall, or is too close to a corner
        // it's unusable
        if (ld.reliable == 2 || ld.corner_error)
        {
            continue;
        }
        // if the laser is too close to a wall, we *can* use it if it
        // *shouldn't* be too close to a wall.
        if (ld.reliable == 1)
        {
            // TODO
            continue;
        }
        sum += adjust_dir(&ld);
    }
    return 0;
}
#if __TEST__
struct foo {
    uint16_t arr[6];
};
Test_test(Test, calculate_wanted_heading_y)
{
    uint16_t SAVE(g_currentPosY);
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

    RESTORE(g_currentPosY);
}
/*Test_test(Test, calculate_wanted_heading_y_2)
{
    uint16_t SAVE(g_currentPosY);
    struct laser_data ld;
    double res;

    g_currentPosY = GridToMm(10);
    ld.endY = 4400;
    ld.offset = 0;
    ld.collision_type = 1;
    ld.quadrant = 1;
    printf("calc\n");
    res = calculate_wanted_heading(&ld, 114 + 70);
    printf("res: %f\n", res);
    Test_assertEquals(radian_to_heading(res), FULL_TURN/16);

    RESTORE(g_currentPosY);
}*/
Test_test(Test, calculate_wanted_heading_x)
{
    uint16_t SAVE(g_currentPosX);
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

    RESTORE(g_currentPosX);
}
Test_test(Test, adjust_heading_8)
{
    uint16_t SAVE(g_currentHeading);
    uint16_t SAVE(g_currentPosX);
    uint16_t SAVE(g_currentPosY);

    g_currentHeading = FULL_TURN/8*7+FULL_TURN/128;
    g_currentPosX = GridToMm(24);
    g_currentPosY = GridToMm(10);
    update_trig_cache();

    struct foo sd = { .arr = {563, 563, 133, 128, 133, 128}};
    Test_assertEquals(adjust_heading((struct sensor_data*) &sd), true);
    Test_assertEquals(g_currentHeading, 58343); //theory: 57344
    RESTORE(g_currentHeading);
    RESTORE(g_currentPosX);
    RESTORE(g_currentPosY);
}
Test_test(Test, adjust_heading)
{
    uint16_t SAVE(g_currentHeading);
    uint16_t SAVE(g_currentPosX);
    uint16_t SAVE(g_currentPosY);

    g_currentHeading = FULL_TURN/4+FULL_TURN/128;
    g_currentPosX = GridToMm(24);
    g_currentPosY = GridToMm(10);
    update_trig_cache();

    struct foo sd = { .arr = {1280, 1280, 130, 130, 130, 130}};
    Test_assertEquals(adjust_heading((struct sensor_data*) &sd), true);
    Test_assertEquals(g_currentHeading, FULL_TURN/4);

    g_currentHeading = FULL_TURN/4-FULL_TURN/128;
    update_trig_cache();
    Test_assertEquals(adjust_heading((struct sensor_data*) &sd), true);
    Test_assertEquals(g_currentHeading, FULL_TURN/4);

    g_currentHeading = FULL_TURN/4;
    update_trig_cache();
    Test_assertEquals(adjust_heading((struct sensor_data*) &sd), true);
    Test_assertEquals(g_currentHeading, FULL_TURN/4);

    g_currentHeading = FULL_TURN/16+FULL_TURN/128;
    update_trig_cache();

    sd = (struct foo){ .arr = {96, 96, 113, 182, 180, 111}};
    Test_assertEquals(adjust_heading((struct sensor_data*) &sd), true);
    Test_assertEquals(g_currentHeading, 4239); //theory 4096

    g_currentHeading = FULL_TURN/16;
    update_trig_cache();

    sd = (struct foo){ .arr = {96, 96, 113, 182, 180, 111}};
    Test_assertEquals(adjust_heading((struct sensor_data*) &sd), true);
    Test_assertEquals(g_currentHeading, 4215); //theory 4096

    g_currentHeading = FULL_TURN/16*5 + FULL_TURN/64;
    update_trig_cache();

    sd = (struct foo){ .arr = {96, 96, 113, 182, 180, 111}};
    Test_assertEquals(adjust_heading((struct sensor_data*) &sd), true);
    Test_assertEquals(g_currentHeading, 20623); //theory 20480

    RESTORE(g_currentHeading);
    RESTORE(g_currentPosX);
    RESTORE(g_currentPosY);
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

    ld->corner_error = false;
    ld->offset_error = false;

    // extra: calculate if the laser *should* hit a wall we're sure about
    // that it's not hitting, and if so throw out that value

    ld->cos = laser_cos(laser_dir);
    ld->sin = laser_sin(laser_dir);

    calculate_sensor_position(laser_x, laser_y,
            &(ld->startX), &(ld->startY));

    // TODO: fancier stuff given more data
    if (sensor_id < 2 && distance < LIDAR_MIN)
    {
        ld->reliable = 1;
        distance = LIDAR_MIN;
    }
    else if (sensor_id < 2 && distance > LIDAR_MAX)
    {
        ld->reliable = 2;
        distance = LIDAR_MAX;
    }
    else if (sensor_id & 0x6 && distance < IR_MIN)
    {
        ld->reliable = 1;
        distance = IR_MIN;
    }
    else if (sensor_id & 0x6 && distance > IR_MAX)
    {
        ld->reliable = 2;
        distance = IR_MAX;
    }
    else
    {
        ld->reliable = 0;
    }

    ld->endX = ld->startX + ld->cos * distance;
    ld->endY = ld->startY + ld->sin * distance;

    double x_dif = calculate_dif(ld->endX);
    double y_dif = calculate_dif(ld->endY);

    // in which quadrant is the laser pointing
    ld->quadrant = ((g_currentHeading >> 14) + laser_dir) & 3;

    if (abs(x_dif) < abs(y_dif))
    {
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
        if (abs(y_dif) < CORNER_SENSITIVITY)
        {
            ld->corner_error = true;
        }
        if (abs(x_dif) > MAX_ERROR)
        {
            ld->offset_error = true;
        }
    }
    else
    {
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
        if (abs(x_dif) < CORNER_SENSITIVITY)
        {
            ld->corner_error = true;
        }
        if (abs(y_dif) > MAX_ERROR)
        {
            ld->offset_error = true;
        }
    }
    if (ld->reliable == 2)
    {
        ld->offset = 0;
    }
    return !ld->offset_error && !ld->corner_error && ld->reliable == 0;
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

    uint8_t end_x_coord;
    uint8_t end_y_coord;

    // Depending on direction we're hitting different walls of the cell, and
    // that gives different coordinates for the cell
    switch(ld->collision_type)
    {
        case 0:
            //hitting left side
            end_x_coord = (ld->endX - ld->offset) / GRID_SIZE;
            end_y_coord = ld->endY / GRID_SIZE;
            break;
        case 1:
            // bottom
            end_x_coord = ld->endX / GRID_SIZE;
            end_y_coord = (ld->endY - ld->offset) / GRID_SIZE;
            break;
        case 2:
            // right
            end_x_coord = (ld->endX - ld->offset) / GRID_SIZE - 1;
            end_y_coord = ld->endY / GRID_SIZE;
            break;
        case 3:
            // top
            end_x_coord = ld->endX / GRID_SIZE;
            end_y_coord = (ld->endY - ld->offset) / GRID_SIZE - 1;
            break;
        default:
            return -1;
    }

    if (end_x_coord < MAP_X_MAX && end_y_coord < MAP_Y_MAX)
    {
        if (ld->reliable == 0 && !ld->corner_error && !ld->offset_error)
        {
            // if it's a perfectly good hit, mark as wall
            mark_wall(end_x_coord, end_y_coord);
        }
        else if (ld->reliable == 1 && !ld->corner_error)
        {
            // if we're operating too close, mark as wall as long as it's not
            // a corner error
            mark_wall(end_x_coord, end_y_coord);
        }
    }

    // if we're too close, there's no empty spaces
    if (ld->reliable == 1)
    {
        return -1;
    }
    // otherwise we can mark spaces up to the sensors MAX

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

    double endx = ld->endX;
    double endy = ld->endY;
    if (ld->collision_type & 0x1)
    {
        endy -= ld->offset;
    }
    else
    {
        endx -= ld->offset;
    }

    // mark the square the sensor starts in as empty
    if (abs(calculate_dif(ld->startX)) > CORNER_SENSITIVITY &&
            abs(calculate_dif(ld->startY)) > CORNER_SENSITIVITY)
    {
        mark_empty(MmToGrid(ld->startX), MmToGrid(ld->startY));
    }

    switch (ld->quadrant)
    {
        case 0:
            return ( laser_positive_x(ld->startX, ld->startY, endx, tan)
                    || laser_positive_y(ld->startX, ld->startY, endy, cot));
        case 1:
            return (laser_negative_x(ld->startX, ld->startY, endx, tan)
                    || laser_positive_y(ld->startX, ld->startY, endy, -cot));
        case 2:
            return (laser_negative_x(ld->startX, ld->startY, endx, -tan)
                    || laser_negative_y(ld->startX, ld->startY, endy, -cot));
        case 3:
            return (laser_positive_x(ld->startX, ld->startY, endx, -tan)
                    || laser_negative_y(ld->startX, ld->startY, endy, cot));
        default:
            return -1;
    }
}

#define GRID_DIV_CORNER_CEIL(d) (d / GRID_SIZE) + ((d % GRID_SIZE > CORNER_SENSITIVITY) ? 1 : 0)
#define GRID_DIV_CORNER_FLOOR(d) (d / GRID_SIZE) + ((d % GRID_SIZE < CORNER_SENSITIVITY) ? 1 : 0)

#define GRID_DIV_CEIL(d) (d / GRID_SIZE + ((d % GRID_SIZE) ? 1 : 0))
#define GRID_DIV_FLOOR(d) (d / GRID_SIZE)
#define GRID_DIV_ROUND(d) (d / GRID_SIZE + \
        ((d % GRID_SIZE >= GRID_SIZE/2) ? 1 : 0))
// helper functions for draw_laser_line that calls laser_loop,
// used to mark empty squares.
int8_t laser_positive_x(uint16_t x, uint16_t y, uint16_t end_x, double delta_y)
{
    uint8_t  x_0       = GRID_DIV_CEIL(x);
    uint16_t y_0       = round(y + delta_y * (x_0*GRID_SIZE - x));
    uint8_t  max_steps = GRID_DIV_CEIL(end_x) - x_0;
    if (max_steps > MAP_X_MAX)
    {
        return 0;
    }
    return laser_loop(max_steps, x_0, y_0, +1.0, delta_y, mark_empty);
}

int8_t laser_negative_x(uint16_t x, uint16_t y, uint16_t end_x, double delta_y)
{
    uint8_t  x_0       = GRID_DIV_FLOOR(x);
    uint16_t y_0       = round(y + delta_y * (x - x_0*GRID_SIZE));
    uint8_t  max_steps = x_0 - GRID_DIV_FLOOR(end_x);
    if (max_steps > MAP_X_MAX)
    {
        return 0;
    }
    // subtract 1 from x_0 due to hitting the right wall of cells
    return laser_loop(max_steps, x_0-1, y_0, -1.0, delta_y, mark_empty);
}

int8_t laser_positive_y(uint16_t x, uint16_t y, uint16_t end_y, double delta_x)
{
    uint8_t  y_0       = GRID_DIV_CEIL(y);
    uint16_t x_0       = round(x + delta_x * (y_0*GRID_SIZE - y));
    uint8_t  max_steps = GRID_DIV_CEIL(end_y) - y_0;
    if (max_steps > MAP_Y_MAX)
    {
        return 0;
    }
    return laser_loop(max_steps, y_0, x_0, +1.0, delta_x, mark_empty_rev);
}

int8_t laser_negative_y(uint16_t x, uint16_t y, uint16_t end_y, double delta_x)
{
    uint8_t  y_0       = GRID_DIV_FLOOR(y);
    uint16_t x_0       = round(x + delta_x * (y - y_0*GRID_SIZE));
    uint8_t  max_steps = y_0 - GRID_DIV_FLOOR(end_y);
    if (max_steps > MAP_Y_MAX)
    {
        return 0;
    }
    // subtract 1 from y_0 due to hitting the upper wall of cell
    return laser_loop(max_steps, y_0-1, x_0, -1.0, delta_x, mark_empty_rev);
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
bool mark_wall(uint8_t x, uint8_t y)
{
    if (g_navigationMap[x][y] == INT8_MIN)
    {
        send_map_update(x, y, INT8_MIN);
        return false;
    }
    // mark as wall
    g_navigationMap[x][y] -= 1;

    // if the cell state changed, send update to com-unit
    if (g_navigationMap[x][y] == -1)
    {
        send_map_update(x, y, -1);
    }
    else if (g_navigationMap[x][y] == 0)
    {
        send_map_update(x, y, 0);
    }
    else
    {
        // always send update atm
        send_map_update(x, y, g_navigationMap[x][y]);
    }
    return true;
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

#define TEST_RESET_MAP(x, y, v) \
    Test_assertEquals(g_navigationMap[x][y], v) \
    g_navigationMap[x][y] = 0;

Test_test(Test, calc_sensor_pos)
{
    double x, y;
    uint16_t SAVE(g_currentPosX);
    uint16_t SAVE(g_currentPosY);
    uint16_t SAVE(g_currentHeading);

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
    Test_assertEquals(round(y), 40);

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
    Test_assertEquals(round(y), 45);

    g_currentHeading = 49152;
    update_trig_cache();
    calculate_sensor_position(50, 45, &x, &y);
    Test_assertEquals(x, 145);
    Test_assertEquals(y, 40);

    RESTORE(g_currentPosX);
    RESTORE(g_currentPosY);
    RESTORE(g_currentHeading);
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
    uint16_t SAVE(g_currentPosX);
    uint16_t SAVE(g_currentPosY);
    uint16_t SAVE(g_currentHeading);
    enum Direction SAVE(g_wheelDirectionLeft);
    enum Direction SAVE(g_wheelDirectionRight);

    struct sensor_data data;
    data.odometer_left  = 1;
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

    RESTORE(g_currentPosX);
    RESTORE(g_currentPosY);
    RESTORE(g_currentHeading);
    RESTORE(g_wheelDirectionLeft);
    RESTORE(g_wheelDirectionRight);
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
    TEST_RESET_MAP(0, 0, INT8_MAX);
}

Test_test(Test, mark_wall)
{
    g_navigationMap[0][0] = 2;
    Test_assertEquals(mark_wall(0, 0), true);
    Test_assertEquals(g_navigationMap[0][0], 1);

    Test_assertEquals(mark_wall(0, 0), true);
    Test_assertEquals(g_navigationMap[0][0], 0);

    Test_assertEquals(mark_wall(0, 0), true);
    Test_assertEquals(g_navigationMap[0][0], -1);

    Test_assertEquals(mark_wall(0, 0), true);
    Test_assertEquals(g_navigationMap[0][0], -2);

    g_navigationMap[0][0] = INT8_MIN;
    Test_assertEquals(mark_wall(0, 0), false);
    TEST_RESET_MAP(0, 0, INT8_MIN);
}

Test_test(Test, laser_loop_1)
{
    // 2 cells straight to the right
    Test_assertEquals(laser_loop(2, 1, 200, +1.0, 0, mark_empty), 0);

    TEST_RESET_MAP(1, 0, 1);
    TEST_RESET_MAP(2, 0, 1);
        }

Test_test(Test, laser_loop_2)
{
    // 2 cells straight to the left
    Test_assertEquals(laser_loop(2, 2, 200, -1.0, 0, mark_empty), 0);

    TEST_RESET_MAP(2, 0, 1);
    TEST_RESET_MAP(1, 0, 1);
        }

Test_test(Test, laser_loop_3)
{
    // 3 cells straight to the right into the wall
    Test_assertEquals(laser_loop(3, MAP_X_MAX-3, 200,
                +1.0, 0, mark_empty), 0);

    TEST_RESET_MAP(MAP_X_MAX-1, 0, 1);
    TEST_RESET_MAP(MAP_X_MAX-2, 0, 1);
    TEST_RESET_MAP(MAP_X_MAX-3, 0, 1);
            }

Test_test(Test, laser_loop_4)
{
    // 4 cells straight to the left into the wall
    Test_assertEquals(laser_loop(4, 3, 200, -1.0, 0, mark_empty), 0);

    TEST_RESET_MAP(3, 0, 1);
    TEST_RESET_MAP(2, 0, 1);
    TEST_RESET_MAP(1, 0, 1);
    TEST_RESET_MAP(0, 0, 1);

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

    TEST_RESET_MAP(1, 0, 1);
    TEST_RESET_MAP(2, 1, 1);
    TEST_RESET_MAP(3, 2, 1);
    TEST_RESET_MAP(4, 3, 1);
    TEST_RESET_MAP(5, 4, 1);

                    }

Test_test(Test, laser_loop_7)
{
    // 5/16 laser, starting in (,0) and hitting the first x-wall
    // at (GRID_SIZE, 200).
    Test_assertEquals(laser_loop(4, 4, 200, -1, 2.4142135, mark_empty), 0);

    TEST_RESET_MAP(4, 0, 1);
    TEST_RESET_MAP(3, 2, 1);
    TEST_RESET_MAP(2, 5, 1);
    TEST_RESET_MAP(1, 7, 1);

                }

Test_test(Test, laser_loop_8)
{
    // 11/16 laser, hitting the first wall at (1600,3400)
    Test_assertEquals(laser_loop(4, 4, 8*GRID_SIZE+200, -1, -2.41421356, mark_empty), 0);

    TEST_RESET_MAP(4, 8, 1);
    TEST_RESET_MAP(3, 6, 1);
    TEST_RESET_MAP(2, 3, 1);
    TEST_RESET_MAP(1, 1, 1);

                }

Test_test(Test, laser_loop_9)
{
    // 25/32 laser, hitting the first wall at (1600,3400)
    Test_assertEquals(laser_loop(4, 8, (MAP_Y_MAX-1)*GRID_SIZE+200, 1,
                -5.02733949, mark_empty), 0);

    TEST_RESET_MAP(8, 24, 1);
    TEST_RESET_MAP(9, 19, 1);
    TEST_RESET_MAP(10, 14, 1);
    TEST_RESET_MAP(11, 9, 1);

                }

Test_test(Test, laser_loop_10)
{
    Test_assertEquals(laser_loop(4, 3, 200, -1.0, 0, mark_empty_rev), 0);

    TEST_RESET_MAP(0, 3, 1);
    TEST_RESET_MAP(0, 2, 1);
    TEST_RESET_MAP(0, 1, 1);
    TEST_RESET_MAP(0, 0, 1);

                }

Test_test(Test, laser_positive_x_1)
{
    // 2 cells straight to the right
    // there's a wall at (3, 0)
    Test_assertEquals(laser_positive_x(200, 200, 3*GRID_SIZE, 0), 0);

    TEST_RESET_MAP(1, 0, 1);
    TEST_RESET_MAP(2, 0, 1);
        }

Test_test(Test, laser_positive_x_2)
{
    // 2 cells straight to the right
    // there's a wall at (3, 0)
    for (uint16_t start_x = 199; start_x < 202; ++start_x)
    {
        for (uint16_t start_y = 199; start_y < 202; ++start_y)
        {
            Test_assertEquals(laser_positive_x(start_x, start_y,
                        3*GRID_SIZE, 0), 0);

            TEST_RESET_MAP(1, 0, 1);
            TEST_RESET_MAP(2, 0, 1);
                                }
    }
}

Test_test(Test, laser_negative_x_1)
{
    // 2 cells straight to the left from (1400, 200)
    Test_assertEquals(laser_negative_x(1400, 200, 400, 0), 0);

    TEST_RESET_MAP(2, 0, 1);
    TEST_RESET_MAP(1, 0, 1);
        }

Test_test(Test, laser_negative_x_2)
{
    // 2 cells straight to the left from (1400, 200)
    for (uint16_t start_x = 1399; start_x < 1402; ++start_x)
    {
        for (uint16_t start_y = 199; start_y < 202; ++start_y)
        {
            Test_assertEquals(laser_negative_x(start_x, start_y, 400, 0), 0);

            TEST_RESET_MAP(2, 0, 1);
            TEST_RESET_MAP(1, 0, 1);
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
            Test_assertEquals(laser_positive_y(start_x, start_y, 25*GRID_SIZE, 0), 0);

            TEST_RESET_MAP(0, MAP_Y_MAX-1, 1);
            TEST_RESET_MAP(0, MAP_Y_MAX-2, 1);
            TEST_RESET_MAP(0, MAP_Y_MAX-3, 1);

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
            Test_assertEquals(laser_negative_y(start_x, start_y, 0, 0), 0);

            TEST_RESET_MAP(0, 3, 1);
            TEST_RESET_MAP(0, 2, 1);
            TEST_RESET_MAP(0, 1, 1);
            TEST_RESET_MAP(0, 0, 1);

                                                        }
    }
}
Test_test(Test, draw_laser_line)
{
    uint16_t SAVE(g_currentPosX);
    uint16_t SAVE(g_currentPosY);
    uint16_t SAVE(g_currentHeading);

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
    Test_assertEquals(ld.reliable, 0);
    Test_assertEquals(ld.corner_error, 0);
    Test_assertEquals(ld.offset_error, 0);

    Test_assertEquals(draw_laser_line(&ld), 0);

    TEST_RESET_MAP(1, 0, 1);

    TEST_RESET_MAP(2, 0, 1);
    TEST_RESET_MAP(3, 0, 1);
    TEST_RESET_MAP(4, 0, 1);
    TEST_RESET_MAP(5, 0, -1);

    RESTORE(g_currentPosX);
    RESTORE(g_currentPosY);
    RESTORE(g_currentHeading);
}
Test_test(Test, draw_laser_line__)
{
    uint16_t SAVE(g_currentPosX);
    uint16_t SAVE(g_currentPosY);
    uint16_t SAVE(g_currentHeading);

    g_currentPosX = 500;
    g_currentPosY = 200;
    g_currentHeading = 0;

    update_trig_cache();
    // a laser positioned 120 behind the robot
    g_currentPosX = 1120;

    struct sensor_data sd;
    struct laser_data ld;
    sd.lidar_backward = 1000;

    calculate_laser_data(&sd, &ld, 1);
    Test_assertEquals(ld.collision_type, 2);
    Test_assertEquals(ld.quadrant, 2);
    Test_assertEquals(draw_laser_line(&ld), 0);

    TEST_RESET_MAP(0, 0, 1);
    TEST_RESET_MAP(1, 0, 1);
    TEST_RESET_MAP(2, 0, 1);

    RESTORE(g_currentPosX);
    RESTORE(g_currentPosY);
    RESTORE(g_currentHeading);
}

Test_test(Test, draw_laser_line_2)
{
    uint16_t SAVE(g_currentPosX);
    uint16_t SAVE(g_currentPosY);
    uint16_t SAVE(g_currentHeading);

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
    Test_assertEquals(ld.startX, 9771);
    Test_assertEquals(ld.startY, 4980);
    Test_assertEquals(ld.endX, 9406);
    Test_assertEquals(round(ld.endY), 65518);
    Test_assertEquals(ld.reliable, 0);

    Test_assertEquals(draw_laser_line(&ld), 0);
    //Test_assertEquals(draw_laser_line(120, 0, 0, 5012), 0);
    TEST_RESET_MAP(24, 12, 1);
    TEST_RESET_MAP(24, 11, 1);
    TEST_RESET_MAP(24, 10, 1);
    TEST_RESET_MAP(24, 9, 1);
    TEST_RESET_MAP(24, 8, 1);
    TEST_RESET_MAP(24, 7, 1);

    TEST_RESET_MAP(23, 6, 1);

    TEST_RESET_MAP(23, 4, 1);
    TEST_RESET_MAP(23, 3, 1);
    TEST_RESET_MAP(23, 2, 1);
    TEST_RESET_MAP(23, 1, 1);
    TEST_RESET_MAP(23, 0, 1);

    RESTORE(g_currentPosX);
    RESTORE(g_currentPosY);
    RESTORE(g_currentHeading);
}

Test_test(Test, draw_laser_line_2_2)
{
    uint16_t SAVE(g_currentPosX);
    uint16_t SAVE(g_currentPosY);
    uint16_t SAVE(g_currentHeading);

    g_currentHeading = 48392;

    update_trig_cache();

    g_currentPosX = GridToMm(24) - 20;
    g_currentPosY = GridToMm(12)+100;

    Test_assertEquals(g_currentPosX, 9780);
    Test_assertEquals(g_currentPosY, 5100);
    struct sensor_data sd;
    struct laser_data ld;
    sd.lidar_forward = 4592;
    calculate_laser_data(&sd, &ld, 0);
    Test_assertEquals(ld.collision_type, 3);
    Test_assertEquals(ld.quadrant, 2);
    Test_assertEquals(draw_laser_line(&ld), 0);
    //Test_assertEquals(draw_laser_line(100, 0, 0, 4612), 0);

    TEST_RESET_MAP(24, 12, 1);
    TEST_RESET_MAP(24, 11, 1);
    TEST_RESET_MAP(24, 10, 1);
    TEST_RESET_MAP(24, 9, 1);
    TEST_RESET_MAP(24, 8, 1);
    TEST_RESET_MAP(24, 7, 1);

    TEST_RESET_MAP(23, 6, 1);

    TEST_RESET_MAP(23, 4, 1);
    TEST_RESET_MAP(23, 3, 1);
    TEST_RESET_MAP(23, 2, 1);
    TEST_RESET_MAP(23, 1, 1);
    TEST_RESET_MAP(23, 0, -1);

    RESTORE(g_currentPosX);
    RESTORE(g_currentPosY);
    RESTORE(g_currentHeading);
}

Test_test(Test, draw_laser_line_3)
{
    uint16_t SAVE(g_currentPosX);
    uint16_t SAVE(g_currentPosY);
    uint16_t SAVE(g_currentHeading);

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

    TEST_RESET_MAP(24, 0, 1);
    TEST_RESET_MAP(24, 1, 1);
    TEST_RESET_MAP(24, 2, 1);
    TEST_RESET_MAP(24, 3, -1);

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

    TEST_RESET_MAP(24, 0, 1);
    TEST_RESET_MAP(23, 0, -1);

    RESTORE(g_currentPosX);
    RESTORE(g_currentPosY);
    RESTORE(g_currentHeading);
}
Test_test(Test, draw_laser_line_3_2)
{
    uint16_t SAVE(g_currentPosX);
    uint16_t SAVE(g_currentPosY);
    uint16_t SAVE(g_currentHeading);

    struct laser_data ld;
    g_currentPosX = GridToMm(24);
    g_currentPosY = GridToMm(0);
    g_currentHeading = FULL_TURN/4;

    update_trig_cache();

    struct sensor_data sd;
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

    TEST_RESET_MAP(24, 0, 1);
    TEST_RESET_MAP(25, 0, -1);

    RESTORE(g_currentPosX);
    RESTORE(g_currentPosY);
    RESTORE(g_currentHeading);
}

Test_test(Test, ir_min)
{
    uint16_t SAVE(g_currentPosX);
    uint16_t SAVE(g_currentPosY);
    uint16_t SAVE(g_currentHeading);

    Test_assertEquals(laser_positive_x(
                GridToMm(24)+70,
                GridToMm(10)+200+80,
                GridToMm(24)+70+IR_MAX,
                0), 0);
    TEST_RESET_MAP(25, 11, 1);

    g_currentPosX = GridToMm(24);
    g_currentPosY = GridToMm(10) + 200;
    g_currentHeading = FULL_TURN/4;
    struct laser_data ld;

    struct sensor_data sd;
    sd.ir_rightfront = 530;
    calculate_laser_data(&sd, &ld, 4);

    Test_assertEquals(ld.startX, GridToMm(24)+70); //9870
    Test_assertEquals(ld.startY, GridToMm(10)+200+80); //8480
    Test_assertEquals(ld.endX, GridToMm(24)+70+IR_MAX); //10220
    Test_assertEquals(ld.reliable, 2);

    draw_laser_line(&ld);
    TEST_RESET_MAP(25, 11, 1);
        TEST_RESET_MAP(24, 11, 1);

    RESTORE(g_currentPosX);
    RESTORE(g_currentPosY);
    RESTORE(g_currentHeading);
}

Test_test(Test, update_map)
{
    uint16_t SAVE(g_currentPosX);
    uint16_t SAVE(g_currentPosY);
    uint16_t SAVE(g_currentHeading);

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
    //detected by multiple sensors
    TEST_RESET_MAP(24, 10, 3);
    TEST_RESET_MAP(24, 11, 3);

    TEST_RESET_MAP(24, 12, 1);
    TEST_RESET_MAP(24, 13, -1);

    TEST_RESET_MAP(24, 9, 1);
    TEST_RESET_MAP(24, 8, -1);

    TEST_RESET_MAP(23, 11, -1);

    TEST_RESET_MAP(23, 10, 1);
    // (22, 10) above IR_MAX

    TEST_RESET_MAP(25, 11, 1);
    // (26, 11) above IR_MAX

    TEST_RESET_MAP(25, 10, -1);

    RESTORE(g_currentPosX);
    RESTORE(g_currentPosY);
    RESTORE(g_currentHeading);
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
    uint16_t SAVE(g_currentPosX);
    uint16_t SAVE(g_currentPosY);
    uint16_t SAVE(g_currentHeading);

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
    Test_assertEquals(g_currentPosY, GridToMm(10)+10);

    g_currentPosX = GridToMm(24);
    g_currentPosY = GridToMm(10);
    sd = (struct foo){ .arr = {480, 480, 130, 130, 120, 120}};
    Test_assertEquals(adjust_position((struct sensor_data*) &sd), true);
    Test_assertEquals(g_currentPosX, GridToMm(24)+5);
    Test_assertEquals(g_currentPosY, GridToMm(10));

    g_currentHeading = FULL_TURN/2;
    update_trig_cache();

    g_currentPosX = GridToMm(24)+8;
    g_currentPosY = GridToMm(10);
    sd = (struct foo){ .arr = {470, 490, 130, 130, 120, 120}};
    Test_assertEquals(adjust_position((struct sensor_data*) &sd), true);
    Test_assertEquals(g_currentPosX, GridToMm(24)-10);
    Test_assertEquals(g_currentPosY, GridToMm(10)+5);

    // TODO broken since lowering MIN_ADJUST_SENSORS
    /*g_currentPosX = GridToMm(24)+8;
    g_currentPosY = GridToMm(10);
    sd = (struct foo){ .arr = {470, 480, 0, 0, 120, 120}};
    Test_assertEquals(adjust_position((struct sensor_data*) &sd), false);
    Test_assertEquals(g_currentPosX, GridToMm(24)+8);
    Test_assertEquals(g_currentPosY, GridToMm(10));

    g_currentPosX = GridToMm(24);
    g_currentPosY = GridToMm(10);
    sd = (struct foo){ .arr = {480, 490, 330, 30, 120, 120}};
    Test_assertEquals(adjust_position((struct sensor_data*) &sd), false);
    Test_assertEquals(g_currentPosX, GridToMm(24));
    Test_assertEquals(g_currentPosY, GridToMm(10));*/

    g_currentHeading = FULL_TURN/8;
    update_trig_cache();

    g_currentPosX = GridToMm(24)+10; //9810
    g_currentPosY = GridToMm(10);
    sd = (struct foo){ .arr = {728, 728, 293, 127, 133, 298}};
    Test_assertEquals(adjust_position((struct sensor_data*) &sd), true);

    Test_assertEquals(g_currentPosX, GridToMm(24)); //9800
    Test_assertEquals(g_currentPosY, GridToMm(10)); //4200

    RESTORE(g_currentPosX);
    RESTORE(g_currentPosY);
    RESTORE(g_currentHeading);
}
Test_test(Test, corner_sensitivity)
{
    uint16_t SAVE(g_currentPosX);
    uint16_t SAVE(g_currentPosY);
    uint16_t SAVE(g_currentHeading);

    struct laser_data ld;
    ld.startX = 400;
    ld.startY = 400;
    ld.endX = 1200;
    ld.endY = 400;
    ld.collision_type = 0;
    ld.quadrant = 0;
    ld.offset = 0;
    ld.cos = 1;
    ld.sin = 0;

    ld.reliable = 0;
    ld.corner_error = true;
    ld.offset_error = false;

    draw_laser_line(&ld);
    ld.startX = 600;
    draw_laser_line(&ld);

    RESTORE(g_currentPosX);
    RESTORE(g_currentPosY);
    RESTORE(g_currentHeading);
}
Test_test(Test, corner_sensitivity_y)
{
    uint16_t SAVE(g_currentPosX);
    uint16_t SAVE(g_currentPosY);
    uint16_t SAVE(g_currentHeading);

    struct laser_data ld;
    ld.startX = 400;
    ld.startY = 600;
    ld.endX = 400;
    ld.endY = 1200;
    ld.collision_type = 1;
    ld.quadrant = 1;
    ld.offset = 0;
    ld.cos = 0;
    ld.sin = 1;
    ld.reliable = 0;
    ld.corner_error = true;
    ld.offset_error = false;
    draw_laser_line(&ld);

    RESTORE(g_currentPosX);
    RESTORE(g_currentPosY);
    RESTORE(g_currentHeading);
}
// test max error
#endif // __TEST__
