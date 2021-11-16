#include <stdlib.h> //abs

#include "../AVR_common/robot.h"
#include "../AVR_common/sensors.h"
#include "../AVR_testing/test.h"
#include "navigation_unit.h"

// robot will turn to a precision of at least 2.8 degrees
#define TURN_SENSITIVITY FULL_TURN / 128
// robot will stop when the middle of the robot is within this many mm of
// the middle of the target square along both the x and y axis
#define POS_SENSITIVITY 50

struct sensor_data next_sensor_data;
uint8_t            sensor_count = 0;

int8_t nav_main(struct sensor_data* data);
bool   arrived_at_goal(void);

int8_t handle_sensor_data(struct data_packet* data)
{
    // Check for parity error?

    // check packet count
    if (data->address != DEBUG)
    {
        if (data->byte_count != ADR_DATA_PACKETS[data->address])
        {
            // Invalid number of data packets
            return -1;
        }
    }

    // TODO Forward to com unit

    // Save the data
    sensor_count++;
    switch (data->address)
    {
        case LIDAR_FORWARD:
            next_sensor_data.lidar_forward = BYTES_TO_UINT16(data);
            break;
        case LIDAR_BACKWARD:
            next_sensor_data.lidar_backward = BYTES_TO_UINT16(data);
            break;
        case IR_LEFTFRONT:
            next_sensor_data.ir_leftfront = BYTES_TO_UINT16(data);
            break;
        case IR_LEFTBACK:
            next_sensor_data.ir_leftback = BYTES_TO_UINT16(data);
            break;
        case IR_RIGHTFRONT:
            next_sensor_data.ir_rightfront = BYTES_TO_UINT16(data);
            break;
        case IR_RIGHTBACK:
            next_sensor_data.ir_rightback = BYTES_TO_UINT16(data);
            break;
        case ODOMETER:
            next_sensor_data.odometer_left  = data->bytes[0];
            next_sensor_data.odometer_right = data->bytes[1];
            break;
        case GYRO:
            next_sensor_data.gyro = BYTES_TO_UINT16(data);
            break;
        default:
            return -1;
    }

    // Check if we've received all data
    if (sensor_count == SENSOR_PACKETS)
    {
        // copy the data, so we can begin writing the next set of data
        // while still working on it
        struct sensor_data current_sensor_data = next_sensor_data;
        sensor_count                           = 0;

        return nav_main(&current_sensor_data);
    }
    return 0;
}

// EXTRA: assumes we've checked for parity error
int8_t nav_main(struct sensor_data* data)
{
    // TODO calculate heading and position
    // uses data, updates g_currentHeading, g_currentPosX and g_currentPosY

    if (g_navigationGoalType != NONE)
    {
        // have we arrived at the navigation goal?
        if (arrived_at_goal())
        {
            // clear navigation goal
            g_navigationGoalType = NONE;
            // TODO stop moving
        }
        else
        {
            // TODO drive motors with pd towards the goal
            __asm__("nop");
            // TODO send debug values to com unit?
        }
    }

    // TODO send g_currentHeading, g_currentPosX and g_currentPosY to com-unit

    // run map update algorithm
    // which if there's updates, sends them to com-unit
    __asm__("nop");

    // Check if we should run nav algo
    if (g_navigationMode == AUTONOMOUS && g_navigationGoalType == NONE)
    {
        // run navigation algorithm, setting g_navigationGoal
        // sample_search()
    }

    return 0;
}

bool arrived_at_goal(void)
{
    if (g_navigationGoalType == TURN)
    {
        if (abs(g_currentHeading - g_navigationGoalHeading) < TURN_SENSITIVITY)
        {
            return true;
        }
        return false;
    }
    if (g_navigationGoalType == MOVE)
    {
        if (abs(g_currentPosX - g_navigationGoalX) < POS_SENSITIVITY &&
            abs(g_currentPosY - g_navigationGoalY) < POS_SENSITIVITY)
        {
            return true;
        }
        return false;
    }
    // if g_navigationGoalType == none
    return true;
}

/* #### UNIT TESTS #### */

// test handle sensor data
Test_test(Test, handle_sensor_data_lidar_forward)
{
    // save old values to restore later
    struct sensor_data old_sensor_data  = next_sensor_data;
    uint8_t            old_sensor_count = sensor_count;

    // set up environment
    sensor_count                   = 0;
    next_sensor_data.lidar_forward = 0;

    // construct parameter
    struct data_packet data;
    data.address    = LIDAR_FORWARD;
    data.byte_count = 2;
    data.bytes[0]   = UINT16_TO_BYTE_0(2000);
    data.bytes[1]   = UINT16_TO_BYTE_1(2000);

    // test
    Test_assertEquals(handle_sensor_data(&data), 0);
    Test_assertEquals(sensor_count, 1);
    Test_assertEquals(next_sensor_data.lidar_forward, 2000);

    // restore old values
    next_sensor_data = old_sensor_data;
    sensor_count     = old_sensor_count;
}

Test_test(Test, handle_sensor_data_odometer)
{
    // save old values to restore later
    struct sensor_data old_sensor_data  = next_sensor_data;
    uint8_t            old_sensor_count = sensor_count;

    // set up environment
    sensor_count                    = 3;
    next_sensor_data.odometer_left  = 0;
    next_sensor_data.odometer_right = 0;

    // construct parameter
    struct data_packet data;
    data.address    = ODOMETER;
    data.byte_count = 2;
    data.bytes[0]   = 5;
    data.bytes[1]   = 6;

    Test_assertEquals(handle_sensor_data(&data), 0);
    Test_assertEquals(sensor_count, 4);
    Test_assertEquals(next_sensor_data.odometer_left, 5);
    Test_assertEquals(next_sensor_data.odometer_right, 6);

    // restore old values
    next_sensor_data = old_sensor_data;
    sensor_count     = old_sensor_count;
}

Test_test(Test, arrived_at_goal_turn)
{
    enum NavigationGoal oldGoalType              = g_navigationGoalType;
    uint16_t            oldNavigationGoalHeading = g_navigationGoalHeading;
    uint16_t            oldCurrentHeading        = g_currentHeading;

    g_navigationGoalType = TURN;

    g_navigationGoalHeading = FULL_TURN / 2;
    g_currentHeading        = FULL_TURN / 2 + TURN_SENSITIVITY - 1;
    Test_assertTrue(arrived_at_goal());

    g_currentHeading = FULL_TURN / 2 - TURN_SENSITIVITY + 1;
    Test_assertTrue(arrived_at_goal());

    g_navigationGoalHeading = 0;
    g_currentHeading        = TURN_SENSITIVITY - 1;
    Test_assertTrue(arrived_at_goal());

    g_navigationGoalHeading = 0;
    g_currentHeading        = 1 - TURN_SENSITIVITY;
    Test_assertTrue(arrived_at_goal());

    // test falsity
    g_navigationGoalHeading = FULL_TURN / 2;
    g_currentHeading        = FULL_TURN / 2 + TURN_SENSITIVITY;
    Test_assertTrue(!arrived_at_goal());

    g_currentHeading = FULL_TURN / 2 - TURN_SENSITIVITY;
    Test_assertTrue(!arrived_at_goal());

    g_navigationGoalHeading = 0;
    g_currentHeading        = TURN_SENSITIVITY;
    Test_assertTrue(!arrived_at_goal());

    g_navigationGoalHeading = 0;
    g_currentHeading        = -TURN_SENSITIVITY;
    Test_assertTrue(!arrived_at_goal());

    // check that we didn't accidentally modify navgoaltype
    Test_assertEquals(g_navigationGoalType, TURN);

    // restore old values
    g_navigationGoalType    = oldGoalType;
    g_navigationGoalHeading = oldNavigationGoalHeading;
    g_currentHeading        = oldCurrentHeading;
}

Test_test(Test, arrived_at_goal_pos)
{
    enum NavigationGoal oldGoalType        = g_navigationGoalType;
    uint16_t            oldNavigationGoalX = g_navigationGoalX;
    uint16_t            oldNavigationGoalY = g_navigationGoalY;
    uint16_t            oldCurrentPosX     = g_currentPosX;
    uint16_t            oldCurrentPosY     = g_currentPosY;

    g_navigationGoalType = MOVE;

    g_currentPosX     = GridToMm(27);
    g_currentPosY     = GridToMm(5);
    g_navigationGoalX = g_currentPosX + POS_SENSITIVITY - 1;
    g_navigationGoalY = g_currentPosY + POS_SENSITIVITY - 1;
    Test_assertTrue(arrived_at_goal());

    g_navigationGoalX = g_currentPosX + POS_SENSITIVITY - 1;
    g_navigationGoalY = g_currentPosY + POS_SENSITIVITY;
    Test_assertTrue(!arrived_at_goal());

    g_navigationGoalX = g_currentPosX + POS_SENSITIVITY;
    g_navigationGoalY = g_currentPosY + POS_SENSITIVITY - 1;
    Test_assertTrue(!arrived_at_goal());

    g_navigationGoalX = g_currentPosX + POS_SENSITIVITY;
    g_navigationGoalY = g_currentPosY + POS_SENSITIVITY;
    Test_assertTrue(!arrived_at_goal());

    // check that we didn't accidentally modify navgoaltype
    Test_assertEquals(g_navigationGoalType, MOVE);

    g_navigationGoalType = oldGoalType;
    g_navigationGoalX    = oldNavigationGoalX;
    g_navigationGoalY    = oldNavigationGoalY;
    g_currentPosX        = oldCurrentPosX;
    g_currentPosY        = oldCurrentPosY;
}
