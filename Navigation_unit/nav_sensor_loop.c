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
uint8_t sensor_count = 0;

int8_t nav_main(struct sensor_data* data);
bool arrived_at_goal(void);

int8_t handle_sensor_data(struct data_packet* data)
{
    // Check for parity error?

    // check packet count
    if (data->address != debug)
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
        case lidar_forward:
            next_sensor_data.lidar_forward = BYTES_TO_UINT16(data);
            break;
        case lidar_backward:
            next_sensor_data.lidar_backward = BYTES_TO_UINT16(data);
            break;
        case ir_leftfront:
            next_sensor_data.ir_leftfront = BYTES_TO_UINT16(data);
            break;
        case ir_leftback:
            next_sensor_data.ir_leftback = BYTES_TO_UINT16(data);
            break;
        case ir_rightfront:
            next_sensor_data.ir_rightfront = BYTES_TO_UINT16(data);
            break;
        case ir_rightback:
            next_sensor_data.ir_rightback = BYTES_TO_UINT16(data);
            break;
        case odometer:
            next_sensor_data.odometer_left = data->bytes[0];
            next_sensor_data.odometer_right = data->bytes[1];
            break;
        case gyro:
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
        sensor_count = 0;

        return nav_main(&current_sensor_data);
    }
    return 0;
}

// EXTRA: assumes we've checked for parity error
int8_t nav_main(struct sensor_data* data)
{

    // TODO calculate heading and position
    // uses data, updates currentHeading, currentPosX and currentPosY

    if (navigationGoalType != none)
    {
        // have we arrived at the navigation goal?
        if (arrived_at_goal())
        {
            // clear navigation goal
            navigationGoalType = none;
            // TODO stop moving
        }
        else
        {
            // TODO drive motors with pd towards the goal
            __asm__("nop");
            // TODO send debug values to com unit?
        }
    }

    // TODO send currentHeading, currentPosX and currentPosY to com-unit

    // run map update algorithm
    // which if there's updates, sends them to com-unit
    __asm__("nop");

    // Check if we should run nav algo
    if (navigationMode == autonomous && navigationGoalType == none)
    {
        // run navigation algorithm, setting navigationGoal
        // sample_search()
    }

    return 0;
}

bool arrived_at_goal(void)
{
    if (navigationGoalType == turn)
    {
        if (abs(currentHeading - navigationGoalHeading) < TURN_SENSITIVITY)
        {
            return true;
        }
        return false;
    }
    if (navigationGoalType == move)
    {
        if (abs(currentPosX - navigationGoalX) < POS_SENSITIVITY &&
            abs(currentPosY - navigationGoalY) < POS_SENSITIVITY)
        {
            return true;
        }
        return false;
    }
    // if navigationGoalType == none
    return true;
}

/* #### UNIT TESTS #### */

// test handle sensor data
Test_test(Test, handle_sensor_data_lidar_forward)
{
    // save old values to restore later
    struct sensor_data old_sensor_data = next_sensor_data;
    uint8_t old_sensor_count = sensor_count;

    // set up environment
    sensor_count = 0;
    next_sensor_data.lidar_forward = 0;

    // construct parameter
    struct data_packet data;
    data.address = lidar_forward;
    data.byte_count = 2;
    data.bytes[0] = UINT16_TO_BYTE_0(2000);
    data.bytes[1] = UINT16_TO_BYTE_1(2000);

    // test
    Test_assertEquals(handle_sensor_data(&data), 0);
    Test_assertEquals(sensor_count, 1);
    Test_assertEquals(next_sensor_data.lidar_forward, 2000);

    // restore old values
    next_sensor_data = old_sensor_data;
    sensor_count = old_sensor_count;
}

Test_test(Test, handle_sensor_data_odometer)
{
    // save old values to restore later
    struct sensor_data old_sensor_data = next_sensor_data;
    uint8_t old_sensor_count = sensor_count;

    // set up environment
    sensor_count = 3;
    next_sensor_data.odometer_left = 0;
    next_sensor_data.odometer_right = 0;

    // construct parameter
    struct data_packet data;
    data.address = odometer;
    data.byte_count = 2;
    data.bytes[0] = 5;
    data.bytes[1] = 6;

    Test_assertEquals(handle_sensor_data(&data), 0);
    Test_assertEquals(sensor_count, 4);
    Test_assertEquals(next_sensor_data.odometer_left, 5);
    Test_assertEquals(next_sensor_data.odometer_right, 6);

    // restore old values
    next_sensor_data = old_sensor_data;
    sensor_count = old_sensor_count;
}

Test_test(Test, arrived_at_goal_turn)
{
    enum NavigationGoal oldGoalType = navigationGoalType;
    uint16_t oldNavigationGoalHeading = navigationGoalHeading;
    uint16_t oldCurrentHeading = currentHeading;

    navigationGoalType = turn;

    navigationGoalHeading = FULL_TURN / 2;
    currentHeading = FULL_TURN / 2 + TURN_SENSITIVITY - 1;
    Test_assertTrue(arrived_at_goal());

    currentHeading = FULL_TURN / 2 - TURN_SENSITIVITY + 1;
    Test_assertTrue(arrived_at_goal());

    navigationGoalHeading = 0;
    currentHeading = TURN_SENSITIVITY - 1;
    Test_assertTrue(arrived_at_goal());

    navigationGoalHeading = 0;
    currentHeading = 1 - TURN_SENSITIVITY;
    Test_assertTrue(arrived_at_goal());

    // test falsity
    navigationGoalHeading = FULL_TURN / 2;
    currentHeading = FULL_TURN / 2 + TURN_SENSITIVITY;
    Test_assertTrue(!arrived_at_goal());

    currentHeading = FULL_TURN / 2 - TURN_SENSITIVITY;
    Test_assertTrue(!arrived_at_goal());

    navigationGoalHeading = 0;
    currentHeading = TURN_SENSITIVITY;
    Test_assertTrue(!arrived_at_goal());

    navigationGoalHeading = 0;
    currentHeading = -TURN_SENSITIVITY;
    Test_assertTrue(!arrived_at_goal());

    // check that we didn't accidentally modify navgoaltype
    Test_assertEquals(navigationGoalType, turn);

    // restore old values
    navigationGoalType = oldGoalType;
    navigationGoalHeading = oldNavigationGoalHeading;
    currentHeading = oldCurrentHeading;
}

Test_test(Test, arrived_at_goal_pos)
{
    enum NavigationGoal oldGoalType = navigationGoalType;
    uint16_t oldNavigationGoalX = navigationGoalX;
    uint16_t oldNavigationGoalY = navigationGoalY;
    uint16_t oldCurrentPosX = currentPosX;
    uint16_t oldCurrentPosY = currentPosY;

    navigationGoalType = move;

    currentPosX = grid_to_mm(27);
    currentPosY = grid_to_mm(5);
    navigationGoalX = currentPosX + POS_SENSITIVITY - 1;
    navigationGoalY = currentPosY + POS_SENSITIVITY - 1;
    Test_assertTrue(arrived_at_goal());

    navigationGoalX = currentPosX + POS_SENSITIVITY - 1;
    navigationGoalY = currentPosY + POS_SENSITIVITY;
    Test_assertTrue(!arrived_at_goal());

    navigationGoalX = currentPosX + POS_SENSITIVITY;
    navigationGoalY = currentPosY + POS_SENSITIVITY - 1;
    Test_assertTrue(!arrived_at_goal());

    navigationGoalX = currentPosX + POS_SENSITIVITY;
    navigationGoalY = currentPosY + POS_SENSITIVITY;
    Test_assertTrue(!arrived_at_goal());

    // check that we didn't accidentally modify navgoaltype
    Test_assertEquals(navigationGoalType, move);

    navigationGoalType = oldGoalType;
    navigationGoalX = oldNavigationGoalX;
    navigationGoalY = oldNavigationGoalY;
    currentPosX = oldCurrentPosX;
    currentPosY = oldCurrentPosY;
}
