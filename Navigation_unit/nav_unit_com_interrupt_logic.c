#include "../AVR_common/robot.h"
#include "navigation_unit.h"
#include "nav_unit_com_interrupt_logic.h"

//assumes data is not corrupt, does not check parity
int8_t communication_unit_interrupt(struct data_packet *data)
{
    // verify valid data packet count
    if (data->address != debug)
    {
        if (data->byte_count != ADR_DATA_PACKETS[data->address])
        {
            //Invalid number of data packets
            return -1;
        }
    }

    /*if (data->address == parity_error)
    {
        //send the data associated with address bytes[0];
        return resend(data->bytes[0]);
    }*/

    switch (data->address)
    {
    case command:
        return handle_command((enum directionID)data->bytes[0]);
    case pd_kp:
        return set_pd_kp(data->bytes[0]);
    case pd_kd:
        return set_pd_kd(data->bytes[0]);
    default:
        return -1;
    }
}

int8_t handle_command(enum directionID id)
{
    switch (id)
    {
    case stop:
        return command_stop();
    case start:
        return command_start();
    default:
        if (navigationMode != manual)
        {
            return -1;
        }
        return command_set_target_square(id);
    }
}

// resend the data last sent with that address
// might not be needed
/*int8_t resend(uint8_t _address)
{
    return -1;
}*/
// Set the PD-constant KP
int8_t set_pd_kp(uint8_t kp)
{
    pdkp = kp;
    return 0;
}
// Set the PD-constant KD
int8_t set_pd_kd(uint8_t kd)
{
    pdkd = kd;
    return 0;
}

// Manual stop
// Stop both wheels
// clear the navigation goal
// set navigation to manual
int8_t command_stop()
{
    wheelSpeedLeft = 0;
    wheelSpeedRight = 0;
    navigationGoalType = none;
    navigationMode = manual;
    return 0;
}

// Start exploring the maze autonomously
// Set navigation to automatic
int8_t command_start()
{
    navigationGoalType = none;
    navigationMode = autonomous;
    return 0;
}

int8_t navigate_forward(uint8_t dir)
{
    switch (dir)
    {
    case 0:
        navigationGoalX = grid_to_mm(mm_to_grid(currentPosX) + 1);
        navigationGoalY = currentPosY;
        break;
    case 1:
        navigationGoalX = currentPosX;
        navigationGoalY = grid_to_mm(mm_to_grid(currentPosY) + 1);
        break;
    case 2:
        navigationGoalX = grid_to_mm(mm_to_grid(currentPosX) - 1);
        navigationGoalY = currentPosY;
        break;
    case 3:
        navigationGoalX = currentPosX;
        navigationGoalY = grid_to_mm(mm_to_grid(currentPosY) - 1);
        break;
    default:
        return -1;
    }
    navigationGoalType = move;
    return 0;
}

int8_t command_set_target_square(uint8_t id)
{
    // get current heading, rounded to nearest quarter turn
    // 0 = straight right
    // FULL_TURN / 2 = straight left
    // FULL_TURN / 4 = straight up
    // FULL_TURN * 3 / 4 = straight down
    uint8_t dir;

    // right
    if (currentHeading < FULL_TURN / 8 || currentHeading > FULL_TURN * 7 / 8)
    {
        dir = 0;
    }
    // up
    else if (currentHeading < FULL_TURN * 3 / 8)
    {
        dir = 1;
    }
    // left
    else if (currentHeading < FULL_TURN * 5 / 8)
    {
        dir = 2;
    }
    // down
    else
    {
        dir = 3;
    }

    switch (id)
    {
    case forward:
        return navigate_forward(dir);
    case backward:
        //going backward is the same as a half-turn and forward
        return navigate_forward((dir + 2) % 4);
    case fw_left:
        return navigate_forward((dir + 1) % 4);
    case fw_right:
        return navigate_forward((dir + 3) % 4);
    case turn_left:
        navigationGoalHeading = ((dir + 1) % 4) / 4 * FULL_TURN;
        navigationGoalType = turn;
        return 0;
    case turn_right:
        navigationGoalHeading = ((dir + 3) % 4) / 4 * FULL_TURN;
        navigationGoalType = turn;
        return 0;
    default:
        return -1;
    }
}

#include "../AVR_testing/test.h"
Test_test(Test, uartCommandStart)
{
    enum NavigationGoal oldGoalType = navigationGoalType;
    enum NavigationMode oldNavigationMode = navigationMode;
    struct data_packet data;
    data.address = command;
    data.byte_count = 1;
    data.bytes[0] = start;
    Test_assertEquals(communication_unit_interrupt(&data), 0);

    Test_assertEquals(navigationGoalType, none);
    Test_assertEquals(navigationMode, autonomous);

    navigationGoalType = oldGoalType;
    navigationMode = oldNavigationMode;
}
Test_test(Test, uartCommand_turn_left)
{
    enum NavigationGoal oldGoalType = navigationGoalType;
    enum NavigationMode oldNavigationMode = navigationMode;
    uint16_t oldNavigationGoalHeading = navigationGoalHeading;

    struct data_packet data;
    data.address = command;
    data.byte_count = 1;

    data.bytes[0] = turn_left;

    // Test it doesn't do anything in auto mode
    navigationMode = autonomous;
    Test_assertEquals(communication_unit_interrupt(&data), -1);
    Test_assertEquals(navigationGoalType, oldGoalType);
    Test_assertEquals(navigationMode, autonomous);
    Test_assertEquals(oldNavigationGoalHeading, navigationGoalHeading);

    // Test actual move
    navigationMode = manual;
    currentHeading = 0;

    Test_assertEquals(communication_unit_interrupt(&data), 0);
    Test_assertEquals(navigationGoalType, turn);
    Test_assertEquals(navigationMode, manual);
    //Test_assertEquals(navigationGoalHeading, FULL_TURN/4);

    navigationGoalType = oldGoalType;
    navigationMode = oldNavigationMode;
    navigationGoalHeading = oldNavigationGoalHeading;
}

Test_test(Test, uartCommand_fw_left)
{
    enum NavigationGoal oldGoalType = navigationGoalType;
    enum NavigationMode oldNavigationMode = navigationMode;
    uint8_t oldNavigationGoalX = navigationGoalX;
    uint8_t oldNavigationGoalY = navigationGoalY;

    struct data_packet data;
    data.address = command;
    data.byte_count = 1;

    data.bytes[0] = fw_left;

    // Test it doesn't do anything in auto mode
    navigationMode = autonomous;
    Test_assertEquals(communication_unit_interrupt(&data), -1);
    Test_assertEquals(navigationGoalType, oldGoalType);
    Test_assertEquals(navigationMode, autonomous);

    // Test actual move
    navigationMode = manual;
    currentHeading = 0;
    currentPosX = grid_to_mm(24);
    currentPosY = 0;

    Test_assertEquals(communication_unit_interrupt(&data), 0);
    Test_assertEquals(navigationGoalType, move);
    Test_assertEquals(navigationMode, manual);
    Test_assertEquals(navigationGoalX, grid_to_mm(24));
    Test_assertEquals(navigationGoalY, grid_to_mm(1));

    navigationGoalType = oldGoalType;
    navigationMode = oldNavigationMode;
    navigationGoalX = oldNavigationGoalX;
    navigationGoalY = oldNavigationGoalY;
}
