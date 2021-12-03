#include "nav_unit_com_interrupt_logic.h"
#include "../AVR_common/robot.h"
#include "navigation_unit.h"
#include <stdio.h>

// assumes data is not corrupt, does not check parity
int8_t communication_unit_interrupt(struct data_packet* data)
{
    // verify valid data packet count
    if (data->address != ADR_DEBUG)
    {
        if (data->byte_count != ADR_DATA_PACKETS[data->address])
        {
            // Invalid number of data packets
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
        case COMMAND:
            return handle_command((enum directionID)data->bytes[0]);
        case PD_KP:
            return set_pd_kp(data->bytes[0]);
        case PD_KD:
            return set_pd_kd(data->bytes[0]);
        default:
            return -1;
    }
}

int8_t handle_command(enum directionID id)
{
    switch (id)
    {
        case STOP:
            return command_stop();
        case START:
            return command_start();
        default:
            if (g_navigationMode != MANUAL)
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
    g_pdKp = kp;
    return 0;
}
// Set the PD-constant KD
int8_t set_pd_kd(uint8_t kd)
{
    g_pdKd = kd;
    return 0;
}

// Manual stop
// Stop both wheels
// clear the navigation goal
// set navigation to manual
int8_t command_stop()
{
    g_wheelSpeedLeft    = 0;
    g_wheelSpeedRight   = 0;
    g_navigationGoalSet = false;
    g_navigationMode    = MANUAL;
    return 0;
}

// Start exploring the maze autonomously
// Set navigation to automatic
int8_t command_start()
{
    g_navigationGoalSet = false;
    g_navigationMode    = AUTONOMOUS;
    return 0;
}

int8_t navigate_forward(uint8_t dir)
{
    switch (dir)
    {
        case 0:
            g_navigationGoalX = GridToMm(MmToGrid(g_currentPosX) + 1);
            g_navigationGoalY = g_currentPosY;
            break;
        case 1:
            g_navigationGoalX = g_currentPosX;
            g_navigationGoalY = GridToMm(MmToGrid(g_currentPosY) + 1);
            break;
        case 2:
            g_navigationGoalX = GridToMm(MmToGrid(g_currentPosX) - 1);
            g_navigationGoalY = g_currentPosY;
            break;
        case 3:
            g_navigationGoalX = g_currentPosX;
            g_navigationGoalY = GridToMm(MmToGrid(g_currentPosY) - 1);
            break;
        default:
            return -1;
    }
    g_navigationGoalHeading = dir * FULL_TURN / 4;
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
    if (g_currentHeading < FULL_TURN / 8 ||
        g_currentHeading > FULL_TURN / 8 * 7)
    {
        dir = 0;
    }
    // up
    else if (g_currentHeading < FULL_TURN / 8 * 3)
    {
        dir = 1;
    }
    // left
    else if (g_currentHeading < FULL_TURN / 8 * 5)
    {
        dir = 2;
    }
    // down
    else
    {
        dir = 3;
    }
    g_navigationGoalSet = true;

    switch (id)
    {
        case FORWARD:
            return navigate_forward(dir);
        case BACKWARD:
            // going backward is the same as a half-turn and forward
            return navigate_forward((dir + 2) % 4);
        case FW_LEFT:
            return navigate_forward((dir + 1) % 4);
        case FW_RIGHT:
            return navigate_forward((dir + 3) % 4);
        case TURN_LEFT:
            g_navigationGoalHeading = ((dir + 1) % 4) * FULL_TURN / 4;
			g_navigationGoalX = g_currentPosX;
			g_navigationGoalY = g_currentPosY;
            return 0;
        case TURN_RIGHT:
            g_navigationGoalHeading = ((dir + 3) % 4) * FULL_TURN / 4;
			g_navigationGoalX = g_currentPosX;
			g_navigationGoalY = g_currentPosY;
            return 0;
        case TURN_AROUND:
            g_navigationGoalHeading = ((dir + 2) % 4) * FULL_TURN / 4;
			g_navigationGoalX = g_currentPosX;
			g_navigationGoalY = g_currentPosY;
            return 0;
        default:
            return -1;
    }
}
#if __TEST__
#include "../AVR_testing/test.h"
Test_test(Test, uartCommandStart)
{
    bool                oldGoalSet        = g_navigationGoalSet;
    enum NavigationMode oldNavigationMode = g_navigationMode;
    struct data_packet  data;
    data.address    = COMMAND;
    data.byte_count = 1;
    data.bytes[0]   = START;
    Test_assertEquals(communication_unit_interrupt(&data), 0);

    Test_assertEquals(g_navigationGoalSet, false);
    Test_assertEquals(g_navigationMode, AUTONOMOUS);

    g_navigationGoalSet = oldGoalSet;
    g_navigationMode    = oldNavigationMode;
}
Test_test(Test, uartCommand_turn_left)
{
    bool                oldGoalSet               = g_navigationGoalSet;
    enum NavigationMode oldNavigationMode        = g_navigationMode;
    uint16_t            oldNavigationGoalHeading = g_navigationGoalHeading;
    uint16_t oldHeading = g_currentHeading;

    struct data_packet data;
    data.address    = COMMAND;
    data.byte_count = 1;

    data.bytes[0] = TURN_LEFT;

    // Test it doesn't do anything in auto mode
    g_navigationMode = AUTONOMOUS;
    Test_assertEquals(communication_unit_interrupt(&data), -1);
    Test_assertEquals(g_navigationGoalSet, oldGoalSet);
    Test_assertEquals(g_navigationMode, AUTONOMOUS);
    Test_assertEquals(oldNavigationGoalHeading, g_navigationGoalHeading);

    // Test actual move
    g_navigationMode = MANUAL;
    g_currentHeading = 0;

    Test_assertEquals(communication_unit_interrupt(&data), 0);
    Test_assertEquals(g_navigationGoalSet, true);
    Test_assertEquals(g_navigationMode, MANUAL);
    // Test_assertEquals(g_navigationGoalHeading, FULL_TURN/4);

    g_navigationGoalSet     = oldGoalSet;
    g_navigationMode        = oldNavigationMode;
    g_navigationGoalHeading = oldNavigationGoalHeading;
    g_currentHeading = oldHeading;
}

Test_test(Test, uartCommand_fw_left)
{
    bool                oldGoalSet         = g_navigationGoalSet;
    enum NavigationMode oldNavigationMode  = g_navigationMode;
    uint8_t             oldNavigationGoalX = g_navigationGoalX;
    uint8_t             oldNavigationGoalY = g_navigationGoalY;
    uint8_t             oldNavigationGoalHeading = g_navigationGoalHeading;
    uint16_t oldHeading = g_currentHeading;

    struct data_packet data;
    data.address    = COMMAND;
    data.byte_count = 1;

    data.bytes[0] = FW_LEFT;

    // Test it doesn't do anything in auto mode
    g_navigationMode = AUTONOMOUS;
    Test_assertEquals(communication_unit_interrupt(&data), -1);
    Test_assertEquals(g_navigationGoalSet, oldGoalSet);
    Test_assertEquals(g_navigationMode, AUTONOMOUS);

    // Test actual move
    g_navigationMode = MANUAL;
    g_currentHeading = 0;
    g_currentPosX    = GridToMm(24);
    g_currentPosY    = 0;

    Test_assertEquals(communication_unit_interrupt(&data), 0);
    Test_assertEquals(g_navigationGoalSet, true);
    Test_assertEquals(g_navigationMode, MANUAL);
    Test_assertEquals(g_navigationGoalX, GridToMm(24));
    Test_assertEquals(g_navigationGoalY, GridToMm(1));

    g_navigationGoalSet = oldGoalSet;
    g_navigationMode    = oldNavigationMode;
    g_navigationGoalX   = oldNavigationGoalX;
    g_navigationGoalY   = oldNavigationGoalY;
    g_currentHeading = oldHeading;
    g_navigationGoalHeading = oldNavigationGoalHeading;
}
#endif
