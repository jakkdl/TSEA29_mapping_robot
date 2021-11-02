#include "robot.h"
#include "navigation_unit.h"


//assumes data is not corrupt, does not check parity
uint8_t communication_unit_interrupt(struct Com_packet* data) {
    // verify valid data packet count
    if (data->adress != ADR_DEBUG)
    {
        if (data->packet_count != ADR_DATA_PACKETS[data->adress])
        {
            //Invalid number of data packets
            return -1;
        }
    }

    if (data->adress == ADR_PARITY_ERROR)
    {
        //send the data associated with adress data_packets[0];
        return resend(data->data_packets[0]);
    }

    switch (data->adress)
    {
        case ADR_COMMAND:
            return handle_command(data->data_packets[0]);
        case ADR_PD_KP:
            return set_pd_kp(data->data_packets[0]);
        case ADR_PD_KD:
            return set_pd_kd(data->data_packets[0]);
        default:
            return -1;
    }
}




uint8_t handle_command(uint8_t id)
{
    switch (id)
    {
        case ID_STOP:
            return command_stop();
        case ID_START:
            return command_start();
        default:
            if (navigationMode != NAVIGATION_MODE_MANUAL)
            {
                return -1;
            }
            return command_set_target_square(id);
    }

}


// resend the data last sent with that adress
// might not be needed
uint8_t resend(uint8_t _adress)
{
    return -1;
}
// Set the PD-constant KP
uint8_t set_pd_kp(uint8_t kp)
{
    pdkp = kp;
    return 0;
}
// Set the PD-constant KD
uint8_t set_pd_kd(uint8_t kd)
{
    pdkd = kd;
    return 0;
}

// Manual stop
// Stop both wheels
// clear the navigation goal
// set navigation to manual
uint8_t command_stop()
{
    wheelSpeedLeft = 0;
    wheelSpeedRight = 0;
    navigationGoalType = NAVIGATION_GOAL_NONE;
    navigationMode = NAVIGATION_MODE_MANUAL;
    return 0;
}

// Start exploring the maze autonomously
// Set navigation to automatic
uint8_t command_start()
{
    navigationGoalType = NAVIGATION_GOAL_NONE;
    navigationMode = NAVIGATION_MODE_AUTONOMOUS;
    return 0;
}

uint8_t navigate_forward(uint8_t dir)
{
    switch (dir)
    {
        case 0:
            navigationGoalX += 1;
            break;
        case 1:
            navigationGoalY += 1;
            break;
        case 2:
            navigationGoalX -= 1;
            break;
        case 3:
            navigationGoalY -= 1;
            break;
        default:
            return -1;
    }
    navigationGoalType = NAVIGATION_GOAL_MOVE;
    return 0;
}

uint8_t command_set_target_square(uint8_t id)
{
    // get current heading, rounded to nearest quarter turn
    // 0 = straight right
    // FULL_TURN / 2 = straight left
    // FULL_TURN / 4 = straight up
    // FULL_TURN * 3 / 4 = straight down
    uint8_t dir;

    // right
    if (currentHeading < FULL_TURN/8 || currentHeading > FULL_TURN*7/8)
    {
        dir = 0;
    }
    // up
    else if (currentHeading < FULL_TURN*3/8)
    {
        dir = 1;
    }
    // left
    else if (currentHeading < FULL_TURN*5/8)
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
        case ID_FORWARD:
            return navigate_forward(dir);
        case ID_BACKWARD:
            //going backward is the same as a half-turn and forward
            return navigate_forward((dir+2) % 4);
        case ID_FW_LEFT:
            return navigate_forward((dir+1) % 4);
        case ID_FW_RIGHT:
            return navigate_forward((dir+3) % 4);
        case ID_TURN_LEFT:
            navigationGoalHeading = ((dir+1) % 4) / 4 * FULL_TURN;
            navigationGoalType = NAVIGATION_GOAL_TURN;
            return 0;
        case ID_TURN_RIGHT:
            navigationGoalHeading = ((dir+3) % 4) / 4 * FULL_TURN;
            navigationGoalType = NAVIGATION_GOAL_TURN;
            return 0;
        default:
            return -1;
    }
}
