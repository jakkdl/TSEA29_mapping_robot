#include "robot.h"
#include "navigation_unit.h"


//assumes data is not corrupt, does not check parity
short communication_unit_interrupt(struct Com_packet* data) {
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




short handle_command(short id)
{
    switch (id)
    {
        case ID_STOP:
            return command_stop();
        case ID_START:
            return command_start();
        default:
            if (NAVIGATION_MODE != NAVIGATION_MODE_MANUAL)
            {
                return -1;
            }
            return command_set_target_square(id);
    }

}


// resend the data last sent with that adress
// might not be needed
short resend(short _adress)
{
    return -1;
}
// Set the PD-constant KP
short set_pd_kp(short kp)
{
    PD_KP = kp;
    return 0;
}
// Set the PD-constant KD
short set_pd_kd(short kd)
{
    PD_KD = kd;
    return 0;
}

// Manual stop
// Stop both wheels
// clear the navigation goal
// set navigation to manual
short command_stop()
{
    WHEEL_SPEED_LEFT = 0;
    WHEEL_SPEED_RIGHT = 0;
    NAVIGATION_GOAL_TYPE = NAVIGATION_GOAL_NONE;
    NAVIGATION_MODE = NAVIGATION_MODE_MANUAL;
    return 0;
}

// Start exploring the maze autonomously
// Set navigation to automatic
short command_start()
{
    NAVIGATION_GOAL_TYPE = NAVIGATION_GOAL_NONE;
    NAVIGATION_MODE = NAVIGATION_MODE_AUTONOMOUS;
    return 0;
}

short navigate_forward(short dir) {
    switch (dir) {
        case 0:
            NAVIGATION_GOAL_X += 1;
            break;
        case 1:
            NAVIGATION_GOAL_Y += 1;
            break;
        case 2:
            NAVIGATION_GOAL_X -= 1;
            break;
        case 3:
            NAVIGATION_GOAL_Y -= 1;
            break;
        default:
            return -1;
    }
    NAVIGATION_GOAL_TYPE = NAVIGATION_GOAL_MOVE;
    return 0;
}

short command_set_target_square(short id)
{
    // get current heading, rounded to nearest quarter turn
    // 0 = straight right
    // FULL_TURN / 2 = straight left
    // FULL_TURN / 4 = straight up
    // FULL_TURN * 3 / 4 = straight down
    short dir;

    // right
    if (CURRENT_HEADING < FULL_TURN/8 || CURRENT_HEADING > FULL_TURN*7/8) {
        dir = 0;
    }
    // up
    else if (CURRENT_HEADING < FULL_TURN*3/8) {
        dir = 1;
    }
    // left
    else if (CURRENT_HEADING < FULL_TURN*5/8) {
        dir = 2;
    }
    // down
    else {
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
            NAVIGATION_GOAL_HEADING = ((dir+1) % 4) / 4 * FULL_TURN;
            NAVIGATION_GOAL_TYPE = NAVIGATION_GOAL_TURN;
            return 0;
        case ID_TURN_RIGHT:
            NAVIGATION_GOAL_HEADING = ((dir+3) % 4) / 4 * FULL_TURN;
            NAVIGATION_GOAL_TYPE = NAVIGATION_GOAL_TURN;
            return 0;
        default:
            return -1;
    }
}

int main() {
    return 0;
}
