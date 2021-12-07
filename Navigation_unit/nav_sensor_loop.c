#include <stdlib.h> //abs
#include <avr/interrupt.h>

#include "../AVR_common/robot.h"
#include "../AVR_common/sensors.h"
#include "../AVR_common/uart.h"
#include "../AVR_testing/test.h"
#include "navigation_unit.h"
#include "pd.h"
#include "navigation.h"
#include "rotation_math.h"
#include "nav_sensor_loop.h"


struct sensor_data sensor_data_0;
struct sensor_data sensor_data_1;

struct sensor_data *next_sensor_data = &sensor_data_0;
struct sensor_data *current_sensor_data = &sensor_data_1;

volatile bool g_SensorDataReady = false;
uint8_t            sensor_count = 0;

bool   arrived_at_goal(void);

int8_t handle_sensor_data(struct data_packet* data)
{

    // check packet count
    if (data->address != ADR_DEBUG)
    {
        if (data->byte_count != ADR_DATA_PACKETS[data->address])
        {
            // Invalid number of data packets
            return -1;
        }
    }
    // Check for parity error?
    // TODO Forward to com unit

    // Save the data
    sensor_count++;
    switch (data->address)
    {
        case LIDAR_FORWARD:
            next_sensor_data->lidar_forward = BYTES_TO_UINT16(data);
            break;
        case LIDAR_BACKWARD:
            next_sensor_data->lidar_backward = BYTES_TO_UINT16(data);
            break;
        case IR_LEFTFRONT:
            next_sensor_data->ir_leftfront = BYTES_TO_UINT16(data);
            break;
        case IR_LEFTBACK:
            next_sensor_data->ir_leftback = BYTES_TO_UINT16(data);
            break;
        case IR_RIGHTFRONT:
            next_sensor_data->ir_rightfront = BYTES_TO_UINT16(data);
            break;
        case IR_RIGHTBACK:
            next_sensor_data->ir_rightback = BYTES_TO_UINT16(data);
            break;
        case ODOMETER:
            next_sensor_data->odometer_left  = data->bytes[0];
            next_sensor_data->odometer_right = data->bytes[1];
            break;
        case GYRO:
            next_sensor_data->gyro = BYTES_TO_UINT16(data);
            break;
        default:
            return -1;
    }




    // Check if we've received all data
    if (sensor_count >= SENSOR_PACKETS && data->address == ODOMETER)
    {
        // copy the data, so we can begin writing the next set of data
        // while still working on it
		if (next_sensor_data == &sensor_data_0)
		{
			next_sensor_data = &sensor_data_1;
			current_sensor_data = &sensor_data_0;
		}
		else
		{
			next_sensor_data = &sensor_data_0;
			current_sensor_data = &sensor_data_1;
		}
        sensor_count = 0;
		g_SensorDataReady = true;
    }

    return 0;
}

void send_sensor_data(struct sensor_data* data)
{
    struct data_packet packet;
    packet.byte_count = 2;

    uint16_t* value = (uint16_t*) data;
    for (int i=0; i < 7; ++i)
    {
        packet.address = i;
        packet.bytes[0] = Uint16ToByte0(*value);
        packet.bytes[1] = Uint16ToByte1(*value);
        Uart_Send_0(&packet);
        ++value;
    }
    packet.address = ODOMETER;
    packet.bytes[0] = data->odometer_left;
    packet.bytes[1] = data->odometer_right;
    Uart_Send_0(&packet);
}

int8_t nav_main(void)
{
    struct sensor_data* data = current_sensor_data;

    // send sensor data to display
    send_sensor_data(data);

    // updates g_currentHeading, g_currentPosX and g_currentPosY
    // sends them to display if updated
    if (calculate_heading_and_position(data) == -1)
    {
        return -1;
    }

    if (g_navigationGoalSet)
    {
        // have we arrived at the navigation goal?
        if (PDcontroller_Update())
        {
            // clear navigation goal
            g_navigationGoalSet = false;
        }
    }


    // run map update algorithm
    // which if there's updates, sends them to com-unit
    update_map(data);

    // Check if we should run nav algo
    if (g_navigationMode == AUTONOMOUS && !g_navigationGoalSet)
    {
        // run navigation algorithm
        // sets g_navigationGoal
        // and calls PDcontroller_NewGoal()
        wall_follow();
    }

    return 0;
}


/* #### UNIT TESTS #### */

// test handle sensor data

#if __TEST__

Test_test(Test, handle_sensor_data_lidar_forward)
{
    // save old values to restore later
    struct sensor_data* old_sensor_data  = next_sensor_data;
    uint8_t            old_sensor_count = sensor_count;

    // set up environment
    sensor_count                   = 0;
    next_sensor_data->lidar_forward = 0;

    // construct parameter
    struct data_packet data;
    data.address    = LIDAR_FORWARD;
    data.byte_count = 2;
    data.bytes[0]   = Uint16ToByte0(2000);
    data.bytes[1]   = Uint16ToByte1(2000);

    // test
    Test_assertEquals(handle_sensor_data(&data), 0);
    Test_assertEquals(sensor_count, 1);
    Test_assertEquals(next_sensor_data->lidar_forward, 2000);

    // restore old values
    next_sensor_data = old_sensor_data;
    sensor_count     = old_sensor_count;
}


Test_test(Test, handle_sensor_data_odometer)
{
    // save old values to restore later
    struct sensor_data* old_sensor_data  = next_sensor_data;
    uint8_t            old_sensor_count = sensor_count;

    // set up environment
    sensor_count                    = 3;
    next_sensor_data->odometer_left  = 0;
    next_sensor_data->odometer_right = 0;

    // construct parameter
    struct data_packet data;
    data.address    = ODOMETER;
    data.byte_count = 2;
    data.bytes[0]   = 5;
    data.bytes[1]   = 6;

    Test_assertEquals(handle_sensor_data(&data), 0);
    Test_assertEquals(sensor_count, 4);
    Test_assertEquals(next_sensor_data->odometer_left, 5);
    Test_assertEquals(next_sensor_data->odometer_right, 6);

    // restore old values
    next_sensor_data = old_sensor_data;
    sensor_count     = old_sensor_count;
}
#endif
