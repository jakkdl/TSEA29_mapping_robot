#ifndef _SENSORS_H
#define _SENSORS_H

struct sensor_data
{
    uint16_t lidar_forward;
    uint16_t lidar_backward;
    uint16_t ir_leftfront;
    uint16_t ir_leftback;
    uint16_t ir_rightfront;
    uint16_t ir_rightback;
    uint8_t odometer_left;
    uint8_t odometer_right;
    uint16_t gyro;
};

#define SENSOR_PACKETS 8
// odometers are sent in the same packet

// used to unpack a byte array into a uint16_t value
#define BYTES_TO_UINT16(data) (uint16_t)(data->bytes[1] << 4 | data->bytes[0])

// used to pack a uint16_t value into two bytes
#define UINT16_TO_BYTE_0(value) (value) & 0b1111
#define UINT16_TO_BYTE_1(value) (value) >> 4

#endif //_SENSORS_H
