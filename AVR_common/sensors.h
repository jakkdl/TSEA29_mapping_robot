#ifndef AVR_COMMON_SENSORS_H_
#define AVR_COMMON_SENSORS_H_

#include <stdint.h>

struct sensor_data
{
    uint16_t lidar_forward;
    uint16_t lidar_backward;
    uint16_t ir_leftfront;
    uint16_t ir_leftback;
    uint16_t ir_rightfront;
    uint16_t ir_rightback;
    int16_t gyro;
    uint8_t odometer_right;
    uint8_t odometer_left;
};

// these don't strictly have to be defined here, but since it's required
// that the data in the arrays are in the same order as in the struct
// I thought it good. /JL
extern const int8_t LASER_POSITION_X[];
extern const int8_t LASER_POSITION_Y[];
extern const int8_t LASER_DIRECTION[];
// check the same byte offset into the array as in the struct
#define AdressInStruct(s, m) (void*)&(s) - (void*)(m)
#define LaserPositionX(sensor_data, sensor) LASER_POSITION_X[AdressInStruct(sensor, sensor_data)]
#define LaserPositionY(sensor_data, sensor) LASER_POSITION_Y[AdressInStruct(sensor, sensor_data)]
#define LaserDirection(sensor_data, sensor) LASER_DIRECTION[AdressInStruct(sensor, sensor_data)]

#define SENSOR_PACKETS 8
// odometers are sent in the same packet

// used to unpack a byte array into a uint16_t value
#define BYTES_TO_UINT16(data) (uint16_t)(data->bytes[1] << 8 | data->bytes[0])

// used to pack a uint16_t value into two bytes
#define Uint16ToByte0(value) ((value) & 0xff)
#define Uint16ToByte1(value) ((value) >> 8)

#endif // AVR_COMMON_SENSORS_H_
