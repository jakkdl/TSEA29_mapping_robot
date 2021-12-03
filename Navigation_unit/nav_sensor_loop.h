#include <stdbool.h>
#ifndef NAVIGATION_UNIT_NAV_SENSOR_LOOP_H_
#define NAVIGATION_UNIT_NAV_SENSOR_LOOP_H_
int8_t handle_sensor_data(struct data_packet* data);
int8_t nav_main(void);

extern volatile bool g_SensorDataReady;
#endif // NAVIGATION_UNIT_NAV_SENSOR_LOOP_H_
