#ifndef NAVIGATION_UNIT_NAV_UNIT_COM_INTERRUPT_LOGIC_H_
#define NAVIGATION_UNIT_NAV_UNIT_COM_INTERRUPT_LOGIC_H_

#include "../AVR_common/robot.h"

int8_t communication_unit_interrupt(struct data_packet* data);
int8_t handle_command(enum directionID id);
int8_t resend(uint8_t adress);
int8_t set_pd_kd(uint8_t kd);
int8_t set_pd_kp(uint8_t kp);
int8_t command_stop();
int8_t command_start();
int8_t command_set_target_square(uint8_t id);
#endif // NAVIGATION_UNIT_NAV_UNIT_COM_INTERRUPT_LOGIC_H_
