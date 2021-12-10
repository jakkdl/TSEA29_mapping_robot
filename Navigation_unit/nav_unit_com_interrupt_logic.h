#ifndef NAVIGATION_UNIT_NAV_UNIT_COM_INTERRUPT_LOGIC_H_
#define NAVIGATION_UNIT_NAV_UNIT_COM_INTERRUPT_LOGIC_H_

#include "../AVR_common/robot.h"

int8_t communication_unit_interrupt(struct data_packet* data);
int8_t command_set_target_square(uint8_t id);
#endif // NAVIGATION_UNIT_NAV_UNIT_COM_INTERRUPT_LOGIC_H_
