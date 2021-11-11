#ifndef NAV_UNIT_COM_INTERRUPT_LOGIC_H
#define NAV_UNIT_COM_INTERRUPT_LOGIC_H

#include "../AVR_common/robot.h"
struct Com_packet
{
    enum Address address;
    uint8_t packet_count;
    uint8_t data_packets[7];
};

uint8_t handle_command(enum directionID id);
uint8_t resend(uint8_t adress);
uint8_t set_pd_kd(uint8_t kd);
uint8_t set_pd_kp(uint8_t kp);
uint8_t command_stop();
uint8_t command_start();
uint8_t command_set_target_square(uint8_t id);
#endif
