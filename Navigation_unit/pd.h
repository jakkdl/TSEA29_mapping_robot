#ifndef PD_CONTROLLER_H
#define PD_CONTROLLER_H
#include <stdint.h>
#include <stdbool.h>

bool PDcontroller_Update(void);
void PDcontroller_NewGoal(void);
#endif
