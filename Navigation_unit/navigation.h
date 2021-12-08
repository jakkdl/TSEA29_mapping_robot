#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "../AVR_common/robot.h"
#include "../AVR_testing/test.h"
#include "nav_unit_com_interrupt_logic.h"
#include "navigation_unit.h"

#define QUEUE_ROWS 1000
#define COLS 3
#define ROWS_ADJACENT 4
#define COORD_SIZE 2

void init_wall_follow();
bool wall_follow();
void sample_search();

#endif
