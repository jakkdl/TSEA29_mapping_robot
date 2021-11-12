#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "nav_unit_com_interrupt_logic.h"
#include "navigation_unit.h"
#include "map.h"

void init_map()
{
    for (int row = 0; row < MAP_ROWS; row++)
    {
        for (int col = 0; col < MAP_COLS; col++)
        {
            map[row][col] = unknown_cell;
        }
    }
}

void update_cell()
{
}

bool unexplored_cells_exist()
{
    for (int row = 0; row < MAP_ROWS; row++)
    {
        for (int col = 0; col < MAP_COLS; col++)
        {
            if (map[row][col] == unknown_cell)
            {
                return true;
            }
        }
    }
}