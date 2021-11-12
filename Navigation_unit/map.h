#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

int const MAP_ROWS = 49;
int const MAP_COLS = 25;
int unknown_cell = 0;
uint8_t map[MAP_ROWS][MAP_COLS];
void init_map();
void update_cell();
bool unexplored_cells_exist();