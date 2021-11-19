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

void wall_follow();

// for easy iterations through arrays
int queueSize            = 0;
int adjacentCellsSize    = 0;
int traversableCellsSize = 0;

uint8_t  queue[QUEUE_ROWS][COLS];
uint8_t  adjacentCells[ROWS_ADJACENT][COLS];
uint8_t  traversableCells[ROWS_ADJACENT][COLS];
uint8_t  endPoint[COORD_SIZE];
uint16_t startPosX;
uint16_t startPosY;

void    sample_search();
uint8_t get_robot_adjacent_coord(int direction, int xy);
uint8_t get_adjacent_cell(int direction, int xy, uint8_t* currentCell);
bool    cell_is_wall(
     uint8_t cell[COLS]); // Confusing function names at the moment. Will fix.
bool    is_wall(uint8_t dir);
void    move_one_cell(uint8_t queue[QUEUE_ROWS][COLS]);
bool    left_opening();
bool    right_opening();
bool    wall_in_front();
bool    is_wall(uint8_t dir);
uint8_t get_heading();
bool    at_start_pos();
void    save_start_pos();

#endif
