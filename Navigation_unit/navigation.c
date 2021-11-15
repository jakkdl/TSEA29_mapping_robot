#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
//#include "nav_unit_com_interrupt_logic.h"
//#include "navigation_unit.h"
//#include "map.h"
#include "nav_unit_com_interrupt_logic.c"
#include "map.c"

int const QUEUE_ROWS = 1000;
int const COLS = 3;
int const ROWS_ADJACENT = 4;
int const COORD_SIZE = 2;
int queueSize = 0; //for easy iterations through arrays
int adjacentCellsSize = 0;

int startPosition[COORD_SIZE];
//int endPoint[COORD_SIZE];
int islandStartPosition[COORD_SIZE];

uint8_t queue[QUEUE_ROWS][COLS];
uint8_t adjacentCells[ROWS_ADJACENT][COLS];
uint8_t traversableCells[ROWS_ADJACENT][COLS];

int main(void);
void wall_follow();
void sample_search();
uint8_t get_robot_adjacent_cell(int direction, int xy);
uint8_t get_adjacent_cell(int direction, int xy, uint8_t *currentCell);
bool cell_is_wall(uint8_t cell[COLS]);
void move_one_cell(uint8_t queue[QUEUE_ROWS][COLS]);
bool left_opening();
bool right_opening();
void reverse_controls();
bool wall_in_front();
bool reverseControls = false;

/*
Changed so that the algorithm looks for the end point instead of the 
position of the robot. Same same but backwards.
*/

bool left_opening()
{
    //get some sensordata
    return true;
}
bool right_opening()
{
    //get some sensordata
    return true;
}

bool wall_in_front()
{
    return false;
}

void wall_follow()
{
    //Save start position somehow. This is temporary
    while (mm_to_grid(currentPosX) != startPosition[0] && mm_to_grid(currentPosY) != startPosition[1])
    {
        if (left_opening())
        {
            command_set_target_square(fw_left);
        }
        else
        {
            if (wall_in_front())
            {
                if (right_opening())
                {
                    command_set_target_square(fw_right);
                }
                else
                {
                    //Turn around 180 degrees (this was "reverse controls" earlier)
                    for (int i = 0; i < 2; i++)
                    {
                        command_set_target_square(turn_right);
                    }
                }
            }
            else
            {
                command_set_target_square(forward);
            }
        }
    }
    if (unexplored_cells_exist())
    {
        sample_search(map[cellToExplore]);
    }
}

//Path finding algorithm that might not work as expected. Can probably be replaced easily if so.
void sample_search(uint8_t endPoint[2])
{
    int counter = 0;
    bool adjacentInQueue = false;
    bool endPointInQueue = false;
    int queueIndex = 0;

    queue[0][0] = mm_to_grid(currentPosX);
    queue[0][1] = mm_to_grid(currentPosY);
    queue[0][2] = counter;
    counter++;
    queueSize++;

    while (queueIndex < queueSize)
    {
        for (int direction = 0; direction < 4; direction++)
        {
            for (int j = 0; j < 3; j++)
            {
                if (j < 2)
                {
                    adjacentCells[direction][j] = get_adjacent_cell(direction, j, queue[queueIndex]);
                }
                else
                {
                    adjacentCells[direction][j] = counter;
                }
            }
        }
        for (int i = 0; i < 4; i++)
        {
            if (!cell_is_wall(adjacentCells[i]))
            {
                for (int j = 0; j < queueSize; j++)
                {
                    //Check if the queue contains the adjacent cell
                    if (adjacentCells[i] == queue[j])
                    {
                        adjacentInQueue = true;
                        break;
                    }
                }
                if (!adjacentInQueue)
                {
                    //Add the cells to the queue
                    queueIndex++;
                    queueSize++;
                    queue[queueIndex][0] = adjacentCells[i][0];
                    queue[queueIndex][1] = adjacentCells[i][1];
                    queue[queueIndex][2] = adjacentCells[i][2];
                    adjacentInQueue = false;
                }
            }
        }
        for (int i = 0; i < queueSize; i++)
        {
            if (queue[i] == endPoint)
            {
                endPointInQueue = true;
                break;
            }
        }
        if (!endPointInQueue)
        {
            queueIndex++;
            counter++;
        }
        else
        {
            //break the loop, a path has been found
            endPointInQueue = false;
            queueIndex = queueSize;
        }
    }
    move_one_cell(queue);
}

//For sample search
uint8_t get_adjacent_cell(int direction, int xy, uint8_t *currentCell)
{
    //each case returns the x & y coordinate for the adjacent cell separately.
    //direction == 0: right cell
    //direction == 1: upper cell
    //direction == 2: left cell
    //direction == 3: lower cell

    switch (direction)
    {
    case 0:
        if (xy == 0)
        {
            return currentCell[0] + 1;
        }
        else
        {
            return currentCell[1];
        }
        break;
    case 1:
        if (xy == 0)
        {
            return currentCell[0];
        }
        else
        {
            return currentCell[1] + 1;
        }
        break;
    case 2:
        if (xy == 0)
        {
            return currentCell[0] - 1;
        }
        else
        {
            return currentCell[1];
        }
        break;
    case 3:
        if (xy == 0)
        {
            return currentCell[0];
        }
        else
        {
            return currentCell[1] - 1;
        }
        break;
    default:
        return 0;
        break;
    }
}

//For robot
uint8_t get_robot_adjacent_cell(int direction, int xy)
{
    //each case returns the x & y coordinate for the adjacent cell separately.
    //direction == 0: right cell
    //direction == 1: upper cell
    //direction == 2: left cell
    //direction == 3: lower cell

    switch (direction)
    {
    case 0:
        if (xy == 0)
        {
            return mm_to_grid(currentPosX) + 1;
        }
        else
        {
            return mm_to_grid(currentPosY);
        }
        break;
    case 1:
        if (xy == 0)
        {
            return mm_to_grid(currentPosX);
        }
        else
        {
            return mm_to_grid(currentPosY) + 1;
        }
        break;
    case 2:
        if (xy == 0)
        {
            return mm_to_grid(currentPosX) - 1;
        }
        else
        {
            return mm_to_grid(currentPosY);
        }
        break;
    case 3:
        if (xy == 0)
        {
            return mm_to_grid(currentPosX);
        }
        else
        {
            return mm_to_grid(currentPosY) - 1;
        }
        break;
    default:
        return 0;
        break;
    }
}

bool cell_is_wall(uint8_t cell[COLS])
{
    /*     if(){
        false;
    }
    else{
        return true;
    } */
    return false;
}

void move_one_cell(uint8_t queue[QUEUE_ROWS][COLS])
{
    //Create array of the robot's adjacent cells

    for (int direction = 0; direction < 4; direction++)
    {
        for (int i = 0; i < 3; i++)
        {
            if (i < 2)
            {
                adjacentCells[direction][i] = get_robot_adjacent_cell(direction, i);
            }
            else
            {
                adjacentCells[direction][i] = 0;
            }
        }
    }

    //Compare the robot's adjacent cells to those in the queue.
    //If one or more of them exist in the queue, add them to the array of
    //traversable cells.

    for (int i = 0; i < ROWS_ADJACENT; i++)
    {
        for (int j = 0; j < queueSize; j++)
        {
            if (adjacentCells[i][0] == queue[j][0] && adjacentCells[i][1] == queue[j][1])
            {
                traversableCells[i][0] = queue[j][0];
                traversableCells[i][1] = queue[j][1];
                traversableCells[i][2] = queue[j][2];
            }
        }
    }

    //Pick the traversable cell with the lowest counter number
    int temp = 1000;
    int index;
    for (int i = 0; i < sizeof(traversableCells); i++)
    {
        if (traversableCells[i][2] < temp && traversableCells[i][2] != 0)
        {
            temp = traversableCells[i][2];
            index = i;
        }
    }

    //The grid's direction is stationary, but the robot is not.
    //How do we know which way the robot is facing and so in which direction it should go?

    /*     turn_towards_cell();
    if (!wall_in_front)
    {
        go_to_cell(traversableCells[index], queue);
    }
    else
    {
    } */
}

void turn_towards_cell()
{
}

/* int wall_in_front()
{
    //if(wall_one_cell_ahead())
    //return true
    //else
    //return false
} */

/* void go_to_cell(int *cell, int **queue)
{
    //update robotCoord
    if (robotPosition == endPoint)
    {
        if (robotPosition == islandStartPosition)
        {
            //return to initial starting position
        }
        else
        {
            if (unexplored_cells_exist())
            {
                sample_search();
            }
            else
            {
                //return to initial starting position
            }
        }
    }
    else
    {
        move_one_cell(queue);
    }
} */

//For debugging purposes only

void test(int rows, int cols, int queue[rows][cols])
{
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            printf("%d", queue[i][j]);
        }
        printf("\n");
    }
}

int main(void)
{
    for (int i = 0, kk = 0; i < QUEUE_ROWS; i++)
    {
        for (int j = 0; j < COLS; j++)
        {
            queue[i][j] = kk;
            kk++;
        }
    }

    //test(rows, cols, queue);
    move_one_cell(queue);

    return 0;
}
