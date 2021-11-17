#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "nav_unit_com_interrupt_logic.h"
#include "navigation_unit.h"
#include "../AVR_common/robot.h"
#include "navigation.h"
#include "../AVR_testing/test.h"

//for easy iterations through arrays
int queueSize = 0;
int adjacentCellsSize = 0;
int traversableCellsSize = 0;

uint8_t queue[QUEUE_ROWS][COLS];
uint8_t adjacentCells[ROWS_ADJACENT][COLS];
uint8_t traversableCells[ROWS_ADJACENT][COLS];
uint8_t endPoint[COORD_SIZE];
uint16_t startPosX;
uint16_t startPosY;

void sample_search();
uint8_t get_robot_adjacent_coord(int direction, int xy);
uint8_t get_adjacent_cell(int direction, int xy, uint8_t *currentCell);
bool cell_is_wall(uint8_t cell[COLS]); //Confusing function names at the moment. Will fix.
bool is_wall(uint8_t dir);
void move_one_cell(uint8_t queue[QUEUE_ROWS][COLS]);
bool left_opening();
bool right_opening();
bool wall_in_front();
bool is_wall(uint8_t dir);
uint8_t get_heading();
bool at_start_pos();
void save_start_pos();

/*
Changed so that the algorithm looks for the end point instead of the 
position of the robot. Same same but backwards.
*/

uint16_t get_start_pos(int xy)
{
    if (xy == 0)
    {
        return startPosX;
    }
    else
    {
        return startPosY;
    }
}

void save_start_pos()
{
    startPosX = MmToGrid(g_currentPosX);
    startPosY = MmToGrid(g_currentPosY);
}

void init_map()
{
    for (int x = 0; x < 49; x++)
    {
        for (int y = 0; y < 25; y++)
        {
            g_navigationMap[x][y] = 0;
        }
    }
}

bool unexplored_cells_exist()
{
    for (int x = 0; x < 49; x++)
    {
        for (int y = 0; y < 25; y++)
        {
            if (g_navigationMap[x][y] == 0)
            {
                endPoint[0] = x;
                endPoint[1] = y;
                return true;
            }
        }
    }
    return false;
}
/*
bool opening()
{
}
*/
bool left_opening()
{
    uint8_t dir = get_heading();

    switch (dir)
    {
    case 0:
        return !is_wall(dir + 1);
    case 1:
        return !is_wall(dir + 1);
    case 2:
        return !is_wall(dir + 1);
    case 3:
        return !is_wall(dir - 3);
    default:
        return -1;
    }
}

//redundant
bool right_opening()
{
    uint8_t dir = get_heading();
    return !is_wall(dir + 3);
}

//checks if cell at g_navigationMap[x][y] is a wall
bool is_wall(uint8_t dir)
{
    int x;
    int y;

    x = get_robot_adjacent_coord(dir % 4, 0);
    y = get_robot_adjacent_coord(dir % 4, 1);
    return g_navigationMap[x][y] == 1;
}

uint8_t get_heading()
{

    // right
    if (g_currentHeading < FULL_TURN / 8 || g_currentHeading > FULL_TURN * 7 / 8)
    {
        return 0;
    }
    // up
    else if (g_currentHeading < FULL_TURN * 3 / 8)
    {
        return 1;
    }
    // left
    else if (g_currentHeading < FULL_TURN * 5 / 8)
    {
        return 2;
    }
    // down
    else
    {
        return 3;
    }
}

bool at_start_pos()
{
    if (MmToGrid(g_currentPosX) != startPosX && MmToGrid(g_currentPosY) != startPosY)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void wall_follow()
{
    uint8_t dir = get_heading();

    if (left_opening())
    {
        command_set_target_square(FW_LEFT);
    }
    else
    {
        //wall in front of robot?
        if (is_wall(get_heading()))
        {
            //previously right_opening()
            if (!is_wall(dir + 3))
            {
                command_set_target_square(FW_RIGHT);
            }
            else
            {
                //Turn around 180 degrees (this was "reverse controls" earlier)
                command_set_target_square(TURN_AROUND);
            }
        }
        else
        {
            command_set_target_square(FORWARD);
        }
    }

    if (unexplored_cells_exist())
    {
        //sample_search();
    }
}

//Path finding algorithm that might not work as expected. Can probably be replaced easily if so.
/* void sample_search()
{
    int counter = 0;
    bool adjacentInQueue = false;
    bool endPointInQueue = false;
    int queueIndex = 0;

    queue[0][0] = MmToGrid(g_currentPosX);
    queue[0][1] = MmToGrid(g_currentPosY);
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
} */

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
uint8_t get_robot_adjacent_coord(int dir, int xy)
{
    //each case returns the x & y coordinate for the adjacent cell separately.
    //direction == 0: right cell
    //direction == 1: upper cell
    //direction == 2: left cell
    //direction == 3: lower cell

    switch (dir)
    {
    case 0:
        if (xy == 0)
        {
            return MmToGrid(g_currentPosX) + 1;
        }
        else
        {
            return MmToGrid(g_currentPosY);
        }
        break;
    case 1:
        if (xy == 0)
        {
            return MmToGrid(g_currentPosX);
        }
        else
        {
            return MmToGrid(g_currentPosY) + 1;
        }
        break;
    case 2:
        if (xy == 0)
        {
            return MmToGrid(g_currentPosX) - 1;
        }
        else
        {
            return MmToGrid(g_currentPosY);
        }
        break;
    case 3:
        if (xy == 0)
        {
            return MmToGrid(g_currentPosX);
        }
        else
        {
            return MmToGrid(g_currentPosY) - 1;
        }
        break;
    default:
        return 0;
        break;
    }
}

/* bool cell_is_wall(uint8_t cell[COLS])
{
    if(){
        false;
    }
    else{
        return true;
    } 
    return false;
} */

/* void move_one_cell(uint8_t queue[QUEUE_ROWS][COLS])
{
    //Create array of the robot's adjacent cells

    for (int direction = 0; direction < 4; direction++)
    {
        for (int i = 0; i < 3; i++)
        {
            if (i < 2)
            {
                adjacentCells[direction][i] = get_robot_adjacent_coord(direction, i);
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

    turn_towards_cell();
    if (!wall_in_front)
    {
        go_to_cell(traversableCells[index], queue);
    }
    else
    {
    } 
} */

/* void turn_towards_cell()
{
} */

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

/* TESTS */

Test_test(Test, test_unexplored_cells_exist)
{
    Test_assertTrue(unexplored_cells_exist());
}

Test_test(Test, test_cells_exist)
{
    for (int x = 0; x < 49; x++)
    {
        for (int y = 0; y < 25; y++)
        {
            g_navigationMap[x][y] = 1;
        }
    }
    Test_assertEquals(unexplored_cells_exist(), false);
    g_navigationMap[0][1] = 0;
    Test_assertEquals(unexplored_cells_exist(), true);
    //reset map
    for (int x = 0; x < 49; x++)
    {
        for (int y = 0; y < 25; y++)
        {
            g_navigationMap[x][y] = 0;
        }
    }
}

//write more specific tests. The robot starts in (24, 0)
Test_test(Test, test_is_wall)
{
    /* for (int x = 0; x < 49; x++)
    {
        for (int y = 0; y < 25; y++)
        {
            g_navigationMap[x][y] = 1;
        }
    } */
    g_navigationMap[25][0] = 1;
    Test_assertEquals(is_wall(0), true);
    Test_assertEquals(is_wall(1), false);
    Test_assertEquals(is_wall(2), false);
    Test_assertEquals(is_wall(3), false);
    for (int x = 0; x < 49; x++)
    {
        for (int y = 0; y < 25; y++)
        {
            g_navigationMap[x][y] = 0;
        }
    }
}

/* Test_test(Test, test_left_opening)
{
}

Test_test(Test, test_right_opening)
{
} */
