#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

void start_exploring()
{
}

int returned_to_start()
{
}

int left_opening()
{
}

int right_opening()
{
}

int wall_in_front()
{
}

void turn_left()
{
}

void turn_right()
{
}

void go_forward()
{
}

void reverse_controls()
{
}

void return_to_start()
{
}

int unexplored_cells()
{
}

void sample_search(int robotCoord[], int endPoint[])
{
    int queue[1000][3];
    int adjacentCells[4][3];
    int counter = 0;
    bool adjacentInQueue = false;
    bool startPointInQueue = false;
    int queueIndex = 0;

    queue[0][0] = endPoint[0];
    queue[0][1] = endPoint[1];
    queue[0][2] = counter;
    counter++;

    while (queueIndex < sizeof(queue))
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
            if (!isWall(adjacentCells[i]))
            {
                for (int j = 0; j < sizeof(queue); j++)
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
                    queueIndex++;
                    queue[queueIndex][0] = adjacentCells[i][0];
                    queue[queueIndex][1] = adjacentCells[i][1];
                    queue[queueIndex][2] = adjacentCells[i][2];
                }
                adjacentInQueue = false;
            }
        }
        for (int i = 0; i < sizeof(queue); i++)
        {
            if (queue[i] == robotCoord)
            {
                startPointInQueue = true;
                break;
            }
        }
        if (!startPointInQueue)
        {
            queueIndex++;
            counter++;
        }
    }
}

int get_adjacent_cell(int direction, int xy, int currentNode[])
{
    /*  each case returns the x & y coordinate for the adjacent cell separately.
    direction == 0: left cell
    direction == 1: upper cell
    direction == 2: right cell
    direction == 3: lower cell */

    switch (direction)
    {
    case 0:
        if (xy == 0)
        {
            return currentNode[0] - 1;
        }
        else
        {
            return currentNode[1];
        }
        break;
    case 1:
        if (xy == 0)
        {
            return currentNode[0];
        }
        else
        {
            return currentNode[1] - 1;
        }
        break;
    case 2:
        if (xy == 0)
        {
            return currentNode[0] + 1;
        }
        else
        {
            return currentNode[1];
        }
        break;
    case 3:
        if (xy == 0)
        {
            return currentNode[0];
        }
        else
        {
            return currentNode[1] + 1;
        }
        break;
    }
}
int is_wall(int cell[])
{
}
int main(void)
{
    sample_search();
}
