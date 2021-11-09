#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

int startPosition[];
int islandStartPosition[];
int robotPosition[] = {10, 3};
int endPoint[];

/*
Changed so that the algorithm looks for the end point instead of the 
position of the robot. Same same but backwards.
*/

void sample_search()
{
    int queue[1300][3];
    int adjacentCells[4][3];
    int counter = 0;
    bool adjacentInQueue = false;
    bool endPointInQueue = false;
    int queueIndex = 0;

    queue[0][0] = robotPosition[0];
    queue[0][1] = robotPosition[1];
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
            queueIndex = sizeof(queue);
        }
    }
    move_one_cell(queue);
}

void move_one_cell(int **queue)
{
    int rows = 4;
    int cols = 3;

    //memory allocation

    //int robotPosition[] = {10, 3};
    int **adjacentCells = malloc(rows * sizeof(int *));
    int **traversableCells = malloc(rows * sizeof(int *));

    for (int i = 0; i < rows; i++)
    {
        adjacentCells[i] = malloc(cols * sizeof(int));
        traversableCells[i] = malloc(cols * sizeof(int));
    }

    //Create array of the robot's adjacent cells

    for (int direction = 0; direction < 4; direction++)
    {
        for (int i = 0; i < 3; i++)
        {
            if (i < 2)
            {
                adjacentCells[direction][i] = get_adjacent_cell(direction, i, robotPosition);
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

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
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

    turn_towards_cell();
    if (!wall_in_front)
    {
        go_to_cell(traversableCells[index], queue);
    }
    else
    {
    }

    //The grid's direction is stationary, but the robot is not.
    //How do we know which way the robot is facing and so in which direction it should go?
    //Also, check for wall before attempting driving to the cell. This is an error in the flowchart.
    //go_to_cell();

    //free memory
    for (int i = 0; i < rows; i++)
    {
        free(adjacentCells[i]);
        free(traversableCells[i]);
    }
    free(adjacentCells);
    free(traversableCells);
}

int get_adjacent_cell(int direction, int xy, int *currentCell)
{
    //each case returns the x & y coordinate for the adjacent cell separately.
    //direction == 0: left cell
    //direction == 1: upper cell
    //direction == 2: right cell
    //direction == 3: lower cell

    switch (direction)
    {
    case 0:
        if (xy == 0)
        {
            return currentCell[0] - 1;
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
            return currentCell[1] - 1;
        }
        break;
    case 2:
        if (xy == 0)
        {
            return currentCell[0] + 1;
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
            return currentCell[1] + 1;
        }
        break;
    default:
        return 0;
        break;
    }
}

void turn_towards_cell()
{
}

int wall_in_front()
{
    //if(wall_one_cell_ahead())
    //return true
    //else
    //return false
}

void go_to_cell(int *cell, int **queue)
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
            if (unexplored_cells())
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
}

//For debugging purposes only

void test(int rows, int cols, int **queue)
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
    int rows = 4;
    int cols = 3;
    int **queue = malloc(rows * sizeof(int *));

    for (int i = 0; i < rows; i++)
    {
        queue[i] = malloc(cols * sizeof(int));
    }

    for (int i = 0, kk = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            queue[i][j] = kk;
            kk++;
        }
    }

    //test(rows, cols, queue);
    move_one_cell(queue);

    for (int i = 0; i < rows; i++)
    {
        free(queue[i]);
    }
    free(queue);

    return 0;
}
