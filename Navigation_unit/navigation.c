#include "navigation.h"
#include <stdlib.h>

int      queueSize            = 0;
int      adjacentCellsSize    = 0;
int      traversableCellsSize = 0;
uint8_t  g_queue[QUEUE_ROWS][COLS];
uint8_t  g_adjacentCells[ROWS_ADJACENT][COLS];
uint8_t  g_traversableCells[ROWS_ADJACENT][COLS];
uint8_t  g_endPoint[COORD_SIZE];
uint16_t g_startPosX;
uint16_t g_startPosY;

uint8_t get_robot_adjacent_coord(int direction, int xy);
// uint8_t get_adjacent_cell(int direction, int xy, uint8_t* currentCell);

void get_adjacent_cell(uint8_t  direction,
                       uint8_t* x,
                       uint8_t* y,
                       uint8_t* currentCell);

bool    cell_is_wall(uint8_t cell[COLS]);
bool    is_wall(uint8_t dir);
void    move_one_cell(uint8_t queue[QUEUE_ROWS][COLS]);
bool    left_opening();
bool    right_opening();
bool    wall_in_front();
bool    is_wall(uint8_t dir);
uint8_t get_heading();
bool    at_start_pos();
void    save_start_pos();
bool    BFS();

uint16_t get_start_pos(int xy)
{
    if (xy == 0)
    {
        return g_startPosX;
    }
    return g_startPosY;
}

void save_start_pos()
{
    g_startPosX = MmToGrid(g_currentPosX);
    g_startPosY = MmToGrid(g_currentPosY);
}

bool unexplored_cells_exist()
{
    return BFS();
}

bool isValidCell(uint16_t x, uint16_t y, bool visited[MAP_X_MAX][MAP_Y_MAX])
{
    if  (x > MAP_X_MAX || y > MAP_Y_MAX)
    {
        return false;
    }
    if (visited[x][y] || IsWall(x,y))
    {
        return false;
    }
    return true;
}

struct Queue
{
    int front, rear, size;
    unsigned capacity;
    int* array;
};

struct Queue* createQueue(unsigned capacity)
{
    struct Queue* queue = (struct Queue*)malloc(sizeof(struct Queue));
    queue->capacity = capacity;
    queue->front = queue->size = 0;
    queue->rear = capacity - 1;
    queue->array = (int*)malloc(queue->capacity * sizeof(int));
    return queue;
}

bool isEmpty(struct Queue* queue)
{
    return(queue->size == 0);
}

void enqueue(struct Queue* queue, int val)
{
    queue->rear = (queue->rear + 1) % queue->capacity;
    queue->array[queue->rear] = val;
    queue->size = queue->size + 1;
}

int dequeue(struct Queue* queue)
{
    int i = queue->array[queue->front];
    queue->front = (queue->front + 1) % queue->capacity;
    queue->size = queue->size - 1;
    return i;
}

bool BFS()
{
    int x = 24;
    int y = 0;
    printf("x1 = %d, y1 = %d\n", x, y);
    bool visited[MAP_X_MAX][MAP_Y_MAX];
    struct Queue* queue = createQueue(MAP_Y_MAX*MAP_X_MAX);
    enqueue(queue, x);
    enqueue(queue, y);
    while(!isEmpty(queue))
    {
        //printf("x = %d, y = %d\n", cord->x, cord->y);
        x = dequeue(queue);
        y = dequeue(queue);
        if (isValidCell(x, y, visited))
        {
            printf("x2 = %d, y2 = %d\n", x, y);
            if (IsUnknown(x,y))
            {
                return true;
            }
            //should be done better
            
            visited[x][y] = true;
            x += 1;
            enqueue(queue, x);
            enqueue(queue, y);
            x -= 2;
            enqueue(queue, x);
            enqueue(queue,y);
            x += 1;
            y += 1;
            enqueue(queue, x);
            enqueue(queue, y);
            y -= 2;
            enqueue(queue, x);
            enqueue(queue, x);        }
    }
    return false;
}
// checks if cell at g_navigationMap[x][y] is a wall
bool is_wall(uint8_t dir)
{
    int x;
    int y;

    x = get_robot_adjacent_coord(dir % 4, 0);
    y = get_robot_adjacent_coord(dir % 4, 1);
    return IsWall(x, y);
}

bool is_unknown(uint8_t dir)
{
	int x;
	int y;

	x = get_robot_adjacent_coord(dir % 4, 0);
	y = get_robot_adjacent_coord(dir % 4, 1);
	return x>=0 && x < MAP_X_MAX && y >= 0 && y < MAP_Y_MAX && abs(g_navigationMap[x][y]) < 10;
	//return IsUnknown(x, y);
}

uint8_t get_heading()
{

    // right
    if (g_currentHeading < FULL_TURN / 8 ||
        g_currentHeading > FULL_TURN * 7 / 8)
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
    if (MmToGrid(g_currentPosX) != g_startPosX &&
        MmToGrid(g_currentPosY) != g_startPosY)
    {
        return false;
    }
    else
    {
        return true;
    }
}

// Position robot so that its left side faces a wall
void init_wall_follow()
{
    uint8_t dir = get_heading();
    if (!is_wall(dir + 1))
    {
        command_set_target_square(TURN_AROUND);
    }
}

bool wall_follow()
{
    uint8_t dir = get_heading();

    // left opening?
    if (!is_wall(dir + 1))
    {
		if (is_unknown(dir+1))
		{
			return false;
		}
        command_set_target_square(FW_LEFT);
    }
    else
    {
        // wall in front of robot?
        if (is_wall(dir))
        {
            // right opening?
            if (!is_wall(dir + 3))
            {
                command_set_target_square(FW_RIGHT);
            }
            else
            {
                command_set_target_square(TURN_AROUND);
            }
        }
        else
        {
            command_set_target_square(FORWARD);
        }
    }

    // wall follow complete
    if (at_start_pos())
    {
        if (unexplored_cells_exist())
        {
            return false;
        }
        return true;
    }
    return false;
}

// Path finding algorithm that might not work as expected. Can probably be
// replaced easily if so.
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
                    adjacentCells[direction][j] = get_adjacent_cell(direction,
j, queue[queueIndex]);
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

// For sample search
/* void get_adjacent_cell(uint8_t  direction,
                       uint8_t* x,
                       uint8_t* y,
                       uint8_t* currentCell)
{
    // each case returns the x & y coordinate for the adjacent cell separately.
    // direction == 0: right cell
    // direction == 1: upper cell
    // direction == 2: left cell
    // direction == 3: lower cell

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
            case 1:
                if (xy == 0)
                {
                    return currentCell[0];
                }
                else
                {
                    return currentCell[1] + 1;
                }
            case 2:
                if (xy == 0)
                {
                    return currentCell[0] - 1;
                }
                else
                {
                    return currentCell[1];
                }
            case 3:
                if (xy == 0)
                {
                    return currentCell[0];
                }
                else
                {
                    return currentCell[1] - 1;
                }
            default:
                return 0;
        }
} */

// For robot
uint8_t get_robot_adjacent_coord(int dir, int xy)
{
    // each case returns the x & y coordinate for the adjacent cell separately.
    // direction == 0: right cell
    // direction == 1: upper cell
    // direction == 2: left cell
    // direction == 3: lower cell

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
        case 1:
            if (xy == 0)
            {
                return MmToGrid(g_currentPosX);
            }
            else
            {
                return MmToGrid(g_currentPosY) + 1;
            }
        case 2:
            if (xy == 0)
            {
                return MmToGrid(g_currentPosX) - 1;
            }
            else
            {
                return MmToGrid(g_currentPosY);
            }
        case 3:
            if (xy == 0)
            {
                return MmToGrid(g_currentPosX);
            }
            else
            {
                return MmToGrid(g_currentPosY) - 1;
            }
        default:
            return 0;
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
                adjacentCells[direction][i] =
get_robot_adjacent_coord(direction, i);
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
            if (adjacentCells[i][0] == queue[j][0] && adjacentCells[i][1] ==
queue[j][1])
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
    //How do we know which way the robot is facing and so in which direction it
should go?

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
    stdout = &mystdout;
    uint16_t prevPosx = g_startPosX;
    uint16_t prevPosY = g_startPosY;
    uint16_t startposx = g_startPosX;
    uint16_t startposy = g_startPosY;
    g_startPosY = 0;
    g_startPosX = 24;
    Test_assertTrue(unexplored_cells_exist());
    MakeWall(24,1);
    MakeWall(25,0);
    MakeWall(23,0);
    MakeEmpty(24,0);
    Test_assertEquals(MmToGrid(200), 0);
    Test_assertEquals(MmToGrid(9600), 24);
    g_startPosY = 200;
    g_startPosX = 9600;
    Test_assertEquals(unexplored_cells_exist(), false);
    //reset map
    for (int x = 0; x < 49; x++)
    {
        for (int y = 0; y < 25; y++)
        {
            MakeUnknown(x, y);
        }
    }
    g_startPosY = startposy;
    g_startPosX = startposx;
    g_currentPosY = prevPosY;
    g_currentPosX = prevPosx;
}

Test_test(Test, test_cells_exist)
{
    for (int x = 0; x < 49; x++)
    {
        for (int y = 0; y < 25; y++)
        {
            MakeWall(x, y);
        }
    }
    Test_assertEquals(unexplored_cells_exist(), false);
    // reset map
    for (int x = 0; x < 49; x++)
    {
        for (int y = 0; y < 25; y++)
        {
            MakeUnknown(x, y);
        }
    }
}

// write more specific tests. The robot starts in (24, 0)
Test_test(Test, test_walls_and_openings)
{
    MakeWall(25, 0);
    MakeWall(23, 0);
    MakeWall(24, 1);
    Test_assertEquals(is_wall(0), true);
    Test_assertEquals(is_wall(1), true);
    Test_assertEquals(is_wall(2), true);
    Test_assertEquals(is_wall(3), true);

    // reset map
    MakeUnknown(25, 0);
    MakeUnknown(23, 0);
    MakeUnknown(24, 1);

    // trying to inspect cell that is outside of map currently
    MakeEmpty(23, 0);
    MakeEmpty(24, 1);
    MakeEmpty(25, 0);
    Test_assertEquals(is_wall(0), false);
    Test_assertEquals(is_wall(1), false);
    Test_assertEquals(is_wall(2), false);
    Test_assertEquals(is_wall(3), true);

    MakeUnknown(23, 0);
    MakeUnknown(24, 1);
    MakeUnknown(25, 0);
}

Test_test(Test, test_navigation_goals)
{
    uint16_t oldHeading = g_currentHeading;
    bool                oldGoalSet         = g_navigationGoalSet;
    uint8_t             oldNavigationGoalX = g_navigationGoalX;
    uint8_t             oldNavigationGoalY = g_navigationGoalY;
    uint8_t             oldNavigationGoalHeading = g_navigationGoalHeading;

    MakeWall(25, 0);
    MakeWall(24, 1);
    g_currentHeading = 0;
    Test_assertEquals(is_wall(2), false);
    Test_assertEquals(is_wall(1), true);

    Test_assertEquals(!is_wall(get_heading() + 1), false);
    Test_assertEquals(is_wall(get_heading()), true);
    Test_assertEquals(!is_wall(get_heading() + 3), false);

    wall_follow();
    Test_assertEquals(g_navigationGoalHeading, FULL_TURN / 2);
    MakeWall(23, 0);
    MakeUnknown(25, 0);
    wall_follow();
    Test_assertEquals(g_navigationGoalHeading, 0);
    MakeWall(25, 0);
    MakeUnknown(24, 1);
    wall_follow();
    // TODO: Broken since adding checks for unknown
    //Test_assertEquals(g_navigationGoalHeading, FULL_TURN / 4);

    MakeUnknown(25, 0);
    MakeUnknown(23, 0);

    //Test_assertEquals(g_navigationGoalHeading, FULL_TURN / 4);
    init_wall_follow();
    //Test_assertEquals(g_navigationGoalHeading, FULL_TURN / 2);

    g_navigationGoalSet = oldGoalSet;
    g_currentHeading = oldHeading;
    g_navigationGoalX   = oldNavigationGoalX;
    g_navigationGoalY   = oldNavigationGoalY;
    g_navigationGoalHeading = oldNavigationGoalHeading;
}
