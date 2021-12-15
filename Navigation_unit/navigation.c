#include "navigation.h"
#include <stdlib.h>

uint16_t g_startPosX = 24;
uint16_t g_startPosY = 0;

uint8_t get_robot_adjacent_coord(int direction, int xy);

bool    cell_is_wall(uint8_t cell[COLS]);
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

bool isValidCell(uint16_t x, uint16_t y)
{
    return x < MAP_X_MAX && y < MAP_Y_MAX && !IsWall(x, y);
}

struct Queue
{
    int16_t front, rear, size;
    unsigned capacity;
    int8_t array[MAP_X_MAX*MAP_Y_MAX];
};
struct Queue* createQueue(struct Queue* queue, unsigned capacity)
{
    queue->capacity = capacity;
    queue->front = queue->size = 0;
    queue->rear = capacity - 1;
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
void printQueue(struct Queue* queue)
{
    //purely for academic purposes
    printf("queue:\n");
    for(int i = queue->front; i <= queue->rear; i++)
    {
        if(i%2 == 0)
        {
            printf("x = %d", queue->array[i]);
        }
        else
        {
            printf("y = %d\n", queue->array[i]);
        }
    }
}

void addSquare(int8_t x, int8_t y, struct Queue* queue,
        bool visited[MAP_X_MAX][MAP_Y_MAX])
{
    if (!visited[x][y])
    {
        visited[x][y] = true;
        enqueue(queue, x);
        enqueue(queue, y);
    }
}
bool BFS()
{
    int8_t x = MmToGrid(g_currentPosX);
    int8_t y = MmToGrid(g_currentPosY);
    bool visited[MAP_X_MAX][MAP_Y_MAX] = {0};
    struct Queue queue;
    createQueue(&queue, MAP_Y_MAX*MAP_X_MAX);

    enqueue(&queue, x);
    enqueue(&queue, y);
    while(!isEmpty(&queue))
    {
        //printQueue(queue);
        x = dequeue(&queue);
        y = dequeue(&queue);
        if (isValidCell(x, y))
        {
            if (IsUnknown(x,y))
            {
                return true;
            }
            addSquare(x+1, y, &queue, visited);
            addSquare(x-1, y, &queue, visited);
            addSquare(x, y+1, &queue, visited);
            addSquare(x, y-1, &queue, visited);
        }
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
    save_start_pos();
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

/* TESTS */

Test_test(Test, test_unexplored_cells_exist)
{
    //stdout = &mystdout;
    uint16_t prevPosX = g_currentPosX;
    uint16_t prevPosY = g_currentPosY;
    Test_assertEquals(unexplored_cells_exist(),true);

    MakeWall(24,1);
    MakeWall(25,0);
    MakeWall(23,0);
    MakeEmpty(24,0);
    Test_assertEquals(unexplored_cells_exist(), false);
    MakeUnknown(24,1);

    g_currentPosX = GridToMm(26);
    g_currentPosY = GridToMm(0);
    Test_assertEquals(unexplored_cells_exist(), true);
    MakeUnknown(25,0);

    g_currentPosX = GridToMm(24);
    g_currentPosY = GridToMm(0);
    MakeEmpty(24,1);
    MakeEmpty(24,2);
    MakeEmpty(25,0);
    Test_assertEquals(unexplored_cells_exist(), true);

    MakeWall(25,1);
    MakeWall(26,0);
    MakeWall(25,2);
    MakeWall(23,1);
    MakeWall(23,2);
    MakeWall(24,3);
    Test_assertEquals(unexplored_cells_exist(),false);

    //reset map
    for (int x = 0; x < 49; x++)
    {
        for (int y = 0; y < 25; y++)
        {
            MakeUnknown(x, y);
        }
    }
    g_currentPosX = prevPosX;
    g_currentPosY = prevPosY;
}

Test_test(Test, test_queue)
{
    struct Queue queue;
    createQueue(&queue, 10);
    enqueue(&queue,10);
    Test_assertEquals(dequeue(&queue), 10);
}

Test_test(Test, test_cells_exist)
{
    for (int x = 0; x < 49; x++)
    {
        for (int y = 0; y < 25; y++)
        {
            MakeEmpty(x, y);
        }
    }
    Test_assertEquals(unexplored_cells_exist(), false);
    MakeUnknown(48,24);
    Test_assertEquals(unexplored_cells_exist(), true);
    MakeUnknown(48,23);
    MakeUnknown(47,24);
    MakeWall(48,23);
    MakeWall(47,24);
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
