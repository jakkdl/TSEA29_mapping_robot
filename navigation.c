#include <stdio.h>
#include <stdlib.h>

typedef struct
{
    int *array;
    size_t used;
    size_t size;
} Array;

void initArray(Array *a, size_t initialSize)
{
    a->array = malloc(initialSize * sizeof(int));
    a->used = 0;
    a->size = initialSize;
}

void insertArray(Array *a, int element)
{
    // a->used is the number of used entries, because a->array[a->used++] updates a->used only *after* the array has been accessed.
    // Therefore a->used can go up to a->size
    if (a->used == a->size)
    {
        a->size *= 2;
        a->array = realloc(a->array, a->size * sizeof(int));
    }
    a->array[a->used++] = element;
}

void freeArray(Array *a)
{
    free(a->array);
    a->array = NULL;
    a->used = a->size = 0;
}

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

void sample_search(int endPoint)
{
    int counter = 0;

    Array a;
    int i;

    initArray(&a, 5);
}
