#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "test.h"

int main(void)
{
    // Run tests
    Test_run();

    // This is supposed to quit simavr "since interrupts are off"
    // Does not seem to work.
    sleep_cpu();
}
