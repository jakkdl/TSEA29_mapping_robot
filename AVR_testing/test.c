#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../AVR_common/uart.h"
#include "test.h"

/*#ifndef F_CPU
#define F_CPU 8000000
#endif
#include "simavr/simavr/sim/avr/avr_mcu_section.h"
AVR_MCU(F_CPU, "atmega1284p");
*/
Test_TestHolder* m_Test_head;

Test_TestHolder* m_Test_activeTest;

typedef struct Test_ResultType
{
    uint16_t totalTests;
    uint16_t successCount;
    uint16_t failureCount;
} Test_ResultType;

Test_ResultType m_Test_result;

// This crashes simavr, and I have no clue why, so we can't get
/*
   const struct avr_mmcu_vcd_trace_t _mytrace[]  _MMCU_ = {
   { AVR_MCU_VCD_SYMBOL("UDR0"), .what = (void*)&UDR0, },
   { AVR_MCU_VCD_SYMBOL("UDRE0"), .mask = (1 << UDRE0), .what = (void*)&UCSR0A,
   },
   };*/

static int uart_putchar(char c, FILE* stream)
{
    if (c == '\n')
        uart_putchar('\r', stream);
    // included with avr/io.h
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    return 0;
}

// define a FILE to be used by custom printf
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
// Initialise the test framework
void Test_init(void)
{
    m_Test_head       = NULL;
    m_Test_activeTest = NULL;

    m_Test_result.totalTests   = 0;
    m_Test_result.successCount = 0;
    m_Test_result.failureCount = 0;
}

void Test_add(Test_TestHolder* test)
{
    // Put to front of chain
    test->next  = m_Test_head;
    m_Test_head = test;
}

void Test_assertTrueLog(uint8_t condition, uint16_t lineNumber)
{
    // We have the active test
    if (!(condition))
    {
        m_Test_activeTest->testResult = FAILURE;
        m_Test_activeTest->line       = lineNumber;
        m_Test_activeTest->actual     = 1;
        m_Test_activeTest->expected   = 0;
    }
}

void Test_assertEqualLog(uint16_t actual,
                         uint16_t expected,
                         uint16_t lineNumber)
{
    if (expected != actual)
    {
        m_Test_activeTest->testResult = FAILURE;
        m_Test_activeTest->line       = lineNumber;
        m_Test_activeTest->actual     = actual;
        m_Test_activeTest->expected   = expected;
    }
}

// Run through all the tests
void Test_runall(void)
{
    // Print to UART, which simavr for some reason
    // prints?
    stdout = &mystdout;

    // Reset counts
    m_Test_result.totalTests   = 0;
    m_Test_result.successCount = 0;
    m_Test_result.failureCount = 0;

    // Reset status of all
    m_Test_activeTest          = m_Test_head;
    Test_TestHolder* prev_test = NULL;
    Test_TestHolder* curr_test = NULL;
    Test_TestHolder* next_test = NULL;
    while (m_Test_activeTest != NULL)
    {
        m_Test_result.totalTests++;

        m_Test_activeTest->testResult = NOT_RUN;
        m_Test_activeTest->line       = 0;

        // next in the chain
        // and reverse order
        curr_test               = m_Test_activeTest;
        next_test               = m_Test_activeTest->next;
        m_Test_activeTest->next = prev_test;
        m_Test_activeTest       = next_test;
        prev_test               = curr_test;

        m_Test_head = prev_test;
    }

    // Now execute the tests
    m_Test_activeTest = m_Test_head;
    while (m_Test_activeTest != NULL)
    {
        m_Test_activeTest->testFunction();

        if (m_Test_activeTest->testResult == NOT_RUN)
        {
            m_Test_activeTest->testResult = SUCCESS;
            m_Test_result.successCount++;
        }
        else
        {
            printf("FAIL: %s @ %d\n    got %d expected %d\n",
                   m_Test_activeTest->name,
                   m_Test_activeTest->line,
                   m_Test_activeTest->actual,
                   m_Test_activeTest->expected);
            m_Test_result.failureCount++;
        }

        __asm__("nop");

        // next in the chain
        m_Test_activeTest = m_Test_activeTest->next;
    }

    // Get the results
    __asm__("nop");
    /*UART_Init(0);
    UART_Transmit(0, m_Test_result.successCount);*/

    printf("Total Tests: %d\n", m_Test_result.totalTests);
    printf("Success Count: %d\n", m_Test_result.successCount);
    printf("Fail Count: %d\n", m_Test_result.failureCount);
}

// Examples
/*
Test_test(Test, testWillFail)
{
    Test_assertEquals(1, 0);
}*/
/*
Test_test(Test, testWillPass)
{
    Test_assertTrue(1);
}*/
