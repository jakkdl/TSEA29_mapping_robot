#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "../AVR_common/uart.h"
#include "test.h"

#if __NAVIGATION_UNIT__
#include "../Navigation_unit/navigation_unit.h"
#endif

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

int uart_putchar(char c, FILE* stream);
FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

// This crashes simavr, and I have no clue why, so we can't get
/*
   const struct avr_mmcu_vcd_trace_t _mytrace[]  _MMCU_ = {
   { AVR_MCU_VCD_SYMBOL("UDR0"), .what = (void*)&UDR0, },
   { AVR_MCU_VCD_SYMBOL("UDRE0"), .mask = (1 << UDRE0), .what = (void*)&UCSR0A,
   },
   };*/

int uart_putchar(char c, FILE* stream)
{
    if (c == '\n')
        uart_putchar('\r', stream);
    // included with avr/io.h
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    return 0;
}

// define a FILE to be used by custom printf
//static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
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

bool Test_assertTrueLog(uint8_t condition,
        uint16_t lineNumber,
        char* file)
{
    // We have the active test
    if (!(condition))
    {
        m_Test_activeTest->testResult = FAILURE;
        m_Test_activeTest->line       = lineNumber;
        snprintf(
                m_Test_activeTest->message,
                MSG_LEN,
                "FAIL: %s @ %u in %s\n    not true\n",
                m_Test_activeTest->name,
                lineNumber,
                file);
        return false;
    }
    return true;
}

bool Test_assertEqualLog(uint16_t actual,
                         uint16_t expected,
                         uint16_t lineNumber,
                         char* file)
{
    if (expected != actual)
    {
        m_Test_activeTest->testResult = FAILURE;
        m_Test_activeTest->line       = lineNumber;
        snprintf(
                m_Test_activeTest->message,
                MSG_LEN,
                "FAIL: %s @ %u in %s\n    got %u expected %u\n",
                m_Test_activeTest->name,
                lineNumber,
                file,
                actual,
                expected
                );
        return false;
    }
    return true;
}

bool Test_assertFloatEqualLog(double actual,
                              double expected,
                              uint16_t lineNumber,
                              char* file)
{
    if (expected != actual)
    {
        m_Test_activeTest->testResult = FAILURE;
        m_Test_activeTest->line       = lineNumber;
        snprintf(
                m_Test_activeTest->message,
                MSG_LEN,
                "FAIL: %s @ %u in %s\n    got %f expected %f\n",
                m_Test_activeTest->name,
                lineNumber,
                file,
                actual,
                expected
                );
        return false;
    }
    return true;
}

#if __NAVIGATION_UNIT__
bool check_reset_map(void)
{
    bool result = true;
    for (uint8_t x = 0; x < MAP_X_MAX; ++x)
    {
        for (uint8_t y = 0; y < MAP_Y_MAX; ++y)
        {
            if (g_navigationMap[x][y] != 0)
            {
                printf("(%u, %u) = %d\n", x, y, g_navigationMap[x][y]);
                result = false;
                g_navigationMap[x][y] = 0;
            }
        }
    }
    return result;
}
#endif


#define test_reset_global(var, value) \
        if (var != value) \
        { \
            if (m_Test_activeTest->testResult == SUCCESS) \
            { \
                printf("FAIL: %s after test\n    " #var " is %u and not reset to " #value "\n", \
                        m_Test_activeTest->name, \
                        var); \
                m_Test_result.successCount--; \
                m_Test_result.failureCount++; \
                m_Test_activeTest->testResult = FAILURE; \
            } \
            var = value; \
        }

// Run through all the tests
void Test_runall(void)
{
    uint8_t oldKd = g_pdKd;
    uint8_t oldKp = g_pdKp;
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
            printf(m_Test_activeTest->message);
            m_Test_result.failureCount++;
        }
        //printf("%s\n", m_Test_activeTest->name);
#if __NAVIGATION_UNIT__
        test_reset_global(g_navigationMode, MANUAL)
        test_reset_global(g_pdKd, oldKd)
        test_reset_global(g_pdKp, oldKp)
        test_reset_global(g_currentHeading, FULL_TURN/4)
        test_reset_global(g_currentPosX, GridToMm(24))
        test_reset_global(g_currentPosY, 200)
        test_reset_global(g_wheelDirectionLeft, DIR_FORWARD)
        test_reset_global(g_wheelDirectionRight, DIR_FORWARD)
        test_reset_global(g_wheelSpeedLeft, 0)
        test_reset_global(g_wheelSpeedRight, 0)
        test_reset_global(g_navigationGoalSet, false)
        test_reset_global(g_navigationGoalX, 24)
        test_reset_global(g_navigationGoalY, 0)
        test_reset_global(g_navigationGoalHeading, 0)
        test_reset_global(g_uart_tx_0.length, 0)
        test_reset_global(g_uart_rx_0.length, 0)
#if __UART_TX_1__
        test_reset_global(g_uart_tx_1.length, 0)
#endif
        test_reset_global(g_uart_rx_1.length, 0)
        if (!check_reset_map() && m_Test_activeTest->testResult == SUCCESS)
        {
            printf("FAIL: \"%s\" left garbage in the map according to above\n",
                    m_Test_activeTest->name);
            m_Test_result.successCount--;
            m_Test_result.failureCount++;
            m_Test_activeTest->testResult = FAILURE;
        }
#endif

        __asm__("nop");

        // next in the chain
        m_Test_activeTest = m_Test_activeTest->next;
    }

    // Get the results
    __asm__("nop");
    /*UART_Init(0);
    UART_Transmit(0, m_Test_result.successCount);*/

    printf("Total Tests: %u\n", m_Test_result.totalTests);
    printf("Success Count: %u\n", m_Test_result.successCount);
    printf("Fail Count: %u\n", m_Test_result.failureCount);
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
