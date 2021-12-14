#ifndef AVR_TESTING_TEST_H_
#define AVR_TESTING_TEST_H_

#if (! __TEST__)

void Test_init(void);

#define Test_run()

#define Test_test(MODULE, NAME)                                                \
    void MODULE##_test_##NAME(void);                                           \
void MODULE##_test_##NAME(void)

// gcc gives warnings if we don't use actual & expected
#define Test_assertTrue(condition) if (condition) return;
#define Test_assertEquals(actual, expected) if(actual != expected) return;
#define Test_assertFloatEquals(actual, expected) if(actual != expected) return;

#else

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>

#define MSG_LEN 120
// typedefs
typedef enum TestResult
{
    NOT_RUN = 0,
    SUCCESS,
    FAILURE
} TestResult;

typedef struct Test_TestHolder
{
    char* name;
    void (*testFunction)(void);
    char* file;

    int        line;
    char message[MSG_LEN];
    TestResult testResult;

    struct Test_TestHolder* next;
} Test_TestHolder;


extern FILE mystdout;

// Initialise the test framework
void Test_add(Test_TestHolder* test);

bool Test_assertTrueLog(uint8_t condition, uint16_t lineNumber,
        char* file);

bool Test_assertEqualLog(uint16_t actual,
        uint16_t expected,
        uint16_t lineNumber,
        char* file);

bool Test_assertFloatEqualLog(double actual,
        double expected,
        uint16_t lineNumber,
        char* file);

void Test_runall(void);


void Test_init(void) __attribute__((naked)) __attribute__((section(".init7")));

#define Test_run()                                                             \
{                                                                          \
    Test_runall();                                                         \
    for (;;)                                                               \
    ;                                                                  \
}

#define Test_test(MODULE, NAME)                                                \
    void MODULE##_test_##NAME(void); /*declare mod_test_name */                \
void MODULE##_appendtest_##NAME(void) __attribute__((naked))               \
__attribute__((section(".init8"))); /* declare mod_appendtest_name */      \
Test_TestHolder m_##MODULE##_test_##NAME = {                               \
#NAME, MODULE##_test_##NAME, __FILE__, __LINE__, "", NOT_RUN, 0               \
}; /* create struct m_mod_test_name*/                                      \
void MODULE##_appendtest_##NAME(void)                                      \
{                                                                          \
    Test_add(&m_##MODULE##_test_##NAME);                                   \
};                              /*define mod_appendtest_name*/             \
void MODULE##_test_##NAME(void) /*define mod_test_name*/

#define SAVE(x) old_ ## x = x
#define RESTORE(x) x = old_ ## x
#define Test_assertTrue(condition)                                             \
    if (!Test_assertTrueLog((condition), __LINE__, __FILE__)) \
{ \
    return; \
}

#define Test_assertEquals(actual, expected)                                    \
    if (!Test_assertEqualLog((actual), (expected), __LINE__, __FILE__)) \
{ \
    return; \
}

#define Test_assertFloatEquals(actual, expected)                               \
    if (!Test_assertFloatEqualLog((actual), (expected), __LINE__, __FILE__)) \
{ \
    return; \
}

#endif // __TEST__

#endif //AVR_TESTING_TEST_H_
