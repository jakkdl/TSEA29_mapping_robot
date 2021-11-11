#ifndef _TEST_H
#define _TEST_H

#include <inttypes.h>

// typedefs
typedef enum TestResult
{
	NOT_RUN = 0,
	SUCCESS,
	FAILURE
} TestResult;

typedef struct Test_TestHolder
{
	char * name;
	void (*testFunction)(void);
	char * file;

	int line;
	TestResult testResult;

	struct Test_TestHolder* next;
} Test_TestHolder;

// Initialise the test framework

void Test_add(Test_TestHolder* test);

void Test_assertTrueLog(uint8_t condition, uint16_t lineNumber);

void Test_assertEqualLog(uint16_t actual, uint16_t expected,
    uint16_t lineNumber);

void Test_runall(void);

#if __TEST__

void Test_init(void) __attribute__ ((naked))
    __attribute__ ((section (".init7")));

#define Test_run() {Test_runall(); for(;;); }

#define Test_test(MODULE, NAME) \
	void MODULE##_test_##NAME(void); /*declare mod_test_name */ \
	void MODULE##_appendtest_##NAME(void) \
            __attribute__ ((naked)) \
            __attribute__ ((section (".init8"))); /* declare mod_appendtest_name */ \
	Test_TestHolder m_##MODULE##_test_##NAME = { #NAME, \
            MODULE##_test_##NAME, __FILE__, 0, NOT_RUN, 0 }; /* create struct m_mod_test_name*/ \
	void MODULE##_appendtest_##NAME(void) { \
            Test_add( &m_##MODULE##_test_##NAME ); }; /*define mod_appendtest_name*/ \
	void MODULE##_test_##NAME(void) /*define mod_test_name*/
#else

void Test_init(void);

#define Test_run()

#define Test_test(MODULE, NAME) \
	void MODULE##_test_##NAME(void); \
	void MODULE##_test_##NAME(void)

#endif

#define Test_assertTrue(condition) \
    Test_assertTrueLog((condition), __LINE__); \
    if (!(condition)) { \
        return; \
	}

#define Test_assertEquals(expected, actual) \
    Test_assertEqualLog((expected), (actual), __LINE__); \
    if ((expected) != (actual)) { \
        return; \
    }

#endif /* TEST_H */
