#include "test.c"

int main(void)
{
    Test_init();
    Test_appendtest_testWillFail();
    Test_appendtest_testWillPass();
	Test_run();        // System code
}
