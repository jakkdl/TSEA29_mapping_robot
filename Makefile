GCC = avr-gcc

CFLAGS	= -mmcu=atmega1284p
CFLAGS	+= -Os -Wall -Wextra
CFLAGS	+= -g

COMMON_FILES = AVR_common/robot.c AVR_common/sensors.c AVR_common/uart.c
NAVIGATION_FILES = Navigation_unit/nav_sensor_loop.c Navigation_unit/navigation_unit.c Navigation_unit/nav_unit_com_interrupt_logic.c Navigation_unit/navigation.c Navigation_unit/pd.c Navigation_unit/rotation_math.c
NAVIGATION_MAIN = Navigation_unit/main.c
NAVIGATION_FLAGS = -D __NAVIGATION_UNIT__

COMMON_FILES_WALLFOLLOW = AVR_common/robot.c
NAVIGATION_FILES_WALLFOLLOW = Navigation_unit/navigation_unit.c Navigation_unit/nav_unit_com_interrupt_logic.c Navigation_unit/navigation.c

# required to print floats, as per https://stackoverflow.com/a/26525329
TEST_FLAGS = -Wl,-u,vfprintf -lprintf_flt -lm -D __TEST__
TEST_FILES = AVR_testing/*.c

navigation-test:
	$(GCC) $(COMMON_FILES) $(NAVIGATION_FILES) $(TEST_FILES) $(CFLAGS) $(TEST_FLAGS) $(NAVIGATION_FLAGS) -o navigation.elf

navigation:
	$(GCC) $(COMMON_FILES) $(NAVIGATION_FILES) $(NAVIGATION_MAIN) $(CFLAGS) $(NAVIGATION_FLAGS) -o navigation.elf

navigation-test-wf:
	$(GCC) $(COMMON_FILES_WALLFOLLOW) $(NAVIGATION_FILES_WALLFOLLOW) $(TEST_FILES) $(CFLAGS) $(TEST_FLAGS) -o navigation.elf
