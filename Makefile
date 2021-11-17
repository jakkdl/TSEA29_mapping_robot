GCC = avr-gcc

CFLAGS	= -mmcu=atmega1284p
CFLAGS	+= -Os -Wall -Wextra
CFLAGS	+= -g

COMMON_FILES = AVR_common/robot.c
NAVIGATION_FILES = Navigation_unit/nav_sensor_loop.c Navigation_unit/navigation_unit.c Navigation_unit/nav_unit_com_interrupt_logic.c Navigation_unit/navigation.c Navigation_unit/Pd.c Navigation_unit/rotation_math.c
NAVIGATION_MAIN = Navigation_unit/main.c

# required to print floats, as per https://stackoverflow.com/a/26525329
TEST_FLAGS = -Wl,-u,vfprintf -lprintf_flt -lm
TEST_FILES = AVR_testing/*.c

navigation-test:
	$(GCC) $(COMMON_FILES) $(NAVIGATION_FILES) $(TEST_FILES) $(CFLAGS) $(TEST_FLAGS) -D __TEST__ -o navigation.elf

navigation:
	$(GCC) $(COMMON_FILES) $(NAVIGATION_FILES) $(NAVIGATION_MAIN) $(CFLAGS) -o navigation.elf
