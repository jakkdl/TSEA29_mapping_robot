GCC = avr-gcc

CFLAGS	= -mmcu=atmega1284p
CFLAGS	+= -O2 -Wall -Wextra
CFLAGS	+= -g

COMMON_FILES = AVR_common/robot.c
TESTING_FILES = AVR_testing/*.c
NAVIGATION_FILES = Navigation_unit/nav_sensor_loop.c Navigation_unit/navigation_unit.c Navigation_unit/nav_unit_com_interrupt_logic.c
NAVIGATION_MAIN = Navigation_unit/main.c


navigation-test:
	$(GCC) $(COMMON_FILES) $(NAVIGATION_FILES) $(TESTING_FILES) $(CFLAGS) -D __TEST__ -o navigation.elf

navigation:
	$(GCC) $(COMMON_FILES) $(NAVIGATION_FILES) $(NAVIGATION_MAIN) $(CFLAGS) -o navigation.elf
