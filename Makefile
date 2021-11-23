RM = rm -f
CFLAGS	= -mmcu=atmega1284p
CFLAGS	+= -Os -Wall -Wextra -std=gnu99
CFLAGS	+= -g2
CFLAGS	+= -mrelax

ifdef OS
	GCC = "C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe"
	OBJCOPY = "C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe"
	OBJDUMP = "C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objdump.exe"
	SIZE = "C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-size.exe"
	CFLAGS += -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.3.300\gcc\dev\atmega1284p"
	CFLAGS += -funsigned-char -funsigned-bitfields -DDEBUG  -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.3.300\include" -ffunction-sections -fdata-sections -fpack-struct -fshort-enums
	CFLAGS += -Wl,-Map="$(FILE_NAME).map" -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections
else
	GCC = avr-gcc
endif

COMMON_FILES = AVR_common/robot.c AVR_common/sensors.c AVR_common/uart.c
NAVIGATION_FILES = Navigation_unit/nav_sensor_loop.c Navigation_unit/navigation_unit.c Navigation_unit/nav_unit_com_interrupt_logic.c Navigation_unit/navigation.c Navigation_unit/pd.c Navigation_unit/rotation_math.c
NAVIGATION_MAIN = Navigation_unit/main.c
NAVIGATION_FLAGS = -D __NAVIGATION_UNIT__

COMMON_FILES_WALLFOLLOW = AVR_common/robot.c
NAVIGATION_FILES_WALLFOLLOW = Navigation_unit/navigation_unit.c Navigation_unit/nav_unit_com_interrupt_logic.c Navigation_unit/navigation.c

# required to print floats, as per https://stackoverflow.com/a/26525329
TEST_FLAGS = -Wl,-u,vfprintf -lprintf_flt -lm -D __TEST__
TEST_FILES = AVR_testing/*.c

FILE_NAME = navigation

all:	navigation-atmel

navigation-test:
	$(GCC) $(COMMON_FILES) $(NAVIGATION_FILES) $(TEST_FILES) $(CFLAGS) $(TEST_FLAGS) $(NAVIGATION_FLAGS) -o $(FILE_NAME).elf

navigation:
	$(GCC) $(COMMON_FILES) $(NAVIGATION_FILES) $(NAVIGATION_MAIN) $(CFLAGS) $(NAVIGATION_FLAGS) -o $(FILE_NAME).elf

navigation-atmel: navigation-test
	$(OBJCOPY) -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "$(FILE_NAME).elf" "$(FILE_NAME).hex"
	$(OBJCOPY) -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "$(FILE_NAME).elf" "$(FILE_NAME).eep" || exit 0
	$(OBJDUMP) -h -S "$(FILE_NAME).elf" > "$(FILE_NAME).lss"
	$(OBJCOPY) -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "$(FILE_NAME).elf" "$(FILE_NAME).srec"
	$(SIZE) "$(FILE_NAME).elf"

navigation-test-wf:
	$(GCC) $(COMMON_FILES_WALLFOLLOW) $(NAVIGATION_FILES_WALLFOLLOW) $(TEST_FILES) $(CFLAGS) $(TEST_FLAGS) -o navigation.elf

clean:
	#$(RM) navigation.elf
	$(RM) "$(FILE_NAME).elf" "$(FILE_NAME).a" "$(FILE_NAME).hex" "$(FILE_NAME).lss" "$(FILE_NAME).eep" "$(FILE_NAME).map" "$(FILE_NAME).srec" "$(FILE_NAME).usersignatures"
