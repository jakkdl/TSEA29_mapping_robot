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
	CFLAGS += -Wl,-Map="navigation_unit.map" -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections
else
	GCC = avr-gcc
	OBJCOPY = avr-objcopy
	OBJDUMP = avr-objdump
	SIZE = avr-size
endif

COMMON_FILES = AVR_common/robot.c AVR_common/sensors.c AVR_common/uart.c
NAVIGATION_FILES = Navigation_unit/nav_sensor_loop.c Navigation_unit/navigation_unit.c Navigation_unit/nav_unit_com_interrupt_logic.c Navigation_unit/navigation.c Navigation_unit/pd.c Navigation_unit/rotation_math.c Navigation_unit/main.c Navigation_unit/pwm_timer.c
NAVIGATION_FLAGS = -D __NAVIGATION_UNIT__

SENSOR_FILES = Sensorenhet/adc.c Sensorenhet/gyro.c Sensorenhet/lidar.c Sensorenhet/main.c
SENSOR_FLAGS = -D __SENSOR_UNIT__

COMMUNICATION_FILES = Com_unit/main.c
COMMUNICATION_FLAGS = -D __COMMUNICATION_UNIT__

COMMON_FILES_WALLFOLLOW = AVR_common/robot.c
NAVIGATION_FILES_WALLFOLLOW = Navigation_unit/navigation_unit.c Navigation_unit/nav_unit_com_interrupt_logic.c Navigation_unit/navigation.c

# required to print floats, as per https://stackoverflow.com/a/26525329
TEST_FLAGS = -Wl,-u,vfprintf -lprintf_flt -lm -D __TEST__
TEST_FILES = AVR_testing/test.c

FILE_NAME = navigation

all:	navigation-atmel

navigation-test:
	$(GCC) $(COMMON_FILES) $(NAVIGATION_FILES) $(TEST_FILES) $(CFLAGS) $(TEST_FLAGS) $(NAVIGATION_FLAGS) -o navigation_unit.elf

navigation:
	$(GCC) $(COMMON_FILES) $(NAVIGATION_FILES) $(CFLAGS) $(NAVIGATION_FLAGS) -o navigation_unit.elf

sensor:
	$(GCC) $(COMMON_FILES) $(SENSOR_FILES) $(CFLAGS) $(SENSOR_FLAGS) -o sensor_unit.elf

communication:
	$(GCC) $(COMMON_FILES) $(COMMUNICATION_FILES) $(CFLAGS) $(COMMUNICATION_FLAGS) -o communication_unit.elf

sensor-atmel: sensor
	$(OBJCOPY) -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "sensor_unit.elf" "sensor_unit.hex"
	$(OBJCOPY) -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "sensor_unit.elf" "sensor_unit.eep" || exit 0
	$(OBJDUMP) -h -S "sensor_unit.elf" > "sensor_unit.lss"
	$(OBJCOPY) -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "sensor_unit.elf" "sensor_unit.srec"
	$(SIZE) "sensor_unit.elf"

navigation-atmel: navigation
	$(OBJCOPY) -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "navigation_unit.elf" "navigation_unit.hex"
	$(OBJCOPY) -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "navigation_unit.elf" "navigation_unit.eep" || exit 0
	$(OBJDUMP) -h -S "navigation_unit.elf" > "navigation_unit.lss"
	$(OBJCOPY) -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "navigation_unit.elf" "navigation_unit.srec"
	$(SIZE) "navigation_unit.elf"

communication-atmel: communication
	$(OBJCOPY) -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "communication_unit.elf" "communication_unit.hex"
	$(OBJCOPY) -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "communication_unit.elf" "communication_unit.eep" || exit 0
	$(OBJDUMP) -h -S "communication_unit.elf" > "communication_unit.lss"
	$(OBJCOPY) -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "communication_unit.elf" "communication_unit.srec"
	$(SIZE) "communication_unit.elf"

navigation-test-wf:
	$(GCC) $(COMMON_FILES_WALLFOLLOW) $(NAVIGATION_FILES_WALLFOLLOW) $(TEST_FILES) $(CFLAGS) $(TEST_FLAGS) -o navigation.elf

clean:
	#$(RM) navigation.elf
	$(RM) "navigation_unit.elf" "navigation_unit.a" "navigation_unit.hex" "navigation_unit.lss" "navigation_unit.eep" "navigation_unit.map" "navigation_unit.srec" "navigation_unit.usersignatures"
	$(RM) "sensor_unit.elf" "sensor_unit.a" "sensor_unit.hex" "sensor_unit.lss" "sensor_unit.eep" "sensor_unit.map" "sensor_unit.srec" "sensor_unit.usersignatures"
	$(RM) "communication_unit.elf" "communication_unit.a" "communication_unit.hex" "communication_unit.lss" "communication_unit.eep" "communication_unit.map" "communication_unit.srec" "communication_unit.usersignatures"
