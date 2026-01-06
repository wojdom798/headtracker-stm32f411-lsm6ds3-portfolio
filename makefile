TARGET = main

LD_SCRIPT = stm32f411ce.ld
MCU_SPEC  = cortex-m4

STARTUP_FILE = startup_stm32f4xx

# toolchain tool definitions
CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
OC = arm-none-eabi-objcopy
OD = arm-none-eabi-objdump
OS = arm-none-eabi-size


SOURCE_DIR := "./src"
BUILD_DIR := "./build"
LINKER_DIR := "./linker"


# assembler directives
ASFLAGS = -c
ASFLAGS += -O0
ASFLAGS += -mcpu=$(MCU_SPEC)
ASFLAGS += -mthumb
ASFLAGS += -Wall
# set error messages to appear on a single line
ASFLAGS += -fmessage-length=0

# C compiler directives
CFLAGS = -mcpu=$(MCU_SPEC)
CFLAGS += -mthumb
CFLAGS += -Wall
CFLAGS += -g
# set error messages to appear on a single line
CFLAGS += -fmessage-length=0
CFLAGS += --specs=nosys.specs # ?

# linker directives
LSCRIPT = $(LINKER_DIR)/$(LD_SCRIPT)
LFLAGS = -mcpu=$(MCU_SPEC)
LFLAGS += -mthumb
LFLAGS += -Wall
LFLAGS += --specs=nosys.specs
LFLAGS += -nostdlib
LFLAGS += -lgcc
LFLAGS += -fmessage-length=0
LFLAGS += -fsigned-char
LFLAGS += -ffunction-sections
LFLAGS += -fdata-sections
LFLAGS += -g3
LFLAGS += -T"$(LSCRIPT)"

AS_SRC   =  $(SOURCE_DIR)/$(STARTUP).S
C_SRC    =  $(SOURCE_DIR)/$(TARGET).c

INCLUDE  =  -I./
INCLUDE  += -I./stm32f411_headers
INCLUDE  += -I./src
INCLUDE  += -I./src/stm32f411Blackpill
INCLUDE  += -I./src/debugger
INCLUDE  += -I./src/usb
INCLUDE  += -I./src/lsm6ds3

OBJ_FILES = $(BUILD_DIR)/$(TARGET).o
OBJ_FILES += $(BUILD_DIR)/$(STARTUP_FILE).o
OBJ_FILES += $(BUILD_DIR)/hardwareSetup.o
OBJ_FILES += $(BUILD_DIR)/stm32f411Blackpill.o
OBJ_FILES += $(BUILD_DIR)/makeshiftDebugger.o
OBJ_FILES += $(BUILD_DIR)/usb.o
OBJ_FILES += $(BUILD_DIR)/usbDebug.o
OBJ_FILES += $(BUILD_DIR)/usbDemoDescriptors.o
OBJ_FILES += $(BUILD_DIR)/usbIrqHandlers.o
OBJ_FILES += $(BUILD_DIR)/lsm6ds3.o


.PHONY: compile_startup_s
compile_startup_s:
	$(CC) -x assembler-with-cpp $(ASFLAGS) -fsigned-char -ffunction-sections -fdata-sections -g3 $(SOURCE_DIR)/$(STARTUP_FILE).S -o $(BUILD_DIR)/$(STARTUP_FILE).o


.PHONY: compile_hardwareSetup
compile_hardwareSetup:
	$(CC) -c $(CFLAGS) $(INCLUDE) $(SOURCE_DIR)/hardwareSetup.c -o $(BUILD_DIR)/hardwareSetup.o


.PHONY: compile_stm32f411Blackpill
compile_stm32f411Blackpill:
	$(CC) -c $(CFLAGS) $(INCLUDE) $(SOURCE_DIR)/stm32f411Blackpill/stm32f411Blackpill.c -o $(BUILD_DIR)/stm32f411Blackpill.o


.PHONY: compile_makeshiftDebugger
compile_makeshiftDebugger:
	$(CC) -c $(CFLAGS) $(INCLUDE) $(SOURCE_DIR)/debugger/makeshiftDebugger.c -o $(BUILD_DIR)/makeshiftDebugger.o


.PHONY: compile_usb
compile_usb:
	$(CC) -c $(CFLAGS) $(INCLUDE) $(SOURCE_DIR)/usb/usb.c -o $(BUILD_DIR)/usb.o


.PHONY: compile_usbDebug
compile_usbDebug:
	$(CC) -c $(CFLAGS) $(INCLUDE) $(SOURCE_DIR)/usb/usbDebug.c -o $(BUILD_DIR)/usbDebug.o


.PHONY: compile_usbDemoDescriptors
compile_usbDemoDescriptors:
	$(CC) -c $(CFLAGS) $(INCLUDE) $(SOURCE_DIR)/usb/usbDemoDescriptors.c -o $(BUILD_DIR)/usbDemoDescriptors.o


.PHONY: compile_usbIrqHandlers
compile_usbIrqHandlers:
	$(CC) -c $(CFLAGS) $(INCLUDE) $(SOURCE_DIR)/usb/usbIrqHandlers.c -o $(BUILD_DIR)/usbIrqHandlers.o


.PHONY: compile_lsm6ds3
compile_lsm6ds3:
	$(CC) -c $(CFLAGS) $(INCLUDE) $(SOURCE_DIR)/lsm6ds3/lsm6ds3.c -o $(BUILD_DIR)/lsm6ds3.o


.PHONY: compile
compile:
	$(CC) -c $(CFLAGS) $(INCLUDE) $(SOURCE_DIR)/$(TARGET).c -o $(BUILD_DIR)/$(TARGET).o


.PHONY: link
link:
	$(CC) $(OBJ_FILES) $(LFLAGS) -o $(BUILD_DIR)/$(TARGET).elf


.PHONY: build
build:
	$(OC) -S -O binary $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin
	$(OS) $(BUILD_DIR)/$(TARGET).elf


.PHONY: all
all: compile_startup_s compile_hardwareSetup compile_stm32f411Blackpill compile_makeshiftDebugger compile_usb compile_usbDebug compile_usbDemoDescriptors compile_usbIrqHandlers compile_lsm6ds3 compile link build


.PHONY: flash_mcu
flash_mcu:
	ST-LINK_CLI.exe -P $(BUILD_DIR)/$(TARGET).bin 0x08000000 -V -Rst