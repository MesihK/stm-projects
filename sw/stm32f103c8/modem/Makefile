DEVICE          = stm32f103c8t6
OPENCM3_DIR     = ../../../lib/libopencm3
BINARY          = modem

SRCS            += main.c
SRCS            += uart.c
SRCS            += ring.c
SRCS            += spi.c
SRCS            += flash.c
SRCS            += si4463.c
SRCS            += systick.c
SRCS            += rl.c
SRCS            += ../../../lib/microrl/src/microrl.c
SRCS            += ../../../lib/tiny-regex-c/re.c
INCLUDES        += -I.
INCLUDES        += -I../../../lib/microrl/src/
INCLUDES        += -I../../../lib/tiny-regex-c/

STFLASH			= $(shell which st-flash)

ARCHFLAGS 		= -mthumb -mcpu=cortex-m3 -msoft-float
CFLAGS          += -std=c99 $(INCLUDES)
CFLAGS 			+= -Wall -Wextra -Wimplicit-function-declaration -Wredundant-decls -Wstrict-prototypes -Wundef -Wshadow -MD
CFLAGS			+= -fno-common -ffunction-sections -fdata-sections
LDFLAGS         += -static -nostartfiles -mfix-cortex-m3-ldrd  
LDFLAGS			+= -Wl,-Map=$(*).map -Wl,--cref
LDLIBS          += -Wl,--start-group -lc -lgcc -lnosys  -Wl,--end-group  -Wl,--gc-sections 

include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk


#LDFLAGS 		+= -T$(LDSCRIPT) 
OBJS 			= $(SRCS:.c=.o)

.PHONY: clean all

all: $(BINARY).elf

clean:
	$(RM) -rf $(BINARY).o $(BINARY).d $(BINARY).elf $(BINARY).bin

flash: 
	@printf "  FLASH  $<\n"
	$(OBJCOPY) -Obinary $(BINARY).elf $(BINARY).bin
	$(STFLASH) write $(BINARY).bin 0x8000000

include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk
