DEVICE          = stm32f051r8t6
OPENCM3_DIR     = ../../../lib/libopencm3
BINARY          = rl

SRCS            += rl.c
SRCS            += ../../../lib/microrl/src/microrl.c
INCLUDES        += -I.
INCLUDES        += -I../../../lib/microrl/src/

STFLASH			= $(shell which st-flash)

#ARCHFLAGS 		= -mthumb -mcpu=cortex-m0 -msoft-float
CFLAGS          += -std=c99  $(INCLUDES)
LDFLAGS         += -static -nostartfiles 
LDLIBS          += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group 

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
