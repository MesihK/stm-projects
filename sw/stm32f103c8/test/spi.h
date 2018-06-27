#ifndef SPIH
#define SPIH

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/spi.h>
#include <stdio.h>
#include <errno.h>
#include "ring.h"

void spi_setup(void) ;
uint8_t spi_send_data(uint8_t data);
void spi1_isr(void);

#endif
