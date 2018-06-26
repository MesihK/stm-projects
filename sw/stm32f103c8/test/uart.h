#ifndef UARTH 
#define UARTH

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <stdio.h>
#include <errno.h>
#include "ring.h"
#include <sys/unistd.h>
#include  <sys/stat.h>

int _write(int file, char *ptr, int len);
void usart_setup(int baudrate);
void usart1_isr(void);

#endif
