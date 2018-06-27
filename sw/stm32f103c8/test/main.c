#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/usart.h>
#include <string.h>
#include <stdio.h>
//#include "syscalls.h"
#include "uart.h"
#include "spi.h"
#include "flash.h"

volatile uint32_t system_millis;

void sys_tick_handler(void)
{
	system_millis++;
}

static void msleep(uint32_t delay)
{
	uint32_t wake = system_millis + delay;
	while (wake > system_millis);
}

static void systick_setup(void)
{
    systick_set_reload(72000/8-1);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
    systick_counter_enable();
    systick_interrupt_enable();
}

static void clock_setup(void)
{
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_AFIO);
}

static void gpio_setup(void)
{
	/* Setup GPIO pin GPIO13 on GPIO port C for LED. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    gpio_clear(GPIOC, GPIO13);
}

int main(void)
{
    uint32_t i=0, verify = 0;
    
	clock_setup();
    systick_setup();
	gpio_setup();
	spi_setup();
    usart_setup(57600);


    gpio_clear(GPIOC, GPIO13);
    i = 0xDEADBEEF;
    printf("i: %u Verify:%u \n",i, verify);
    msleep(100);
    flash_program_data(0, &i, 4);
    printf("programmed\n");
    msleep(100);
    flash_read_data(0, 4, &verify);
    printf("read\n");
    if(i == verify) gpio_set(GPIOC, GPIO13);
    else gpio_clear(GPIOC, GPIO13);

    msleep(1000);

    i = 0;
    printf("Verify:%u \n", verify);
    while(1)
    {
        printf("Merhaba:%d ", i++);
        gpio_clear(GPIOA, GPIO4);
		printf("spi: %d \r\n", spi_send_data((uint8_t) i));
        gpio_set(GPIOA, GPIO4);
		//rx_value = spi_read(SPI1);
        gpio_toggle(GPIOC, GPIO13);	/* LED on/off */
        msleep(500);
    }
}
