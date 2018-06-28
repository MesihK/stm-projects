#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <string.h>
#include <stdio.h>
//#include "syscalls.h"
#include "uart.h"
#include "spi.h"
#include "flash.h"
#include "si4463.h"
#include "systick.h"
uint8_t rxComplateFlag = 0;
uint8_t si4463IRQFlag = 0;
int16_t rxRSSI = 0;


void SI446X_IRQ(){
    si4463IRQFlag = 1;
}
void SI446X_CB_RXCOMPLETE(uint8_t length, int16_t rssi)
{
    //gpio_toggle(GPIOC, GPIO13);
    rxRSSI = rssi;
    rxComplateFlag = 1;
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
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    gpio_clear(GPIOC, GPIO13);

    //ent enp
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO1);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);
    gpio_set(GPIOA, GPIO1);
    gpio_set(GPIOA, GPIO2);
}

static void tim_setup(void)
{
	rcc_periph_clock_enable(RCC_TIM2);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	rcc_periph_reset_pulse(RST_TIM2);

	/* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 * (These are actually default values after reset above, so this call
	 * is strictly unnecessary, but demos the api for alternative settings)
	 */
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT,
		TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	/*
	 * Please take note that the clock source for STM32 timers
	 * might not be the raw APB1/APB2 clocks.  In various conditions they
	 * are doubled.  See the Reference Manual for full details!
	 * In our case, TIM2 on APB1 is running at double frequency, so this
	 * sets the prescaler to have the timer run at 5kHz
	 */
	timer_set_prescaler(TIM2, ((rcc_apb1_frequency * 2) / 5000));

	timer_disable_preload(TIM2);
	timer_continuous_mode(TIM2);
	/* 5khz/20 = .25khz / 256 ~ 10hz = 100ms de bir sira degisecek*/
	timer_set_period(TIM2, 19);

	timer_enable_counter(TIM2);
	timer_enable_irq(TIM2, TIM_DIER_UIE);
}
void tim2_isr(void)
{
	if (timer_get_flag(TIM2, TIM_SR_UIF)) {
		/* Clear update interrupt flag. */
		timer_clear_flag(TIM2, TIM_SR_UIF);
        gpio_toggle(GPIOC, GPIO13);
    }
}
int main(void)
{
    uint32_t i=0, verify = 0;
    uint8_t rxBuffer[64];
    
	clock_setup();
    systick_setup();
	gpio_setup();
    usart_setup(57600);
    printf("Init si4463\r\n");
    tim_setup();
    Si446x_init();


    gpio_clear(GPIOC, GPIO13);
    i = 0xDEADBEEF;
    printf("i: %u Verify:%u \r\n",i, verify);
    msleep(100);
    flash_program_data(0, &i, 4);
    printf("programmed\r\n");
    msleep(100);
    flash_read_data(0, 4, &verify);
    printf("read\n");
    if(i == verify) gpio_set(GPIOC, GPIO13);
    else gpio_clear(GPIOC, GPIO13);

    msleep(1000);

    i = 0;
    printf("Verify:%u \r\n", verify);
    Si446x_RX(0);
    printf("Rx mode\r\n");

    while(1)
    {
        if(si4463IRQFlag){
            si4463IRQFlag = 0;
            Si446x_SERVICE();
            printf("Interrupt!\r\n");
        }
        if(rxComplateFlag){
            rxComplateFlag = 0;
            Si446x_read(rxBuffer, 64);
            rxBuffer[rxBuffer[0]] = 0;
            printf("data received %s\r\n", &rxBuffer[2]);
        }
        //printf("RSSI: %d\r\n", Si446x_getRSSI());
        //msleep(100);
        //gpio_toggle(GPIOC, GPIO13);
    }
}