#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/timer.h>
#include <string.h>
#include <stdio.h>

int _write(int file, char *ptr, int len)
{
	int ret=len;

	if (file == 1) {
        for(int i=0;i<len;i++){
            usart_send_blocking(USART1, ptr[i]); /* USART1: Send byte. */
        }

		return ret;
	}

	return -1;
}

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
    systick_set_reload(48000);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    systick_interrupt_enable();
}

static void clock_setup(void)
{
    rcc_clock_setup_in_hsi_out_48mhz();
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_USART1);
}

uint8_t channel_array[] = { 2, 2, ADC_CHANNEL_TEMP};
static void adc_setup(void)
{
	rcc_periph_clock_enable(RCC_ADC);
	rcc_periph_clock_enable(RCC_GPIOA);

	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2);
	//gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);

	adc_power_off(ADC1);
	adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);
	adc_calibrate(ADC1);
	adc_set_operation_mode(ADC1, ADC_MODE_SCAN);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	adc_enable_temperature_sensor();
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_071DOT5);
	adc_set_regular_sequence(ADC1, 1, channel_array);
	adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
	adc_disable_analog_watchdog(ADC1);
	adc_power_on(ADC1);

	/* Wait for ADC starting up. */
	int i;
	for (i = 0; i < 800000; i++) {    /* Wait a bit. */
		__asm__("nop");
	}

}

static void dac_setup(){
	rcc_periph_clock_enable(RCC_DAC);
	rcc_periph_clock_enable(RCC_GPIOA);

	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO4);
    dac_disable(CHANNEL_1);
    dac_disable_waveform_generation(CHANNEL_1);
    dac_enable(CHANNEL_1);
    dac_set_trigger_source(DAC_CR_TSEL1_SW);
}

void set_dac(uint16_t data){
    dac_load_data_buffer_single(data, RIGHT12, CHANNEL_1);
    dac_software_trigger(CHANNEL_1);
}

static void usart_setup(void)
{
    nvic_enable_irq(NVIC_USART1_IRQ);
    
	/* Setup USART2 parameters. */
	usart_set_baudrate(USART1, 9600);
	usart_set_databits(USART1, 8);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    usart_enable_rx_interrupt(USART1);

	/* Finally enable the USART. */
	usart_enable(USART1);
}

void usart1_isr(void)
{
    if( (USART_CR1(USART1) & USART_CR1_RXNEIE) ) 
    {
		gpio_toggle(GPIOC, GPIO9);	/* LED on/off */
    }
}

static void gpio_setup(void)
{
	/* Setup GPIO pin GPIO8/9 on GPIO port C for LEDs. */
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8 | GPIO9);

	/* Setup GPIO pins for USART2 transmit. */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);

	/* Setup USART1 TX pin as alternate function. */
	gpio_set_af(GPIOB, GPIO_AF0, GPIO6 | GPIO7 );
}

static void tim_setup(void)
{
    //TIM1_CH1 = A8, TIM1_CH2 = A9 AF2
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO8 | GPIO9);
	gpio_set_af(GPIOA, GPIO_AF2, GPIO8 | GPIO9 );

    //TIM2_CH1 = A0, TIM2_CH2 = A1 AF2
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO0 | GPIO1);
	gpio_set_af(GPIOA, GPIO_AF2, GPIO0 | GPIO1 );

    //TIM3_CH1 = B4, TIM2_CH2 = B5 AF1
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO4 | GPIO5);
	gpio_set_af(GPIOB, GPIO_AF1, GPIO4 | GPIO5 );


	rcc_periph_clock_enable(RCC_TIM1);
    timer_reset(TIM1);
    timer_set_period(TIM1, 4096);
    timer_slave_set_mode(TIM1, 0x3); //encoder
    timer_ic_set_input(TIM1, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_input(TIM1, TIM_IC2, TIM_IC_IN_TI2);
    timer_enable_counter(TIM1);

	rcc_periph_clock_enable(RCC_TIM2);
    timer_reset(TIM2);
    timer_set_period(TIM2, 4096);
    timer_slave_set_mode(TIM2, 0x3); //encoder
    timer_ic_set_input(TIM2, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_input(TIM2, TIM_IC2, TIM_IC_IN_TI2);
    timer_enable_counter(TIM2);

	rcc_periph_clock_enable(RCC_TIM3);
    timer_reset(TIM3);
    timer_set_period(TIM3, 4096);
    timer_slave_set_mode(TIM3, 0x3); //encoder
    timer_ic_set_input(TIM3, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_input(TIM3, TIM_IC2, TIM_IC_IN_TI2);
    timer_enable_counter(TIM3);

}

int main(void)
{
	uint16_t adc_val;
	clock_setup();
    systick_setup();
	gpio_setup();
	usart_setup();
	adc_setup();
    dac_setup();
    tim_setup();

    while(1)
    {
		adc_start_conversion_regular(ADC1);
		while (!(adc_eoc(ADC1)));

		adc_val = adc_read_regular(ADC1);
        set_dac(adc_val);

        printf("adc=%04d\tenc1=%04d\tenc2=%04d\tenc3=%04d\r\n",
                adc_val,
                timer_get_counter(TIM1),
                timer_get_counter(TIM2),
                timer_get_counter(TIM3));
        //msleep(500);
    }
}

