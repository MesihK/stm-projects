/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Copyright (C) 2012 Karl Palsson <karlp@tweak.net.au>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/timer.h>
#include  "dsin.h"

#define PORT_LED GPIOC
#define PIN_LED GPIO8
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
}

static void gpio_setup(void)
{
	/* Enable GPIOB clock. */
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOA);


	/* Set GPIO6 (in GPIO port B) to 'output push-pull'. */
	gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED);

    //debug pins
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 );
}

static void timer_setup(void)
{
	rcc_periph_clock_enable(RCC_TIM1);

    /*
     * A8  AF2 TIM1_CH1
     * A9  AF2 TIM1_CH2
     * A10 AF2 TIM1_CH3
     * A7  AF2 TIM1_CH1N
     * B0  AF2 TIM1_CH2N
     * B1  AF2 TIM1_CH2N
     */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7 | GPIO8 | GPIO9 | GPIO10);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO0 | GPIO1 );
	gpio_set_af(GPIOA, GPIO_AF2, GPIO7 | GPIO8 | GPIO9 | GPIO10);
	gpio_set_af(GPIOB, GPIO_AF2, GPIO0 | GPIO1 );
	nvic_enable_irq(NVIC_TIM1_BRK_UP_TRG_COM_IRQ);
	nvic_enable_irq(NVIC_TIM1_CC_IRQ);

    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(TIM1, 48-1);
	timer_set_repetition_counter(TIM1, 0);
	timer_enable_preload(TIM1);
	timer_continuous_mode(TIM1);
	timer_set_period(TIM1, 100-1); //10khz

	timer_set_deadtime(TIM1, 40);
	timer_set_enabled_off_state_in_idle_mode(TIM1);
	timer_set_enabled_off_state_in_run_mode(TIM1);
	timer_disable_break(TIM1);
	timer_set_break_polarity_high(TIM1);
	timer_disable_break_automatic_output(TIM1);
	timer_set_break_lock(TIM1, TIM_BDTR_LOCK_OFF);

	/* -- OC1 and OC1N configuration -- */

	/* Disable outputs. */
	timer_disable_oc_output(TIM1, TIM_OC1);
	timer_disable_oc_output(TIM1, TIM_OC1N);

	/* Configure global mode of line 1. */
	timer_disable_oc_clear(TIM1, TIM_OC1);
	timer_enable_oc_preload(TIM1, TIM_OC1);
	timer_set_oc_slow_mode(TIM1, TIM_OC1);
	timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);

	/* Configure OC1. */
	timer_set_oc_polarity_high(TIM1, TIM_OC1);
	timer_set_oc_idle_state_set(TIM1, TIM_OC1);

	/* Configure OC1N. */
	timer_set_oc_polarity_high(TIM1, TIM_OC1N);
	timer_set_oc_idle_state_set(TIM1, TIM_OC1N);

	/* Set the capture compare value for OC1. */
	timer_set_oc_value(TIM1, TIM_OC1, 10);

	/* Reenable outputs. */
	timer_enable_oc_output(TIM1, TIM_OC1);
	timer_enable_oc_output(TIM1, TIM_OC1N);

	/* -- OC2 and OC2N configuration -- */

	/* Disable outputs. */
	timer_disable_oc_output(TIM1, TIM_OC2);
	timer_disable_oc_output(TIM1, TIM_OC2N);

	/* Configure global mode of line 2. */
	timer_disable_oc_clear(TIM1, TIM_OC2);
	timer_enable_oc_preload(TIM1, TIM_OC2);
	timer_set_oc_slow_mode(TIM1, TIM_OC2);

	/* -- OC2 and OC2N configuration -- */

	/* Disable outputs. */
	timer_disable_oc_output(TIM1, TIM_OC2);
	timer_disable_oc_output(TIM1, TIM_OC2N);

	/* Configure global mode of line 2. */
	timer_disable_oc_clear(TIM1, TIM_OC2);
	timer_enable_oc_preload(TIM1, TIM_OC2);
	timer_set_oc_slow_mode(TIM1, TIM_OC2);
	timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM1);

	/* Configure OC2. */
	timer_set_oc_polarity_high(TIM1, TIM_OC2);
	timer_set_oc_idle_state_set(TIM1, TIM_OC2);

	/* Configure OC2N. */
	timer_set_oc_polarity_high(TIM1, TIM_OC2N);
	timer_set_oc_idle_state_set(TIM1, TIM_OC2N);

	/* Set the capture compare value for OC1. */
	timer_set_oc_value(TIM1, TIM_OC2, 20);

	/* Reenable outputs. */
	timer_enable_oc_output(TIM1, TIM_OC2);
	timer_enable_oc_output(TIM1, TIM_OC2N);

	/* -- OC3 and OC3N configuration -- */

	/* Disable outputs. */
	timer_disable_oc_output(TIM1, TIM_OC3);
	timer_disable_oc_output(TIM1, TIM_OC3N);

	/* Configure global mode of line 3. */
	timer_disable_oc_clear(TIM1, TIM_OC3);
	timer_enable_oc_preload(TIM1, TIM_OC3);
	timer_set_oc_slow_mode(TIM1, TIM_OC3);

	/* -- OC3 and OC3N configuration -- */

	/* Disable outputs. */
	timer_disable_oc_output(TIM1, TIM_OC3);
	timer_disable_oc_output(TIM1, TIM_OC3N);

	/* Configure global mode of line 3. */
	timer_disable_oc_clear(TIM1, TIM_OC3);
	timer_enable_oc_preload(TIM1, TIM_OC3);
	timer_set_oc_slow_mode(TIM1, TIM_OC3);
	timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_PWM1);

	/* Configure OC3. */
	timer_set_oc_polarity_high(TIM1, TIM_OC3);
	timer_set_oc_idle_state_set(TIM1, TIM_OC3);

	/* Configure OC3N. */
	timer_set_oc_polarity_high(TIM1, TIM_OC3N);
	timer_set_oc_idle_state_set(TIM1, TIM_OC3N);

	/* Set the capture compare value for OC3. */
	timer_set_oc_value(TIM1, TIM_OC3, 30);

	/* Reenable outputs. */
	timer_enable_oc_output(TIM1, TIM_OC3);
	timer_enable_oc_output(TIM1, TIM_OC3N);

	/* ---- */

	/* ARR reload enable. */
	timer_enable_preload(TIM1);

	/*
	 * Enable preload of complementary channel configurations and
	 * update on COM event.
	 */
	timer_enable_preload_complementry_enable_bits(TIM1);

	/* Enable outputs in the break subsystem. */
	timer_enable_break_main_output(TIM1);

	/* Counter enable. */
	timer_enable_counter(TIM1);

	/* Enable commutation interrupt. */
	timer_enable_irq(TIM1, TIM_DIER_COMIE);
	timer_enable_irq(TIM1, TIM_DIER_CC1IE);
	timer_enable_irq(TIM1, TIM_DIER_CC2IE);
	timer_enable_irq(TIM1, TIM_DIER_CC3IE);
	timer_enable_irq(TIM1, TIM_DIER_UIE);
}
void tim1_brk_up_trg_com_isr(void)
{
    if(timer_get_flag(TIM1, TIM_SR_COMIF)){
		gpio_set(GPIOA, GPIO0);	
        timer_clear_flag(TIM1, TIM_SR_COMIF);
		gpio_clear(GPIOA, GPIO0);	
    }
    if(timer_get_flag(TIM1, TIM_SR_UIF)){
		gpio_set(GPIOA, GPIO1);	
        timer_clear_flag(TIM1, TIM_SR_UIF);
		gpio_clear(GPIOA, GPIO1);	
    }
}
void tim1_cc_isr(void)
{
    if(timer_get_flag(TIM1, TIM_SR_CC1IF)){
		gpio_set(GPIOA, GPIO2);	
        timer_clear_flag(TIM1, TIM_SR_CC1IF);
		gpio_clear(GPIOA, GPIO2);	
    }
    if(timer_get_flag(TIM1, TIM_SR_CC2IF)){
		gpio_set(GPIOA, GPIO3);	
        timer_clear_flag(TIM1, TIM_SR_CC2IF);
		gpio_clear(GPIOA, GPIO3);	
    }
    if(timer_get_flag(TIM1, TIM_SR_CC3IF)){
		gpio_set(GPIOA, GPIO4);	
        timer_clear_flag(TIM1, TIM_SR_CC3IF);
		gpio_clear(GPIOA, GPIO4);	
    }
}

void set_pwm_angle(uint16_t angle, uint16_t amp)
{
    int16_t U = amp*_sin(angle);
    int16_t V = amp*_sin(angle+120);
    int16_t W = amp*_sin(angle+240);
    int16_t u = 0, v = 0, w = 0;

    if (angle <= 60 ) {
        u = 0;
        v = u + U;
        w = u - W;
    }
    else if (angle <= 120 ) {
        v = 100;
        u = v - U;
        w = V + v;
    }
    else if (angle <= 180 ) {
        w = 0;
        u = W + w;
        v = w - V;
    }
    else if (angle <= 240 ) {
        u = 100;
        v = u + U;
        w = u - W;
    }
    else if (angle <= 300 ) {
        v = 0;
        u = v - U;
        w = V + v;
    }
    else if (angle <= 360 ) {
        w = 100;
        u = W + w;
        v = w - V;
    }

	timer_set_oc_value(TIM1, TIM_OC1, u);
	timer_set_oc_value(TIM1, TIM_OC2, v);
	timer_set_oc_value(TIM1, TIM_OC3, w);
    timer_generate_event(TIM1, TIM_EGR_COMG); // sanirim ihtiyac yok
    //com eventi baska bir timer ile cagrilip bu timer da is yapilmasini sagliyor
    //bizim buna ihtiyacimiz yok cunku sabit v/f kontrolu yapacagiz
}

int main(void)
{
    clock_setup();
    systick_setup();
	gpio_setup();
    timer_setup();

    uint16_t angle = 0;
	while (1) {
		gpio_toggle(PORT_LED, PIN_LED);	/* LED on/off */
		gpio_set(GPIOA, GPIO5);	
        set_pwm_angle(angle, 1);
		gpio_clear(GPIOA, GPIO5);	
        angle = angle + 1;
        if(angle > 360) angle = angle - 360;
        msleep(1); //360 * 1 ms = .36ms T = ~2.8H

	}

	return 0;
}
