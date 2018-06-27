#include "uart.h"

#define BUFFER_SIZE 128

struct ring output_ring;
uint8_t output_ring_buffer[BUFFER_SIZE];

int _write(int file, char *ptr, int len)
{
    gpio_toggle(GPIOC, GPIO13);
	int ret;

	if (file == 1) {
		ret = ring_write(&output_ring, (uint8_t *)ptr, len);

		if (ret < 0)
			ret = -ret;

		USART_CR1(USART1) |= USART_CR1_TXEIE;

		return ret;
	}

	errno = EIO;
	return -1;
}

void usart_setup(int baudrate)
{
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_GPIOA);

	ring_init(&output_ring, output_ring_buffer, BUFFER_SIZE);

	nvic_enable_irq(NVIC_USART1_IRQ);

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

	usart_set_baudrate(USART1, baudrate);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX_RX);

	USART_CR1(USART1) |= USART_CR1_RXNEIE;

	usart_enable(USART1);
}

void usart1_isr(void)
{
    gpio_toggle(GPIOC, GPIO13);

	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {

		ring_write_ch(&output_ring, usart_recv(USART1));

		USART_CR1(USART1) |= USART_CR1_TXEIE;
	}

	if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_TXE) != 0)) {

		int32_t data;

		data = ring_read_ch(&output_ring, NULL);

		if (data == -1) {
			USART_CR1(USART1) &= ~USART_CR1_TXEIE;
		} else {
			usart_send(USART1, data);
		}
	}
}

