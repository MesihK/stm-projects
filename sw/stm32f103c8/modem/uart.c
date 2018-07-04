#include "uart.h"

#define BUFFER_SIZE 1280

struct ring tx_ring;
uint8_t tx_ring_buffer[BUFFER_SIZE];
struct ring rx_ring;
uint8_t rx_ring_buffer[BUFFER_SIZE];

int _write(int file, char *ptr, int len)
{
	int ret=len;

	if (file == 1) {
        if(ring_get_count(&tx_ring)) 
            USART_CR1(USART1) |= USART_CR1_TXEIE;
        while(ring_get_count(&tx_ring) > BUFFER_SIZE-len-1);
        //USART_CR1(USART1) &= ~USART_CR1_TXEIE;
		ret = ring_write(&tx_ring, (uint8_t *)ptr, len);
		USART_CR1(USART1) |= USART_CR1_TXEIE;
        
        //for(int i =0; i< len; i++)
         //   usart_send_blocking(USART1, ptr[i]);

		if (ret < 0)
			ret = -ret;


		return ret;
	}

	errno = EIO;
	return -1;
}

int uart_read(uint8_t *buffer, int len){
    if(len > ring_get_count(&rx_ring)) len = ring_get_count(&rx_ring);
    ring_read(&rx_ring, buffer, len);
    return len;
}
uint8_t uart_read_ch(){
    return ring_read_ch(&rx_ring, NULL);
}
int uart_rx_available(){
    return ring_get_count(&rx_ring);
}
int uart_tx_available(){
    return BUFFER_SIZE - ring_get_count(&tx_ring);
}

void usart_setup(int baudrate)
{
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_GPIOA);

	ring_init(&tx_ring, tx_ring_buffer, BUFFER_SIZE);
	ring_init(&rx_ring, rx_ring_buffer, BUFFER_SIZE);

	nvic_enable_irq(NVIC_USART1_IRQ);
    //uart inturrpt should be top priorty else we miss character
    nvic_set_priority(NVIC_USART1_IRQ, 0);

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

    USART_CR1(USART1) &= ~USART_CR1_TCIE;
    USART_CR1(USART1) &= ~USART_CR1_IDLEIE;
    USART_CR1(USART1) &= ~USART_CR1_PEIE;
    USART_CR1(USART1) &= ~USART_CR1_TXEIE;
	usart_enable(USART1);
}

void usart1_isr(void)
{
    //gpio_clear(GPIOC, GPIO13);
	if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_TXE) != 0)) {

		int32_t data;

		data = ring_read_ch(&tx_ring, NULL);

		if (data == -1) {
			USART_CR1(USART1) &= ~USART_CR1_TXEIE;
		} else {
			usart_send(USART1, data);
		}
	}
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {

		ring_write_ch(&rx_ring, usart_recv(USART1));
	}

}

