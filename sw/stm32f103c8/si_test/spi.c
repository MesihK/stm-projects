#include "spi.h"

#define BUFFER_SIZE 128
struct ring spi_rx_ring;
uint8_t spi_rx_ring_buffer[BUFFER_SIZE];
struct ring spi_tx_ring;
uint8_t spi_tx_ring_buffer[BUFFER_SIZE];

void spi_setup(void) {
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_SPI1);

	ring_init(&spi_rx_ring, spi_rx_ring_buffer, BUFFER_SIZE);
	ring_init(&spi_tx_ring, spi_tx_ring_buffer, BUFFER_SIZE);

  /* Configure GPIOs: SS=PA4, SCK=PA5, MISO=PA6 and MOSI=PA7 */
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5 |
   //                                         GPIO4 |
                                            GPIO7 );

  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
          GPIO6);

  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO4);

  nvic_enable_irq(NVIC_SPI1_IRQ);
  /* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
  spi_reset(SPI1);

  /* Set up SPI in Master mode with:
   * Clock baud rate: 1/64 of peripheral clock frequency
   * Clock polarity: Idle High
   * Clock phase: Data valid on 2nd clock pulse
   * Data frame format: 8-bit
   * Frame format: MSB First
   */
  spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_8, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_enable_software_slave_management(SPI1);
  spi_disable_ss_output(SPI1);
  spi_set_nss_high(SPI1);
  //spi_enable_software_slave_management(SPI1);

  //SPI_CR1(USART1) |= SPI_CR2_RXNEIE;
  //SPI_CR1(USART1) |= SPI_CR2_TXEIE;
  /* Enable SPI1 periph. */
  spi_enable(SPI1);
  gpio_set(GPIOA, GPIO4);
}

uint8_t spi_transfer(uint8_t data){
  while(SPI_SR(SPI1) & SPI_SR_BSY);
  SPI_DR(SPI1) = data;
  while(!SPI_SR(SPI1) & SPI_SR_RXNE);
  while(SPI_SR(SPI1) & SPI_SR_BSY);
  return SPI_DR(SPI1);
  //gpio_clear(GPIOA, GPIO4);
  //ring_write_ch(&spi_tx_ring, data);
  //SPI_CR2(SPI1) |= SPI_CR2_TXEIE;
}

/*
void spi1_isr(void){
	if (((SPI_CR2(SPI1) & SPI_CR2_RXNEIE) != 0) &&
	    ((SPI_SR(USART1) & SPI_SR_RXNE) != 0)) {
		ring_write_ch(&spi_rx_ring, spi_read(SPI1));
    }
	if (((SPI_CR2(SPI1) & SPI_CR2_TXEIE) != 0) &&
	    ((SPI_SR(USART1) & SPI_SR_TXE) != 0)) {
        int32_t data;

		data = ring_read_ch(&spi_tx_ring, NULL);

		if (data == -1) {
			SPI_CR2(SPI1) &= ~SPI_CR2_TXEIE;
            gpio_set(GPIOA, GPIO4);
		} else {
			spi_send(SPI1, data);
		}
    }
}
*/

