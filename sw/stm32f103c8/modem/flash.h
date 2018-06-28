#ifndef FLASHH
#define FLASHH

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>

/*flash operations*/
uint32_t flash_program_data(uint32_t start_address, uint8_t *input_data, uint16_t num_elements);
void flash_read_data(uint32_t start_address, uint16_t num_elements, uint8_t *output_data);

#define FLASH_WRONG_DATA_WRITTEN 0x80
#define RESULT_OK 0

//result = flash_program_data(0, str_send, SEND_BUFFER_SIZE);
//flash_read_data(0, SEND_BUFFER_SIZE, str_verify);

#endif
