#ifndef SI4463H
#define SI4463H

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/iwdg.h>
#include <stdint.h>
#include <string.h>
#include "Si446x_defs.h"
#include "spi.h"
#include "systick.h"
#include "rl.h"

#define SI446X_MAX_PACKET_LEN	128 ///< Maximum packet length

#define SI446X_MAX_TX_POWER		127 ///< Maximum TX power (+20dBm/100mW)

#define SI446X_WUT_RUN	1 ///< Wake the microcontroller when the WUT expires
#define SI446X_WUT_BATT	2 ///< Take a battery measurement when the WUT expires
#define SI446X_WUT_RX	4 ///< Go into RX mode for LDC time (not supported yet!)

#define SI446X_GPIO_PULL_EN		0x40 ///< Pullup enable for GPIO pins
#define SI446X_GPIO_PULL_DIS	0x00 ///< Pullup disable for GPIO pins
#define SI446X_NIRQ_PULL_EN		0x40 ///< Pullup enable for NIRQ pin
#define SI446X_NIRQ_PULL_DIS	0x00 ///< Pullup disable for NIRQ pin
#define SI446X_SDO_PULL_EN		0x40 ///< Pullup enable for SDO pin
#define SI446X_SDO_PULL_DIS		0x00 ///< Pullup disable for SDO pin
#define SI446X_PIN_PULL_EN		0x40 ///< Pullup enable for any pin
#define SI446X_PIN_PULL_DIS		0x00 ///< Pullup disable for any pin

#define SI446X_GPIO_DRV_HIGH		0x00 ///< GPIO drive strength high
#define SI446X_GPIO_DRV_MED_HIGH	0x20 ///< GPIO drive strength medium-high
#define SI446X_GPIO_DRV_MED_LOW		0x40 ///< GPIO drive strength medium-low
#define SI446X_GPIO_DRV_LOW			0x60 ///< GPIO drive strength low

#define SI446X_PROP_GROUP_GLOBAL		0x00 ///< Property group global
#define SI446X_PROP_GROUP_INT			0x01 ///< Property group interrupts
#define SI446X_PROP_GROUP_FRR			0x02 ///< Property group fast response registers
#define SI446X_PROP_GROUP_PREAMBLE		0x10 ///< Property group preamble
#define SI446X_PROP_GROUP_SYNC			0x11 ///< Property group sync
#define SI446X_PROP_GROUP_PKT			0x12 ///< Property group packet config
#define SI446X_PROP_GROUP_MODEM			0x20 ///< Property group modem
#define SI446X_PROP_GROUP_MODEM_CHFLT	0x21 ///< Property group RX coefficients
#define SI446X_PROP_GROUP_PA			0x22 ///< Property group power amp
#define SI446X_PROP_GROUP_SYNTH			0x23 ///< Property group synthesizer 
#define SI446X_PROP_GROUP_MATCH			0x30 ///< Property group address match
#define SI446X_PROP_GROUP_FREQ_CONTROL	0x40 ///< Property group frequency control
#define SI446X_PROP_GROUP_RX_HOP		0x50 ///< Property group RX hop
#define SI446X_PROP_GROUP_PTI			0xF0 ///< Property group packet trace interface

/**
* @brief GPIO pin modes (see the Si446x API docs for what they all mean)
*/
typedef enum
{
	SI446X_GPIO_MODE_DONOTHING	= 0x00,
	SI446X_GPIO_MODE_TRISTATE	= 0x01,
	SI446X_GPIO_MODE_DRIVE0		= 0x02,
	SI446X_GPIO_MODE_DRIVE1		= 0x03,
	SI446X_GPIO_MODE_INPUT		= 0x04,
	SI446X_GPIO_MODE_32K_CLK	= 0x05,
	SI446X_GPIO_MODE_BOOT_CLK	= 0x06,
	SI446X_GPIO_MODE_DIV_CLK	= 0x07,
	SI446X_GPIO_MODE_CTS		= 0x08,
	SI446X_GPIO_MODE_INV_CTS	= 0x09,
	SI446X_GPIO_MODE_CMD_OVERLAP	= 0x0A,
	SI446X_GPIO_MODE_SDO		= 0x0B,
	SI446X_GPIO_MODE_POR		= 0x0C,
	SI446X_GPIO_MODE_CAL_WUT	= 0x0D,
	SI446X_GPIO_MODE_WUT		= 0x0E,
	SI446X_GPIO_MODE_EN_PA		= 0x0F,
	SI446X_GPIO_MODE_TX_DATA_CLK	= 0x10,
	SI446X_GPIO_MODE_RX_DATA_CLK	= 0x11,
	SI446X_GPIO_MODE_EN_LNA			= 0x12,
	SI446X_GPIO_MODE_TX_DATA		= 0x13,
	SI446X_GPIO_MODE_RX_DATA		= 0x14,
	SI446X_GPIO_MODE_RX_RAW_DATA		= 0x15,
	SI446X_GPIO_MODE_ANTENNA_1_SW		= 0x16,
	SI446X_GPIO_MODE_ANTENNA_2_SW		= 0x17,
	SI446X_GPIO_MODE_VALID_PREAMBLE		= 0x18,
	SI446X_GPIO_MODE_INVALID_PREAMBLE	= 0x19,
	SI446X_GPIO_MODE_SYNC_WORD_DETECT	= 0x1A,
	SI446X_GPIO_MODE_CCA			= 0x1B,
	SI446X_GPIO_MODE_IN_SLEEP		= 0x1C,
	SI446X_GPIO_MODE_PKT_TRACE		= 0x1D,
// Nothing for 0x1E (30)
	SI446X_GPIO_MODE_TX_RX_DATA_CLK	= 0x1F,
	SI446X_GPIO_MODE_TX_STATE		= 0x20,
	SI446X_GPIO_MODE_RX_STATE		= 0x21,
	SI446X_GPIO_MODE_RX_FIFO_FULL	= 0x22,
	SI446X_GPIO_MODE_TX_FIFO_EMPTY	= 0x23,
	SI446X_GPIO_MODE_LOW_BATT		= 0x24,
	SI446X_GPIO_MODE_CCA_LATCH		= 0x25,
	SI446X_GPIO_MODE_HOPPED			= 0x26,
	SI446X_GPIO_MODE_HOP_TABLE_WRAP	= 0x27
} si446x_gpio_mode_t;

/**
* @brief NIRQ pin modes (see the Si446x API docs for what they all mean)
*/
typedef enum
{
	SI446X_NIRQ_MODE_DONOTHING	= 0x00,
	SI446X_NIRQ_MODE_TRISTATE	= 0x01,
	SI446X_NIRQ_MODE_DRIVE0		= 0x02,
	SI446X_NIRQ_MODE_DRIVE1		= 0x03,
	SI446X_NIRQ_MODE_INPUT		= 0x04,
//	SI446X_NIRQ_MODE_32K_CLK	= 0x05,
//	SI446X_NIRQ_MODE_BOOT_CLK	= 0x06,
	SI446X_NIRQ_MODE_DIV_CLK	= 0x07,
	SI446X_NIRQ_MODE_CTS		= 0x08,
//	SI446X_NIRQ_MODE_INV_CTS	= 0x09,
//	SI446X_NIRQ_MODE_CMD_OVERLAP	= 0x0A,
	SI446X_NIRQ_MODE_SDO		= 0x0B,
	SI446X_NIRQ_MODE_POR		= 0x0C,
//	SI446X_NIRQ_MODE_CAL_WUT	= 0x0D,
//	SI446X_NIRQ_MODE_WUT		= 0x0E,
	SI446X_NIRQ_MODE_EN_PA		= 0x0F,
	SI446X_NIRQ_MODE_TX_DATA_CLK	= 0x10,
	SI446X_NIRQ_MODE_RX_DATA_CLK	= 0x11,
	SI446X_NIRQ_MODE_EN_LNA			= 0x12,
	SI446X_NIRQ_MODE_TX_DATA		= 0x13,
	SI446X_NIRQ_MODE_RX_DATA		= 0x14,
	SI446X_NIRQ_MODE_RX_RAW_DATA	= 0x15,
	SI446X_NIRQ_MODE_ANTENNA_1_SW	= 0x16,
	SI446X_NIRQ_MODE_ANTENNA_2_SW	= 0x17,
	SI446X_NIRQ_MODE_VALID_PREAMBLE	= 0x18,
	SI446X_NIRQ_MODE_INVALID_PREAMBLE	= 0x19,
	SI446X_NIRQ_MODE_SYNC_WORD_DETECT	= 0x1A,
	SI446X_NIRQ_MODE_CCA			= 0x1B,
//	SI446X_NIRQ_MODE_IN_SLEEP		= 0x1C,
	SI446X_NIRQ_MODE_PKT_TRACE		= 0x1D,
// Nothing for 0x1E (30)
	SI446X_NIRQ_MODE_TX_RX_DATA_CLK	= 0x1F,
//	SI446X_NIRQ_MODE_TX_STATE		= 0x20,
//	SI446X_NIRQ_MODE_RX_STATE		= 0x21,
//	SI446X_NIRQ_MODE_RX_FIFO_FULL	= 0x22,
//	SI446X_NIRQ_MODE_TX_FIFO_EMPTY	= 0x23,
//	SI446X_NIRQ_MODE_LOW_BATT		= 0x24,
//	SI446X_NIRQ_MODE_CCA_LATCH		= 0x25,
//	SI446X_NIRQ_MODE_HOPPED			= 0x26,
	SI446X_NIRQ_MODE_NIRQ			= 0x27
} si446x_nirq_mode_t;

/**
* @brief SDO pin modes (see the Si446x API docs for what they all mean)
*/
typedef enum
{
	SI446X_SDO_MODE_DONOTHING	= 0x00,
	SI446X_SDO_MODE_TRISTATE	= 0x01,
	SI446X_SDO_MODE_DRIVE0		= 0x02,
	SI446X_SDO_MODE_DRIVE1		= 0x03,
	SI446X_SDO_MODE_INPUT		= 0x04,
	SI446X_SDO_MODE_32K_CLK	= 0x05,
//	SI446X_SDO_MODE_BOOT_CLK	= 0x06,
	SI446X_SDO_MODE_DIV_CLK	= 0x07,
	SI446X_SDO_MODE_CTS		= 0x08,
//	SI446X_SDO_MODE_INV_CTS	= 0x09,
//	SI446X_SDO_MODE_CMD_OVERLAP	= 0x0A,
	SI446X_SDO_MODE_SDO		= 0x0B,
	SI446X_SDO_MODE_POR		= 0x0C,
//	SI446X_SDO_MODE_CAL_WUT	= 0x0D,
	SI446X_SDO_MODE_WUT		= 0x0E,
	SI446X_SDO_MODE_EN_PA		= 0x0F,
	SI446X_SDO_MODE_TX_DATA_CLK	= 0x10,
	SI446X_SDO_MODE_RX_DATA_CLK	= 0x11,
	SI446X_SDO_MODE_EN_LNA			= 0x12,
	SI446X_SDO_MODE_TX_DATA			= 0x13,
	SI446X_SDO_MODE_RX_DATA			= 0x14,
	SI446X_SDO_MODE_RX_RAW_DATA		= 0x15,
	SI446X_SDO_MODE_ANTENNA_1_SW		= 0x16,
	SI446X_SDO_MODE_ANTENNA_2_SW		= 0x17,
	SI446X_SDO_MODE_VALID_PREAMBLE		= 0x18,
	SI446X_SDO_MODE_INVALID_PREAMBLE	= 0x19,
	SI446X_SDO_MODE_SYNC_WORD_DETECT	= 0x1A,
	SI446X_SDO_MODE_CCA			= 0x1B,
//	SI446X_SDO_MODE_IN_SLEEP		= 0x1C,
//	SI446X_SDO_MODE_PKT_TRACE		= 0x1D,
// Nothing for 0x1E (30)
//	SI446X_SDO_MODE_TX_RX_DATA_CLK	= 0x1F,
//	SI446X_SDO_MODE_TX_STATE		= 0x20,
//	SI446X_SDO_MODE_RX_STATE		= 0x21,
//	SI446X_SDO_MODE_RX_FIFO_FULL	= 0x22,
//	SI446X_SDO_MODE_TX_FIFO_EMPTY	= 0x23,
//	SI446X_SDO_MODE_LOW_BATT		= 0x24,
//	SI446X_SDO_MODE_CCA_LATCH		= 0x25,
//	SI446X_SDO_MODE_HOPPED			= 0x26,
//	SI446X_SDO_MODE_HOP_TABLE_WRAP	= 0x27
} si446x_sdo_mode_t;

/**
* @brief Data structure for storing chip info from ::Si446x_getInfo()
*/
typedef struct {
	uint8_t chipRev; ///< Chip revision
	uint16_t part; ///< Part ID
	uint8_t partBuild; ///< Part build
	uint16_t id; ///< ID
	uint8_t customer; ///< Customer
	uint8_t romId; ///< ROM ID (3 = revB1B, 6 = revC2A)
	
	uint8_t revExternal; ///< Revision external
	uint8_t revBranch; ///< Revision branch
	uint8_t revInternal; ///< Revision internal
	uint16_t patch; ///< Patch
	uint8_t func; ///< Function
} si446x_info_t;

/**
* @brief GPIOs for passing to ::Si446x_writeGPIO(), or for masking when reading from ::Si446x_readGPIO()
*/
typedef enum
{
	SI446X_GPIO0 = 0, ///< GPIO 1
	SI446X_GPIO1 = 1, ///< GPIO 2
	SI446X_GPIO2 = 2, ///< GPIO 3
	SI446X_GPIO3 = 3, ///< GPIO 4
	SI446X_NIRQ = 4, ///< NIRQ
	SI446X_SDO = 5 ///< SDO
} si446x_gpio_t;

/**
* @brief Radio states, returned from ::Si446x_getState()
*/
typedef enum
{
	SI446X_STATE_NOCHANGE	= 0x00,
	SI446X_STATE_SLEEP		= 0x01, ///< This will never be returned since SPI activity will wake the radio into ::SI446X_STATE_SPI_ACTIVE
	SI446X_STATE_SPI_ACTIVE	= 0x02,
	SI446X_STATE_READY		= 0x03,
	SI446X_STATE_READY2		= 0x04, ///< Will return as ::SI446X_STATE_READY
	SI446X_STATE_TX_TUNE	= 0x05, ///< Will return as ::SI446X_STATE_TX
	SI446X_STATE_RX_TUNE	= 0x06, ///< Will return as ::SI446X_STATE_RX
	SI446X_STATE_TX			= 0x07,
	SI446X_STATE_RX			= 0x08
} si446x_state_t;

/**
* @brief Initialise, must be called before anything else!
*
* @return (none)
*/
void Si446x_init(void);

/**
* @brief Get chip info, see ::si446x_info_t
*
* @see ::si446x_info_t
* @param [info] Pointer to allocated ::si446x_info_t struct to place data into
* @return (none)
*/
void Si446x_getInfo(si446x_info_t* info);

/**
* @brief Get the current RSSI, the chip needs to be in receive mode for this to work
*
* @return The current RSSI in dBm (usually between -130 and 0)
*/
int16_t Si446x_getRSSI(void);

/**
* @brief Set the transmit power. The output power does not follow the \p pwr value, see the Si446x datasheet for a pretty graph
*
* 0 = -32dBm (<1uW)\n
* 7 = 0dBm (1mW)\n
* 12 = 5dBm (3.2mW)\n
* 22 = 10dBm (10mW)\n
* 40 = 15dBm (32mW)\n
* 100 = 20dBm (100mW)
*
* @param [pwr] A value from 0 to 127
* @return (none)
*/
void Si446x_setTxPower(uint8_t pwr);

/**
 * MODEM_RSSI_COMP property is added to the internally measured RSSI level, and thus changes the 
 * RSSI value returned by the GET_MODEM_STATUS command. The MODEM_RSSI_COMP value is in increments 
 * of 1 dB, and thus changing the compensation value by 1 code will result in the RSSI value returned 
 * by the GET_MODEM_STATUS command changing by 2 codes.
 *
 * The default value of the MODEM_RSSI_COMP property is 0x40 = 64 decimal. This is a sufficient offset 
 * value such that the returned RSSI value for measuring noise (i.e., no signal) is slightly greater 
 * than zero. A larger compensation value will adjust the returned RSSI value upwards, and a lower 
 * value will adjust the RSSI value downwards.
 */
void Si446x_setRSSIComp(uint8_t comp);


/**
 * The chip provides internal adjustable capacitor banks for the purpose of 
 * tuning the frequency of the crystal oscillator.
 * The total frequency adjustment range (for a typical 30 MHz crystal blank) is approximately ±100 ppm.
 * Range 0x00 - 0x7F
 */
void Si446x_setXOTune(uint8_t tune);
uint8_t Si446x_getXOTune(void);


void Si446x_setFreq(double freq);

/**
* @brief Read received data from FIFO
*
* @param [buff] Pointer to buffer to place data
* @param [len] Number of bytes to read, make sure not to read more bytes than what the FIFO has stored. The number of bytes that can be read is passed in the ::SI446X_CB_RXCOMPLETE() callback.
* @return (none)
*/
void Si446x_read(void* buff, uint8_t len);

/**
* @brief Transmit a packet
*
* @param [packet] Pointer to packet data
* @param [len] Number of bytes to transmit, maximum of ::SI446X_MAX_PACKET_LEN If configured for fixed length packets then this parameter is ignored and the length is set by ::SI446X_FIXED_LENGTH in Si446x_config.h
* @param [channel] Channel to transmit data on (0 - 255)
* @param [onTxFinish] What state to enter when the packet has finished transmitting. Usually ::SI446X_STATE_SLEEP or ::SI446X_STATE_RX
* @return 0 on failure (already transmitting), 1 on success (has begun transmitting)
*/
uint8_t Si446x_TX(void* packet, uint8_t len, uint8_t channel, si446x_state_t onTxFinish);

/**
* @brief Enter receive mode
*
* Entering RX mode will abort any transmissions happening at the time
*
* @param [channel] Channel to listen to (0 - 255)
* @return (none)
*/
void Si446x_RX(uint8_t channel);

/*-*
* @brief Changes will be applied next time the radio enters RX mode (NOT SUPPORTED)
*
* @param [mode] TODO
* @param [address] TODO
* @return (none)
*/
//void Si446x_setAddress(si446x_addrMode_t mode, uint8_t address);

/**
* @brief Enter sleep mode
*
* If WUT is enabled then the radio will keep the internal 32KHz RC enabled with a current consumption of 740nA, otherwise the current consumption will be 40nA without WUT.
* Sleep will fail if the radio is currently transmitting.
*
* @note Any SPI communications with the radio will wake the radio into ::SI446X_STATE_SPI_ACTIVE mode. ::Si446x_sleep() will need to called again to put it back into sleep mode.
*
* @return 0 on failure (busy transmitting something), 1 on success
*/
uint8_t Si446x_sleep(void);

/**
* @brief Get the radio status
*
* @see ::si446x_state_t
* @return The current radio status
*/
si446x_state_t Si446x_getState(void);

void Si446x_SERVICE(void);
#endif
