#include "si4463.h"
#include "radio_config.h"

#define IRQ_PIN GPIO3
#define IRQ_PRT GPIOA
#define IRQ_RCC RCC_GPIOA

#define SDN_PIN GPIO0
#define SDN_PRT GPIOA
#define SDN_RCC RCC_GPIOA

#define CSN_PIN GPIO4
#define CSN_PRT GPIOA
#define CSN_RCC RCC_GPIOA

#define SI446X_IDLE_MODE SI446X_STATE_READY
#define SI446X_FIXED_LENGTH 64
#define IDLE_STATE SI446X_IDLE_MODE

#define IRQ_PACKET				0
#define IRQ_MODEM				1
#define IRQ_CHIP				2

static const uint8_t config[] = RADIO_CONFIGURATION_DATA_ARRAY;
static volatile uint8_t enabledInterrupts[3];

#define rssi_dBm(val)			((val / 2) - 134)

static inline uint8_t cselect(void)
{
    gpio_clear(CSN_PRT, CSN_PIN);
	return 1;
}

static inline uint8_t cdeselect(void)
{
    gpio_set(CSN_PRT, CSN_PIN);
	return 0;
}

#define CHIPSELECT()	for(uint8_t _cs = cselect(); _cs; _cs = cdeselect())

// If an interrupt might do some SPI communications with another device then we
// need to turn global interrupts off while communicating with the radio.
// Otherwise, just turn off our own radio interrupt while doing SPI stuff.
static inline uint8_t interrupt_off(void){
	exti_disable_request(EXTI3);
    return 1;
}
static inline uint8_t interrupt_on(void){
	exti_enable_request(EXTI3);
    return 0;
}
#define SI446X_ATOMIC() for(uint8_t _cs2 = interrupt_off(); _cs2; _cs2 = interrupt_on())
#define SI446X_NO_INTERRUPT() for(uint8_t _cs3 = interrupt_off(); _cs3; _cs3 = interrupt_on())

static void __empty_callback0(void){}
static void __empty_callback1(int16_t param1){(void)(param1);}
void __attribute__((weak, alias ("__empty_callback0"))) SI446X_CB_IRQ(void);
void __attribute__((weak, alias ("__empty_callback0"))) SI446X_CB_CMDTIMEOUT(void);
void __attribute__((weak, alias ("__empty_callback1"))) SI446X_CB_RXBEGIN(int16_t rssi);
void __attribute__((weak)) SI446X_CB_RXCOMPLETE(uint8_t length, int16_t rssi){(void)(length);(void)(rssi);}
void __attribute__((weak, alias ("__empty_callback1"))) SI446X_CB_RXINVALID(int16_t rssi);
void __attribute__((weak, alias ("__empty_callback0"))) SI446X_CB_SENT(void);
void __attribute__((weak, alias ("__empty_callback0"))) SI446X_CB_WUT(void);
void __attribute__((weak, alias ("__empty_callback0"))) SI446X_CB_LOWBATT(void);
void __attribute__((weak, alias ("__empty_callback0"))) SI446X_CB_RXINVALIDSYNC(void);

static void exti_setup(void);

// Read CTS and if its ok then read the command buffer
static uint8_t getResponse(void* buff, uint8_t len)
{
	uint8_t cts = 0;

	SI446X_ATOMIC()
	{
		CHIPSELECT()
		{
			// Send command
			spi_transfer(SI446X_CMD_READ_CMD_BUFF);

			// Get CTS value
			cts = (spi_transfer(0xFF) == 0xFF);

			if(cts)
			{
				// Get response data
				for(uint8_t i=0;i<len;i++)
					((uint8_t*)buff)[i] = spi_transfer(0xFF);
			}
		}
	}
	return cts;
}

// Keep trying to read the command buffer, with timeout of around 500ms
static uint8_t waitForResponse(void* out, uint8_t outLen, uint8_t useTimeout)
{
    int i=0;
	// With F_CPU at 8MHz and SPI at 4MHz each check takes about 7us + 10us delay
	uint32_t timeout = 40000;
	while(!getResponse(out, outLen))
	{
        //f=72Mhz / 72 000 0 ~ 10us delay
		for (i = 0; i < 120; i++)	/* Wait a bit. */
			__asm__("nop");
		if(useTimeout && !--timeout)
		{
			SI446X_CB_CMDTIMEOUT();
			return 0;
		}
	}
	return 1;
}

static void doAPI(void* data, uint8_t len, void* out, uint8_t outLen)
{
	SI446X_NO_INTERRUPT()
	{
		if(waitForResponse(NULL, 0, 1)) // Make sure it's ok to send a command
		{
			SI446X_ATOMIC()
			{
				CHIPSELECT()
				{
					for(uint8_t i=0;i<len;i++)
						spi_transfer(((uint8_t*)data)[i]); // (pgm_read_byte(&((uint8_t*)data)[i]));
				}
			}

			if(((uint8_t*)data)[0] == SI446X_CMD_IRCAL) // If we're doing an IRCAL then wait for its completion without a timeout since it can sometimes take a few seconds
				waitForResponse(NULL, 0, 0);
			else if(out != NULL) // If we have an output buffer then read command response into it
				waitForResponse(out, outLen, 1);
		}
	}
}

// Configure a bunch of properties (up to 12 properties in one go)
static void setProperties(uint16_t prop, void* values, uint8_t len)
{
	// len must not be greater than 12

	uint8_t data[16] = {
		SI446X_CMD_SET_PROPERTY,
		(uint8_t)(prop>>8),
		len,
		(uint8_t)prop
	};

	// Copy values into data, starting at index 4
	memcpy(data + 4, values, len);

	doAPI(data, len + 4, NULL, 0);
}

// Set a single property
static inline void setProperty(uint16_t prop, uint8_t value)
{
	setProperties(prop, &value, 1);
}

// Read a bunch of properties
static void getProperties(uint16_t prop, void* values, uint8_t len)
{
	uint8_t data[] = {
		SI446X_CMD_GET_PROPERTY,
		(uint8_t)(prop>>8),
		len,
		(uint8_t)prop
	};

	doAPI(data, sizeof(data), values, len);
}

// Read a single property
static inline uint8_t getProperty(uint16_t prop)
{
	uint8_t val;
	getProperties(prop, &val, 1);
	return val;
}

// Do an ADC conversion
static uint16_t getADC(uint8_t adc_en, uint8_t adc_cfg, uint8_t part)
{
	uint8_t data[6] = {
		SI446X_CMD_GET_ADC_READING,
		adc_en,
		adc_cfg
	};
	doAPI(data, 3, data, 6);
	return (data[part]<<8 | data[part + 1]);
}

// Read a fast response register
static uint8_t getFRR(uint8_t reg)
{
	uint8_t frr = 0;
	SI446X_ATOMIC()
	{
		CHIPSELECT()
		{
			spi_transfer(reg);
			frr = spi_transfer(0xFF);
		}
	}
	return frr;
}

// Ge the patched RSSI from the beginning of the packet
static int16_t getLatchedRSSI(void)
{
	uint8_t frr = getFRR(SI446X_CMD_READ_FRR_A);
	int16_t rssi = rssi_dBm(frr);
	return rssi;
}

// Get current radio state
static si446x_state_t getState(void)
{
	uint8_t state = getFRR(SI446X_CMD_READ_FRR_B);
	if(state == SI446X_STATE_TX_TUNE)
		state = SI446X_STATE_TX;
	else if(state == SI446X_STATE_RX_TUNE)
		state = SI446X_STATE_RX;
	else if(state == SI446X_STATE_READY2)
		state = SI446X_STATE_READY;
	return (si446x_state_t)state;
}

// Set new state
static void setState(si446x_state_t newState)
{
	uint8_t data[] = {
		SI446X_CMD_CHANGE_STATE,
		newState
	};
	doAPI(data, sizeof(data), NULL, 0);
}

// Clear RX and TX FIFOs
static void clearFIFO(void)
{
	// 'static const' saves 20 bytes of flash here, but uses 2 bytes of RAM
	static const uint8_t clearFifo[] = {
		SI446X_CMD_FIFO_INFO,
		SI446X_FIFO_CLEAR_RX | SI446X_FIFO_CLEAR_TX
	};
	doAPI((uint8_t*)clearFifo, sizeof(clearFifo), NULL, 0);
}

// Read pending interrupts
// Reading interrupts will also clear them
// Buff should either be NULL (just clear interrupts) or a buffer of atleast 8 bytes for storing statuses
static void interrupt(void* buff)
{
	uint8_t data = SI446X_CMD_GET_INT_STATUS;
	doAPI(&data, sizeof(data), buff, 8);
}

// Similar to interrupt() but with the option of not clearing certain interrupt flags
static void interrupt2(void* buff, uint8_t clearPH, uint8_t clearMODEM, uint8_t clearCHIP)
{
	uint8_t data[] = {
		SI446X_CMD_GET_INT_STATUS,
		clearPH,
		clearMODEM,
		clearCHIP
	};
	doAPI(data, sizeof(data), buff, 8);
}

// Reset the RF chip
static void resetDevice(void)
{
    gpio_set(SDN_PRT, SDN_PIN);
	msleep(50);
    gpio_clear(SDN_PRT, SDN_PIN);
	msleep(50);
}

static void applyStartupConfig(void)
{
	uint8_t buff[17];
	for(uint16_t i=0;i<sizeof(config);i++)
	{
		memcpy(buff, &config[i], sizeof(buff));
		doAPI(&buff[1], buff[0], NULL, 0);
		i += buff[0];
	}
}

void Si446x_init()
{
	rcc_periph_clock_enable(SDN_RCC);
	rcc_periph_clock_enable(CSN_RCC);

	gpio_set_mode(SDN_PRT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, SDN_PIN);
	gpio_set_mode(CSN_PRT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, CSN_PIN);

    gpio_set(CSN_PRT, CSN_PIN);
    gpio_clear(SDN_PRT, SDN_PIN);

    spi_setup();

	resetDevice();
	applyStartupConfig();
	interrupt(NULL);
	Si446x_sleep();

	enabledInterrupts[IRQ_PACKET] = (1<<SI446X_PACKET_RX_PEND) | (1<<SI446X_CRC_ERROR_PEND);
    enabledInterrupts[IRQ_MODEM] = (1<<SI446X_INVALID_SYNC_PEND);
	//enabledInterrupts[IRQ_MODEM] = (1<<SI446X_SYNC_DETECT_PEND);

    exti_setup();
}

void Si446x_getInfo(si446x_info_t* info)
{
	uint8_t data[8] = {
		SI446X_CMD_PART_INFO
	};
	doAPI(data, 1, data, 8);

	info->chipRev	= data[0];
	info->part		= (data[1]<<8) | data[2];
	info->partBuild	= data[3];
	info->id		= (data[4]<<8) | data[5];
	info->customer	= data[6];
	info->romId		= data[7];

	data[0] = SI446X_CMD_FUNC_INFO;
	doAPI(data, 1, data, 6);

	info->revExternal	= data[0];
	info->revBranch		= data[1];
	info->revInternal	= data[2];
	info->patch			= (data[3]<<8) | data[4];
	info->func			= data[5];
}

int16_t Si446x_getRSSI()
{
	uint8_t data[3] = {
		SI446X_CMD_GET_MODEM_STATUS,
		0xFF
	};
	doAPI(data, 2, data, 3);
	int16_t rssi = rssi_dBm(data[2]);
	return rssi;
}

si446x_state_t Si446x_getState()
{
	// TODO what about the state change delay with transmitting?
	return getState();
}

void Si446x_setTxPower(uint8_t pwr)
{
	setProperty(SI446X_PA_PWR_LVL, pwr);
}

void Si446x_setRSSIComp(uint8_t comp)
{
	setProperty(SI446X_MODEM_RSSI_COMP, comp);
}

void Si446x_setXOTune(uint8_t tune)
{
	setProperty(SI446X_GLOBAL_XO_TUNE, tune);
}

uint8_t Si446x_getXOTune()
{
	getProperty(SI446X_GLOBAL_XO_TUNE);
}

void Si446x_setFreq(double freq)
{
    if(freq < 142 || freq > 1050) return;
    uint8_t clkgen = 0;
    uint8_t outdiv = 0;
    if(freq < 194) {clkgen = 5; outdiv = 24;}      //DIV/24
    else if(freq < 273) {clkgen = 4; outdiv = 16;}//DIV/16
    else if(freq < 385) {clkgen = 3; outdiv = 12;}//DIV/12
    else if(freq < 546) {clkgen = 2; outdiv = 8;}//DIV/8
    else if(freq < 760) {clkgen = 1; outdiv = 6;}//DIV/6
    else {clkgen = 0; outdiv = 4;}//DIV/4
    clkgen |= 8; // SYS_SEL = 1, force recalibration
	setProperty(SI446X_MODEM_CLKGEN_BAND, clkgen);

    double div = ( 2.0f*26000000.0f/(double)(outdiv) );
    double fraction = freq/div;
    uint8_t inte = (int)(fraction) - 1;
    uint32_t frac = (uint32_t)((fraction - (double)inte) * (double)(2^19));
	setProperty(SI446X_FREQ_CONTROL_INTE, inte);
    uint8_t properties[3];
    properties[0] = (frac&0xF0000) >> 16;
    properties[1] = (frac&0x0FF00) >> 8;
    properties[2] = (frac&0x000FF);
    setProperties(SI446X_FREQ_CONTROL_FRAC, properties, sizeof(properties));
}

uint8_t Si446x_sleep()
{
	if(getState() == SI446X_STATE_TX)
		return 0;
	setState(SI446X_STATE_SLEEP);
	return 1;
}

void Si446x_read(void* buff, uint8_t len)
{
	SI446X_ATOMIC()
	{
		CHIPSELECT()
		{
			spi_transfer(SI446X_CMD_READ_RX_FIFO);
			for(uint8_t i=0;i<len;i++)
				((uint8_t*)buff)[i] = spi_transfer(0xFF);
		}
	}
}

uint8_t Si446x_TX(void* packet, uint8_t len, uint8_t channel, si446x_state_t onTxFinish)
{
    //fixed length
	// Stop the unused parameter warning
	((void)(len));

	SI446X_NO_INTERRUPT()
	{
		if(getState() == SI446X_STATE_TX) // Already transmitting
			return 0;

		// TODO collision avoid or maybe just do collision detect (RSSI jump)

		setState(IDLE_STATE);
		clearFIFO();
		interrupt2(NULL, 0, 0, 0xFF);

		SI446X_ATOMIC()
		{
			// Load data to FIFO
			CHIPSELECT()
			{
				spi_transfer(SI446X_CMD_WRITE_TX_FIFO);
				for(uint8_t i=0;i<SI446X_FIXED_LENGTH;i++)
					spi_transfer(((uint8_t*)packet)[i]);
			}
		}

		// Begin transmit
		uint8_t data[] = {
			SI446X_CMD_START_TX,
			channel,
			(uint8_t)(onTxFinish<<4),
			0,
			SI446X_FIXED_LENGTH,
			0,
			0
		};
		doAPI(data, sizeof(data), NULL, 0);
	}
	return 1;
}

void Si446x_RX(uint8_t channel)
{
	SI446X_NO_INTERRUPT()
	{
		setState(IDLE_STATE);
		clearFIFO();
		//fix_invalidSync_irq(0);
		//setProperty(SI446X_PKT_FIELD_2_LENGTH_LOW, MAX_PACKET_LEN); // TODO ?
		interrupt2(NULL, 0, 0, 0xFF); // TODO needed?

		// TODO RX timeout to sleep if WUT LDC enabled

		uint8_t data[] = {
			SI446X_CMD_START_RX,
			channel,
			0,
			0,
			SI446X_FIXED_LENGTH,
			SI446X_STATE_NOCHANGE, // RX Timeout
			SI446X_STATE_RX, // RX Valid
			SI446X_STATE_RX// IDLE_STATE // RX Invalid (using SI446X_STATE_SLEEP for the INVALID_SYNC fix)
		};
		doAPI(data, sizeof(data), NULL, 0);
	}
}

static void exti_setup(void)
{
	/* Enable IRQ port clock. */
	rcc_periph_clock_enable(IRQ_RCC);

	/* Enable AFIO clock. */
	rcc_periph_clock_enable(RCC_AFIO);

	/* Enable EXTI3 interrupt. */
	nvic_enable_irq(NVIC_EXTI3_IRQ);

	gpio_set_mode(IRQ_PRT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, IRQ_PIN);
    gpio_set(IRQ_PRT, IRQ_PIN); //pull up

	/* Configure the EXTI subsystem. */
	exti_select_source(EXTI3, IRQ_PRT);
	exti_set_trigger(EXTI3, EXTI_TRIGGER_FALLING);
	exti_enable_request(EXTI3);
}

void exti3_isr(void)
{
	exti_reset_request(EXTI3);
    //gpio_toggle(GPIOC, GPIO13);
    //SI446X_CB_IRQ();
    Si446x_SERVICE();
}
void Si446x_SERVICE(void)
{
	uint8_t interrupts[8];


	interrupt(interrupts);

	// TODO remove
	//SI446X_CB_DEBUG(interrupts);

	//printf_P(PSTR("INT %hhu/%hhu %hhu/%hhu %hhu/%hhu\n"), interrupts[2], interrupts[3], interrupts[4], interrupts[5], interrupts[6], interrupts[7]);

	// We could read the enabled interrupts properties instead of keep their states in RAM, but that would be much slower
	interrupts[2] &= enabledInterrupts[IRQ_PACKET];
	interrupts[4] &= enabledInterrupts[IRQ_MODEM];
	interrupts[6] &= enabledInterrupts[IRQ_CHIP];

	// Valid PREAMBLE and SYNC, packet data now begins
	if(interrupts[4] & (1<<SI446X_SYNC_DETECT_PEND))
	{
		//fix_invalidSync_irq(1);
		SI446X_CB_RXBEGIN(getLatchedRSSI());
	}
/*
	// Disable INVALID_SYNC
	if((interrupts[4] & (1<<SI446X_INVALID_SYNC_PEND)) || (interrupts[2] & ((1<<SI446X_PACKET_SENT_PEND)|(1<<SI446X_CRC_ERROR_PEND))))
	{
		//fix_invalidSync_irq(0);
	}
*/

	// INVALID_SYNC detected, sometimes the radio gets messed up in this state and requires a RX restart
	if(interrupts[4] & (1<<SI446X_INVALID_SYNC_PEND))
		SI446X_CB_RXINVALIDSYNC();

	// Valid packet
	if(interrupts[2] & (1<<SI446X_PACKET_RX_PEND))
	{
		uint8_t len = SI446X_FIXED_LENGTH;
		SI446X_CB_RXCOMPLETE(len, getLatchedRSSI());
	}

	// Corrupted packet
	// NOTE: This will still be called even if the address did not match, but the packet failed the CRC
	// This will not be called if the address missed, but the packet passed CRC
	if(interrupts[2] & (1<<SI446X_CRC_ERROR_PEND))
	{
		SI446X_CB_RXINVALID(getLatchedRSSI()); // TODO remove RSSI stuff for invalid packets, entering SLEEP mode looses the latched value?
	}

	// Packet sent
	if(interrupts[2] & (1<<SI446X_PACKET_SENT_PEND))
		SI446X_CB_SENT();

	if(interrupts[6] & (1<<SI446X_LOW_BATT_PEND))
		SI446X_CB_LOWBATT();

	if(interrupts[6] & (1<<SI446X_WUT_PEND))
		SI446X_CB_WUT();
}
