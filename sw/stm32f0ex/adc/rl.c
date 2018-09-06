#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>
#include <string.h>
#include "microrl.h"
#include <stdio.h>

void print (const char * str);
void print_help ();
int execute (int argc, const char * const * argv);
char ** complet (int argc, const char * const * argv);
void sigint (void);

microrl_t rl;
microrl_t * prl = &rl;
// definition commands word
#define _CMD_HELP  "help"
#define _CMD_CLEAR "clear"
#define _CMD_LIST  "list"
#define _CMD_LISP  "lisp" // for demonstration completion on 'l + <TAB>'
#define _CMD_NAME  "name"
#define _CMD_VER   "version"
// sub commands for version command
#define _SCMD_MRL  "microrl"
#define _SCMD_DEMO "demo"

#define _NUM_OF_CMD 6
#define _NUM_OF_VER_SCMD 2

//available  commands
char * keyworld [] = {_CMD_HELP, _CMD_CLEAR, _CMD_LIST, _CMD_NAME, _CMD_VER, _CMD_LISP};
// version subcommands
char * ver_keyworld [] = {_SCMD_MRL, _SCMD_DEMO};

// array for comletion
char * compl_world [_NUM_OF_CMD + 1];

// 'name' var for store some string
#define _NAME_LEN 8
char name [_NAME_LEN];
int val;

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

uint8_t channel_array[] = { 1, 1, ADC_CHANNEL_TEMP};
static void adc_setup(void)
{
	rcc_periph_clock_enable(RCC_ADC);
	rcc_periph_clock_enable(RCC_GPIOA);

	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);

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
        microrl_insert_char(prl, usart_recv(USART1));
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

int main(void)
{
	uint16_t temp;
	clock_setup();
    systick_setup();
	gpio_setup();
	usart_setup();
	adc_setup();
    dac_setup();

	//microrl_init (prl, print);
	//microrl_set_execute_callback (prl, execute);
	//microrl_set_complete_callback (prl, complet);
	//microrl_set_sigint_callback (prl, sigint);

    while(1)
    {
        //gpio_toggle(GPIOC, GPIO8);	/* LED on/off */
		adc_start_conversion_regular(ADC1);
		while (!(adc_eoc(ADC1)));

		temp = adc_read_regular(ADC1);
        set_dac(temp);

        //printf("adc=%d\r\n",temp);
        //msleep(500);
    }
}


void print (const char * str)
{
    int i=0;
    while(str[i] != 0){
		gpio_toggle(GPIOC, GPIO8);	/* LED on/off */
        usart_send_blocking(USART1, str[i++]); /* USART1: Send byte. */
    }
}

void print_help ()
{
	print ("Use TAB key for completion\r\nCommand:\r\n");
	print ("\tversion {microrl | demo} - print version of microrl lib or version of this demo src\r\n");
	print ("\thelp  - this message\r\n");
	print ("\tclear - clear screen\r\n");
	print ("\tlist  - list all commands in tree\r\n");
	print ("\tname [string] - print 'name' value if no 'string', set name value to 'string' if 'string' present\r\n");
	print ("\tlisp - dummy command for demonstation auto-completion, while inputed 'l+<TAB>'\r\n");
}

int execute (int argc, const char * const * argv)
{
	int i = 0;
	// just iterate through argv word and compare it with your commands
	while (i < argc) {
		if (strcmp (argv[i], _CMD_HELP) == 0) {
			print ("microrl library based shell v 1.0\r\n");
			print_help ();        // print help
		} else if (strcmp (argv[i], _CMD_NAME) == 0) {
			if ((++i) < argc) { // if value preset
				if (strlen (argv[i]) < _NAME_LEN) {
					strcpy (name, argv[i]);
				} else {
					print ("name value too long!\r\n");
				}
			} else {
				print (name);
				print ("\r\n");
			}
		} else if (strcmp (argv[i], _CMD_VER) == 0) {
			if (++i < argc) {
				if (strcmp (argv[i], _SCMD_DEMO) == 0) {
					print ("demo v 1.0\r\n");
				} else if (strcmp (argv[i], _SCMD_MRL) == 0) {
					print ("microrl v 1.2\r\n");
				} else {
					print ((char*)argv[i]);
					print (" wrong argument, see help\r\n");
				}
			} else {
				print ("version needs 1 parametr, see help\r\n");
			}
		} else if (strcmp (argv[i], _CMD_CLEAR) == 0) {
			print ("\033[2J");    // ESC seq for clear entire screen
			print ("\033[H");     // ESC seq for move cursor at left-top corner
		} else if (strcmp (argv[i], _CMD_LIST) == 0) {
			print ("available command:\n");// print all command per line
			for (int i = 0; i < _NUM_OF_CMD; i++) {
				print ("\t");
				print (keyworld[i]);
				print ("\r\n");
			}
		} else {
			print ("command: '");
			print ((char*)argv[i]);
			print ("' Not found.\r\n");
		}
		i++;
	}
	return 0;
}

char ** complet (int argc, const char * const * argv)
{
	int j = 0;

	compl_world [0] = NULL;

	// if there is token in cmdline
	if (argc == 1) {
		// get last entered token
		char * bit = (char*)argv [argc-1];
		// iterate through our available token and match it
		for (int i = 0; i < _NUM_OF_CMD; i++) {
			// if token is matched (text is part of our token starting from 0 char)
			if (strstr(keyworld [i], bit) == keyworld [i]) {
				// add it to completion set
				compl_world [j++] = keyworld [i];
			}
		}
	}	else if ((argc > 1) && (strcmp (argv[0], _CMD_VER)==0)) { // if command needs subcommands
		// iterate through subcommand for command _CMD_VER array
		for (int i = 0; i < _NUM_OF_VER_SCMD; i++) {
			if (strstr (ver_keyworld [i], argv [argc-1]) == ver_keyworld [i]) {
				compl_world [j++] = ver_keyworld [i];
			}
		}
	} else { // if there is no token in cmdline, just print all available token
		for (; j < _NUM_OF_CMD; j++) {
			compl_world[j] = keyworld [j];
		}
	}

	// note! last ptr in array always must be NULL!!!
	compl_world [j] = NULL;
	// return set of variants
	return compl_world;
}

void sigint (void)
{
	print ("^C catched!\r\n");
}

