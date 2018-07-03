#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/iwdg.h>
#include <string.h>
#include <stdio.h>
//#include "syscalls.h"
#include "microrl.h"
#include "uart.h"
#include "spi.h"
#include "flash.h"
#include "si4463.h"
#include "systick.h"
#include "rl.h"

#define MAX_PACKET_SIZE 64
#define TIMEOUT 100
#define TX_CIRCULAR_LENGTH 90
#define RX_CIRCULAR_LENGTH 30
//#define EEPROM_TEST 0xABCDEF00
//#define CHANNEL_TRESHOLD -80.0
//Special Address
//#define BROADCAST 255
//Commands
#define CMD_PING            1
#define CMD_PING_RESP       2
//Packet Format
#define SOURCE_ADDR         2
#define DEST_ADDR           3
#define CMD                 4

EEPROM_STRUCT eeStruct;
extern volatile uint32_t system_millis;

volatile uint8_t timer_cnt = 0;
volatile uint8_t rxbuffer[RX_CIRCULAR_LENGTH][MAX_PACKET_SIZE];
volatile uint8_t txbuffer[TX_CIRCULAR_LENGTH][MAX_PACKET_SIZE];
volatile uint8_t rxBufferCnt;
volatile uint8_t rxReadBufCnt;
volatile uint8_t rxOverflowed;
volatile uint8_t txBufferCnt;
volatile uint8_t txReadBufCnt;
volatile uint8_t txCounter;
uint32_t lastSerialDataTime;
uint8_t commandModeEnabled = 0;
volatile uint8_t specialPacketArrivedFlag = 0;
volatile uint8_t specialPacketBuffer = 0;
volatile int16_t rxRSSI = -120;
volatile uint8_t invalidRxFlag = 0;
volatile uint8_t cmdTimeoutFlag = 0;
volatile uint8_t invalidSyncFlag = 0;
volatile uint32_t rxPingRespCnt = 0;

uint8_t rxComplateFlag = 0;
uint8_t si4463IRQFlag = 0;

extern microrl_t rl;
extern microrl_t * prl;

void send_ping(uint8_t addr);
void send_ping_resp(uint8_t addr, int16_t rssi);
void process_special_packet(void);

void SI446X_CB_RXCOMPLETE(uint8_t length, int16_t rssi)
{
    rxRSSI = rssi;
    Si446x_read((uint8_t*)rxbuffer[rxBufferCnt], MAX_PACKET_SIZE);
    uint8_t len = rxbuffer[rxBufferCnt][0];
    if(len <= MAX_PACKET_SIZE)
        timer_cnt = rxbuffer[rxBufferCnt][1] + eeStruct.transferDelay;
    if(len > MAX_PACKET_SIZE) {
        //invalid packet len!
        len = MAX_PACKET_SIZE;
        return;
    }
    else if(len == 0){
        //if len == 0 then it is special packet that contains api spesific data
        specialPacketArrivedFlag = 1;
        specialPacketBuffer = rxBufferCnt;
    }
    if((rxBufferCnt + 1) % RX_CIRCULAR_LENGTH == rxReadBufCnt )
        rxOverflowed = 1;
    rxBufferCnt++;
    if( rxBufferCnt >= RX_CIRCULAR_LENGTH ) rxBufferCnt = 0;
}

void SI446X_CB_RXINVALID(int16_t rssi)
{
    invalidRxFlag = 1;
}
void SI446X_CB_TXTIMEOUT()
{
    iwdg_reset();
    if(eeStruct.debug == 1) printf("\n\033[31mTX timeout!!!\033[0m\r\n");
    init_modem();
}
void SI446X_CB_CMDTIMEOUT()
{
    cmdTimeoutFlag = 1; 
}
void SI446X_CB_RXINVALIDSYNC(void)
{
    invalidSyncFlag = 1;
}

int rf_tx()
{
  if((timer_cnt > (128-eeStruct.deadTime/2) && timer_cnt < (128+eeStruct.deadTime/2)) || 
      timer_cnt > (255-eeStruct.deadTime/2) || timer_cnt < (eeStruct.deadTime/2)) return 0;
  if(eeStruct.id % 2 == 0 && timer_cnt > 128) return 0;
  if(eeStruct.id % 2 == 1 && timer_cnt < 128) return 0;

  txbuffer[txReadBufCnt][1] = timer_cnt;
  if( Si446x_TX(txbuffer[txReadBufCnt], MAX_PACKET_SIZE, eeStruct.channel, SI446X_STATE_RX) ){
    //_write(1, &txbuffer[txReadBufCnt][2], 4);
    //printf(" \r\n");
    return 1;
  }
  return 0;
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
    gpio_set(GPIOC, GPIO13);

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
	timer_set_period(TIM2, 1);

	timer_enable_counter(TIM2);
	timer_enable_irq(TIM2, TIM_DIER_UIE);
}
void tim2_isr(void)
{
	if (timer_get_flag(TIM2, TIM_SR_UIF)) {
		/* Clear update interrupt flag. */
		timer_clear_flag(TIM2, TIM_SR_UIF);
        timer_cnt++;
    }
}

void init_modem(){
    iwdg_reset();
    Si446x_init();
    iwdg_reset();
    Si446x_setTxPower(eeStruct.rfpower);
    Si446x_setRSSIComp(eeStruct.rssiComp);
    Si446x_RX(eeStruct.channel);
}

void send_ping(uint8_t addr)
{
    memset(txbuffer[txReadBufCnt], 0, MAX_PACKET_SIZE);
    txbuffer[txReadBufCnt][0] = 0;
    txbuffer[txReadBufCnt][SOURCE_ADDR] = eeStruct.id;
    txbuffer[txReadBufCnt][DEST_ADDR] = addr;
    txbuffer[txReadBufCnt][CMD] = CMD_PING;
    while (rf_tx() <= 0);
}
void send_ping_resp(uint8_t addr, int16_t rssi)
{
    memset(txbuffer[txReadBufCnt], 0, MAX_PACKET_SIZE);
    txbuffer[txReadBufCnt][0] = 0;
    txbuffer[txReadBufCnt][SOURCE_ADDR] = eeStruct.id;
    txbuffer[txReadBufCnt][DEST_ADDR] = addr;
    txbuffer[txReadBufCnt][CMD] = CMD_PING_RESP;
    txbuffer[txReadBufCnt][5] = (uint8_t)rssi;
    txbuffer[txReadBufCnt][6] = (uint8_t)(rssi >> 8);
    while (rf_tx() <= 0);
}
void process_special_packet()
{
    if(specialPacketArrivedFlag == 0)
        return;
    specialPacketArrivedFlag = 0;
    if( rxbuffer[specialPacketBuffer][DEST_ADDR] != eeStruct.id &&
        rxbuffer[specialPacketBuffer][DEST_ADDR] != BROADCAST){
        //address missmatch
        if(eeStruct.debug == 1) printf("\033[31mAddress missmatch!\033[0m\r\n");
        rl_reset_cursor();
        return;
    }
    switch(rxbuffer[specialPacketBuffer][CMD]) {
        case CMD_PING:
            send_ping_resp(rxbuffer[specialPacketBuffer][SOURCE_ADDR], rxRSSI);
            //Serial.print(P("Respond to ping coming from: "));
            //Serial.print(rxbuffer[specialPacketBuffer][SOURCE_ADDR]);
            //Serial.print(P(" rssi: "));
            //Serial.print(rxRSSI);
            //rl_reset_cursor();
        break;
        case CMD_PING_RESP: {
            int16_t rssi = rxbuffer[specialPacketBuffer][5];
            rssi += ((int16_t)rxbuffer[specialPacketBuffer][6]<<8);
            printf("Ping received from: %d with rssi: %f, our rssi: %f",
                   rxbuffer[specialPacketBuffer][SOURCE_ADDR], 
                   rssi/2.0-134.0, rxRSSI/2.0-134.0);
            rxPingRespCnt++;
            //rl_reset_cursor();
        break;
        }
        default:
            printf("\033[31mUnknown Command!\033[0m\r\n");
            rl_reset_cursor();
        break;
    }
}

int main(void)
{
    uint8_t p1=1,p2=1;
    uint32_t commandModeTimer = 0;
    txCounter = 2;
    lastSerialDataTime = 0;
    rxBufferCnt = 0;
    txBufferCnt = 0;
    rxReadBufCnt = 0;
    txReadBufCnt = 0;
    rxOverflowed = 0;

    
	clock_setup();
    systick_setup();
	gpio_setup();
    tim_setup();

    eeprom_load();
    usart_setup(eeStruct.baudrate);

    init_modem();

    microrl_init (prl, rl_print);
    microrl_set_execute_callback (prl, execute);
    microrl_set_complete_callback (prl, complet);
    microrl_set_sigint_callback (prl, rl_sigint);
    commandModeTimer = system_millis;
    while (1) {
        if(uart_rx_available() > 5){
            while(uart_rx_available()){
                if(uart_read_ch() == 'a') commandModeEnabled++;
                else commandModeEnabled = 0;
                if(system_millis - commandModeTimer > 100) break;
            }
            if(commandModeEnabled > 4) commandModeEnabled = 1;
            else commandModeEnabled = 0;
            break;
        }
        if(system_millis - commandModeTimer > 2000) break;
    }
    while( commandModeEnabled ){
        // put received char from stdin to microrl lib
        process_special_packet();
        if(uart_rx_available() > 0)
            microrl_insert_char (prl, uart_read_ch());
    }

    //todo debug only. Do not forget to delete this
    if(eeStruct.debug == 1) printf("\033[31minit\033[0m\r\n");

    /*special packets has different buffer pointer. 
    so wrorng overflowed signal may ocur. */
    rxOverflowed = 0; 

    iwdg_set_period_ms(150);
    iwdg_start();
    while(1)
    {
        iwdg_reset();

        /*if( timer_cnt > 128 && p1){
            printf("turn 1\r\n");
            p1 = 0;
            p2 = 1;
        }
        if( timer_cnt < 128 && p2){
            printf("turn 2\r\n");
            p1 = 1;
            p2 = 0;
        }*/
        //Si446x_SERVICE();
        if(cmdTimeoutFlag == 1){
            cmdTimeoutFlag = 0;
            if(eeStruct.debug == 1) printf("\n\033[31mcmd timeout resetting device!\033[0m\r\n");
            init_modem();
        }
        if(invalidRxFlag == 1) {
            invalidRxFlag = 0;
            if(eeStruct.debug == 1) printf("\n\033[31minvalid rx!\033[0m\r\n");
            init_modem();
        }
        if(invalidSyncFlag == 1) {
            invalidSyncFlag = 0;
            if(eeStruct.debug == 1) printf("\n\033[31minvalid sync!\033[0m\r\n");
            //init_modem();
        }
        if(rxOverflowed == 1){
            if(eeStruct.debug == 1) printf("\n\033[31mRX overflowed!!!\033[0m\r\n");
            rxOverflowed = 0;
        }
        while(rxReadBufCnt != rxBufferCnt) {
            //if our turn than do not wait and send the data
            /*if(txBufferCnt != txReadBufCnt && (
                  (eeStruct.id % 2 == 0 && timer_cnt < 128) ||
                  (eeStruct.id % 2 == 1 && timer_cnt > 128) ))
                break;*/
            iwdg_reset();
            if(uart_tx_available() < MAX_PACKET_SIZE) break;
            //char c = '+'; _write(1, &c, 1); //debug packet boundary
            if(rxbuffer[rxReadBufCnt][0] > 2 && rxbuffer[rxReadBufCnt][0] <= MAX_PACKET_SIZE)
                _write(1, (char*)&rxbuffer[rxReadBufCnt][2], rxbuffer[rxReadBufCnt][0]-2);
            rxReadBufCnt++;
            if( rxReadBufCnt >= RX_CIRCULAR_LENGTH ) rxReadBufCnt = 0;
        }
        if(uart_rx_available()  >= MAX_PACKET_SIZE ||
          (uart_rx_available() > 0 && (system_millis - lastSerialDataTime > TIMEOUT)))
        {
            gpio_clear(GPIOC, GPIO13);
            while(uart_rx_available() > 0){
                //if our turn than do not wait and send the data
                /*if(txBufferCnt != txReadBufCnt && (
                      (eeStruct.id % 2 == 0 && timer_cnt < 128) ||
                      (eeStruct.id % 2 == 1 && timer_cnt > 128) ))
                    break;*/
                txbuffer[txBufferCnt][txCounter++] = uart_read_ch();
                if(txCounter >= MAX_PACKET_SIZE) {
                    iwdg_reset();
                    txCounter = 2;
                    if((txBufferCnt + 1) % TX_CIRCULAR_LENGTH == txReadBufCnt ){
                        if(eeStruct.debug == 1) printf("\n\033[31mTX overflowed!!!\033[0m\r\n");
                    }
                    txBufferCnt++;
                    if ( txBufferCnt >= TX_CIRCULAR_LENGTH ) txBufferCnt = 0;
                }
            }
            lastSerialDataTime = system_millis;
            gpio_set(GPIOC, GPIO13);
        }

        if(txBufferCnt != txReadBufCnt){
            //we are here because a buffer full
            /*if(txBufferCnt >= txReadBufCnt)
                printf(" %d,%d \r\n", uart_rx_available(), txBufferCnt-txReadBufCnt);
            else
                printf(" %d,%d \r\n", uart_rx_available(), TX_CIRCULAR_LENGTH - txReadBufCnt-txBufferCnt);
                */
            while(txBufferCnt != txReadBufCnt && (
                  (eeStruct.id % 2 == 0 && timer_cnt < 128) ||
                  (eeStruct.id % 2 == 1 && timer_cnt > 128) )){
                iwdg_reset();
                txbuffer[txReadBufCnt][0] = MAX_PACKET_SIZE;
                if( rf_tx() ){
                    txReadBufCnt++;
                    if ( txReadBufCnt >= TX_CIRCULAR_LENGTH ) txReadBufCnt = 0;
                }
            }
        }

        else if ( (system_millis - lastSerialDataTime > TIMEOUT && txCounter > 2 ))
        {
            //we are here because of timeout
            txbuffer[txReadBufCnt][0] = txCounter;
            if( rf_tx() ){
                txCounter = 2;
            }
        }

    }
}
