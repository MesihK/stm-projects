#ifndef RLH
#define RLH

#include <string.h>
#include <stdlib.h>
#include "microrl.h"
#include "re.h"
#include "uart.h"
#include "si4463.h"
#include "flash.h"

#define EEPROM_TEST 0xABCDEF00
#define BROADCAST 255
#define CHANNEL_TRESHOLD -80.0
//extern char __BUILD_NUMBER;
extern volatile uint8_t timer_cnt;
extern uint8_t commandModeEnabled;
extern volatile uint32_t rxPingRespCnt;

extern void send_ping(uint8_t addr);
extern void send_ping_resp(uint8_t addr, int16_t rssi);
extern void process_special_packet(void);
extern void init_modem(void);

typedef struct {
  uint32_t eepromTest;
  uint8_t channel;
  uint8_t rfpower;
  uint32_t baudrate;
  uint8_t id;
  uint8_t debug;
  uint8_t rssiComp;
  uint8_t transferDelay;
  uint8_t deadTime;
}EEPROM_STRUCT;
extern EEPROM_STRUCT eeStruct;

int execute (int argc, const char * const * argv);
char ** complet (int argc, const char * const * argv);
void rl_sigint (void);
void rl_reset_cursor(void);
void rl_help(void);
void rl_print(const char *str);

void eeprom_save(void);
void eeprom_load(void);

#endif

