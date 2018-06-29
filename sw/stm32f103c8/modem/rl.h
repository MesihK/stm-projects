#ifndef RLH
#define RLH

#include <string.h>
#include <stdlib.h>
#include "microrl.h"
#include "re.h"
#include "uart.h"
#include "si4463.h"

#define EEPROM_TEST 0xABCDEF00
#define BROADCAST 255
#define CHANNEL_TRESHOLD -80.0
//extern char __BUILD_NUMBER;
extern uint8_t timer_cnt;
extern uint8_t commandModeEnabled;
extern volatile uint32_t rxPingRespCnt;

typedef struct {
  uint32_t eepromTest;
  uint8_t channel;
  uint8_t rfpower;
  uint32_t baudrate;
  uint8_t id;
  uint8_t debug;
  uint8_t rssiComp;
}EEPROM_STRUCT;
extern EEPROM_STRUCT eeStruct;

int execute (int argc, const char * const * argv);
char ** complet (int argc, const char * const * argv);
void rl_sigint (void);
void rl_reset_cursor(void);
void rl_help(void);
void rl_print(const char *str);

#endif

