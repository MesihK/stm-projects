#include "rl2.h"

microrl_t rl;
microrl_t * prl = &rl;

#define _NUM_OF_CMD 9
#define _NUM_OF_SET_SCMD 6
#define _NUM_OF_BAUDRATES 6

char * keyworld [] = {"help", "set", "ver", "clear", "save", "show", "exit",
                      "rssi", "ping"};
char * set_keyworld [] = {"channel", "baudrate", "rfpower", "id", "debug", "rssiComp"};
char * baudrates [] = {"9600", "19200", "38400", "57600", "115200", "230400"};
char * compl_world [_NUM_OF_CMD + 1];

void rl_reset_cursor(void)
{
	printf ("\n\r\033[32m-> \033[0m");
	memset(prl->cmdline, 0, _COMMAND_LINE_LEN);
	prl->cursor = 0;
}
void rl_sigint (void)
{
	rl_reset_cursor();
}

void rl_help()
{
	printf("Si4463 transceiver modem, parameter setter mode.\n");
	printf("\thelp: prints this text\n");
	printf("\tset:  (channel, baudrate, rfpower)\n");
	printf("\t      channel: sets transceiver to desired channel\n");
	printf("\t      baudrate: sets comminucaton baudrate, needs reset\n");
	printf("\t      rfpower: sets transceiver output rf power\n");
	printf("\t      id: sets transceiver address\n");
	printf("\tclear: clears screen\n");
	printf("\tsave: saves channel, baudrate and rfpower parameters\n");
	printf("\tshow: shows channel, baudrate and rfpower parameters\n");
	printf("\trssi: takes a channel number as argument, shows rssi\n");
	printf("\tping: takes an address and pings it.\n");
	printf("\t      If no address given then boradcasts\n");
	printf("\texit: returns to transceiver operation mode\n");
}

void rl_print(const char *str)
{
	printf("%s", str);
}

//*****************************************************************************
// execute callback for microrl library
// do what you want here, but don't write to argv!!! read only!!
int execute (int argc, const char * const * argv)
{
	int i = 0;
	// just iterate through argv word and compare it with your commands
	while (i < argc) {
		if (strcmp (argv[i], P("help")) == 0) {
			rl_help();
		} else if (strcmp (argv[i], P("set")) == 0) {
			if (++i < argc) {
				if (strcmp (argv[i], P("channel")) == 0) {
					if (++i < argc) {
						if( re_match("^ *[0-9]+$", argv[i]) != -1){
							uint8_t chl = atoi(argv[i]);
							if( chl <= 20){
								eeStruct.channel = chl;
								Si446x_RX(eeStruct.channel);
								printf("Setting channel to: ");
								printf("%d\n",eeStruct.channel);
							} else {
								printf("Channel should be between 0-20\n");
							}
						} else {
							printf("Channel should be a number!\n");
						}
					} else {
						printf("need channel number argument!\n");
					}
				} else if (strcmp (argv[i], P("baudrate")) == 0) {
					if (++i < argc) {
                        if( re_match("^ *[0-9]+$", argv[i]) != -1){
                            uint32_t baudrate = atol(argv[i]);
                            if(baudrate == 9600 || baudrate == 19200 ||
                               baudrate == 38400 || baudrate == 57600 ||
                               baudrate == 115200 || baudrate == 230400){
                                eeStruct.baudrate = baudrate; 
                                printf("Setting baudrate to: ");
                                printf("%d\n", eeStruct.baudrate);
                            } else {
                                printf("Wrong buadrate! Baudrate can be:\n");
                                printf("\t9600\t19200\r\n\t38400\t57600\r\n\t115200\t230400\n");
                            }
                        } else {
                            printf("Baudrate should be a number!\n");
                            printf("\t9600\t19200\r\n\t38400\t57600\r\n\t115200\t230400\n");
                        }
                    } else {
                        printf("need baudrate argument!\n");
                    }
				} else if (strcmp (argv[i], P("rfpower")) == 0) {
			        if (++i < argc) {
                        if( re_match("^ *[0-9]+$", argv[i]) != -1){
                            uint8_t pwr = atoi(argv[i]);
                            if( pwr <= 127){
                                eeStruct.rfpower = pwr; 
                                Si446x_setTxPower(eeStruct.rfpower);
                                printf("Setting RF power to: ");
                                printf("%d\n", eeStruct.rfpower);
                            } else {
                                printf("RF power should be in 0 - 127 range!\n");
                            }
                        } else {
                            printf("RF Power should be a number!\n");
                        }
                    } else {
                        printf("need RF power argument!\n");
                    }
				} else if (strcmp (argv[i], P("id")) == 0) {
			        if (++i < argc) {
                        if( re_match("^ *[0-9]+$", argv[i]) != -1){
                            uint8_t id = atoi(argv[i]);
                            if( id >= 1 && id <= 254){
                            eeStruct.id = id;
                            printf("Setting id to: ");
                            printf("%d\n", eeStruct.id);
                            } else {
                                printf("ID should be in 1 - 254 range!\n");
                            }
                        } else {
                            printf("ID should be a number!\n");
                        }
                    } else {
                        printf("need id argument!\n");
                    }
				} else if (strcmp (argv[i], P("debug")) == 0) {
			        if (++i < argc) {
                        if( re_match("^ *[0-9]+$", argv[i]) != -1){
                            eeStruct.debug = atoi(argv[i]);
                            if( eeStruct.debug == 1)
                                printf("Debug on!\n");
                            else{
                                eeStruct.debug = 0;
                                printf("Debug off!\n");
                            }
                        } else {
                            printf("Debug can be 0 or 1!\n");
                        }
                    } else {
                        printf("Debug can be 1 or 0!\n");
                    }
				} else if (strcmp (argv[i], P("rssiComp")) == 0) {
			        if (++i < argc) {
                        if( re_match("^ *[0-9]+$", argv[i]) != -1){
                            if(atoi(argv[i]) <= 0x7F){
                                eeStruct.rssiComp = atoi(argv[i]);
                                Si446x_setRSSIComp(eeStruct.rssiComp);
                                printf("RSSI compansation value succesfully setted!\n");
                            } else {
                                printf("RSSI compansation shouldn't exceed 127!\n");
                            }
                        } else {
                            printf("RSSI compansation value should be an integer!\n");
                        }
                    } else {
                        printf("Debug can be 1 or 0!\n");
                    }
				} else {
					printf("%s\n", (char*)argv[i]);
					printf(" wrong argument, see help\n");
				}
			} else {
				printf("set needs 2 parameter, see help\n");
			}
		} else if (strcmp (argv[i], P("clear")) == 0) {
			printf ("\033[2J");    // ESC seq for clear entire screen
			printf ("\033[H");     // ESC seq for move cursor at left-top corner
		} else if (strcmp (argv[i], P("ver")) == 0) {
            printf("Si4463 transceiver modem ");
            printf(__DGIT_DESCRIBE);
            printf(" build: ");
            //printf("%l\n", (unsigned long) &__BUILD_DATE);
            //printf(" build: ");
            printf("%d\n", (unsigned long) &__BUILD_NUMBER);
		} else if (strcmp (argv[i], P("save")) == 0) {
            eeStruct.eepromTest = EEPROM_TEST;
            //TODO: EEPROM.put(0, eeStruct);
            printf("Paramterers saved to eeprom\n");
		} else if (strcmp (argv[i], P("show")) == 0) {
            printf("Channel: ");
            printf("%d", eeStruct.channel);
            printf(" RF Power: ");
            printf("%d", eeStruct.rfpower);
            printf(" Baudrate: ");
            printf("%d", eeStruct.baudrate);
            printf(" ID: ");
            printf("%d", eeStruct.id);
            printf(" Debug: ");
            printf("%d\n", eeStruct.debug);
            printf(" RSSI Compansation: ");
            printf("%d\n", eeStruct.rssiComp);
            printf("%d\n", timer_cnt);
		} else if (strcmp (argv[i], P("rssi")) == 0) {
            if (++i < argc) {
                if( re_match("^ *[0-9]+$", argv[i]) != -1){
                    uint8_t chnl = atoi(argv[i]);
                    if( chnl <= 20){
                        double min = 100.0, max = -200.0;
                        Si446x_RX(chnl);
                        while(Serial.available() == 0){
                            printf("rssi: ");
                            double rssi = getRSSI();
                            if (rssi > max) max = rssi;
                            if (rssi < min) min = rssi;
                            if(rssi > CHANNEL_TRESHOLD){
                                printf("\033[31m");
                                printf("%f", rssi);
                                printf("dBm\033[0m\n");
                            } else {
                                printf("%f", rssi);
                                printf("dBm \n");
                            }
                            delay(250);
                            process_special_packet();
                        }
                        printf("\n\033[33mChannel: ");
                        printf("%d", chnl);
                        printf(" rssi max: ");
                        printf("%f", max);
                        printf("dBm min: ");
                        printf("%f", min);
                        printf("dBm\033[0m\n");
                        Si446x_RX(eeStruct.channel);
                    } else {
                        printf("Channel should be in 0 - 20!\n");
                    }
                } else {
                    printf("Channel should be a number!\n");
                }
            } else {
                printf("need a channel!\n");
            }
		} else if (strcmp (argv[i], P("ping")) == 0) {
            uint8_t addr = BROADCAST;
            uint8_t cntr = 100;
            uint8_t send_cnt = 0;
            rxPingRespCnt = 0;
            if (++i < argc) {
                addr = atoi(argv[i]);
            }
            while(Serial.available() == 0){
                if(cntr >= 100){
                    send_ping(addr);
                    send_cnt++;
                    cntr = 0;
                }
                cntr++;
                process_special_packet();
                delay(10);
            }
            printf("\n\033[33mPings send: ");
            printf("%d", send_cnt);
            printf(" pings received: ");
            printf("%d", rxPingRespCnt);
            printf("\033[0m\n");
		} else if (strcmp (argv[i], P("exit")) == 0) {
            commandModeEnabled = 0;
		} else {
			printf ("command: '");
			printf("%s", (char*)argv[i]);
			printf("' Not found.\n");
		}
		i++;
	}
	return 0;
}

//*****************************************************************************
// completion callback for microrl library
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
	}	else if ((argc == 2) && (strcmp (argv[0], P("set"))==0)) { // if command needs subcommands
		// iterate through subcommand for command set array
		for (int i = 0; i < _NUM_OF_SET_SCMD; i++) {
			if (strstr (set_keyworld [i], argv [argc-1]) == set_keyworld [i]) {
				compl_world [j++] = set_keyworld [i];
			}
		}
	}	else if ((argc == 3) && (strcmp (argv[0], P("set"))==0) && 
                 (strcmp (argv[1], P("baudrate"))==0)) { // if command needs subcommands
		for (int i = 0; i < _NUM_OF_BAUDRATES; i++) {
			if (strstr (baudrates [i], argv [argc-1]) == baudrates [i]) {
				compl_world [j++] = baudrates [i];
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

