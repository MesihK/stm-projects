#include "rl.h"

char __BUILD_NUMBER = 1;
microrl_t rl;
microrl_t * prl = &rl;

#define _NUM_OF_CMD 9
#define _NUM_OF_SET_SCMD 8
#define _NUM_OF_BAUDRATES 6

char * keyworld [] = {"help", "set", "ver", "clear", "save", "show", "exit",
                      "rssi", "ping"};
char * set_keyworld [] = {"channel", "baudrate", "rfpower", "id", "debug", "rssiComp",
                          "transferDelay", "deadTime"};
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
	printf("Si4463 transceiver modem, parameter setter mode.\r\n");
	printf("\thelp: prints this text\r\n");
	printf("\tset:  (channel, baudrate, rfpower)\r\n");
	printf("\t      channel: sets transceiver to desired channel\r\n");
	printf("\t      baudrate: sets comminucaton baudrate, needs reset\r\n");
	printf("\t      rfpower: sets transceiver output rf power\r\n");
	printf("\t      id: sets transceiver address\r\n");
	printf("\tclear: clears screen\r\n");
	printf("\tsave: saves channel, baudrate and rfpower parameters\r\n");
	printf("\tshow: shows channel, baudrate and rfpower parameters\r\n");
	printf("\trssi: takes a channel number as argument, shows rssi\r\n");
	printf("\tping: takes an address and pings it.\r\n");
	printf("\t      If no address given then boradcasts\r\n");
	printf("\texit: returns to transceiver operation mode\r\n");
}

void rl_print(const char *str)
{
    while(*str != 0)
        _write(1, str++, 1);
	//printf("%s", str);
}

//*****************************************************************************
// execute callback for microrl library
// do what you want here, but don't write to argv!!! read only!!
int execute (int argc, const char * const * argv)
{
	int i = 0;
	// just iterate through argv word and compare it with your commands
	while (i < argc) {
		if (strcmp (argv[i], ("help")) == 0) {
			rl_help();
		} else if (strcmp (argv[i], ("set")) == 0) {
			if (++i < argc) {
				if (strcmp (argv[i], ("channel")) == 0) {
					if (++i < argc) {
						if( re_match("^ *[0-9]+$", argv[i]) != -1){
							uint8_t chl = atoi(argv[i]);
							if( chl <= 200){
								eeStruct.channel = chl;
								Si446x_RX(eeStruct.channel);
								printf("Setting channel to: ");
								printf("%d\r\n",eeStruct.channel);
							} else {
								printf("Channel should be between 0-200\r\n");
							}
						} else {
							printf("Channel should be a number!\r\n");
						}
					} else {
						printf("need channel number argument!\r\n");
					}
				} else if (strcmp (argv[i], ("baudrate")) == 0) {
					if (++i < argc) {
                        if( re_match("^ *[0-9]+$", argv[i]) != -1){
                            uint32_t baudrate = atol(argv[i]);
                            if(baudrate == 9600 || baudrate == 19200 ||
                               baudrate == 38400 || baudrate == 57600 ||
                               baudrate == 115200 || baudrate == 230400){
                                eeStruct.baudrate = baudrate; 
                                printf("Setting baudrate to: ");
                                printf("%d\r\n", eeStruct.baudrate);
                            } else {
                                printf("Wrong buadrate! Baudrate can be:\r\n");
                                printf("\t9600\t19200\r\n\t38400\t57600\r\n\t115200\t230400\r\n");
                            }
                        } else {
                            printf("Baudrate should be a number!\r\n");
                            printf("\t9600\t19200\r\n\t38400\t57600\r\n\t115200\t230400\r\n");
                        }
                    } else {
                        printf("need baudrate argument!\r\n");
                    }
				} else if (strcmp (argv[i], ("rfpower")) == 0) {
			        if (++i < argc) {
                        if( re_match("^ *[0-9]+$", argv[i]) != -1){
                            uint8_t pwr = atoi(argv[i]);
                            if( pwr <= 127){
                                eeStruct.rfpower = pwr; 
                                Si446x_setTxPower(eeStruct.rfpower);
                                printf("Setting RF power to: ");
                                printf("%d\r\n", eeStruct.rfpower);
                            } else {
                                printf("RF power should be in 0 - 127 range!\r\n");
                            }
                        } else {
                            printf("RF Power should be a number!\r\n");
                        }
                    } else {
                        printf("need RF power argument!\r\n");
                    }
				} else if (strcmp (argv[i], ("id")) == 0) {
			        if (++i < argc) {
                        if( re_match("^ *[0-9]+$", argv[i]) != -1){
                            uint8_t id = atoi(argv[i]);
                            if( id >= 1 && id <= 254){
                            eeStruct.id = id;
                            printf("Setting id to: ");
                            printf("%d\r\n", eeStruct.id);
                            } else {
                                printf("ID should be in 1 - 254 range!\r\n");
                            }
                        } else {
                            printf("ID should be a number!\r\n");
                        }
                    } else {
                        printf("need id argument!\r\n");
                    }
				} else if (strcmp (argv[i], ("debug")) == 0) {
			        if (++i < argc) {
                        if( re_match("^ *[0-9]+$", argv[i]) != -1){
                            eeStruct.debug = atoi(argv[i]);
                            if( eeStruct.debug == 1)
                                printf("Debug on!\r\n");
                            else{
                                eeStruct.debug = 0;
                                printf("Debug off!\r\n");
                            }
                        } else {
                            printf("Debug can be 0 or 1!\r\n");
                        }
                    } else {
                        printf("Debug can be 1 or 0!\r\n");
                    }
				} else if (strcmp (argv[i], ("rssiComp")) == 0) {
			        if (++i < argc) {
                        if( re_match("^ *[0-9]+$", argv[i]) != -1){
                            if(atoi(argv[i]) <= 0x7F){
                                eeStruct.rssiComp = atoi(argv[i]);
                                Si446x_setRSSIComp(eeStruct.rssiComp);
                                printf("RSSI compansation value succesfully setted!\r\n");
                            } else {
                                printf("RSSI compansation shouldn't exceed 127!\r\n");
                            }
                        } else {
                            printf("RSSI compansation value should be an integer!\r\n");
                        }
                    } else {
                        printf("Debug can be 1 or 0!\r\n");
                    }
				} else if (strcmp (argv[i], ("transferDelay")) == 0) {
			        if (++i < argc) {
                        if( re_match("^ *[0-9]+$", argv[i]) != -1){
                            if(atoi(argv[i]) <= 0x7F){
                                eeStruct.transferDelay = atoi(argv[i]);
                                printf("Transfer delay succesfuly setted!\r\n");
                            } else {
                                printf("Transfer delay shouldn't exceed 127!\r\n");
                            }
                        } else {
                            printf("Transfer delay should be an integer!\r\n");
                        }
                    } else {
                        printf("Need an integer!\r\n");
                    }
				} else if (strcmp (argv[i], ("deadTime")) == 0) {
			        if (++i < argc) {
                        if( re_match("^ *[0-9]+$", argv[i]) != -1){
                            if(atoi(argv[i]) <= 0x7F){
                                eeStruct.deadTime = atoi(argv[i]);
                                printf("Dead time succesfuly setted!\r\n");
                            } else {
                                printf("Dead time shouldn't exceed 127!\r\n");
                            }
                        } else {
                            printf("Dead time should be an integer!\r\n");
                        }
                    } else {
                        printf("Need an integer!\r\n");
                    }
				} else {
					printf("%s\r\n", (char*)argv[i]);
					printf(" wrong argument, see help\r\n");
				}
			} else {
				printf("set needs 2 parameter, see help\r\n");
			}
		} else if (strcmp (argv[i], ("clear")) == 0) {
			printf ("\033[2J");    // ESC seq for clear entire screen
			printf ("\033[H");     // ESC seq for move cursor at left-top corner
		} else if (strcmp (argv[i], ("ver")) == 0) {
            printf("Si4463 transceiver modem ");
            //printf(__DGIT_DESCRIBE);
            printf(" build: ");
            //printf("%l\n", (unsigned long) &__BUILD_DATE);
            //printf(" build: ");
            printf("%d\r\n", (unsigned long) &__BUILD_NUMBER);
		} else if (strcmp (argv[i], ("save")) == 0) {
            eeStruct.eepromTest = EEPROM_TEST;
            eeprom_save();
            printf("Paramterers saved to eeprom\r\n");
		} else if (strcmp (argv[i], ("show")) == 0) {
            printf("Channel: ");
            printf("%d", eeStruct.channel);
            printf(" RF Power: ");
            printf("%d", eeStruct.rfpower);
            printf(" Baudrate: ");
            printf("%d", eeStruct.baudrate);
            printf(" ID: ");
            printf("%d", eeStruct.id);
            printf(" Debug: ");
            printf("%d\r\n", eeStruct.debug);
            printf(" RSSI Compansation: ");
            printf("%d", eeStruct.rssiComp);
            printf(" Transfer Delay: ");
            printf("%d", eeStruct.transferDelay);
            printf(" Dead time: ");
            printf("%d\r\n", eeStruct.deadTime);
            printf("%d\r\n", timer_cnt);
		} else if (strcmp (argv[i], ("rssi")) == 0) {
            if (++i < argc) {
                if( re_match("^ *[0-9]+$", argv[i]) != -1){
                    uint8_t chnl = atoi(argv[i]);
                    if( chnl <= 200){
                        double min = 100.0, max = -200.0;
                        Si446x_RX(chnl);
                        while(uart_rx_available() == 0){
                            printf("rssi: ");
                            double rssi = Si446x_getRSSI()/2.0-134.0;
                            if (rssi > max) max = rssi;
                            if (rssi < min) min = rssi;
                            if(rssi > CHANNEL_TRESHOLD){
                                printf("\033[31m");
                                printf("%f", rssi);
                                printf("dBm\033[0m\r\n");
                            } else {
                                printf("%f", rssi);
                                printf("dBm \r\n");
                            }
                            msleep(250);
                            process_special_packet();
                        }
                        printf("\r\n\033[33mChannel: ");
                        printf("%d", chnl);
                        printf(" rssi max: ");
                        printf("%f", max);
                        printf("dBm min: ");
                        printf("%f", min);
                        printf("dBm\033[0m\r\n");
                        Si446x_RX(eeStruct.channel);
                    } else {
                        printf("Channel should be in 0 - 200!\r\n");
                    }
                } else {
                    printf("Channel should be a number!\r\n");
                }
            } else {
                printf("need a channel!\r\n");
            }
		} else if (strcmp (argv[i], ("ping")) == 0) {
            uint8_t addr = BROADCAST;
            uint8_t cntr = 100;
            uint8_t send_cnt = 0;
            rxPingRespCnt = 0;
            init_modem();
            if (++i < argc) {
                addr = atoi(argv[i]);
            }
            while(uart_rx_available() == 0){
                if(cntr >= 100){
                    
                    send_ping(addr);
                    send_cnt++;
                    cntr = 0;
                }
                cntr++;
                process_special_packet();
                msleep(10);
            }
            printf("\r\n\033[33mPings send: ");
            printf("%d", send_cnt);
            printf(" pings received: ");
            printf("%d", rxPingRespCnt);
            printf("\033[0m\r\n");
		} else if (strcmp (argv[i], ("exit")) == 0) {
            commandModeEnabled = 0;
		} else {
			printf ("command: '");
			printf("%s", (char*)argv[i]);
			printf("' Not found.\r\n");
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
	}	else if ((argc == 2) && (strcmp (argv[0], ("set"))==0)) { // if command needs subcommands
		// iterate through subcommand for command set array
		for (int i = 0; i < _NUM_OF_SET_SCMD; i++) {
			if (strstr (set_keyworld [i], argv [argc-1]) == set_keyworld [i]) {
				compl_world [j++] = set_keyworld [i];
			}
		}
	}	else if ((argc == 3) && (strcmp (argv[0], ("set"))==0) && 
                 (strcmp (argv[1], ("baudrate"))==0)) { // if command needs subcommands
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

void eeprom_save(void){
    uint32_t buffer[9];

    buffer[0] = eeStruct.eepromTest;
    buffer[1] = eeStruct.channel;
    buffer[2] = eeStruct.rfpower;
    buffer[3] = eeStruct.baudrate;
    buffer[4] = eeStruct.id;
    buffer[5] = eeStruct.debug;
    buffer[6] = eeStruct.rssiComp;
    buffer[7] = eeStruct.transferDelay;
    buffer[8] = eeStruct.deadTime;

    flash_program_data(0, buffer, 4*9);
}
void eeprom_load(void){
    uint32_t buffer[9];
    flash_read_data(0, 4*9, buffer);

    if(buffer[0] == EEPROM_TEST){
        eeStruct.eepromTest = buffer[0];
        eeStruct.channel = buffer[1];
        eeStruct.rfpower = buffer[2];
        eeStruct.baudrate = buffer[3];
        eeStruct.id = buffer[4];
        eeStruct.debug = buffer[5];
        eeStruct.rssiComp = buffer[6];
        eeStruct.transferDelay = buffer[7];
        eeStruct.deadTime = buffer[8];
    } else {
        //eeprom default
        eeStruct.eepromTest = EEPROM_TEST;
        eeStruct.channel = 0;
        eeStruct.rfpower = 50;
        eeStruct.baudrate = 57600;
        eeStruct.id = 1;
        eeStruct.debug = 1;
        eeStruct.rssiComp = 48;
        eeStruct.transferDelay = 5;
        eeStruct.deadTime = 10;
    }
}
