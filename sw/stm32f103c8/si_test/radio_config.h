
#ifndef RADIO_CONFIG_H_
#define RADIO_CONFIG_H_

#define RF_POWER_UP 0x02, 0x01, 0x01, 0x01, 0x8C, 0xBA, 0x80
#define RF_GPIO_PIN_CFG 0x13, 0x08, 0x1B, 0x61, 0x60, 0x67, 0x4B, 0x00
#define GLOBAL_2_0 0x11, 0x00, 0x04, 0x00, 0x00, 0x00, 0x18, 0x30
#define INT_CTL_2_0 0x11, 0x01, 0x04, 0x00, 0x07, 0x38, 0x21, 0x00
#define FRR_CTL_2_0 0x11, 0x02, 0x03, 0x00, 0x0A, 0x09, 0x00
#define PREAMBLE_2_0 0x11, 0x10, 0x01, 0x04, 0x31
#define SYNC_2_0 0x11, 0x11, 0x06, 0x00, 0x11, 0xB4, 0x2B, 0x00, 0x00, 0x80
#define PKT_2_0 0x11, 0x12, 0x07, 0x00, 0x04, 0x00, 0x30, 0xFF, 0xFF, 0x20, 0x11
#define PKT_2_1 0x11, 0x12, 0x05, 0x0E, 0x40, 0x04, 0xAA, 0x00, 0x80
#define MODEM_2_0 0x11, 0x20, 0x0C, 0x01, 0x00, 0x07, 0x1E, 0x84, 0x80, 0x01, 0x8C, 0xBA, 0x80, 0x00, 0x1F, 0x82
#define MODEM_2_1 0x11, 0x20, 0x0C, 0x1C, 0x80, 0x00, 0x00, 0x20, 0x00, 0xE8, 0x00, 0x41, 0x07, 0xE0, 0x7E, 0x07
#define MODEM_2_2 0x11, 0x20, 0x0A, 0x28, 0xE0, 0x02, 0x00, 0x00, 0x00, 0x23, 0x8F, 0xC1, 0x00, 0xCD
#define MODEM_2_3 0x11, 0x20, 0x0B, 0x39, 0x0E, 0x0E, 0x80, 0x02, 0x40, 0x00, 0x00, 0x28, 0x0C, 0xA4, 0x23
#define MODEM_2_4 0x11, 0x20, 0x09, 0x45, 0x03, 0x01, 0xEC, 0x01, 0x00, 0x80, 0x06, 0x02, 0x18
#define MODEM_2_5 0x11, 0x20, 0x02, 0x50, 0x84, 0x0A
#define MODEM_2_6 0x11, 0x20, 0x02, 0x54, 0x06, 0x07
#define MODEM_2_7 0x11, 0x20, 0x05, 0x5B, 0x40, 0x04, 0x0E, 0x78, 0x20
#define MODEM_CHFLT_2_0 0x11, 0x21, 0x0C, 0x13, 0xBA, 0x0F, 0x51, 0xCF, 0xA9, 0xC9, 0xFC, 0x1B, 0x1E, 0x0F, 0x01, 0xFC
#define MODEM_CHFLT_2_1 0x11, 0x21, 0x05, 0x1F, 0xFD, 0x15, 0xFF, 0x00, 0x0F
#define SYNTH_2_0 0x11, 0x23, 0x06, 0x00, 0x01, 0x05, 0x0B, 0x05, 0x02, 0x00
#define FREQ_CONTROL_2_0 0x11, 0x40, 0x08, 0x00, 0x41, 0x0C, 0xEC, 0x4E, 0x8D, 0xC9, 0x20, 0xFE

#define RADIO_CONFIGURATION_DATA_ARRAY { \
0x07, RF_POWER_UP, \
0x08, RF_GPIO_PIN_CFG, \
0x08, GLOBAL_2_0, \
0x08, INT_CTL_2_0, \
0x07, FRR_CTL_2_0, \
0x05, PREAMBLE_2_0, \
0x0A, SYNC_2_0, \
0x0B, PKT_2_0, \
0x09, PKT_2_1, \
0x10, MODEM_2_0, \
0x10, MODEM_2_1, \
0x0E, MODEM_2_2, \
0x0F, MODEM_2_3, \
0x0D, MODEM_2_4, \
0x06, MODEM_2_5, \
0x06, MODEM_2_6, \
0x09, MODEM_2_7, \
0x10, MODEM_CHFLT_2_0, \
0x09, MODEM_CHFLT_2_1, \
0x0A, SYNTH_2_0, \
0x0C, FREQ_CONTROL_2_0, \
}

#endif
