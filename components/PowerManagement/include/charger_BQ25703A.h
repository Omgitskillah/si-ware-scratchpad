#ifndef CHARGER_BQ257034_H
#define CHARGER_BQ257034_H

#include "powerManagement.h"

#define BQ25703A_CHIP_ADDR      0x6B
#define BQ25703A_TIMOUT_MS      1000

typedef enum {
    REG_CHARGEOPTION0_HIGH = 0x1,
    REG_CHARGECURRENT_WORD = 0x2,
    REG_CHARGER_STATUS_HIGH = 0x21,
    REG_ADCIBAT_WORD = 0x28,
    REG_ADCOPTION_WORD = 0x3A,
    REG_ADCVBUS_HIGH = 0x27,
} ChargerReg_t;

typedef enum {
    OPTION_DISABLE_WATCHDOG = 0x9F,
    OPTION_DISABLE_LOWPOWER = 0x7F,
    OPTION_ENABLE_LOWPOWER = 0x80,
} ChargerOption_t;

typedef enum {
    ADC_VBUS_ENABLE = 0x40,
    ADC_VBUS_DISABLE = 0xBF,
    ADC_CURRENT_ENABLE = 0x0C,
    ADC_CURRENT_DISABLE = 0xF3,
} ChargerADCOption_t;

typedef enum {
    CURRENT_ZERO = 0x0000,
    CURRENT_192mA = 0x00C0,
    CURRENT_256mA = 0x0100,
    CURRENT_512mA = 0x0200,
    CURRENT_768mA =	0x0300,
    CURRENT_1024mA = 0x0400,
    CURRENT_MAX = 0x1FE0,
} ChargerCurrentOption_t;

typedef enum {
    ChargerUnplugged = 0,
    ChargerSlow = 1,
    ChargerFast = 2,
} ChargerState_t;


void charger_readCurrent();
void charger_readStatus();

ScannerError_t charger_selfTest();

ChargerState_t charger_start();
ChargerState_t charger_stop();

#endif