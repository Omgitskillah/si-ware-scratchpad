#ifndef GAUGE_BQ4050_H
#define GAUGE_BQ4050_H

#include "powerManagement.h"
#include <math.h>

typedef enum {
    Flash_Data_Access = 0x4000,
    Flash_Cycle_Count = 0x4140,
    Flash_MfgInit = 0x4340,
	Flash_FET_options = 0x4407,
	Flash_SbsGaugeConfig = 0x4408,
	Flash_SbsConfig = 0x4409,
	Flash_PowerConfig = 0x440b,
	Flash_IOconfig = 0x440c,
	Flash_LEDconfig = 0x442e,
	Flash_SOCflagConfigA = 0x4455,
	Flash_SOCflagConfigB = 0x4457,
	Flash_CEDVsmoothConfig = 0x4470,
	Flash_ProtectionA = 0x447d,
	Flash_ProtectionB = 0x447e,
	Flash_ProtectionC = 0x447f,
	Flash_ProtectionD = 0x4480,
	Flash_CUVThreshold = 0x4481,
    Flash_OCThreshold = 0x44df,
	Flash_ChargeConfig = 0x4534,
    Flash_T1Temp = 0x4535,
    Flash_LowTempCurrentLow = 0x453e,
    Flash_LowTempCurrentMed = 0x4540,
    Flash_LowTempCurrentHigh = 0x4542,
    Flash_PreChargeCurrent = 0x455c,
	Flash_TempEn = 0x4579,
	Flash_TempMode = 0x457a,
	Flash_DAconfig = 0x457b,
	Flash_AFEprotectionControl = 0x457d,
	Flash_CEDVGaugeConfig = 0x458e,
	Flash_EMF = 0x4590,
	Flash_C0 = 0x4592,
	Flash_R0 = 0x4594,
	Flash_T0 = 0x4596,
	Flash_R1 = 0x4598,
	Flash_TC = 0x459a,
	Flash_C1 = 0x459b,
	Flash_EDV0add = 0x459d,
	Flash_EDV1add = 0x45a0,
	Flash_EDV2add = 0x45a3,
	Flash_BattLow = 0x45ec,
	Flash_BalanceConfig = 0x460f
	
} FlashAddr_t;

typedef enum {
    SBS_REMAININGCAPACITYALARM = 0x01,  // R/W, Min 0 Max 700 Default 300 mAh/10 mWh
    SBS_REMAININGTIMEALARM = 0x02,      // R/W, Min 0 Max 30 Default 10 minutes
    SBS_BatteryMode = 0x03,	            // R/W, sets various operating mode options Min 0x0000 Max 0xFFFF
    SBS_AtRate = 0x04,	                // R/W, sets the value used in calculating AtRateTimeToFull and AtRateTimeToEmpty Min -32768 Max 32767 default 0 mA/10 mW
    SBS_ATRATETIMETOFULL = 0x05,        // R, Min 0 Max 65535 minutes, 65535 indicates not being charged
    SBS_ATRATETIMETOEMPTY = 0x06,       // R, Min 0 Max 65535 minutes
    SBS_ATRATEOK = 0x07,                // R, Min 0 Max 65535
    SBS_TEMPERATURE = 0x08,             // R, Min 0 Max 65535 0.1K
    SBS_VOLTAGE = 0x09,                 // R, Min 0 Max 65535 mV
    SBS_CURRENT = 0x0A,                 // R, Min -32767 Max 32768 mA
    SBS_AVERAGECURRENT = 0x0B,          // R, Min -32767 Max 32768 mA-+
    SBS_MAXERROR = 0x0C,                // R, Min 0 Max 100 %, expected margin of error, if it reaches 100% device will restart
    SBS_RELATIVESTATEOFCHARGE = 0x0D,   // R, Min 0 Max 100 % percentage of Full charge capacity
    SBS_ABSOLUTESTATEOFCHARGE = 0x0E,   // R, Min 0 Max 100 %
    SBS_REMAININGCAPACITY = 0x0F,	    // R, Min 0 Max 65535 mAh/10 mWh
    SBS_FULLCHARGECAPACITY = 0x10,	    // R, Min 0 Max 65535 mAh/10 mWh
    SBS_RUNTIMETOEMPTY = 0x11,	        // R, Min 0 Max 65535 minutes, 65535 means not being discharged
    SBS_AVERAGETIMETOEMPTY = 0x12,	    // R, Min 0 Max 65535 minutes, 65535 means not being discharged
    SBS_AVERAGETIMETOFULL = 0x13,	    // R, Min 0 Max 65535 minutes, 65535 means not being discharged
    SBS_CHARGINGCURRENT = 0x14,	        // R, Min 0 Max 65535 mA, 65535 request maximum current
    SBS_CHARGINGVOLTAGE = 0x15,	        // R, Min 0 Max 65535 mV, 65535 request maximum voltage
    SBS_BATTERYSTATUS = 0x16,
    SBS_CYCLECOUNT = 0x17,	            //	Read only in SE, R/W in US or FA, min 0, max 65535
    SBS_DESIGNCAPACITY = 0x18,	        // Read only in SE, R/W in US or FA, min 0, max 65535, theoretical pack capacity, units mAh or 10 mWh
    SBS_DESIGNVOLTAGE = 0x19,	        // Read only in SE, R/W in US or FA, min 7000, max
    SBS_DEVICENAME = 0x21,
    SBS_CELLVOLTAGE4 = 0x3C,
    SBS_CELLVOLTAGE3 = 0x3D,
    SBS_CELLVOLTAGE2 = 0x3E,
    SBS_CELLVOLTAGE1 = 0x3F,
    SBS_STATEOFHEALTH = 0x4F,	        // unit %
} SBSCommand_t;
typedef struct {
    uint16_t address;
    union {
        uint16_t data;
        uint8_t * data_ptr;
    } U;
    uint8_t size;
    bool successfully_written;
}GaugeWriteCommand_t;

void gauge_init();
void gauge_deviceReset();
void gauge_disable();
void gauge_enable();
void gauge_readBatteryInfo(GaugeInfo_t *gaugeInfo);
void gauge_printBatteryInfo(GaugeInfo_t *gaugeInfo);
void gauge_writeGoldenFile(uint8_t* goldenFile);
void gauge_read_life_time_data(life_time_data_t* life_time_data);
void gauge_print_life_time_data(life_time_data_t * life_time_data);

ScannerError_t gauge_selfTest();

#endif