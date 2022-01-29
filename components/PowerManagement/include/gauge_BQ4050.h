#ifndef GAUGE_BQ4050_H
#define GAUGE_BQ4050_H

#include "powerManagement.h"
#include <math.h>

typedef enum {
    MAC_DEVICETYPE = 0x0001,                        // Read only
    MAC_FIRMWAREVERSION = 0x0002,                   // Read only
    MAC_HARDWAREVERSION = 0x0003,                   // Read only
    MAC_IFCHECKSUM = 0x0004,                        // Read only
    MAC_STATICDFSIGNATURE = 0x0005,                 // Read only
    MAC_AIIDFSIGNATURE = 0x0009,                    // Read only
    MAC_SHUTDOWNMODE = 0x0010,                      // Write only
    MAC_SLEEPMODE = 0x0011,                         // Write only
    MAC_FUSETOGGLE = 0x001D,                        // Write only
    MAC_PRECHARGEFET = 0x001E,                      // Write only
    MAC_CHARGEFET = 0x001F,                         // Write only
    MAC_DISCHARGEFET = 0x0020,                      // Write only
    MAC_FETCONTROL = 0x0022,                        // Write only
    MAC_LIFETIMEDATACOLLECTION = 0x0023,            // Write only
    MAC_PERMANENTFAILURE = 0x0024,                  // Write only
    MAC_BLACKBOXRECORDERRESET = 0x002A,             // Write only
    MAC_CALIBRATIONMODE = 0x002D,                   // Write only
    MAC_SEALDEVICE = 0x0030,                        // Write only
    MAC_SECURITYKEYS = 0x0035,                      // R/W
    MAC_AUTHENTICATIONKEY = 0x0037,                 // R/W
    MAC_DEVICERESET = 0x0041,                       // Write only
    MAC_SAFETYALERT = 0x0050,                       // Read only
    MAC_SAFETYSTATUS = 0x0051,                      // Read only
    MAC_PFALERT = 0x0052,                           // Read only
    MAC_PFSTATUS = 0x0053,                          // Read only
    MAC_OPERATIONSTATUS = 0x0054,                   // Read only
    MAC_CHARGINGSTATUS = 0x0055,                    // Read only
    MAC_GAUGINGSTATUS = 0x0056,                     // Read only
    MAC_MANUFACTURINGSTATUS = 0x0057,               // Read only
    MAC_AFEREGISTER = 0x0058,                       // Read only
    MAC_LIFETIMEDATABLOCK1 = 0x0060,                // Read only
    MAC_LIFETIMEDATABLOCK2 = 0x0061,                // Read only
    MAC_LIFETIMEDATABLOCK3 = 0x0062,                // Read only
    MAC_LIFETIMEDATABLOCK4 = 0x0063,                // Read only
    MAC_LIFETIMEDATABLOCK5 = 0x0064,                // Read only
    MAC_MANUFACTURERINFO = 0x0070,                  // Read only
    MAC_DASTATUS1 = 0x0071,                         // Read only
    MAC_DASTATUS2 = 0x0072,                         // Read only
    MAC_MANUFACTURERINFO2 = 0x007A,                 // Read only
    MAC_ROMMODE = 0x0F00,                           // Write only
    MAC_EXITCALIBRATIONOUTPUT = 0xF080,             // R/W
    MAC_OUTPUTCCANDADCFORCALIBRATION = 0xF081,      // R/W
    MAC_OUTPUTSHORTEDCCANDADCFORCALIBR = 0xF082,    // R/W
} MACCommand_t;

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

typedef enum {
    SAFETY_STATUS_FLAG_CUV   = 1 << 0,
    SAFETY_STATUS_FLAG_COV   = 1 << 1,
    SAFETY_STATUS_FLAG_OCC1  = 1 << 2,
    SAFETY_STATUS_FLAG_OCC2  = 1 << 3,
    SAFETY_STATUS_FLAG_OCD1  = 1 << 4,
    SAFETY_STATUS_FLAG_OCD2  = 1 << 5,
    SAFETY_STATUS_FLAG_AOLD  = 1 << 6,
    SAFETY_STATUS_FLAG_AOLDL = 1 << 7,
    SAFETY_STATUS_FLAG_ASCC  = 1 << 8,
    SAFETY_STATUS_FLAG_ASCCL = 1 << 9,
    SAFETY_STATUS_FLAG_ASCD  = 1 << 10,
    SAFETY_STATUS_FLAG_ASCDL = 1 << 11,
    SAFETY_STATUS_FLAG_OTC   = 1 << 12,
    SAFETY_STATUS_FLAG_OTD   = 1 << 13,
    SAFETY_STATUS_FLAG_CUVC  = 1 << 14,
    SAFETY_STATUS_FLAG_OTF   = 1 << 16,
    SAFETY_STATUS_FLAG_PTO   = 1 << 18,
    SAFETY_STATUS_FLAG_CTO   = 1 << 20,
    SAFETY_STATUS_FLAG_OC    = 1 << 22,
    SAFETY_STATUS_FLAG_CHGC  = 1 << 23,
    SAFETY_STATUS_FLAG_CHGV  = 1 << 24,
    SAFETY_STATUS_FLAG_PCHGC = 1 << 25,
    SAFETY_STATUS_FLAG_UTC   = 1 << 26,
    SAFETY_STATUS_FLAG_UTD   = 1 << 27,
}safety_status_flag_t;
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
void gauge_writeGoldenFile( void );
void gauge_read_life_time_data(life_time_data_t* life_time_data);
void gauge_print_life_time_data(life_time_data_t * life_time_data);

ScannerError_t gauge_selfTest();

#endif