#ifndef CENTRALMANAGERDEFS_H
#define CENTRALMANAGERDEFS_H

#include <stdint.h>
#include "centralManagerErrorCodes.h"

#define DATA_TYPE_ATTRIBUTE  __attribute__((packed))

typedef enum {
    OPID_RUN_PSD        = 0x03,
    OPID_RUN_BKGND      = 0x04,
    OPID_RUN_ABSRB      = 0x05,
    OPID_RUN_GNADJ      = 0x06,
    OPID_BRN_GAIN       = 0x07,
    OPID_BRN_SLF        = 0x08,
    OPID_BRN_WLN        = 0x09,
    OPID_RUN_SLFCOR     = 0x0A,
    OPID_RUN_WVL_COR_BG = 0x0B,
    OPID_RUN_WVL_COR    = 0x0C,
    OPID_RSTR_DEFAULT   = 0x0D,
    OPID_SET_SRC_SETT   = 0x16,
    OPID_SET_OPTCL_SET  = 0x1B,
    OPID_CALIB_WELL_1   = 0x5A,
    OPID_CALIB_WELL_2   = 0x5B,
    OPID_RUN_Temp       = 0x60,
    OPID_CHECK_TEMP     = 0xFE,
} P3OperationID_t;

typedef enum {
    OPID_MEM_STATS      = 0x00,
    OPID_SCAN_REQ       = 0x01,
    OPID_MEM_CLEAR      = 0x02,
    OPID_SAVE_SETTINGS  = 0x03,
    OPID_RESTORE_DEFAULT_SETTINGS  = 0x04,
    OPID_SET_SCANNER_ID = 0x23,
    OPID_SCAN_REQ_WITH_SETTINGS = 0x05,
    OPID_SAVE_AMBIENT_TEMP_FILE = 0x24,
    OPID_READ_AMBIENT_TEMP_FILE = 0x25,
    OPID_SAVE_MOTHER_BOARD_VERSION_FILE = 0x26,
    OPID_READ_MOTHER_BOARD_VERSION_FILE = 0x27,
    OPID_SAVE_INACTIVE_TIMEOUT = 0x28,
    OPID_SAVE_TEMPERATURE_WINDOW = 0x29,
    OPID_GET_BATTERY_LOG = 0x30
} MemOperationID_t;

typedef enum {
    OPID_BATT_REQ           = 0x00,
    OPID_VERSION_REQ        = 0x01,
    OPID_SID_REQ            = 0x80,
    OPID_READ_TEMPERATURE_REQ = 0x81,
    OPID_GAUGE_INFO_REQ     = 0x82,
    OPID_POWER_SELFTEST_REQ = 0x83,
    OPID_CHECK_BOARD        = 0x84,
    OPID_RESET_ECU          = 0x85,
    OPID_ENTER_DEBUG_MODE   = 0x86,
    OPID_GET_LIFE_TIME_DATA = 0x87,
} SysOperationID_t;

typedef enum {
    OPID_RSTR_FCTRY_FRMWR   = (uint32_t)0x51D5DAE0,
    OPID_OTA_START          = (uint32_t)0x6A9DFE33,
    OPID_OTA_END            = (uint32_t)0x9EF6835F,
    OPID_OTA_DATA           = (uint32_t)0x8FC35AB3,
    OPID_OTA_OPERATION_DONE = 0,
    OPID_OTA_DATA_WRITTEN   = 1,
} OTAOpertaionID_t;

typedef enum{
    OPID_MAINTENANCE_SAVE_GAUGE_INFO = 0x00,
}MaintenanceOperationID_t;

typedef enum {
    BUTTON_SHORT_PRESS,     //Short press
    BUTTON_LONG_PRESS,      //Long press
    BUTTON_DEFAULT,         //Bad detection maybe it's due to bouncing
} ButtonState_t;
typedef enum{
    EVENT_TYPE_PERIODIC_CHECK = 0x00,
    EVENT_TYPE_SHUT_DOWN = 0x01,
    EVENT_TYPE_POWER_UP = 0x02,
    EVENT_TYPE_MAX = 0xFF,
}event_type_t;

typedef struct {
    uint16_t battVoltage;
    int16_t  current;
    uint16_t capacity;
    uint16_t fullCapacity;
    uint16_t temperature;
    uint16_t cellVoltage1;
    uint16_t cellVoltage2;
    uint16_t timeToEmpty;
    uint16_t timeToFull;
    uint16_t chargingCurrent;
    uint8_t  batteryPercentage;

    uint32_t safetyStatus;
    uint32_t pfStatus;
    uint32_t gaugingStatus;
    uint32_t mfgStatus;
    uint32_t operationStatus;
    uint32_t cycleCount;

    uint8_t fC      :1;
    uint8_t VCT     :1;
    uint8_t xchg    :1;
    uint8_t xdischg :1;
} DATA_TYPE_ATTRIBUTE GaugeInfo_t;

typedef struct {
    uint16_t cell_1_max_voltage;
    uint16_t cell_2_max_voltage;
    uint16_t cell_3_max_voltage;
    uint16_t cell_4_max_voltage;
    uint16_t cell_1_min_voltage;
    uint16_t cell_2_min_voltage;
    uint16_t cell_3_min_voltage;
    uint16_t cell_4_min_voltage;
    uint16_t max_delta_cell_voltage;
    uint16_t max_charge_current;
    uint16_t max_discharge_current;
    uint16_t max_avg_dsg_current;
    uint16_t max_avg_dsg_power;
    uint8_t max_temp_cell;
    uint8_t min_temp_cell;
    uint8_t max_delta_cell_temp;
    uint8_t max_temp_int_sensor;
    uint8_t min_temp_int_sensor;
    uint8_t max_temp_fet;
    uint8_t num_of_shutdowns;
    uint8_t num_of_partial_resets;
    uint8_t num_of_full_resets;
    uint8_t num_of_wdt_resets;
    uint8_t cb_time_cell_1;
    uint8_t cb_time_cell_2;
    uint8_t cb_time_cell_3;
    uint8_t cb_time_cell_4;
    uint16_t total_fw_runtime;
    uint16_t time_spent_in_ut;
    uint16_t time_spent_in_lt;
    uint16_t Time_Spent_in_STL;
    uint16_t time_spent_in_rt;
    uint16_t Time_Spent_in_STH;
    uint16_t time_spent_in_ht;
    uint16_t time_spent_in_ot;
    uint16_t num_of_cov_events;
    uint16_t last_cov_event;
    uint16_t num_of_cuv_events;
    uint16_t last_cuv_event;
    uint16_t num_of_ocd1_events;
    uint16_t last_ocd1_event;
    uint16_t num_of_ocd2_events;
    uint16_t last_ocd2_event;
    uint16_t num_of_occ1_events;
    uint16_t last_occ1_event;
    uint16_t num_of_occ2_events;
    uint16_t last_occ2_event;
    uint16_t num_of_aold_events;
    uint16_t last_aold_event;
    uint16_t num_of_ascd_events;
    uint16_t last_ascd_event;
    uint16_t num_of_ascc_events;
    uint16_t last_ascc_event;
    uint16_t num_of_otc_events;
    uint16_t last_otc_event;
    uint16_t num_of_otd_events;
    uint16_t last_otd_event;
    uint16_t num_of_otf_events;
    uint16_t last_otf_event;
    uint16_t num_valid_charge_term;
    uint16_t last_valid_charge_term;
}DATA_TYPE_ATTRIBUTE life_time_data_t;
typedef struct {
    uint16_t battVoltage;
    int16_t  current;
    uint16_t capacity;
    uint16_t fullCapacity;
    uint16_t temperature;
    uint16_t cellVoltage1;
    uint16_t cellVoltage2;
    uint16_t timeToEmpty;

    uint32_t operation_id1;  //this is added just to ensure compatability with old firmware, every 16 (packet) bytes we need to add operation ID
    uint16_t timeToFull;
    uint16_t chargingCurrent;
    uint16_t fC;
    uint16_t VCT;
    uint16_t xchg;
    uint16_t xdischg;
    uint32_t safetyStatus;
    uint32_t operation_id2;
    uint32_t pfStatus;
    uint32_t operationStatus;
    uint32_t gaugingStatus;
    uint32_t mfgStatus;
    uint8_t batteryPercentage;
    uint16_t cycleCount;
} DATA_TYPE_ATTRIBUTE oldFormatGaugeInfo_t;

typedef struct {
    char id[52];
} DATA_TYPE_ATTRIBUTE ScannerID_t;

typedef struct {
    char id[32];
} DATA_TYPE_ATTRIBUTE AppVersion_t;

typedef struct {
    uint16_t length;
    uint8_t* data_bytes;
}generic_data_packet_t;
typedef struct {
    uint32_t scanTime: 24;
    uint8_t commonWaveNum;
    uint8_t opticalGain;
    uint8_t apodizationSel;
    uint8_t zeroPadding;
    uint8_t mode;        
} DATA_TYPE_ATTRIBUTE MeasurementSettings_t;

typedef struct {
    uint32_t scanTime: 24;
    uint8_t commonWaveNum;
    uint16_t gainvalue;
    uint8_t opticalGain;
    uint8_t apodizationSel;
    uint8_t zeroPadding;
    uint8_t mode;        
} DATA_TYPE_ATTRIBUTE ScannerSettings_t;

typedef struct {
#define SPECTRAL_TYPE_BCKGRND       0
#define SPECTRAL_TYPE_SPECTRUM      1
#define SPECTRAL_TYPE_PSD           2
#define SPECTRAL_TYPE_INVALID       0xff
    uint8_t type;
    double* spectralPTR;
    uint16_t psdLength;
    MeasurementSettings_t settings;
    uint32_t temperature_during_scan;
} DATA_TYPE_ATTRIBUTE SpectralData_t;

typedef struct {
    uint8_t lampsCount;
    uint8_t lampsSelect;
    uint16_t dummy1;
    uint8_t t1;
    uint8_t deltaT;
    uint16_t dummy2;
    uint8_t t2C1;
    uint8_t t2C2;
    uint8_t t2max;
} DATA_TYPE_ATTRIBUTE sourceSettings_t;
typedef struct {
    uint32_t ambientTemp;
    uint32_t tAIFReading;
}TemperatureReading;

typedef struct {
	uint8_t operationID; //P3OperationID_t
    
    union {
        MeasurementSettings_t measurement;
        sourceSettings_t sourceSettings;
	    uint16_t opticalGainVal;
	    uint32_t calibWells_1[3];
	    uint32_t calibWells_2[2];
    } DATA_TYPE_ATTRIBUTE U;
}  DATA_TYPE_ATTRIBUTE P3RequestPacket_t;

typedef struct {
	uint8_t operationID;
}  DATA_TYPE_ATTRIBUTE SysRequestPacket_t;
typedef struct{
    event_type_t event_type;
    GaugeInfo_t* gauge_info_ptr;
}battery_log_data_t;

typedef struct {
    uint8_t operationID;
    union {
        battery_log_data_t  battery_log_request;
    }U;
}  DATA_TYPE_ATTRIBUTE MaintenanceRequestPacket_t;

typedef struct {
	uint8_t operationID; //MemOperationID_t
    union {
        uint16_t scanNum;
        uint64_t scannerID;
        uint32_t temperature;
        uint32_t mother_board_version;
        ScannerSettings_t scannerSettings;
        uint32_t inactive_timeout;
        uint8_t temperature_window;
    }DATA_TYPE_ATTRIBUTE U;
}  DATA_TYPE_ATTRIBUTE MemRequestPacket_t;

typedef struct {
	uint32_t operationID;
    uint8_t* data_ptr;
    uint16_t len;
    uint8_t memory_index;
}  DATA_TYPE_ATTRIBUTE OTARequestPacket_t;

typedef struct {
#define REQUESTPACKET_P3        0
#define REQUESTPACKET_SYSTEM    1
#define REQUESTPACKET_MEMORY    2
#define REQUESTPACKET_OTA       3
#define REQUESTPACKET_MAINTENANCE 4
	uint8_t packetType;
    
    union {
        P3RequestPacket_t p3Packet;
        SysRequestPacket_t sysPacket;
        MemRequestPacket_t memPacket;
        OTARequestPacket_t otaPacket;
        MaintenanceRequestPacket_t maintenancePacket;
    } DATA_TYPE_ATTRIBUTE packet;
}  DATA_TYPE_ATTRIBUTE RequestPacket_t;

typedef struct {
	uint8_t operationID; //P3OperationID_t

    union {
        SpectralData_t spectralData;
        uint64_t moduleID;
        uint32_t fwVersion;
        uint32_t temperature;
        uint16_t gainData;
    } DATA_TYPE_ATTRIBUTE data;
} DATA_TYPE_ATTRIBUTE P3Response_t;

typedef struct{
    uint32_t batteryPercentage;
    uint32_t chargerState;
} battery_data_t;

typedef struct {
	uint32_t operationID;
    union {
        battery_data_t batReq;
        uint64_t sensorID;
        uint32_t temperature;
        struct{
            uint32_t charger_status;
            uint32_t gauge_status;
        } selfTestStatus;
        struct{
            uint16_t life_time_data_length;
            life_time_data_t* life_time_data_ptr;
        } DATA_TYPE_ATTRIBUTE life_time_data_packet;
        AppVersion_t version;
        GaugeInfo_t* dataPtr;
        oldFormatGaugeInfo_t * oldGaugeInfoPtr;
    } DATA_TYPE_ATTRIBUTE U;
}  DATA_TYPE_ATTRIBUTE SysResponse_t;


typedef struct {
	uint32_t operationID;
    union {
        struct {
            uint32_t usedMemory;
            uint32_t version;
        }stat;
        TemperatureReading temperature_reading;
        SpectralData_t spectralData;
        uint32_t mother_board_version;
        generic_data_packet_t battery_log_data;
    }DATA_TYPE_ATTRIBUTE U;
}  DATA_TYPE_ATTRIBUTE MemResponse_t;

typedef struct {
    uint32_t writtenBytes;
    uint8_t operationID;
}  DATA_TYPE_ATTRIBUTE OTAResponse_t;

typedef struct {
#define RESPONSEPACKET_P3        0
#define RESPONSEPACKET_SYSTEM    1
#define RESPONSEPACKET_MEMORY    2
#define RESPONSEPACKET_OTA       3    
	uint8_t packetType;
    ScannerError_t statusCode;
    
    union {
        P3Response_t p3Response;
        SysResponse_t sysResponse;
        MemResponse_t memResponse;
        OTAResponse_t otaResponse;
    } DATA_TYPE_ATTRIBUTE packet;
}  DATA_TYPE_ATTRIBUTE ResponsePacket_t;



#endif