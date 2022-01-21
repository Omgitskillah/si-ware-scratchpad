#ifndef CENTRALMANAGERERRORCODES_H
#define CENTRALMANAGERERRORCODES_H

typedef enum {
	NoError = 0, /* !< No Error */


	/********************************************************
	 * ******************************************************
	 * 					P3 ERRORS (1 ~ 127)					*
	 ********************************************************
	 *******************************************************/




	/********************************************************
	 * ******************************************************
	 * 					Scanner ERRORS (128 ~ 255)			*
	 ********************************************************
	 *******************************************************/

	/* NS Driver Error code (128 ~ 147) */
	NS_DATA_READY_TIMEOUT = 128,
	NS_BACKGROUND_NOT_VALID = 129,
	NS_BACKGROUND_NOT_FOUND = 130,
	NS_BACKGROUND_TEMPERATURE_EXCEEDED = 131,
    NS_PSD_LENGTH_INVALID = 132,

	/* SPIFFS Error Codes (148 ~ 167) */
	OPEN_READ_FILE_ERR = 148,  						//Error in Opening File for Reading
	OPEN_WRITE_FILE_ERR = 149,  					//Error in Opening File for Writing
	READ_FILE_ERR = 150,  							//Error in Reading File
	WRITE_FILE_ERR = 151,  							//Error in Writing File
	SCAN_NUM_FILE_NOT_EXIST = 152,  				//Scan Numbers File does not Exist
	SCAN_FILE_OUT_OF_RANGE = 153,  					//Scan File is out of Range of the stored scan files
	SCAN_FILE_MAX_EXCEEDED = 154,  					//Maximum number of Scan Files is exceeded
	CONFIG_FILE_OPENING_ERROR = 155,				//Failed to open config file
	CONFIG_FILE_LENGTH_ERROR = 156,					//mismatch in config file length
	CONFIG_FILE_PARAMETER_EXTRACTION_ERROR = 157,	//Parameter extraction from config file resturned error
	UNCHARTED_LOCATION_ERROR = 158,					//Uncharted location is being accessed in SPIFFS
	MEMORY_FULL_ERROR = 159, 						//File system is full

	/* BT Classic Codes (168 ~ 177) */
	BTCLASSIC_INIT_CONTROLLER_FAILED = 168,
	BTCLASSIC_ENABLE_CONTROLLER_FAILED = 169,
	BTCLASSIC_INTI_BLUEDROID_FAILED = 170,
	BTCLASSIC_ENABLE_BLUEDROID_FAILED = 171,
	BTCLASSIC_GAP_REGISTER_FAILED = 172,
	BTCLASSIC_SPP_REGISTER_FAILED = 173,
	BTCLASSIC_SPP_INIT_FAILED = 174,

	/* BLE Errors (178 ~ 180) */
	BLE_PACKET_FAILED = 178,

	/* Battery Errors (181 ~ 185) */
	LOW_BATTERY_SCAN_ERROR = 181,
	GAUGE_SELF_TEST_FAILED = 182,
	CHARGER_SELF_TEST_FAILED = 183,

	/* OTA Errors (186 ~ 190) */
	OTA_INIT_FAILED = 186,
	OTA_STEP_FAILED = 187,
	OTA_END_FAILED = 188,
	OTA_RESTORE_FAILED = 189,

	/* Thermal Errors (191 ~ 195) */
	HIGH_TEMP_ERROR = 191,
	TEMP_NOT_STORED_ERROR = 192,

    /*Gauge Safety Errors (200~230) */
    CELL_UNDER_VOLTAGE_ERROR = 200,
    CELL_OVER_VOLTAGE_ERROR = 201,
    OVER_CURRENT_IN_CHARGE_ERROR = 202,
    OVER_CURRENT_IN_DISCHARGE_ERROR = 203,
    OVER_LOAD_IN_DISCHARGE_ERROR = 204,
    SHORT_CIRCUIT_IN_CHARGE_ERROR = 205,
    SHORT_CIRCUIT_IN_DISCHARGE_ERROR = 206,
    OVER_TEMPERATURE_IN_CHARGE_ERROR = 207,
    OVER_TEMPERATURE_IN_DISCHARGE_ERROR = 208,
    OVER_TEMPERATURE_IN_FET_ERROR = 209,
    UNDER_TEMPERATURE_IN_CHARGE_ERROR = 210,
    UNDER_TEMPERATURE_IN_DISCHARGE_ERROR = 211,
    SBS_WATCH_DOG_ERROR = 212,
    PRE_CHARGE_TIMEOUT_ERROR = 213,
    FAST_CHARGE_TIMEOUT_ERROR = 214,
    OVER_CHARGE_ALERT_ERROR = 215,
    OVER_CHARGE_ERROR = 216,
    OVER_CHARGING_VOLTAGE_ERROR = 217,
    OVER_CHARGING_CURRENT_ERROR = 218,
    OVER_PRE_CHARGING_CURRENT_ERROR = 219,
    PRES_BIT_LOW_ERROR = 220,

    NS_SENSOR_TIMEOUT = 255,
} ScannerError_t;

#endif