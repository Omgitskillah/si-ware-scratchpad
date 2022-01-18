#ifndef BLUETOOTHMANAGERDEFS_H
#define BLUETOOTHMANAGERDEFS_H

#include "esp_gatts_api.h"

#define P3_SERVICE_UUID        		"6E400001B5A3F393E0A9E50E24DCCA9E"  	//P3 Service
#define P3_TX_CHAR_UUID	 	    	"6E400003B5A3F393E0A9E50E24DCCA9E"	    //P3 Characteristic 1
#define P3_RX_CHAR_UUID 			"6E400002B5A3F393E0A9E50E24DCCA9E"	    //P3 Characteristic 2

#define SYS_SERVICE_UUID       		"B100B100B100B100B100B100B100B100"	    //System Service
#define SYS_TX_CHAR_UUID 			"B101B101B101B101B101B101B101B101"      //System Characteristic 1
#define SYS_RX_CHAR_UUID			"B102B102B102B102B102B102B102B102"      //System Characteristic 2

#define MEM_SERVICE_UUID	       	"C100C100C100C100C100C100C100C100"	    //Memory Service
#define MEM_TX_CHAR_UUID 			"C101C101C101C101C101C101C101C101"      //Memory Characteristic 1
#define MEM_RX_CHAR_UUID			"C102C102C102C102C102C102C102C102"      //Memory Characteristic 2

#define OTA_SERVICE_UUID	       	"D100D100D100D100D100D100D100D100"  	//OTA Service
#define OTA_TX_CHAR_UUID			"D101D101D101D101D101D101D101D101"      //OTA Characteristic 1
#define OTA_RX_CHAR_UUID			"D102D102D102D102D102D102D102D102"      //OTA Characteristic 2

#define NUMBER_OF_SERVICES          4
#define NUMBER_OF_CHARACTERISTICS   2

#define GATTS_NUM_SERVICE_HANDLE    15

typedef enum {
    CHAR_TX = 0,
    CHAR_RX = 1
} bluetoothManagerChar_t;

typedef struct {
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_char_prop_t property;
    esp_attr_value_t value;
} bluetoothManagerCharInterface_t;

typedef struct {
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    bluetoothManagerCharInterface_t chars[NUMBER_OF_CHARACTERISTICS];
} bluetoothManagerServiceInterface_t;

#endif