#include <string.h>

#include "bluetoothManager.h"
#include "bluetoothManagerUtils.h"
#include "bluetoothManagerDefs.h"
          
#include "centralManager.h"

#include "freertos/semphr.h"

//Includes for BLE
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#define MAX_ADV_NAME_LENGTH     52

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
};

static esp_gatt_if_t _gattsIf; 
static uint16_t _connId;

static char bluetoothManager_adv_name[MAX_ADV_NAME_LENGTH] = {0};

static bluetoothManagerServiceInterface_t serviceInterface[NUMBER_OF_SERVICES];

static CLConnectioncb_t connectionCallback = NULL;

static bool read_for_adv = false;

/**
 * @brief           Callback handler for the gap events.
 *
 * @param[in]       event: enum to identify which gap event triggered the handler.
 *                  param: pointer to the parameters of the event. 
 * 
 */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

/**
 * @brief           Callback handler for the gatt server events.
 *
 * @param[in]       event: enum to identify which gatts event triggered the handler.
 *                  gatts_if: identifer to the application interface.
 *                  param: pointer to the parameters of the event.
 * 
 */
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/**
 * @brief           Initialize serviceInterface structure.
 *
 * @param[in]       gatts_if: value of the gatt server interface
 * 
 */
static void bluetoothManager_initializeServiceInterface();

/**
 * @brief           Create the four main service of the scanner.
 *
 * @param[in]       gatts_if: value of the gatt server interface
 * 
 */
static void bluetoothManager_createServices(esp_gatt_if_t gatts_if);

/**
 * @brief           Handle the create service event.
 *
 * @param[in]       param: Pointer to the event parameters
 * 
 */
static void bluetoothManager_handleCreateEvent(esp_ble_gatts_cb_param_t* param);

/**
 * @brief           Handle the add characteristic event.
 *
 * @param[in]       param: Pointer to the event parameters
 * 
 */
static void bluetoothManager_handleAddCharEvent(esp_ble_gatts_cb_param_t* param);

/**
 * @brief           Handle the write event.
 *
 * @param[in]       param: Pointer to the event parameters
 * 
 */
static void bluetoothManager_handleWriteEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param);

/**
 * @brief           Identify the type of the service according to its UUID.
 *
 * @param[in]       uuid: Pointer to the uuid
 *
 * @return
 *                  -   SERVICE_P3
 *                  -   SERVICE_SYSTEM
 *                  -   SERVICE_MEMORY
 *                  -   SERVICE_OTA
 *                  -   SERVICE_UNDEFINED
 */
static CLService_t bluetoothManagerUtils_identifyServiceByUUID(uint8_t *const uuid);

/**
 * @brief           Identify the type of the service according to its service_handle.
 *
 * @param[in]       uuid: Pointer to the uuid
 *
 * @return
 *                  -   SERVICE_P3
 *                  -   SERVICE_SYSTEM
 *                  -   SERVICE_MEMORY
 *                  -   SERVICE_OTA
 *                  -   SERVICE_UNDEFINED
 */
static CLService_t bluetoothManagerUtils_identifyServiceByHandle(uint16_t service_handle);

static void bluetoothManager_sendNotification(void* char_handle, uint8_t* data, uint16_t length);