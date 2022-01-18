#include "bluetoothManager_prv.h"

#define TAG "bluetoothManager"

uint8_t clientConfigDescriptorValue[] = {0, 0};
uint8_t descriptorHandles[4];

CLService_t bluetoothManagerUtils_identifyServiceByUUID(uint8_t *const uuid)
{
    if (bluetoothManagerUtils_compareUUID128(uuid,
                                             (uint8_t *const) &serviceInterface[SERVICE_P3].service_id.id.uuid.uuid) ==
        0)
        return SERVICE_P3;
    else if (bluetoothManagerUtils_compareUUID128(uuid,
                                                  (uint8_t *const) &serviceInterface[SERVICE_SYSTEM].service_id.id.uuid.uuid) ==
             0)
        return SERVICE_SYSTEM;
    else if (bluetoothManagerUtils_compareUUID128(uuid,
                                                  (uint8_t *const) &serviceInterface[SERVICE_MEMORY].service_id.id.uuid.uuid) ==
             0)
        return SERVICE_MEMORY;
    else if (bluetoothManagerUtils_compareUUID128(uuid,
                                                  (uint8_t *const) &serviceInterface[SERVICE_OTA].service_id.id.uuid.uuid) ==
             0)
        return SERVICE_OTA;
    else
        return SERVICE_UNDEFINED;
}

CLService_t bluetoothManagerUtils_identifyServiceByHandle(uint16_t service_handle)
{
//    if (service_handle == serviceInterface[SERVICE_P3].service_handle ||
//        (service_handle % serviceInterface[SERVICE_P3].service_handle) < GATTS_NUM_SERVICE_HANDLE)
//        return SERVICE_P3;
//    else if (service_handle == serviceInterface[SERVICE_SYSTEM].service_handle ||
//             (service_handle % serviceInterface[SERVICE_SYSTEM].service_handle) < GATTS_NUM_SERVICE_HANDLE)
//        return SERVICE_SYSTEM;
//    else if (service_handle == serviceInterface[SERVICE_MEMORY].service_handle ||
//             (service_handle % serviceInterface[SERVICE_MEMORY].service_handle) < GATTS_NUM_SERVICE_HANDLE)
//        return SERVICE_MEMORY;
//    else if (service_handle == serviceInterface[SERVICE_OTA].service_handle ||
//             (service_handle % serviceInterface[SERVICE_OTA].service_handle) < GATTS_NUM_SERVICE_HANDLE)
//        return SERVICE_OTA;
//    else
//        return SERVICE_UNDEFINED;

    for (int i = 0; i < SERVICE_MAX; ++i)
    {
        if ((service_handle >= serviceInterface[i].service_handle) && (service_handle <  (serviceInterface[i].service_handle + GATTS_NUM_SERVICE_HANDLE))){
            return i;
        }
    }
    return SERVICE_UNDEFINED;
}

static void bluetoothManager_createServices(esp_gatt_if_t gatts_if)
{
    esp_ble_gatts_create_service(gatts_if, &serviceInterface[SERVICE_P3].service_id, GATTS_NUM_SERVICE_HANDLE);
    esp_ble_gatts_create_service(gatts_if, &serviceInterface[SERVICE_SYSTEM].service_id, GATTS_NUM_SERVICE_HANDLE);
    esp_ble_gatts_create_service(gatts_if, &serviceInterface[SERVICE_MEMORY].service_id, GATTS_NUM_SERVICE_HANDLE);
    esp_ble_gatts_create_service(gatts_if, &serviceInterface[SERVICE_OTA].service_id, GATTS_NUM_SERVICE_HANDLE);
}

static void bluetoothManager_handleCreateEvent(esp_ble_gatts_cb_param_t *param)
{
    esp_bt_uuid_t descr_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG
    };
    esp_attr_value_t desc_value = {
            .attr_max_len = sizeof(uint16_t),
            .attr_len = sizeof(clientConfigDescriptorValue),
            .attr_value = clientConfigDescriptorValue,
    };
    esp_attr_control_t desc_control = {
            .auto_rsp = ESP_GATT_AUTO_RSP,
    };
    esp_err_t error;

    CLService_t serviceType = bluetoothManagerUtils_identifyServiceByUUID(
            (uint8_t *const) &param->create.service_id.id.uuid.uuid);

    serviceInterface[serviceType].service_handle = param->create.service_handle;

    esp_attr_control_t control = {
            .auto_rsp = ESP_GATT_RSP_BY_APP,
    };

    error = esp_ble_gatts_start_service(param->create.service_handle);
    if (error)
        ESP_LOGE(TAG, "start service failed, error code =%x", error);

    error = esp_ble_gatts_add_char(serviceInterface[serviceType].service_handle,
                                   &serviceInterface[serviceType].chars[CHAR_TX].char_uuid,
                                   ESP_GATT_PERM_READ /*| ESP_GATT_PERM_WRITE*/,
                                   serviceInterface[serviceType].chars[CHAR_TX].property,
                                   NULL,
                                   &control);
    if (error)
        ESP_LOGE(TAG, "add char failed, error code =%x", error);
    ESP_ERROR_CHECK_WITHOUT_ABORT(
            esp_ble_gatts_add_char_descr(serviceInterface[serviceType].service_handle, &descr_uuid,
                                         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, &desc_value, &desc_control));

    error = esp_ble_gatts_add_char(serviceInterface[serviceType].service_handle,
                                   &serviceInterface[serviceType].chars[CHAR_RX].char_uuid,
                                   /*ESP_GATT_PERM_READ |*/ ESP_GATT_PERM_WRITE,
                                   serviceInterface[serviceType].chars[CHAR_RX].property,
                                   NULL,
                                   &control);
    if (error)
        ESP_LOGE(TAG, "add char failed, error code =%x", error);


}

static void bluetoothManager_handleAddCharEvent(esp_ble_gatts_cb_param_t *param)
{

    CLService_t serviceType = bluetoothManagerUtils_identifyServiceByHandle(param->add_char.service_handle);
    if (bluetoothManagerUtils_compareUUID128((uint8_t *const) &param->add_char.char_uuid.uuid,
                                             (uint8_t *const) &serviceInterface[serviceType].chars[CHAR_TX].char_uuid.uuid) ==
        0)
    {
        serviceInterface[serviceType].chars[CHAR_TX].char_handle = param->add_char.attr_handle;
    } else
    {
        serviceInterface[serviceType].chars[CHAR_RX].char_handle = param->add_char.attr_handle;
    }
}

static void bluetoothManager_handleWriteEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    static esp_gatt_rsp_t gatt_rsp;
    esp_gatt_status_t gatts_status = ESP_GATT_OK;
    ESP_LOGI(TAG, "Recieved data size is %d", param->write.len);
    if (param->write.handle == descriptorHandles[0] || param->write.handle == descriptorHandles[1] ||
        param->write.handle == descriptorHandles[2] || param->write.handle == descriptorHandles[3])
    {
        ESP_LOG_BUFFER_HEX(TAG,param->write.value,param->write.len);
        ESP_LOGI(TAG,"Descriptor Write");
        gatts_status = ESP_GATT_OK;
    } else
    {
//        ESP_LOGI(TAG,"Write handle is %d, OTA Service handle is %d",param->write.handle,serviceInterface[SERVICE_OTA].service_handle);
        CLService_t serviceType = bluetoothManagerUtils_identifyServiceByHandle(param->write.handle);
//        ESP_LOGI(TAG,"Service Type is %d",serviceType);
        if (CL_rxHandler(serviceType, param->write.value, param->write.len) == pdPASS)
            gatts_status = ESP_GATT_OK;
        else
            gatts_status = ESP_GATT_BUSY;
    }
    if (param->write.need_rsp)
    {
        ESP_LOGI(TAG, "Need Response");
        if (param->write.is_prep)
        {
            gatts_status = ESP_GATT_OK;
            ESP_LOGI(TAG, "Prepare Write, Length : %d, Offset : %d", param->write.len, param->write.offset);
            gatt_rsp.attr_value.len = sizeof(esp_gatt_status_t);
            gatt_rsp.attr_value.handle = param->write.handle;
            gatt_rsp.attr_value.offset = param->write.offset;
            gatt_rsp.attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp.attr_value.value, param->write.value, param->write.len);
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, gatts_status, &gatt_rsp);
        } else
        {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, gatts_status, NULL);
        }

    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            read_for_adv = true;
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG,
                     "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                     param->update_conn_params.status,
                     param->update_conn_params.min_int,
                     param->update_conn_params.max_int,
                     param->update_conn_params.conn_int,
                     param->update_conn_params.latency,
                     param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    static int descriptorCounter = 0;
    switch (event)
    {
        case ESP_GATTS_REG_EVT:
        {
            esp_err_t error = esp_ble_gap_set_device_name(bluetoothManager_adv_name);
            if (error)
                ESP_LOGE(TAG, "Set device name failed: %s", esp_err_to_name(error));

            //adv data and scan response data
            esp_ble_adv_data_t adv_data = {
                    .set_scan_rsp = false,
                    .include_name = true,
                    .include_txpower = false,
                    .min_interval = 0x20, //slave connection min interval, Time = min_interval * 1.25 msec
                    .max_interval = 0x40, //slave connection max interval, Time = max_interval * 1.25 msec
                    .appearance = 0x00,
                    .manufacturer_len = 0,
                    .p_manufacturer_data = NULL,
                    .service_data_len = 0,
                    .p_service_data = NULL,
                    .service_uuid_len = 0,
                    .p_service_uuid = NULL,
                    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
            };

            //config adv data
            ESP_LOGI(TAG, "Config ADV data");
            error = esp_ble_gap_config_adv_data(&adv_data);
            if (error)
                ESP_LOGE(TAG, "Config adv data failed: %s", esp_err_to_name(error));

            bluetoothManager_createServices(gatts_if);
            break;
        }

        case ESP_GATTS_CREATE_EVT:
            bluetoothManager_handleCreateEvent(param);
            break;

        case ESP_GATTS_ADD_CHAR_EVT:
            bluetoothManager_handleAddCharEvent(param);
            break;
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            ESP_LOGI(TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d",
                     param->add_char_descr.status, param->add_char_descr.attr_handle,
                     param->add_char_descr.service_handle);
            descriptorHandles[descriptorCounter++] = param->add_char_descr.attr_handle;
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(TAG, "SERVICE_START_EVT, status %d, service_handle %d",
                     param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            _gattsIf = gatts_if;
            _connId = param->connect.conn_id;

            if (connectionCallback != NULL)
            {
                connectionCallback(true);
            }
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            
            /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d,timeout %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                     param->connect.conn_id,param->connect.conn_params.timeout,
                     param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                     param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
            //start sent the update connection parameters to the peer device.
            if(param->connect.conn_params.timeout < conn_params.timeout){
                //update only if the default time out is slower
                esp_ble_gap_update_conn_params(&conn_params);
            }
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            if (connectionCallback != NULL)
            {
                connectionCallback(false);
            }
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_READ_EVT:
        {
            esp_gatt_rsp_t rsp;
            rsp.attr_value.len = 4;
            rsp.attr_value.value[0] = 0xde;
            rsp.attr_value.value[1] = 0xed;
            rsp.attr_value.value[2] = 0xbe;
            rsp.attr_value.value[3] = 0xef;
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                        ESP_GATT_OK, &rsp);
            break;
        }
        case ESP_GATTS_WRITE_EVT:
            bluetoothManager_handleWriteEvent(gatts_if, param);
            break;
        case ESP_GATTS_EXEC_WRITE_EVT:
           ESP_LOGI(TAG, "Exec Write Event");
           esp_ble_gatts_send_response(gatts_if, param->exec_write.conn_id, param->exec_write.trans_id, ESP_GATT_OK,
                                       NULL);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(TAG, "New MTU is %d", param->mtu.mtu);
            break;
        default:
            break;
    }
}

static void bluetoothManager_initializeServiceInterface()
{
    for (int i = 0; i < NUMBER_OF_SERVICES; i++)
    {
        serviceInterface[i].service_id.is_primary = true;
        serviceInterface[i].service_id.id.inst_id = 0;
        serviceInterface[i].service_id.id.uuid.len = ESP_UUID_LEN_128;

        serviceInterface[i].chars[CHAR_TX].char_uuid.len = ESP_UUID_LEN_128;
        serviceInterface[i].chars[CHAR_RX].char_uuid.len = ESP_UUID_LEN_128;

        serviceInterface[i].chars[CHAR_TX].property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        serviceInterface[i].chars[CHAR_RX].property = ESP_GATT_CHAR_PROP_BIT_WRITE;
    }

    bluetoothManagerUtils_setUUID128((uint8_t *const) &serviceInterface[SERVICE_P3].service_id.id.uuid.uuid,
                                     P3_SERVICE_UUID);
    bluetoothManagerUtils_setUUID128((uint8_t *const) &serviceInterface[SERVICE_P3].chars[CHAR_TX].char_uuid.uuid,
                                     P3_TX_CHAR_UUID);
    bluetoothManagerUtils_setUUID128((uint8_t *const) &serviceInterface[SERVICE_P3].chars[CHAR_RX].char_uuid.uuid,
                                     P3_RX_CHAR_UUID);

    bluetoothManagerUtils_setUUID128((uint8_t *const) &serviceInterface[SERVICE_SYSTEM].service_id.id.uuid.uuid,
                                     SYS_SERVICE_UUID);
    bluetoothManagerUtils_setUUID128((uint8_t *const) &serviceInterface[SERVICE_SYSTEM].chars[CHAR_TX].char_uuid.uuid,
                                     SYS_TX_CHAR_UUID);
    bluetoothManagerUtils_setUUID128((uint8_t *const) &serviceInterface[SERVICE_SYSTEM].chars[CHAR_RX].char_uuid.uuid,
                                     SYS_RX_CHAR_UUID);

    bluetoothManagerUtils_setUUID128((uint8_t *const) &serviceInterface[SERVICE_MEMORY].service_id.id.uuid.uuid,
                                     MEM_SERVICE_UUID);
    bluetoothManagerUtils_setUUID128((uint8_t *const) &serviceInterface[SERVICE_MEMORY].chars[CHAR_TX].char_uuid.uuid,
                                     MEM_TX_CHAR_UUID);
    bluetoothManagerUtils_setUUID128((uint8_t *const) &serviceInterface[SERVICE_MEMORY].chars[CHAR_RX].char_uuid.uuid,
                                     MEM_RX_CHAR_UUID);

    bluetoothManagerUtils_setUUID128((uint8_t *const) &serviceInterface[SERVICE_OTA].service_id.id.uuid.uuid,
                                     OTA_SERVICE_UUID);
    bluetoothManagerUtils_setUUID128((uint8_t *const) &serviceInterface[SERVICE_OTA].chars[CHAR_TX].char_uuid.uuid,
                                     OTA_TX_CHAR_UUID);
    bluetoothManagerUtils_setUUID128((uint8_t *const) &serviceInterface[SERVICE_OTA].chars[CHAR_RX].char_uuid.uuid,
                                     OTA_RX_CHAR_UUID);
}

static void bluetoothManager_sendNotification(void *char_handle, uint8_t *data, uint16_t length)
{
    while (esp_ble_get_cur_sendable_packets_num(_connId) <= 1)
    {
        ESP_LOGI(TAG, "Waiting to send BLE message");
        vTaskDelay(2);
    }
//    ESP_LOGI(TAG,"Sending BLE Message");
    ESP_ERROR_CHECK_WITHOUT_ABORT(
            esp_ble_gatts_send_indicate(_gattsIf, _connId, *((uint16_t *) char_handle), length, data, false));
}

void bluetoothManager_responseTask(void *parameters)
{
    ResponsePacket_t respPacket;

    memcpy(&respPacket, parameters, sizeof(ResponsePacket_t));
    xEventGroupSetBits(taskReceivedData, BTMANAGER_RESPONSE_TASK_BIT); //Set the bit after copying the parameters

    switch (respPacket.packetType)
    {
        case RESPONSEPACKET_P3:
            CL_p3ServiceTx(&respPacket, (CLSendHandler_t) bluetoothManager_sendNotification,
                           (void *) &serviceInterface[SERVICE_P3].chars[CHAR_TX].char_handle);
            break;

        case RESPONSEPACKET_SYSTEM:
            CL_systemServiceTx(&respPacket, (CLSendHandler_t) bluetoothManager_sendNotification,
                               (void *) &serviceInterface[SERVICE_SYSTEM].chars[CHAR_TX].char_handle);
            break;

        case RESPONSEPACKET_MEMORY:
            CL_memoryServiceTx(&respPacket, (CLSendHandler_t) bluetoothManager_sendNotification,
                               (void *) &serviceInterface[SERVICE_MEMORY].chars[CHAR_TX].char_handle);
            break;

        case RESPONSEPACKET_OTA:
            CL_OTAServiceTx(&respPacket, (CLSendHandler_t) bluetoothManager_sendNotification,
                            (void *) &serviceInterface[SERVICE_OTA].chars[CHAR_TX].char_handle);
            break;

        default:
            break;
    }

    ESP_LOGI(TAG, "Remaining stack %d bytes", uxTaskGetStackHighWaterMark(NULL));
    xEventGroupSetBits(availableTasks, BTMANAGER_RESPONSE_TASK_BIT); //Set the bit before deleting the task
    vTaskDelete(NULL);  //Delete itself
    while (1);           //This task should be deleted after finishing
}

void bluetoothManager_setConnectionCallback(CLConnectioncb_t callback)
{
    connectionCallback = callback;
}

void bluetoothManager_advertisement(bool start)
{
    esp_err_t error;
    while (!read_for_adv);

    if (start)
    {
        error = esp_ble_gap_start_advertising(&adv_params);
        if (error)
            ESP_LOGE(TAG, "Failed to start advertisement: %s\n", esp_err_to_name(error));
    } else
    {
        error = esp_ble_gap_stop_advertising();
        if (error)
            ESP_LOGE(TAG, "Failed to stop advertisement: %s\n", esp_err_to_name(error));
    }
}


void bluetoothManager_init(ScannerID_t *scannerID)
{
    esp_err_t ret;

    strncpy(bluetoothManager_adv_name, (char *) &scannerID->id, sizeof(ScannerID_t));

    bluetoothManager_initializeServiceInterface();

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ERROR_CHECK(TAG, nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ERROR_CHECK(TAG, ret);
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret)
    {
        ESP_LOGE(TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        ESP_LOGE(TAG, "gap register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(0);
    if (ret)
    {
        ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    //This 517 is the absolute maximum MTU, and it grants you payload of 512 bytes
    //WHATEVER YOU DO, NEVER INCREASE UPON THOSE TWO VALUES
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(517);
    if (local_mtu_ret)
    {
        ESP_LOGE(TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
        return;
    }

}
