#include <sys/cdefs.h>
#include "esp_sleep.h"
#include "uartManager.h"
#include "bluetoothManager.h"
#include "NS_sensor.h"
#include "fileSystem.h"
#include "powerManagement.h"
#include "oledInterface.h"
#include "firmwareUpdate.h"

#include "centralManager.h"
#include "centralManager_priv.h"
#include "centralManagerSystemIOs.h"
#include "esp_spi_flash.h"

#define TAG "centralManager"
#define FORCE_BARQ_INSTALLED 0

static void IRAM_ATTR centralManager_buttonsISR(void *arg);

static uint8_t central_manager_is_barq_installed();

static void centralManager_sleep(uint8_t emergencyShutdown);
static void centralManager_periodicBatteryCheck(TimerHandle_t xTimer);
static void centralManager_prepareBatteryLogPacket(event_type_t event);
static void centralManager_runMaintenanceTask(MaintenanceRequestPacket_t* request_packet);

static io_config_t system_io_config;
static oldFormatGaugeInfo_t oldFormatGaugeInfo;
uint8_t sharedStaticMemory[SHARED_MEMORY_SIZE] = GOLDEN_REGISTER_FILE;
char error_string[ERROR_STRING_SIZE + 1];

static bool _bleConnected = false;
static bool _uartConnected = false;
static battery_data_t _battery_data;

static ScannerSettings_t _scannerSettings = {
        .apodizationSel = 0,    //Boxcar
        .commonWaveNum = 3,     //257-points
        .gainvalue = 0,         //ignored
        .mode = 0,              //single mode
        .opticalGain = 0,       //default
        .scanTime = 5000,       //5 seconds
        .zeroPadding = 3,       //32K padding
};

static QueueHandle_t buttonINTRQueue = NULL;
QueueHandle_t requestQueue = NULL;
QueueHandle_t responseQueue = NULL;
QueueHandle_t memoryAllocationQueue = NULL;

EventGroupHandle_t availableTasks = NULL;
EventGroupHandle_t taskReceivedData = NULL;

static TimerHandle_t bluetoothAdvTimer = NULL;
static TimerHandle_t inactiveTimer = NULL;

static void centralManager_AdvTimerCB(TimerHandle_t pxTimer) {
    bluetoothManager_advertisement(false);
    OLED_bluetooth(BLUETOOTH_OFF);
}

static void centralManager_inactive_timer_callback(TimerHandle_t xTimer) {
    ESP_LOGI(TAG, "Going to Sleep, been inactive for some time now");
    centralManager_sleep(0);
}

static void centralManager_sleep(uint8_t emergencyShutdown) {
    OLED_shutdown(emergencyShutdown);
    PM_sleep();
    ESP_LOGI(TAG, "Going to Sleep");
    centralManager_prepareBatteryLogPacket(EVENT_TYPE_SHUT_DOWN);
    vTaskDelay(20);
    xEventGroupWaitBits(availableTasks,ALL_REQUEST_BITS,pdFALSE,pdTRUE,portMAX_DELAY);//Wait till the request is done

    esp_deep_sleep_start();
}

static void print_scanner_settings(const ScannerSettings_t *scanner_settings) {
    ESP_LOGI(TAG, "Scanner Settings commonWaveNum %d", scanner_settings->commonWaveNum);
    ESP_LOGI(TAG, "Scanner Settings zeroPadding %d", scanner_settings->zeroPadding);
    ESP_LOGI(TAG, "Scanner Settings mode %d", scanner_settings->mode);
    ESP_LOGI(TAG, "Scanner Settings apodizationSel %d", scanner_settings->apodizationSel);
    ESP_LOGI(TAG, "Scanner Settings opticalGain %d", scanner_settings->opticalGain);
    ESP_LOGI(TAG, "Scanner Settings scanTime %d", scanner_settings->scanTime);
}

static void centralManager_temperature_check_timer_callback(TimerHandle_t xTimer) {
    static P3RequestPacket_t rp = {
            .operationID = OPID_CHECK_TEMP,
    };
    centralManager_createTask(NS_requestTask, NS_REQEUST_TASK_BIT, 2560, (void *) &rp);

}

void centralManager_createQueues() {
    buttonINTRQueue = xQueueCreate(1, sizeof(gpio_num_t));
    requestQueue = xQueueCreate(REQUEST_QUEUE_SIZE, sizeof(RequestPacket_t));
    responseQueue = xQueueCreate(RESPONSE_QUEUE_SIZE, sizeof(ResponsePacket_t));
    memoryAllocationQueue = xQueueCreate(MEMORY_ALLOCATION_QUEUE_SIZE, sizeof(uint8_t));
    if (requestQueue == NULL || responseQueue == NULL || buttonINTRQueue == NULL) {
        ESP_LOGE(TAG, "Error while creating the centralManager's queues");
        return;
    }

    availableTasks = xEventGroupCreate();
    taskReceivedData = xEventGroupCreate();
    if (availableTasks == NULL || taskReceivedData == NULL) {
        ESP_LOGE(TAG, "Error while creating the centralManager's event groups");
        return;
    }
    xEventGroupSetBits(availableTasks, 0xFF); //All tasks are available by default
}

void centralManager_centralTasks() {
    BaseType_t ret = xTaskCreate(centralManager_responseTask, "CM_Response", 2048, NULL, 2, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Error while creating the centralManager_responseTask");
        return;
    }

    centralManager_requestsTask(); //continue on the main task
}

static uint8_t centralManager_identify_proper_io(uint32_t mother_board_version) {
    uint8_t barq_installed = FORCE_BARQ_INSTALLED;
    const io_config_t old_mother_board_config = {
            .io_power_button_pin = IO_POWER_BUTTON,
            .io_scan_button_pin = IO_SCAN_BUTTON,
    };
    const io_config_t new_v7_mother_board_config = {
            .io_scan_button_pin = BARQ_SCAN_BUTTON_PIN,
            .io_power_button_pin = IO_POWER_BUTTON,
    };

    /**
     *
     * This may or may not be a v7 board, so we need to see what's installed on it, is it BARQ
     * or m7. In case of BARQ installed then this is surely a v7 or newer board, in case of m7 installed
     * then this is surely older than v7.
     * This check is done using the MODE_SELECT_BAR or the SCAN_BUTTON_PIN, by pulling it up and testing
     * whether it's actually up or not. if this is an m7 chip, this pin will remain high, otherwise
     * this pin will be pulled down to ground
     **/
    gpio_config_t conf = {
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_DISABLE,
            .pin_bit_mask = (1ULL << MODE_SELECT_BAR_M7_SCAN_BUTTON_PIN),
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&conf));
    //Wait for some time for everything to become stable (IO wise)
    //I know 200 cycles is a lot, but why the heck xD
    vTaskDelay(200);
    ESP_LOGI(TAG, "Mode Select Level is %d", gpio_get_level((gpio_num_t) MODE_SELECT_BAR_M7_SCAN_BUTTON_PIN));
    if (1 == gpio_get_level((gpio_num_t) MODE_SELECT_BAR_M7_SCAN_BUTTON_PIN)) {
        //This is m7 chip
        if (mother_board_version >= MOTHER_BOARD_VERSION_G) {
            //We found that this board is a v7 board, so no need to do any further investigation
            //set the configuration and be on your way
            system_io_config = new_v7_mother_board_config;
        } else {
            system_io_config = old_mother_board_config;
        }
    } else {
        //This is barq chip
        barq_installed = 1;
        system_io_config = new_v7_mother_board_config;
        ESP_LOGI(TAG, "Barq Installed");
    }
    return barq_installed;
}

uint8_t central_manager_is_barq_installed() {
    vTaskDelay(200);
    return centralManager_identify_proper_io(0);
}

void centralManager_buttonsInit(uint32_t mother_board_version) {
    centralManager_identify_proper_io(mother_board_version);
    gpio_config_t conf = {
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_NEGEDGE, //both this buttons are pulled high, which means that a negative edge is actually a press
            // HIGH = non pressed, LOW = pressed
            .pin_bit_mask = ((1ULL << system_io_config.io_scan_button_pin) |
                             (1ULL << system_io_config.io_power_button_pin)),
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&conf);

    gpio_isr_handler_add(system_io_config.io_scan_button_pin, centralManager_buttonsISR,
                         (void *) ((gpio_num_t) system_io_config.io_scan_button_pin));
    gpio_isr_handler_add(system_io_config.io_power_button_pin, centralManager_buttonsISR,
                         (void *) ((gpio_num_t) system_io_config.io_power_button_pin));

    BaseType_t ret = xTaskCreate(centralManager_buttonsTask, "CM_ButtonsTask", 2048, NULL, 2, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Error while creating the CM_ButtonsTask");
    }

    esp_sleep_enable_ext0_wakeup(system_io_config.io_power_button_pin, 0);
}

void centralManager_io_isr_init() {
    gpio_install_isr_service(0);
    vTaskDelay(100);
}

void centralManager_systemStart() {
    uint8_t batteryPercentage;
    uint16_t memorySize;
    uint32_t chargerState;
    uint32_t mother_board_version = 0;
    uint32_t inactive_timeout = INACTIVE_TIMER_DEFAULT_PERIOD;
    ScannerID_t scannerID = {
            .id = {0},
    };
    SpectralData_t background;
    PM_wakeup_reason_check();
    OLED_setupFirstStage();
    centralManager_createQueues();
    centralManager_io_isr_init();
    fileSystem_init_flash();
    if (!fileSystem_is_gauge_golden_file_burnt()) {
        PM_init(true, true);
        ESP_LOGI(TAG, "Done with PM Init");
        fileSystem_register_golden_file_burnt();
        //Restart the ECU
//        esp_restart();
    } else {
        ESP_LOGI(TAG,"Reset Reason is %d",esp_reset_reason());
        ESP_LOGI(TAG,"Wakeup Reason is %d",esp_sleep_get_wakeup_cause());
        if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED && esp_reset_reason() == ESP_RST_POWERON){
            ESP_LOGI(TAG,"Initializing Gauge");
            PM_init(true, 0);
        }else{
            PM_init(false, 0);
        }
    }

    //Must init the IOS before asking whether BARQ installed or not
    //Because in this function BARQ is turned on
    NS_sensorInitIOs();
    uint8_t barq_installed = central_manager_is_barq_installed();
    ESP_LOGI(TAG, "Installed Board is %s", barq_installed ? "BARQ" : "M7");
    NS_sensorInit(barq_installed);
    sourceSettings_t sourceSettingsObject = {
            .lampsCount = 2,
            .lampsSelect = 0,
            .t1 = 14,
            .deltaT = 2,
            .t2C1 = 5,
            .t2C2 = 35,
            .t2max = 10,
    };
    TemperatureReading temperature = NS_getTemperature();
    fileSystem_init(&_scannerSettings, &scannerID, &memorySize, &background, &sourceSettingsObject, &temperature,
                    &mother_board_version, &inactive_timeout, &NS_temperature_window);
    print_scanner_settings(&_scannerSettings);
    centralManager_buttonsInit(mother_board_version);

    if (scannerID.id[0] == 0)
        NS_getScannerID(&scannerID);
    ESP_LOGI(TAG, "Scanner ID is %s", scannerID.id);

    NS_setupSettings(&_scannerSettings, &sourceSettingsObject);
    //This maybe the same temperature I just read from the NS module, and it can be the temperature read from the flash
    NS_set_temperature(temperature);
    if (background.spectralPTR != NULL)
        NS_setBackground(&background);

    PM_getBatteryStatus(&batteryPercentage, &chargerState);
    OLED_setupSecondStage(memorySize, batteryPercentage, chargerState);

    bluetoothManager_init(&scannerID);
    bluetoothManager_setConnectionCallback(centralManager_bluetoothConnectionCallback);
    bluetoothManager_advertisement(true);
    bluetoothAdvTimer = xTimerCreate("Bluetooth Advertisement Timer",
                                     BLUETOOTH_TIMER_PERIOD, pdFALSE, NULL, centralManager_AdvTimerCB);
    xTimerStart(bluetoothAdvTimer, 0);
    uartManager_setup();
    uartManager_setConnectionCallback(centralManager_uartConnectionCallback);
    esp_log_set_vprintf(uartManager_log);
//    ESP_LOGI(TAG,"SharedMemoryAddress is %p",&sharedStaticMemory[0]);
    ESP_LOGI(TAG, "Starting Inactive Timeout Timer with %d minutes", inactive_timeout);
    inactiveTimer = xTimerCreate("Active Time Timer", pdMS_TO_TICKS(inactive_timeout * 60 * 1000), pdFALSE, NULL,
                                 centralManager_inactive_timer_callback);
    xTimerStart(inactiveTimer, portMAX_DELAY);
    TimerHandle_t periodic_check_timer = xTimerCreate("Periodic Battery Check", pdMS_TO_TICKS(PERIODIC_CHECK_PERIOD),pdTRUE,NULL,centralManager_periodicBatteryCheck);
    xTimerStart(periodic_check_timer,portMAX_DELAY);
    centralManager_prepareBatteryLogPacket(EVENT_TYPE_POWER_UP);
    centralManager_centralTasks();
}

void centralManager_bluetoothConnectionCallback(bool status) {
    _bleConnected = status;
    if (status) {
        ESP_LOGI(TAG, "Bluetooth connected");
        OLED_bluetooth(BLUETOOTH_CONNECTED);
        xTimerStop(bluetoothAdvTimer, 0);
    } else {
        ESP_LOGI(TAG, "Bluetooth disconnected");
        OLED_bluetooth(BLUETOOTH_ADVERTISEMENT);
        xTimerReset(bluetoothAdvTimer, 0);
    }
}

void centralManager_uartConnectionCallback(bool status) {
    _uartConnected = status;
    if (status) {
        ESP_LOGI(TAG, "uart connected");
        OLED_uart(UART_CONNECTED);
        //quick fix for watch dog reset issue with bluetooth advertisement on and asking for a scan
        if(!_bleConnected){
            bluetoothManager_advertisement(false);
            OLED_bluetooth(BLUETOOTH_OFF);
        }
    } else {
        ESP_LOGI(TAG, "uart disconnected");
        OLED_uart(UART_DISCONNECTED);
    }
}

void centralManager_convertGaugeInfoToOldFormat(const GaugeInfo_t *gaugeInfo, oldFormatGaugeInfo_t *oldFormatGaugeInfo_ptr,
                                          uint32_t operation_id) {
    memset(oldFormatGaugeInfo_ptr, 0, sizeof(oldFormatGaugeInfo_t));
    oldFormatGaugeInfo_ptr->battVoltage = gaugeInfo->battVoltage;
    oldFormatGaugeInfo_ptr->current = gaugeInfo->current;
    oldFormatGaugeInfo_ptr->capacity = gaugeInfo->capacity;
    oldFormatGaugeInfo_ptr->fullCapacity = gaugeInfo->fullCapacity;
    oldFormatGaugeInfo_ptr->temperature = gaugeInfo->temperature;
    oldFormatGaugeInfo_ptr->cellVoltage1 = gaugeInfo->cellVoltage1;
    oldFormatGaugeInfo_ptr->cellVoltage2 = gaugeInfo->cellVoltage2;
    oldFormatGaugeInfo_ptr->timeToEmpty = gaugeInfo->timeToEmpty;
    oldFormatGaugeInfo_ptr->operation_id1 = operation_id;  //th
    oldFormatGaugeInfo_ptr->timeToFull = gaugeInfo->timeToFull;
    oldFormatGaugeInfo_ptr->chargingCurrent = gaugeInfo->chargingCurrent;
    oldFormatGaugeInfo_ptr->fC = gaugeInfo->fC;
    oldFormatGaugeInfo_ptr->VCT = gaugeInfo->VCT;
    oldFormatGaugeInfo_ptr->xchg = gaugeInfo->xchg;
    oldFormatGaugeInfo_ptr->xdischg = gaugeInfo->xdischg;
    oldFormatGaugeInfo_ptr->safetyStatus = gaugeInfo->safetyStatus;
    oldFormatGaugeInfo_ptr->operation_id2 = operation_id;
    oldFormatGaugeInfo_ptr->pfStatus = gaugeInfo->pfStatus;
    oldFormatGaugeInfo_ptr->operationStatus = gaugeInfo->operationStatus;
    oldFormatGaugeInfo_ptr->gaugingStatus = gaugeInfo->gaugingStatus;
    oldFormatGaugeInfo_ptr->mfgStatus = gaugeInfo->mfgStatus;
    oldFormatGaugeInfo_ptr->batteryPercentage = gaugeInfo->batteryPercentage;
    oldFormatGaugeInfo_ptr->cycleCount = gaugeInfo->cycleCount;
}

static void centralManager_systemRequest(SysRequestPacket_t *rp) {
    ResponsePacket_t packet = {
            .packetType = RESPONSEPACKET_SYSTEM,
            .statusCode = NoError,
            .packet.sysResponse.operationID = rp->operationID,
    };

    switch (rp->operationID) {
        case OPID_BATT_REQ:
            packet.packet.sysResponse.operationID = OPID_BATT_REQ;
            PM_getBatteryStatus((uint8_t *) &packet.packet.sysResponse.U.batReq.batteryPercentage,
                                &packet.packet.sysResponse.U.batReq.chargerState);
            break;
        case OPID_VERSION_REQ:
            packet.packet.sysResponse.operationID = OPID_VERSION_REQ;
            FU_getCurrentAppVersion(&packet.packet.sysResponse.U.version);
            break;
        case OPID_SID_REQ:
            packet.packet.sysResponse.operationID = OPID_SID_REQ;
            NS_getSensorID(&packet.packet.sysResponse.U.sensorID);
            break;
        case OPID_POWER_SELFTEST_REQ:
            ESP_LOGI(TAG, "Operation Power Self Test with id %d", rp->operationID);
            packet.packet.sysResponse.operationID = OPID_POWER_SELFTEST_REQ;
            uint32_t gauge, charger;
            PM_selfTest(&charger, &gauge);
            //to avoid unaligned access
            packet.packet.sysResponse.U.selfTestStatus.charger_status = charger;
            packet.packet.sysResponse.U.selfTestStatus.gauge_status = gauge;
            break;
        case OPID_GAUGE_INFO_REQ:
            packet.packet.sysResponse.operationID = OPID_GAUGE_INFO_REQ;
            GaugeInfo_t *gaugeInfo;
            PM_getGaugeInfo(&gaugeInfo);
            centralManager_convertGaugeInfoToOldFormat(gaugeInfo,&oldFormatGaugeInfo,OPID_GAUGE_INFO_REQ);
            packet.packet.sysResponse.U.oldGaugeInfoPtr = &oldFormatGaugeInfo;
            ESP_LOGI(TAG,"OperationID is %d",oldFormatGaugeInfo.operation_id1);
            ESP_LOGI(TAG,"OperationID is %d",packet.packet.sysResponse.U.oldGaugeInfoPtr->operation_id1);
            break;
        case OPID_READ_TEMPERATURE_REQ:
            packet.packet.sysResponse.operationID = OPID_READ_TEMPERATURE_REQ;
            packet.packet.sysResponse.U.temperature = NS_run_temperature();
            break;
        case OPID_CHECK_BOARD:
            ESP_LOGI(TAG, "Operation Check Board with id %d", OPID_CHECK_BOARD);
            packet.packet.sysResponse.operationID = OPID_CHECK_BOARD;
            packet.statusCode = NoError;
            break;
        case OPID_GET_LIFE_TIME_DATA:
            ESP_LOGI(TAG,"Getting Life Time data with id %d",rp->operationID);
            packet.packet.sysResponse.operationID = OPID_GET_LIFE_TIME_DATA;
            packet.packet.sysResponse.U.life_time_data_packet.life_time_data_length = sizeof(life_time_data_t);
            packet.packet.sysResponse.U.life_time_data_packet.life_time_data_ptr = power_management_get_life_time_data();
            break;
        case OPID_RESET_ECU:
            ESP_LOGI(TAG, "Resetting ECU");
            //GOOD BYE CRUEL WORLD, HTW74ONY WLAHY
            esp_restart();  //This will be the last line to be executed before ecu reset, this function doesn't return
            break;
        case OPID_ENTER_DEBUG_MODE:
            ESP_LOGI(TAG, "Entering Debug Mode");
            uartManager_disableTimerForDebug();
            packet.packet.sysResponse.operationID = OPID_ENTER_DEBUG_MODE;
            break;
        default:
            return;
    }

    xQueueSend(responseQueue, (void *) &packet, portMAX_DELAY);
}

_Noreturn void centralManager_requestsTask() {
    static RequestPacket_t requestPacket; //Should be static to pass it as a task parameter

    while (true) {
        xQueueReceive(requestQueue, &requestPacket, portMAX_DELAY);
//        ESP_LOGI(TAG,"RequestsTaskPriority = %d",uxTaskPriorityGet(NULL));  //1
        //Reset the activity timer, so we won't shut down the device
        if(requestPacket.packetType != REQUESTPACKET_MAINTENANCE){
            xTimerReset(inactiveTimer, portMAX_DELAY);
        }

        centralManager_createTask(OLED_requestTask, OLED_REQUEST_TASK_BIT,
                                  2048, (void *) &requestPacket);

        switch (requestPacket.packetType) {
            case REQUESTPACKET_P3:
                centralManager_createTask(NS_requestTask, NS_REQEUST_TASK_BIT,
                                          2560, (void *) &requestPacket.packet.p3Packet);
                OLED_progress(NS_CalculateRequiredTime(&requestPacket.packet.p3Packet));
                break;
            case REQUESTPACKET_SYSTEM:
                centralManager_systemRequest(&requestPacket.packet.sysPacket);
                break;
            case REQUESTPACKET_MEMORY:
                centralManager_createTask(fileSystem_requestTask, FILESYSTEM_REQUEST_TASK_BIT,
                                          2048, (void *) &requestPacket.packet.memPacket);
                break;
            case REQUESTPACKET_OTA:
                centralManager_createTask(FU_requestTask, FIRMWARE_UPDATE_REQUEST_TASK_BIT,
                                          2560, (void *) &requestPacket.packet.otaPacket);
                break;
            case REQUESTPACKET_MAINTENANCE:
                xEventGroupWaitBits(availableTasks, ALL_REQUEST_BITS,pdTRUE,pdTRUE, portMAX_DELAY);   //wait for all request tasks to be done, everything is finished and stop anything new from being initiated
                /*
                 * Now this needs some explanation to what's going on here, this task has a priority of 1, the response task has a priority of 2
                 * That means that this task has less priority than the response task, which is actually now free to execute since all the requests are done
                 * and the responses are free to be done. This will continue to be valid until the response task is blocked for some reason (like waiting for the
                 * bluetooth packet to finish sending. So all I need is to wait for the response task to tell me it's done, and when it's done I am positively sure that there will be no more calls to the
                 * response task anymore
                 */
                xEventGroupWaitBits(availableTasks,RESPONSE_TASK_BIT,pdFALSE,pdTRUE,portMAX_DELAY); //Wait for the main response task to be done
                xEventGroupWaitBits(availableTasks,ALL_RESPONSE_BITS,pdFALSE,pdTRUE,portMAX_DELAY); //Wait for all response tasks to be done
                configASSERT(uxQueueMessagesWaiting(responseQueue) == 0);    //check that the response queue is empty, just a sanity check
                // actually the above assertion may fail if one of the response tasks issue another response from a response task
                centralManager_runMaintenanceTask(&requestPacket.packet.maintenancePacket);
                xEventGroupSetBits(availableTasks,ALL_REQUEST_BITS);    //Set the bits to allow for new requests
            default:
                break;
        }
    }
}
void centralManager_runMaintenanceTask(MaintenanceRequestPacket_t* request_packet){
    ESP_LOGI(TAG,"Maintenance task with operation %d",request_packet->operationID);
    switch (request_packet->operationID) {
        case OPID_MAINTENANCE_SAVE_GAUGE_INFO:{
            battery_log_data_t battery_log_data = request_packet->U.battery_log_request;
            fileSystem_save_gauge_debug_log(&battery_log_data);
        }
            break;
        default:
            break;
    }
}

void centralManager_prepareBatteryLogPacket(event_type_t event){
    RequestPacket_t requestPacket = {
            .packetType = REQUESTPACKET_MAINTENANCE,
            .packet.maintenancePacket.operationID = OPID_MAINTENANCE_SAVE_GAUGE_INFO,
            .packet.maintenancePacket.U.battery_log_request.event_type = event,
    };
    GaugeInfo_t * gaugeInfoPtr;
    PM_getGaugeInfo(&gaugeInfoPtr);
    requestPacket.packet.maintenancePacket.U.battery_log_request.gauge_info_ptr = gaugeInfoPtr;
    xQueueSend(requestQueue,&requestPacket,portMAX_DELAY);
}
void centralManager_periodicBatteryCheck(TimerHandle_t xTimer){
    centralManager_prepareBatteryLogPacket(EVENT_TYPE_PERIODIC_CHECK);
}

uint8_t centralManager_isScannerConnected() {
#ifdef SAVE_ON_CONNECTION
    return false;
#else
    return _bleConnected || _uartConnected;
#endif
}

_Noreturn void centralManager_responseTask(void *parameters) {
    static ResponsePacket_t respPacket;  //Should be static to pass it as a task parameter
    static TimerHandle_t temperature_check_timer = NULL;

    while (true) {
        xQueueReceive(responseQueue, &respPacket, portMAX_DELAY);
        xEventGroupClearBits(availableTasks,RESPONSE_TASK_BIT);
        ESP_LOGI(TAG, "Response came with packet type (%d)", respPacket.packetType);
        if (respPacket.packetType == RESPONSEPACKET_P3) {
            if (respPacket.statusCode == NoError) {
                xEventGroupWaitBits(availableTasks,FILESYSTEM_RESPONSE_TASK_BIT,pdTRUE,pdFALSE,portMAX_DELAY);
                fileSystem_save_scan_data(&respPacket.packet.p3Response);//prevent anybody else from using the fileSystem response task
                if (NULL != temperature_check_timer) {
                    xTimerDelete(temperature_check_timer, 0);
                    temperature_check_timer = NULL;
                }

            } else if (respPacket.statusCode == HIGH_TEMP_ERROR) {
                if (NULL == temperature_check_timer) {
                    temperature_check_timer = xTimerCreate("TempCheck", pdMS_TO_TICKS(60000), pdTRUE,
                                                           NULL, centralManager_temperature_check_timer_callback);
                    xTimerStart(temperature_check_timer, portMAX_DELAY);
                }
            }
        }
        if (_bleConnected)
            centralManager_createTask(bluetoothManager_responseTask, BTMANAGER_RESPONSE_TASK_BIT,
                                      2560, (void *) &respPacket);

        if (_uartConnected)
            centralManager_createTask(uartManager_responseTask, UARTMANAGER_RESPONSE_TASK_BIT,
                                      2560, (void *) &respPacket);


        if (respPacket.packetType == RESPONSEPACKET_MEMORY && respPacket.statusCode == NoError &&
            respPacket.packet.memResponse.operationID == OPID_SAVE_AMBIENT_TEMP_FILE) {
            //Set the temperature after saving it to a file
            NS_set_temperature(respPacket.packet.memResponse.U.temperature_reading);
        }
        if (respPacket.packetType == RESPONSEPACKET_SYSTEM &&
            respPacket.packet.sysResponse.operationID == OPID_BATT_REQ &&
            respPacket.statusCode == NoError) {
            _battery_data = respPacket.packet.sysResponse.U.batReq;
        }

        centralManager_createTask(OLED_responseTask, OLED_RESPONSE_TASK_BIT,
                                  2048, (void *) &respPacket);
        //check that there are no more responses to handle
        if(uxQueueMessagesWaiting(responseQueue) == 0){
            xEventGroupSetBits(availableTasks,RESPONSE_TASK_BIT);
        }
    }
}

void centralManager_createTask(TaskFunction_t func, EventBits_t bit, const uint32_t stackSize, void *parameters) {
    char taskName[12] = {0};
    snprintf(taskName, sizeof(taskName), "Task(0x%0X)", bit);

    xEventGroupWaitBits(availableTasks, bit,
                        pdTRUE,  //bit should be cleared before returning.
                        pdFALSE, portMAX_DELAY);

    xTaskCreate(func, taskName, stackSize, parameters, 2, NULL);

    xEventGroupWaitBits(taskReceivedData, bit,
                        pdTRUE,  //bit should be cleared before returning.
                        pdFALSE, portMAX_DELAY);
}

static void IRAM_ATTR centralManager_buttonsISR(void *arg) {
    gpio_num_t buttonID = (gpio_num_t) arg;
    xQueueSendFromISR(buttonINTRQueue, &buttonID, NULL);
}

_Noreturn void centralManager_buttonsTask(void *parameters) {
    while (1) {
        gpio_num_t buttonID;
        xQueueReceive(buttonINTRQueue, &buttonID, portMAX_DELAY);

        ButtonState_t state = centralManager_getButtonState(buttonID);
        _scannerSettings = *fileSystem_get_scanner_settings();

        if (buttonID == system_io_config.io_scan_button_pin && state == BUTTON_SHORT_PRESS) {
            RequestPacket_t requestPacket = {
                    .packetType = REQUESTPACKET_P3,
                    .packet.p3Packet = {
                            .operationID = OPID_RUN_ABSRB,
                            .U.measurement = {
                                    .apodizationSel = _scannerSettings.apodizationSel,
                                    .commonWaveNum = _scannerSettings.commonWaveNum,
                                    .mode = _scannerSettings.mode,
                                    .opticalGain = _scannerSettings.opticalGain,
                                    .scanTime = _scannerSettings.scanTime,
                                    .zeroPadding = _scannerSettings.zeroPadding,
                            },
                    },
            };
            xQueueSend(requestQueue, &requestPacket, portMAX_DELAY);
        } else if (buttonID == system_io_config.io_scan_button_pin && state == BUTTON_LONG_PRESS) {
            RequestPacket_t requestPacket = {
                    .packetType = REQUESTPACKET_P3,
                    .packet.p3Packet = {
                            .operationID = OPID_RUN_BKGND,
                            .U.measurement = {
                                    .apodizationSel = _scannerSettings.apodizationSel,
                                    .commonWaveNum = _scannerSettings.commonWaveNum,
                                    .mode = _scannerSettings.mode,
                                    .opticalGain = _scannerSettings.opticalGain,
                                    .scanTime = _scannerSettings.scanTime,
                                    .zeroPadding = _scannerSettings.zeroPadding,
                            },
                    },
            };
            xQueueSend(requestQueue, &requestPacket, portMAX_DELAY);
        } else if (buttonID == system_io_config.io_power_button_pin && state == BUTTON_SHORT_PRESS) {
            xTimerReset(bluetoothAdvTimer, 0);
            if(!_bleConnected){
                bluetoothManager_advertisement(true);
                OLED_bluetooth(BLUETOOTH_ADVERTISEMENT);
            }
        } else if (buttonID == system_io_config.io_power_button_pin && state == BUTTON_LONG_PRESS) {
            centralManager_sleep(0);
        }
    }
}

ButtonState_t centralManager_getButtonState(gpio_num_t buttonNum) {
    uint8_t oldLevel = 1, level = 1;
    TickType_t lowLevelTimeStamp = 0;

    for (uint8_t i = 0; i < 25; i++) {
        vTaskDelay(BUTTON_DEBOUNCING_THRESHOLD);
        level = gpio_get_level(buttonNum);
        if (level == 0 && level == oldLevel) //check for low level
        {
            lowLevelTimeStamp = xTaskGetTickCount() * portTICK_PERIOD_MS;

            while (1) {
                vTaskDelay(BUTTON_DEBOUNCING_THRESHOLD);
                level = gpio_get_level(buttonNum);
                if (level == 1 && level == oldLevel) //check for high level
                {
                    if ((xTaskGetTickCount() * portTICK_PERIOD_MS) - lowLevelTimeStamp <=
                        BUTTON_SHORT_PRESS_THRESHOLD) {
                        return BUTTON_SHORT_PRESS;
                    } else {
                        return BUTTON_LONG_PRESS;
                    }
                } else if (level == 0 && level == oldLevel) {
                    //It is still pressed for enough time, I don't need to wait until it's unpressed
                    if ((xTaskGetTickCount() * portTICK_PERIOD_MS) - lowLevelTimeStamp > BUTTON_SHORT_PRESS_THRESHOLD) {
                        return BUTTON_LONG_PRESS;
                    }
                }
                oldLevel = level;
            }
        }
        oldLevel = level;
    }
    return BUTTON_DEFAULT;
}

void
centralManager_esp_error_check_failed_print(const char *tag, const char *msg, esp_err_t rc, const char *file, int line,
                                            const char *function, const char *expression) {
    int current_pos = 0, written_bytes = 0;
    written_bytes = snprintf(&error_string[current_pos], ERROR_STRING_SIZE - current_pos, "%s failed: esp_err_t 0x%x",
                             msg, rc);
    if (written_bytes <= 0) {
        ESP_LOGE(tag, "snprintf Failed");
        return;
    }
    current_pos += written_bytes;
#ifdef CONFIG_ESP_ERR_TO_NAME_LOOKUP
    written_bytes = snprintf(&error_string[current_pos], ERROR_STRING_SIZE - current_pos, " (%s)", esp_err_to_name(rc));
    if (written_bytes <= 0) {
        ESP_LOGE(tag, "snprintf Failed");
        return;
    }
    current_pos += written_bytes;
#endif //CONFIG_ESP_ERR_TO_NAME_LOOKUP
    written_bytes = snprintf(&error_string[current_pos], ERROR_STRING_SIZE - current_pos, " at 0x%08x\n",
                             (intptr_t) __builtin_return_address(0) - 3);
    if (written_bytes <= 0) {
        ESP_LOGE(tag, "snprintf Failed");
        return;
    }
    current_pos += written_bytes;
    if (spi_flash_cache_enabled()) { // strings may be in flash cache
        written_bytes = snprintf(&error_string[current_pos], ERROR_STRING_SIZE - current_pos,
                                 "file: \"%s\" line %d\nfunc: %s\nexpression: %s", file, line, function, expression);
        if (written_bytes <= 0) {
            ESP_LOGE(tag, "snprintf Failed");
            return;
        }
        current_pos += written_bytes;
    }
    ESP_LOGE(tag, "%s", error_string);
}

static void centralManager_lights_off_timer_callback(TimerHandle_t x_timer) {
    OLED_lightsOff();
    xTimerDelete(x_timer, 0);
}

void centralManager_oled_notify_lightsOff(uint32_t lights_off_time) {
    static TimerHandle_t lights_off_timer;
    lights_off_timer = xTimerCreate("LightsOff",
                                    pdMS_TO_TICKS(lights_off_time),
                                    false, NULL, centralManager_lights_off_timer_callback);
    xTimerStart(lights_off_timer, portMAX_DELAY);
    ESP_LOGI(TAG, "Started Lights off timer");
}

ScannerError_t centralManager_check_battery_level(uint8_t requires_scan) {
    //NO charger plugged
    if (_battery_data.chargerState == 0) {
        //Shut down if battery less than 1%
        if (_battery_data.batteryPercentage <= BATTERY_THRESHOLD_SHUT_DOWN) {
            //I should sleep now, but first I need to send a warning to the OLED
            //It is sent before forcing the OLED to sleep
            //never return from here
            centralManager_sleep(1);
        }
        //If this is a scan, issue an error at 3%, other wise issue and error at 20%
        uint8_t threshold = requires_scan ? BATTERY_THRESHOLD_PREVENT_SCAN : BATTERY_THRESHOLD_FINE;
        if (_battery_data.batteryPercentage <= threshold) {
            ESP_LOGE(TAG, "Battery Low, Charger Unplugged, ChargesPercent = %d\n",
                     _battery_data.batteryPercentage);
            return LOW_BATTERY_SCAN_ERROR;
        }
    }
    //The battery is more than the battery threshold or the charger is plugged
    return NoError;

}

esp_err_t centralManager_set_inactive_timeout(uint32_t inactive_timeout) {
    //Time out is in minutes
    xTimerReset(inactiveTimer, portMAX_DELAY);
    return xTimerChangePeriod(inactiveTimer, pdMS_TO_TICKS(inactive_timeout * 60 * 1000), portMAX_DELAY);
}

void centralManager_set_scan_temperature_window(uint8_t temperature_window) {
    NS_temperature_window = temperature_window;
}
