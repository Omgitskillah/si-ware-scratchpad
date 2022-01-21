#include <sys/cdefs.h>
#include "driver/rtc_io.h"
#include "driver/i2c.h"
#include "esp_sleep.h"

#include "centralManagerSystemIOs.h"

#include "charger_BQ25703A.h"
#include "gauge_BQ4050.h"
#include "powerManagement.h"

#define TAG "powerManagement"

static SemaphoreHandle_t power_management_charger_smphr = NULL;
static ChargerState_t power_management_charger_state = ChargerUnplugged;
static GaugeInfo_t power_management_gauge_info = {0};
static oldFormatGaugeInfo_t power_management_old_format_gauge_info = {0};
static life_time_data_t power_management_life_time_data;
//static RTC_DATA_ATTR uint8_t sleep_while_charging = false;


static void IRAM_ATTR PM_chargeOK_ISR(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(power_management_charger_smphr, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR();
}

static void PM_chargeOKInit() {
    gpio_config_t conf = {
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_ANYEDGE,
            .pin_bit_mask = (1ULL << IO_CHRG_OK),
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&conf);

    // gpio_install_isr_service(0); //should be installed before
    gpio_isr_handler_add(IO_CHRG_OK, PM_chargeOK_ISR, (void *) IO_CHRG_OK);
    esp_sleep_enable_ext1_wakeup((1ULL << IO_CHRG_OK), ESP_EXT1_WAKEUP_ANY_HIGH);
}
static void PM_presPinInit(){
    static bool initialized = false;
    if(initialized){
        return;
    }
    if(centralManager_get_mother_board_version() >= MOTHER_BOARD_VERSION_H){
        gpio_config_t conf = {
                .mode = GPIO_MODE_INPUT,
                .intr_type = GPIO_INTR_DISABLE,
                .pin_bit_mask = (1ULL << IO_PM_GAUGE_PRES),
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_ENABLE,   //already pulled down externally
        };
        gpio_config(&conf);
        initialized = true;
    }
}

static void PM_i2cInit() {
    i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .scl_io_num = IO_PM_I2C_SCL,
            .sda_io_num = IO_PM_I2C_SDA,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .master = {
                    .clk_speed = PM_I2C_FREQ_HZ,
            },
            .clk_flags = 0,
    };

    ERROR_CHECK(TAG, i2c_param_config(PM_I2C_NUM, &conf));
    // vTaskDelay(pdMS_TO_TICKS(500));
    ERROR_CHECK(TAG, i2c_driver_install(PM_I2C_NUM, conf.mode, 0, 0, 0));
}
static void power_management_toggle_pres_pin(){
    if(centralManager_get_mother_board_version() >= MOTHER_BOARD_VERSION_H){
        PM_presPinInit();
        gpio_set_level(IO_PM_GAUGE_PRES,1);
        vTaskDelay(3);
        gpio_set_level(IO_PM_GAUGE_PRES,0);
    }
}
static uint32_t power_management_safety_status_check(const GaugeInfo_t * gauge_info){
    typedef struct {
        safety_status_flag_t safety_status_flag;
        safety_status_error_type_t safety_status_error_bit;
        ScannerError_t safety_status_error_code;
        safety_status_action_t safety_status_action;
    }safety_status_operation_t;


    uint64_t safety_errors = 0;
    uint32_t safety_status = gauge_info->safetyStatus;
    ScannerError_t status;
    //static so it doesn't get stored in the function stack
    static safety_status_operation_t safety_status_operations [] = {
            {.safety_status_flag = SAFETY_STATUS_FLAG_CUV,.safety_status_error_bit = SAFETY_STATUS_CELL_UNDER_VOLTAGE,.safety_status_error_code = CELL_UNDER_VOLTAGE_ERROR, .safety_status_action = SAFETY_STATUS_ACTION_REPORT},
            {.safety_status_flag = SAFETY_STATUS_FLAG_COV,.safety_status_error_bit = SAFETY_STATUS_CELL_OVER_VOLTAGE,.safety_status_error_code =CELL_OVER_VOLTAGE_ERROR , .safety_status_action = SAFETY_STATUS_ACTION_REPORT},
            {.safety_status_flag = SAFETY_STATUS_FLAG_OCC1 | SAFETY_STATUS_FLAG_OCC2,.safety_status_error_bit = SAFETY_STATUS_OVER_CURRENT_IN_CHARGE,.safety_status_error_code = OVER_CURRENT_IN_CHARGE_ERROR, .safety_status_action = SAFETY_STATUS_ACTION_REPORT},
            {.safety_status_flag = SAFETY_STATUS_FLAG_OCD1 | SAFETY_STATUS_FLAG_OCD2,.safety_status_error_bit = SAFETY_STATUS_OVER_CURRENT_IN_DISCHARGE,.safety_status_error_code = OVER_CURRENT_IN_DISCHARGE_ERROR, .safety_status_action = SAFETY_STATUS_ACTION_REPORT},
            {.safety_status_flag = SAFETY_STATUS_FLAG_AOLD |SAFETY_STATUS_FLAG_AOLDL ,.safety_status_error_bit = SAFETY_STATUS_OVER_LOAD_IN_DISCHARGE,.safety_status_error_code = OVER_LOAD_IN_DISCHARGE_ERROR, .safety_status_action = SAFETY_STATUS_ACTION_TOGGLE_PRES},
            {.safety_status_flag = SAFETY_STATUS_FLAG_ASCC |SAFETY_STATUS_FLAG_ASCCL ,.safety_status_error_bit = SAFETY_STATUS_SHORT_CIRCUIT_IN_CHARGE,.safety_status_error_code = SHORT_CIRCUIT_IN_CHARGE_ERROR, .safety_status_action = SAFETY_STATUS_ACTION_TOGGLE_PRES},
            {.safety_status_flag = SAFETY_STATUS_FLAG_ASCD |SAFETY_STATUS_FLAG_ASCDL ,.safety_status_error_bit = SAFETY_STATUS_SHORT_CIRCUIT_IN_DISCHARGE,.safety_status_error_code =SHORT_CIRCUIT_IN_DISCHARGE_ERROR , .safety_status_action = SAFETY_STATUS_ACTION_REPORT},
            {.safety_status_flag = SAFETY_STATUS_FLAG_OTC,.safety_status_error_bit = SAFETY_STATUS_OVER_TEMPERATURE_IN_CHARGE,.safety_status_error_code =OVER_TEMPERATURE_IN_CHARGE_ERROR , .safety_status_action = SAFETY_STATUS_ACTION_REPORT},
            {.safety_status_flag = SAFETY_STATUS_FLAG_OTD,.safety_status_error_bit = SAFETY_STATUS_OVER_TEMPERATURE_IN_DISCHARGE,.safety_status_error_code = OVER_TEMPERATURE_IN_DISCHARGE_ERROR, .safety_status_action = SAFETY_STATUS_ACTION_REPORT},
            {.safety_status_flag = SAFETY_STATUS_FLAG_OTF,.safety_status_error_bit = SAFETY_STATUS_OVER_TEMPERATURE_IN_FET,.safety_status_error_code =OVER_TEMPERATURE_IN_FET_ERROR , .safety_status_action = SAFETY_STATUS_ACTION_REPORT},
            {.safety_status_flag = SAFETY_STATUS_FLAG_UTC,.safety_status_error_bit = SAFETY_STATUS_UNDER_TEMPERATURE_IN_CHARGE,.safety_status_error_code =UNDER_TEMPERATURE_IN_CHARGE_ERROR , .safety_status_action = SAFETY_STATUS_ACTION_REPORT},
            {.safety_status_flag = SAFETY_STATUS_FLAG_UTD,.safety_status_error_bit = SAFETY_STATUS_UNDER_TEMPERATURE_IN_DISCHARGE,.safety_status_error_code =UNDER_TEMPERATURE_IN_DISCHARGE_ERROR , .safety_status_action = SAFETY_STATUS_ACTION_REPORT},
            {.safety_status_flag = SAFETY_STATUS_FLAG_PTO,.safety_status_error_bit = SAFETY_STATUS_PRE_CHARGE_TIMEOUT,.safety_status_error_code = PRE_CHARGE_TIMEOUT_ERROR, .safety_status_action = SAFETY_STATUS_ACTION_TOGGLE_PRES},
            {.safety_status_flag = SAFETY_STATUS_FLAG_CTO,.safety_status_error_bit = SAFETY_STATUS_FAST_CHARGE_TIMEOUT,.safety_status_error_code = FAST_CHARGE_TIMEOUT_ERROR, .safety_status_action = SAFETY_STATUS_ACTION_TOGGLE_PRES},
            {.safety_status_flag = SAFETY_STATUS_FLAG_OC,.safety_status_error_bit = SAFETY_STATUS_OVER_CHARGE,.safety_status_error_code = OVER_CHARGE_ERROR, .safety_status_action = SAFETY_STATUS_ACTION_TOGGLE_PRES},
            {.safety_status_flag = SAFETY_STATUS_FLAG_CHGV,.safety_status_error_bit = SAFETY_STATUS_OVER_CHARGING_VOLTAGE,.safety_status_error_code = OVER_CHARGING_VOLTAGE_ERROR, .safety_status_action = SAFETY_STATUS_ACTION_REPORT},
            {.safety_status_flag = SAFETY_STATUS_FLAG_CHGC,.safety_status_error_bit = SAFETY_STATUS_OVER_CHARGING_CURRENT,.safety_status_error_code =OVER_CHARGING_CURRENT_ERROR , .safety_status_action = SAFETY_STATUS_ACTION_REPORT},
            {.safety_status_flag = SAFETY_STATUS_FLAG_PCHGC,.safety_status_error_bit = SAFETY_STATUS_OVER_PRE_CHARGING_CURRENT,.safety_status_error_code =OVER_PRE_CHARGING_CURRENT_ERROR , .safety_status_action = SAFETY_STATUS_ACTION_REPORT},

    };
    uint32_t num_operations = sizeof(safety_status_operations) / sizeof (safety_status_operation_t);
    for (uint32_t i = 0; i < num_operations; ++i) {
        if(safety_status & safety_status_operations[i].safety_status_flag){
            safety_errors |= safety_status_operations[i].safety_status_error_bit;
            status = safety_status_operations[i].safety_status_error_code;  //Should we report errors that the user can't fix?
            switch (safety_status_operations[i].safety_status_action) {

                case SAFETY_STATUS_ACTION_REPORT:
                    break;
                case SAFETY_STATUS_ACTION_TOGGLE_PRES:
                    power_management_toggle_pres_pin();
                    break;
            }
        }
    }
    //only alert handler
    if(gauge_info->safetyAlert & SAFETY_STATUS_FLAG_OC){
        safety_errors |= SAFETY_STATUS_OVER_CHARGE_ALERT;
        status = OVER_CHARGE_ALERT_ERROR;
    }
    if((gauge_info->operationStatus & 0x1) == 0){   //check PRES bit
        safety_errors |= SAFETY_STATUS_OPERATION_PRES_LOW;
        status = PRES_BIT_LOW_ERROR;
    }
    ResponsePacket_t packet = {
            .packetType = RESPONSEPACKET_SYSTEM,
            .statusCode = status,
            .packet.sysResponse.operationID = OPID_SAFETY_STATUS_REQ,
            .packet.sysResponse.U.safety_status_errors = safety_errors,
    };
    xQueueSend(responseQueue, (void *) &packet, portMAX_DELAY); //send the response packet
    return safety_errors;

}

_Noreturn static void PM_powertask(void *parameters) {
    ResponsePacket_t packet = {
            .packetType = RESPONSEPACKET_SYSTEM,
            .statusCode = NoError,
    };

    while (1) {
        bool CHRGOk_level = gpio_get_level(IO_CHRG_OK);
        ESP_LOGI(TAG,"Charger Level is %d",CHRGOk_level);
        if (power_management_charger_state == ChargerUnplugged && CHRGOk_level) {     //charger chip was disabled and charger is plugged (CHRGOk_level is high)
            power_management_charger_state = charger_start();
        } else if (power_management_charger_state != ChargerUnplugged && !CHRGOk_level) { //charger chip was enabled and charger is unplugged (CHRGOk_level is low)
            power_management_charger_state = charger_stop();
        }
        gauge_read_life_time_data(&power_management_life_time_data);
        gauge_readBatteryInfo(&power_management_gauge_info);
        gauge_printBatteryInfo(&power_management_gauge_info);
//        gauge_print_life_time_data(&power_management_life_time_data);

        charger_readCurrent();

        /*********** SEND BATTERY NOTIFICATION ***********/
        packet.packet.sysResponse.operationID = OPID_BATT_REQ;
        packet.packet.sysResponse.U.batReq.batteryPercentage = power_management_gauge_info.batteryPercentage;
        packet.packet.sysResponse.U.batReq.chargerState = power_management_charger_state;
        xQueueSend(responseQueue, (void *) &packet, portMAX_DELAY);

        /*********** SEND Gauge Info NOTIFICATION ***********/
        packet.packet.sysResponse.operationID = OPID_GAUGE_INFO_REQ;
        centralManager_convertGaugeInfoToOldFormat(&power_management_gauge_info,
                                                   &power_management_old_format_gauge_info, OPID_GAUGE_INFO_REQ);
        packet.packet.sysResponse.U.oldGaugeInfoPtr = &power_management_old_format_gauge_info;
        xQueueSend(responseQueue, (void *) &packet, portMAX_DELAY);
        power_management_safety_status_check(&power_management_gauge_info);
        xSemaphoreTake(power_management_charger_smphr, PM_TASK_PERIOD);
    }
}

void PM_wakeup_reason_check() {
    //Waked up by the charger interrupt, and I slept while charging

    esp_sleep_enable_ext1_wakeup((1ULL << IO_CHRG_OK), ESP_EXT1_WAKEUP_ANY_HIGH);
    esp_sleep_enable_ext0_wakeup(IO_POWER_BUTTON, 0);
    rtc_gpio_pullup_en(IO_POWER_BUTTON);
    rtc_gpio_pullup_en(IO_CHRG_OK);

    bool CHRGOk_level = gpio_get_level(IO_CHRG_OK);
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1 && CHRGOk_level == 0) {
        //reset the flag
        //sleep_while_charging = false;
        //enable interrupt to wakeup while charger is plugged in or if power button is pressed
        
        //start sleeping
        esp_deep_sleep_start();
    }
}

void PM_init(bool initializeGauge, bool writeGoldenFile) {
    PM_i2cInit();
    if (writeGoldenFile) {
        gauge_writeGoldenFile(sharedStaticMemory);
        gauge_deviceReset();
    }

    if (initializeGauge) {
        PM_init_reset_gauge();
    }

    power_management_charger_smphr = xSemaphoreCreateBinary();
    if (power_management_charger_smphr == NULL) {
        ESP_LOGE(TAG, "Error while creating the charger semaphore");
        return;
    } else {
        ESP_LOGI(TAG, "Created Charger semaphore");
    }
    PM_chargeOKInit();


    BaseType_t ret = xTaskCreate(PM_powertask, "PM_powertask", 2248, NULL, 2, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Error while creating the centralManager_responseTask");
        return;
    }

}
void  PM_init_reset_gauge(){
    gauge_init();
    gauge_deviceReset();
}

void PM_sleep(uint8_t emergencyShutdown) {
    bool CHRGOk_level = gpio_get_level(IO_CHRG_OK);
    if (emergencyShutdown == 0 && CHRGOk_level) {
        //Wake up when charger is low
        esp_sleep_enable_ext1_wakeup((1ULL << IO_CHRG_OK), ESP_EXT1_WAKEUP_ALL_LOW);
        //This flag is used when waking up, but here is an explanation for the behaviour I need
        /**
         * If the scanner is turned off while charging, it should stay off till the button is pressed.
         * However, if the charger is removed the scanner shouldn't turn on. But if the charger is replugged,
         * the charger should in this case turn on. And since I can't wakeup on an edge I need to actually let the
         * esp wake up. configure the wakeup interrupt and then resleep again. That's what this flag is for
         */
        //sleep_while_charging = true;
    } else {
        //sleep_while_charging = false;
        esp_sleep_enable_ext1_wakeup((1ULL << IO_CHRG_OK), ESP_EXT1_WAKEUP_ANY_HIGH);
    }
//    gauge_disable();
//    gauge_deviceReset();

    ESP_ERROR_CHECK(gpio_set_level(IO_OLED_DCDCEN, 0));
    ESP_ERROR_CHECK(gpio_set_level(IO_NS_ENABLE, 0));
}

void PM_getBatteryStatus(uint8_t *batteryPercentage, uint32_t *chargerState) {
    *batteryPercentage = power_management_gauge_info.batteryPercentage;
    *chargerState = power_management_charger_state;
}

void PM_getGaugeInfo(GaugeInfo_t **infoPTR) {
    *infoPTR = &power_management_gauge_info;
}

life_time_data_t *power_management_get_life_time_data() {
    return &power_management_life_time_data;
}

void PM_selfTest(uint32_t *charger_status, uint32_t *gauge_status) {
    *gauge_status = (uint8_t) gauge_selfTest();
    *charger_status = (uint8_t) charger_selfTest();
}