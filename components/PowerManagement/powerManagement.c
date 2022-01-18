#include <sys/cdefs.h>
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
static RTC_DATA_ATTR uint8_t sleep_while_charging = false;


static void IRAM_ATTR PM_chargeOK_ISR(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(power_management_charger_smphr, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR();
}

static void PM_chargeOKInit() {
    gpio_config_t conf = {
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_PIN_INTR_ANYEDGE,
            .pin_bit_mask = (1ULL << IO_CHRG_OK),
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&conf);

    // gpio_install_isr_service(0); //should be installed before
    gpio_isr_handler_add(IO_CHRG_OK, PM_chargeOK_ISR, (void *) IO_CHRG_OK);
    esp_sleep_enable_ext1_wakeup((1ULL << IO_CHRG_OK), ESP_EXT1_WAKEUP_ANY_HIGH);
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

_Noreturn static void PM_powertask(void *parameters) {
    ResponsePacket_t packet = {
            .packetType = RESPONSEPACKET_SYSTEM,
            .statusCode = NoError,
    };

    while (1) {
        bool CHRGOk_level = gpio_get_level(IO_CHRG_OK);

        if (power_management_charger_state == ChargerUnplugged && CHRGOk_level) {     //charger chip was disabled and charger is plugged (CHRGOk_level is high)
            power_management_charger_state = charger_start();
        } else if (power_management_charger_state != ChargerUnplugged && !CHRGOk_level) { //charger chip was enabled and charger is unplugged (CHRGOk_level is low)
            power_management_charger_state = charger_stop();
        }
        gauge_readBatteryInfo(&power_management_gauge_info);
        gauge_read_life_time_data(&power_management_life_time_data);
//        gauge_printBatteryInfo(&power_management_gauge_info);
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

        xSemaphoreTake(power_management_charger_smphr, PM_TASK_PERIOD);
    }
}

void PM_wakeup_reason_check() {
    //Waked up by the charger interrupt, and I slept while charging
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1 && sleep_while_charging) {
        //reset the flag
        sleep_while_charging = false;
        //enable interrupt to wakeup while charger is plugged in or if button is pressed
        esp_sleep_enable_ext1_wakeup((1ULL << IO_CHRG_OK), ESP_EXT1_WAKEUP_ANY_HIGH);
        ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(IO_POWER_BUTTON, 0));
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
        gauge_init();
        gauge_deviceReset();
    }

    power_management_charger_smphr = xSemaphoreCreateBinary();
    if (power_management_charger_smphr == NULL) {
        ESP_LOGE(TAG, "Error while creating the charger semaphore");
        return;
    } else {
        ESP_LOGI(TAG, "Created Charger semaphore");
    }

    BaseType_t ret = xTaskCreate(PM_powertask, "PM_powertask", 2248, NULL, 2, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Error while creating the centralManager_responseTask");
        return;
    }

    PM_chargeOKInit();
}

void PM_sleep() {
    bool CHRGOk_level = gpio_get_level(IO_CHRG_OK);
    if (CHRGOk_level) {
        //Wake up when charger is low
        esp_sleep_enable_ext1_wakeup((1ULL << IO_CHRG_OK), ESP_EXT1_WAKEUP_ALL_LOW);
        //This flag is used when waking up, but here is an explanation for the behaviour I need
        /**
         * If the scanner is turned off while charging, it should stay off till the button is pressed.
         * However, if the charger is removed the scanner shouldn't turn on. But if the charger is replugged,
         * the charger should in this case turn on. And since I can't wakeup on an edge I need to actually let the
         * esp wake up. configure the wakeup interrupt and then resleep again. That's what this flag is for
         */
        sleep_while_charging = true;
    } else {
        sleep_while_charging = false;
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