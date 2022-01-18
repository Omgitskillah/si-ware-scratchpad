#include <sys/cdefs.h>

#ifndef CENTRALMANAGER_H
#define CENTRALMANAGER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "centralManagerDefs.h"

#define FW_VERSION    2201010208
#define BATTERY_FULL_CAPACITY 3350
#define MOTHER_BOARD_VERSION_G    7

#define SHARED_MEMORY_SIZE     (64 * 1024 + 100)
#define ERROR_STRING_SIZE       1024
#define REQUEST_QUEUE_SIZE     5
#define RESPONSE_QUEUE_SIZE    1
#define SHARED_MEMORY_CHUNCK    512
#define MEMORY_ALLOCATION_QUEUE_SIZE  (SHARED_MEMORY_SIZE / SHARED_MEMORY_CHUNCK)
#define MEMORY_IDX_INVALID      0xFF

#define NS_REQEUST_TASK_BIT                 BIT0
#define FILESYSTEM_REQUEST_TASK_BIT         BIT1
#define OLED_REQUEST_TASK_BIT               BIT2
#define FIRMWARE_UPDATE_REQUEST_TASK_BIT    BIT3

#define BTMANAGER_RESPONSE_TASK_BIT         BIT4
#define FILESYSTEM_RESPONSE_TASK_BIT        BIT5
#define OLED_RESPONSE_TASK_BIT              BIT6
#define UARTMANAGER_RESPONSE_TASK_BIT       BIT7
#define ALL_TASK_BITS                       0xFF
#define ALL_REQUEST_BITS                    0x0F
#define ALL_RESPONSE_BITS                   0xF0
#define RESPONSE_TASK_BIT                   BIT8

#define BUTTON_SHORT_PRESS_THRESHOLD    (1500)                       //1.5 seconds
#define BUTTON_DEBOUNCING_THRESHOLD     (10 / portTICK_PERIOD_MS)    //10  milliseconds

#define BLUETOOTH_TIMER_PERIOD          (3* 60000 / portTICK_PERIOD_MS) //60  seconds
#define PERIODIC_CHECK_PERIOD           (5*60*1000)
#define INACTIVE_TIMER_DEFAULT_PERIOD          15    //15 minutes


#define BATTERY_THRESHOLD_SHUT_DOWN		1
#define BATTERY_THRESHOLD_PREVENT_SCAN	3
#define BATTERY_THRESHOLD_FINE          20


#define ERROR_CHECK(tag, x)    ({                                         \
        esp_err_t __err_rc = (x);                                                   \
        if (__err_rc != ESP_OK) {                                                   \
        centralManager_esp_error_check_failed_print(tag,"ESP_ERROR_CHECK_WITHOUT_ABORT",__err_rc, __FILE__, __LINE__,     \
                                    __ASSERT_FUNC, #x);                             \
        }                                                                           \
        __err_rc;                                                                   \
    })

extern uint8_t sharedStaticMemory[SHARED_MEMORY_SIZE];

extern QueueHandle_t requestQueue;
extern QueueHandle_t responseQueue;
extern QueueHandle_t memoryAllocationQueue;

extern EventGroupHandle_t availableTasks;
extern EventGroupHandle_t taskReceivedData;

void centralManager_createQueues();

void centralManager_centralTasks();

void centralManager_systemStart();

void centralManager_buttonsInit(uint32_t mother_board_version);

void centralManager_buttonsTask(void *parameters);

ButtonState_t centralManager_getButtonState(gpio_num_t buttonNum);

_Noreturn void centralManager_requestsTask();

void centralManager_responseTask(void *parameters);

uint8_t centralManager_isScannerConnected();

void centralManager_bluetoothConnectionCallback(bool status);

void centralManager_uartConnectionCallback(bool status);
void centralManager_convertGaugeInfoToOldFormat(const GaugeInfo_t *gaugeInfo, oldFormatGaugeInfo_t *oldFormatGaugeInfo_ptr,uint32_t operation_id);

void centralManager_createTask(TaskFunction_t func, EventBits_t bit, const uint32_t stackSize, void *parameters);

void
centralManager_esp_error_check_failed_print(const char *tag, const char *msg, esp_err_t rc, const char *file, int line,
                                            const char *function, const char *expression);

void centralManager_oled_notify_lightsOff(uint32_t lights_off_time);

void centralManager_io_isr_init();

ScannerError_t centralManager_check_battery_level(uint8_t requires_scan);

esp_err_t centralManager_set_inactive_timeout(uint32_t inactive_timeout);
void centralManager_set_scan_temperature_window(uint8_t temperature_window);

#endif