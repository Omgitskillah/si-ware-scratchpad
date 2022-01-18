#include <sys/cdefs.h>
#include <oledInterface.h>

#include "oledInterface_prv.h"

#define TAG        "OLED"
static OLED_bluetoothMode_t _bluetooth_start_mode = BLUETOOTH_ADVERTISEMENT;
static TimerHandle_t progress_timer;

static void OLED_delay(TickType_t millis)
{
    xSemaphoreTake(xMutex, (TickType_t) portMAX_DELAY);
    vTaskDelay(millis / portTICK_RATE_MS);
    xSemaphoreGive(xMutex);
}

static void OLED_splashScreen()
{
    //Splash Screen with Logo
    xSemaphoreTake(xMutex, (TickType_t) portMAX_DELAY);

    u8g2_ClearBuffer(&u8g2);
    u8g2_DrawXBM(&u8g2, OLED_LOGO_POSITION_X, OLED_LOGO_POSITION_Y,
                 ICON_OLD_LOGO_WIDTH, ICON_OLD_LOGO_LENGTH, icon_old_logo);
//    u8g2_DrawXBM(&u8g2, OLED_NEW_LOGO_POSITION_X, OLED_LOGO_POSITION_Y,
//                 ICON_NEW_LOGO_WIDTH, ICON_NEW_LOGO_LENGTH, icon_new_logo);

    u8g2_SendBuffer(&u8g2);

    xSemaphoreGive(xMutex);
}

/* Clear the screen and draw skeleton*/
static void OLED_clearScreen()
{
    xSemaphoreTake(xMutex, (TickType_t) portMAX_DELAY);

    //Splash Screen with Logo
    u8g2_ClearBuffer(&u8g2);

    int hight = 0;

    u8g2_DrawHLine(&u8g2, 0, hight, 62);
    u8g2_DrawHLine(&u8g2, 0, 61, 48);
    u8g2_DrawHLine(&u8g2, 0, 64, 46);
    u8g2_DrawHLine(&u8g2, 0, 127, 46);
    u8g2_DrawHLine(&u8g2, 64, hight, 64);
    u8g2_DrawHLine(&u8g2, 64, 45, 64);
    u8g2_DrawHLine(&u8g2, 63, 48, 65);
    u8g2_DrawHLine(&u8g2, 48, 127, 80);

    u8g2_DrawVLine(&u8g2, 0, hight, 62 - hight);
    u8g2_DrawVLine(&u8g2, 61, hight, 48 - hight);
    u8g2_DrawVLine(&u8g2, 0, 64, 64);
    u8g2_DrawVLine(&u8g2, 45, 64, 64);
    u8g2_DrawVLine(&u8g2, 64, hight, 46 - hight);
    u8g2_DrawVLine(&u8g2, 127, hight, 46 - hight);
    u8g2_DrawVLine(&u8g2, 48, 63, 65);
    u8g2_DrawVLine(&u8g2, 127, 48, 80);

    u8g2_DrawLine(&u8g2, 47, 61, 61, 47);
    u8g2_DrawLine(&u8g2, 48, 63, 63, 48);

    u8g2_SendBuffer(&u8g2);

    xSemaphoreGive(xMutex);
}

static void OLED_batteryStatus(uint8_t chargesPercent, OLED_chargingType_t chargingType)
{
    char str[5];

    xSemaphoreTake(xMutex, (TickType_t) portMAX_DELAY);

    if (chargingType == CHARGING_DEFUALT)
    {
        if (chargesPercent <= 100 && chargesPercent >= 80)
            u8g2_DrawXBM(&u8g2, OLED_BATTERY_POSITION_X, OLED_BATTERY_POSITION_Y,
                         ICON_BATTERY_WIDTH, ICON_BATTERY_LENGTH, icon_battery100);
        else if (chargesPercent < 80 && chargesPercent >= 60)
            u8g2_DrawXBM(&u8g2, OLED_BATTERY_POSITION_X, OLED_BATTERY_POSITION_Y,
                         ICON_BATTERY_WIDTH, ICON_BATTERY_LENGTH, icon_battery80);
        else if (chargesPercent < 60 && chargesPercent >= 40)
            u8g2_DrawXBM(&u8g2, OLED_BATTERY_POSITION_X, OLED_BATTERY_POSITION_Y,
                         ICON_BATTERY_WIDTH, ICON_BATTERY_LENGTH, icon_battery60);
        else if (chargesPercent < 40 && chargesPercent >= 20)
            u8g2_DrawXBM(&u8g2, OLED_BATTERY_POSITION_X, OLED_BATTERY_POSITION_Y,
                         ICON_BATTERY_WIDTH, ICON_BATTERY_LENGTH, icon_battery40);
        else if (chargesPercent < 20 && chargesPercent >= 10)
            u8g2_DrawXBM(&u8g2, OLED_BATTERY_POSITION_X, OLED_BATTERY_POSITION_Y,
                         ICON_BATTERY_WIDTH, ICON_BATTERY_LENGTH, icon_battery20);
        else
            u8g2_DrawXBM(&u8g2, OLED_BATTERY_POSITION_X, OLED_BATTERY_POSITION_Y,
                         ICON_BATTERY_WIDTH, ICON_BATTERY_LENGTH, icon_battery0);
    } else
        u8g2_DrawXBM(&u8g2, OLED_BATTERY_POSITION_X, OLED_BATTERY_POSITION_Y,
                     ICON_BATTERY_WIDTH, ICON_BATTERY_LENGTH, icon_batteryCharging);

    u8g2_DrawXBM(&u8g2, OLED_BAT_TEXT_CLR_POSITION_X, OLED_BAT_TEXT_CLR_POSITION_Y,
                 OLED_BAT_TEXT_CLR_SIZE_X, OLED_BAT_TEXT_CLR_SIZE_Y, icon_earse);

    sprintf(str, "%3d%%", chargesPercent);
    u8g2_SetFont(&u8g2, OLED_BATTERY_FONT);
    u8g2_DrawStr(&u8g2, OLED_BAT_TEXT_POSITION_X,
                 OLED_BAT_TEXT_POSITION_Y, str);

    u8g2_SendBuffer(&u8g2);
    xSemaphoreGive(xMutex);

    if (chargingType == CHARGING_FAST && _chargerStatus != chargingType)
    {
        OLED_message("Fast\nCharging", MESSAGE_INFO);
        xTimerStart(readyTimer,portMAX_DELAY);
        OLED_delay(500);
    } else if (chargingType == CHARGING_SLOW && _chargerStatus != chargingType)
    {
        OLED_message("Charging", MESSAGE_INFO);
        xTimerStart(readyTimer,portMAX_DELAY);
        OLED_delay(500);
    }
    _chargerStatus = chargingType;
}

static void OLED_storageSize(int noOfWrittenFiles)
{
    char str[10];

    xSemaphoreTake(xMutex, (TickType_t) portMAX_DELAY);

    // u8g2_DrawXBM(&u8g2, OLED_SDCARD_POSITION_X, OLED_SDCARD_POSITION_Y,
    //              ICON_SDCARD_WIDTH, ICON_SDCARD_LENGTH, icon_sd_card);

    u8g2_DrawXBM(&u8g2, OLED_DISK_POSITION_X, OLED_DISK_POSITION_Y,
                  ICON_DISK_WIDTH, ICON_DISK_LENGTH, icon_disk);

    u8g2_DrawXBM(&u8g2, OLED_SDTEXT_CLR_POSITION_X, OLED_SDTEXT_CLR_POSITION_Y,
                 OLED_SDTEXT_CLR_SIZE_X, OLED_SDTEXT_CLR_SIZE_Y, icon_earse);

    sprintf(str, "%03d/" OLED_SDCARD_MAXSIZE, noOfWrittenFiles);
    u8g2_SetFont(&u8g2, OLED_SDTEXT_FONT);
    u8g2_DrawStr(&u8g2, OLED_SDTEXT_POSITION_X, OLED_SDTEXT_POSITION_Y, str);

    u8g2_SendBuffer(&u8g2);

    xSemaphoreGive(xMutex);
}

void OLED_bluetooth(OLED_bluetoothMode_t bluetoothMode)
{
    if (!_oledStarted)
    {
        _bluetooth_start_mode = bluetoothMode;
        return;
    }
    xSemaphoreTake(xMutex, (TickType_t) portMAX_DELAY);

    switch (bluetoothMode)
    {
        case BLUETOOTH_ADVERTISEMENT:
            u8g2_DrawXBM(&u8g2, OLED_BLUETOOTH_POSITION_X, OLED_BLUETOOTH_POSITION_Y,
                         ICON_BLUETOOTH_WIDTH, ICON_BLUETOOTH_LENGTH, icon_bluetooth);
            break;
        case BLUETOOTH_CONNECTED:
            u8g2_DrawXBM(&u8g2, OLED_BLUETOOTH_POSITION_X, OLED_BLUETOOTH_POSITION_Y,
                         ICON_BLUETOOTH_WIDTH, ICON_BLUETOOTH_LENGTH, icon_bluetooth_connected);
            break;
        case BLUETOOTH_OFF:
            u8g2_DrawXBM(&u8g2, OLED_BLUETOOTH_POSITION_X, OLED_BLUETOOTH_POSITION_Y,
                         ICON_BLUETOOTH_WIDTH, ICON_BLUETOOTH_LENGTH, icon_bluetooth_off);
            break;
        default:
            break;
    }

    u8g2_SendBuffer(&u8g2);
    xSemaphoreGive(xMutex);
}

void OLED_uart(OLED_uartStatus_t uartStatus)
{
    if (!_oledStarted)
        return;

    switch (uartStatus)
    {
        case UART_CONNECTED:
            OLED_message("USB\nconnected!", MESSAGE_INFO);
            break;
        case UART_DISCONNECTED:
            OLED_message("USB\ndetached!", MESSAGE_INFO);
            break;
        default:
            break;
    }
    xTimerReset(readyTimer, 0);
}

static void OLED_message(const char *str, OLED_messageType_t type)
{
    xSemaphoreTake(xMutex, (TickType_t) portMAX_DELAY);

    u8g2_DrawXBM(&u8g2, OLED_MSGTEXT_CLR_POSITION_X, OLED_MSGTEXT_CLR_POSITION_Y,
                 OLED_MSGTEXT_CLR_SIZE_X, OLED_MSGTEXT_CLR_SIZE_Y, icon_earse);        //Text clearing

    switch (type)
    {
        case MESSAGE_INFO:
            u8g2_DrawXBM(&u8g2, OLED_MSG_ICON_POSITION_X, OLED_MSG_ICON_POSITION_Y,
                         ICON_ERROR_WIDTH, ICON_ERROR_WIDTH, icon_info);                    //INFO Icon
            break;
        case MESSAGE_WARNING:
            u8g2_DrawXBM(&u8g2, OLED_MSG_ICON_POSITION_X, OLED_MSG_ICON_POSITION_Y,
                         ICON_ERROR_WIDTH, ICON_ERROR_WIDTH, icon_warning);                //Warning Icon
            break;
        case MESSAGE_ERROR:
            u8g2_DrawXBM(&u8g2, OLED_MSG_ICON_POSITION_X, OLED_MSG_ICON_POSITION_Y,
                         ICON_ERROR_WIDTH, ICON_ERROR_WIDTH, icon_error);                //Error Icon
            break;
        default:    //No icon
            break;
    }
    OLED_printMessage(str);

    u8g2_SendBuffer(&u8g2);
    xSemaphoreGive(xMutex);
}

static void OLED_printMessage(const char *str)
{
    char buffer[25];
    char *tok = NULL;
    int offset = 0;
    int lines = 0;

    memcpy(buffer, str, 25);
    u8g2_SetFont(&u8g2, OLED_MSGTEXT_FONT);

    tok = strtok(buffer, "\n");
    do
    {
        offset = u8g2_GetStrWidth(&u8g2, tok) / 2;
        u8g2_DrawStr(&u8g2, OLED_MSGTEXT_CENTER_X - offset,
                     OLED_MSGTEXT_CENTER_Y + lines * OLED_MSGTEXT_NEWLINE_OFFSET, tok);
        tok = strtok(NULL, "\n");
        lines++;
    } while (tok != NULL);
}

void OLED_progress(uint32_t millis)
{
    if (!millis)
    {
        return;
    }
    ESP_LOGI(TAG,"Progress Time is %d",millis);
    OLED_progress_message_t start_message = {
            .message_type = OLED_PROGRESS_START,
    };
    start_message.time_in_millis = millis;
    if (progress_queue != NULL)
        xQueueSend(progress_queue, &start_message, (TickType_t) portMAX_DELAY);
}

static void OLED_readyTimerCB(TimerHandle_t pxTimer)
{
    OLED_message(OLED_MSGTEXT_READY, MESSAGE_INFO);
}

_Noreturn static void OLED_startingTask(void *parameters)
{
    ESP_LOGI(TAG, "Start OLED_managment Task");
    OLED_clearScreen();
    OLED_batteryStatus(_initialBatteryPercentage, _chargerStatus);
    OLED_storageSize(_initialMemorySize);
    OLED_message(OLED_MSGTEXT_READY, MESSAGE_INFO);
    _oledStarted = true;
    OLED_bluetooth(_bluetooth_start_mode);

    // int progressMillis = 0, delayCycle = 0;
    // //	int i = 100, j = 0, a = 0;
    // while(1)
    // {
    // 	if(xQueueReceive(progress_queue, &progressMillis, (TickType_t)portMAX_DELAY))
    // 	{
    // 		xSemaphoreTake(xMutex, (TickType_t)portMAX_DELAY);

    // 		delayCycle = (int) ceil((progressMillis * 1.0) / (OLED_PROGRESS_LOOP * 1.0));

    // 		if(delayCycle == 0)
    // 		{
    // 			//Fast progress
    // 			for(int i = 0; i < OLED_FAST_PROGRESS_LOOP; i++)
    // 			{
    // 				u8g2_DrawHLine(&u8g2, 48, 120, (i + 1) * OLED_FAST_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 121, (i + 1) * OLED_FAST_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 122, (i + 1) * OLED_FAST_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 123, (i + 1) * OLED_FAST_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 124, (i + 1) * OLED_FAST_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 125, (i + 1) * OLED_FAST_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 126, (i + 1) * OLED_FAST_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 127, (i + 1) * OLED_FAST_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 128, (i + 1) * OLED_FAST_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 129, (i + 1) * OLED_FAST_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 130, (i + 1) * OLED_FAST_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 131, (i + 1) * OLED_FAST_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 132, (i + 1) * OLED_FAST_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 133, (i + 1) * OLED_FAST_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 134, (i + 1) * OLED_FAST_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 135, (i + 1) * OLED_FAST_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 136, (i + 1) * OLED_FAST_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 137, (i + 1) * OLED_FAST_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 138, (i + 1) * OLED_FAST_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 139, (i + 1) * OLED_FAST_PROGRESS_STEP);

    // 				u8g2_SendBuffer(&u8g2);
    // 				//vTaskDelay(delayCycle / portTICK_RATE_MS);
    // 			}
    // 		}
    // 		else
    // 		{
    // 			//Slow progress
    // 			for(int i = 0; i < OLED_PROGRESS_LOOP; i++)
    // 			{
    // 				u8g2_DrawHLine(&u8g2, 48, 120, (i + 1) * OLED_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 121, (i + 1) * OLED_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 122, (i + 1) * OLED_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 123, (i + 1) * OLED_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 124, (i + 1) * OLED_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 125, (i + 1) * OLED_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 126, (i + 1) * OLED_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 127, (i + 1) * OLED_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 128, (i + 1) * OLED_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 129, (i + 1) * OLED_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 130, (i + 1) * OLED_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 131, (i + 1) * OLED_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 132, (i + 1) * OLED_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 133, (i + 1) * OLED_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 134, (i + 1) * OLED_PROGRESS_STEP);
    // 				u8g2_DrawHLine(&u8g2, 48, 135, (i + 1) * OLED_PROGRESS_STEP);

    // 				u8g2_SendBuffer(&u8g2);
    // 				//vTaskDelay(delayCycle / portTICK_RATE_MS);
    // 			}
    // 		}

    // 		xSemaphoreGive(xMutex);
    // 	}
    // }

    ESP_LOGI(TAG, "Remaining stack %d bytes", uxTaskGetStackHighWaterMark(NULL));
    vTaskDelete(NULL);  //Delete itself
    while (1);           //This task should be deleted after finishing
}

static void OLED_printP3Message(P3OperationID_t id)
{
    switch (id)
    {
        case OPID_RUN_PSD:
            OLED_message(OLED_MSGTEXT_PSDSCAN, MESSAGE_INFO);
            break;
        case OPID_RUN_BKGND:
            OLED_message(OLED_MSGTEXT_REFSCAN, MESSAGE_INFO);
            break;
        case OPID_RUN_ABSRB:
            OLED_message(OLED_MSGTEXT_MATSCAN, MESSAGE_INFO);
            break;
        case OPID_RUN_GNADJ:
            OLED_message(OLED_MSGTEXT_GAINADJ, MESSAGE_INFO);
            break;
        case OPID_BRN_GAIN:
            OLED_message(OLED_MSGTEXT_BURN_GAIN, MESSAGE_INFO);
            break;
        case OPID_BRN_SLF:
            OLED_message(OLED_MSGTEXT_BURN_SELF, MESSAGE_INFO);
            break;
        case OPID_BRN_WLN:
            OLED_message(OLED_MSGTEXT_BURN_WLN, MESSAGE_INFO);
            break;
        case OPID_RUN_SLFCOR:
            OLED_message(OLED_MSGTEXT_SELFCORR, MESSAGE_INFO);
            break;
        case OPID_RUN_WVL_COR_BG:
            OLED_message(OLED_MSGTEXT_REFCORR_BG, MESSAGE_INFO);
            break;
        case OPID_RUN_WVL_COR:
            OLED_message(OLED_MSGTEXT_REFCORR_S, MESSAGE_INFO);
            break;
        case OPID_RSTR_DEFAULT:
            OLED_message(OLED_MSGTEXT_RSTR_DEFAULT, MESSAGE_INFO);
            break;
        default:
            break;
    }
}

static void OLED_sleep()
{
    xSemaphoreTake(xMutex, (TickType_t) portMAX_DELAY);

    u8g2_SetPowerSave(&u8g2, 1); // wake up display

    xSemaphoreGive(xMutex);
}
static void OLED_SetupIO(){
    gpio_config_t conf2 = {
            .mode = GPIO_MODE_OUTPUT,
            .intr_type = GPIO_INTR_DISABLE,
            .pin_bit_mask = ((1ULL << IO_OLED_DCDCEN)),
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&conf2);
    gpio_set_level(IO_OLED_DCDCEN, 1);
}
void OLED_setupFirstStage(){
    OLED_SetupIO();
    u8g2_esp32_hal_t u8g2_esp32_hal = {
            .sda = IO_OLED_SDA,
            .scl = IO_OLED_SCL,
    };
    u8g2_esp32_hal_init(u8g2_esp32_hal);

    u8g2_Setup_ssd1327_i2c_ws_128x128_f(&u8g2, U8G2_R0,
                                        u8g2_esp32_i2c_byte_cb, u8g2_esp32_gpio_and_delay_cb);

    u8x8_SetI2CAddress(&u8g2.u8x8, I2C_OLED_ADDRESS);

    u8g2_InitDisplay(&u8g2);     // send init sequence to the display, display is in sleep mode after this,
    u8g2_SetPowerSave(&u8g2, 0); // wake up display
    xTaskCreate(OLED_splashScreenTask, "OLED_splashScreenTask", 4096, NULL, 2, NULL);
}

void OLED_setupSecondStage(uint16_t initialMemorySize, uint8_t initialBatteryPercentage, uint32_t initialChargerStatus)
{
    _initialMemorySize = initialMemorySize;
    _initialBatteryPercentage = initialBatteryPercentage;
    _chargerStatus = initialChargerStatus;

    readyTimer = xTimerCreate("OLED_Ready_Timer", OLED_READY_PERIOD, pdFALSE, NULL, OLED_readyTimerCB);
    progress_timer = xTimerCreate("ProgressTimer", pdMS_TO_TICKS(OLED_PROGRESS_INTERVAL), pdTRUE, NULL,
                                  progress_timer_callback);
    xTaskCreate(OLED_startingTask, "OLED_startingTask", 2048, NULL, 2, NULL);
    xTaskCreate(OLED_progressTask, "OLED_progressTask", 2048, NULL, 1, NULL);
    //Max is 2 items, one item for stop from external source, other is progress from timer
    progress_queue = xQueueCreate(2, sizeof(OLED_progress_message_t));
}

void OLED_shutdown(uint8_t emergencyShutdown)
{
    if(!emergencyShutdown){
        OLED_message(OLED_MSGTEXT_PWROFF, MESSAGE_INFO);
    }else{
        OLED_message(OLED_MSGTEXT_EMERGENCY_PWROFF,MESSAGE_ERROR);

    }
    vTaskDelay(1000 / portTICK_RATE_MS);


    xSemaphoreTake(xMutex, (TickType_t) portMAX_DELAY);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SendBuffer(&u8g2);
    xSemaphoreGive(xMutex);

    OLED_sleep();
}

void OLED_requestTask(void *parameters)
{
    RequestPacket_t rp;

    memcpy(&rp, parameters, sizeof(rp));
    xEventGroupSetBits(taskReceivedData, OLED_REQUEST_TASK_BIT); //Set the bit after copying the parameters

    if (_oledStarted)
    {
        switch (rp.packetType)
        {
            case REQUESTPACKET_P3:
                OLED_printP3Message(rp.packet.p3Packet.operationID);
                break;
            case REQUESTPACKET_SYSTEM:
                break;
            case REQUESTPACKET_MEMORY:
                if (rp.packet.memPacket.operationID == OPID_SCAN_REQ ||
                    rp.packet.memPacket.operationID == OPID_SCAN_REQ_WITH_SETTINGS)
                {
                    char str[20];
                    snprintf(str, sizeof(str), OLED_MSGTEXT_SCANREQ" %d",
                             rp.packet.memPacket.U.scanNum);
                    OLED_message(str, MESSAGE_INFO);
                } else if (rp.packet.memPacket.operationID == OPID_MEM_CLEAR)
                    OLED_message(OLED_MSGTEXT_MEMCLEAR, MESSAGE_INFO);
                else if (rp.packet.memPacket.operationID == OPID_SAVE_SETTINGS)
                    OLED_message(OLED_MSGTEXT_SAVESETT, MESSAGE_INFO);
                else if (rp.packet.memPacket.operationID == OPID_RESTORE_DEFAULT_SETTINGS)
                    OLED_message(OLED_MSGTEXT_RESTORESETT, MESSAGE_INFO);
                break;
            case REQUESTPACKET_OTA:
                break;
        }
    }
    if(REQUESTPACKET_SYSTEM !=  rp.packetType){
        ESP_LOGI(TAG,"Stoppping Ready Timer");
        xTimerStop(readyTimer, 0);
    }
    ESP_LOGI(TAG, "Remaining stack %d bytes", uxTaskGetStackHighWaterMark(NULL));
    xEventGroupSetBits(availableTasks, OLED_REQUEST_TASK_BIT); //Set the bit before deleting the task
    vTaskDelete(NULL);  //Delete itself
    while (1);           //This task should be deleted after finishing
}

void OLED_responseTask(void *parameters)
{
    ResponsePacket_t resp;
    static OLED_progress_message_t stopMessage = {
            .message_type = OLED_PROGRESS_STOP,
    };
    static uint8_t low_battery = 0;

    memcpy(&resp, parameters, sizeof(P3Response_t));
    xEventGroupSetBits(taskReceivedData, OLED_RESPONSE_TASK_BIT); //Set the bit after copying the parameters

    if (_oledStarted)
    {
        if(RESPONSEPACKET_SYSTEM !=  resp.packetType){
            xQueueSend(progress_queue, &stopMessage, portMAX_DELAY);
        }
        if (resp.statusCode != NoError)
        {
            char errorMsg[15];
            sprintf(errorMsg, "Error\n%d", resp.statusCode);
            OLED_message(errorMsg, MESSAGE_ERROR);
        } else
        {
            switch (resp.packetType)
            {
                case RESPONSEPACKET_P3:
                    if (resp.packet.p3Response.operationID < OPID_SET_SRC_SETT)
                        OLED_message(OLED_MSGTEXT_SCAN_COMPLETE, MESSAGE_INFO);
                    break;
                case RESPONSEPACKET_SYSTEM:
                    if (resp.packet.sysResponse.operationID == OPID_BATT_REQ){
                        OLED_batteryStatus((uint8_t) resp.packet.sysResponse.U.batReq.batteryPercentage,
                                           (OLED_chargingType_t) resp.packet.sysResponse.U.batReq.chargerState);
                        if(NoError != centralManager_check_battery_level(0)){
                            OLED_message(OLED_MSGTEXT_LOW_BATTERY,MESSAGE_ERROR);
                            low_battery = 1;
                        }else{
                            if(1 == low_battery){
                                //The system is now ready
                                OLED_message(OLED_MSGTEXT_READY,MESSAGE_INFO);
                            }
                            low_battery = 0;

                        }
                    }
                    break;
                case RESPONSEPACKET_MEMORY:
                    if (resp.packet.memResponse.operationID != OPID_MEM_STATS)
                    {
                        OLED_message(OLED_MSGTEXT_DONE, MESSAGE_INFO);
                        if (resp.packet.memResponse.operationID == OPID_MEM_CLEAR)
                        {
                            OLED_storageSize(resp.packet.memResponse.U.stat.usedMemory);
                        }
                    } else
                        OLED_storageSize(resp.packet.memResponse.U.stat.usedMemory);
                    break;
                case RESPONSEPACKET_OTA:
                    break;
            }
        }
    }
    //These are the cases where the system is not ready
    /**
     * Case 1: The packet type is a system packet, most system packets don't need the ready timer
     * moreover they can interrupt other more important requests like a scan request, you don't need
     * ready in the middle of the scan request
     * Case 2 :is when the error in the scan is HIGH TEMP error
     * Low battery for scan is not considered a reason because the system is ready, it just can't scan
     * Low battery for system is handled above, and since the battery request is a system type
     * packet, the timer won't be reset either way
     */
    if(RESPONSEPACKET_SYSTEM !=  resp.packetType && resp.statusCode != HIGH_TEMP_ERROR){
        //Keep the timer stopped if we encounter a high temp error, only display ready when everything is truly ready
        ESP_LOGI(TAG,"ReEnabling Ready Timer");
        xTimerReset(readyTimer, 0);

    }
    ESP_LOGI(TAG, "Remaining stack %d bytes", uxTaskGetStackHighWaterMark(NULL));
    xEventGroupSetBits(availableTasks, OLED_RESPONSE_TASK_BIT); //Set the bit before deleting the task
    vTaskDelete(NULL);  //Delete itself
    while (1);           //This task should be deleted after finishing
}

void progress_timer_callback(TimerHandle_t xTimer)
{
    static const OLED_progress_message_t timer_progress_message = {
            .message_type = OLED_PROGRESS_STEP,
    };
    xQueueSend(progress_queue, &timer_progress_message, 0);
}

_Noreturn static void OLED_progressTask(void *parameters)
{
    int step_counter = 0;
    uint32_t numSteps = 0;
    double barIncrement = 0;
    uint8_t progressStarted = 0;
    while (true)
    {
        OLED_progress_message_t progress_message;
        if (xQueueReceive(progress_queue, &progress_message, portMAX_DELAY))
        {
            //We have received a proper message
            switch (progress_message.message_type)
            {
                case OLED_PROGRESS_START:
                    numSteps = ceil((double) progress_message.time_in_millis / OLED_PROGRESS_INTERVAL);
                    barIncrement = 80.0 / numSteps;
                    xTimerStart(progress_timer, portMAX_DELAY);
                    progressStarted = 1;
                    break;
                case OLED_PROGRESS_STEP:
                    if (!progressStarted)
                    {
                        break;
                    }
                    //This will draw a box starting from (48,120) start of the progress bar with height of 8
                    //so that it reaches (x,128) this x will increase by (128-48)/NUM_STEPS each steps
                    xSemaphoreTake(xMutex, portMAX_DELAY);
                    if (uxQueueMessagesWaiting(progress_queue) > 0)
                    {
                        //This is to guard against the condition that the timer gave a progress step, then the
                        //task gave a stop message, but the progress task took the progress and is now trying to draw
                        //but it can't because scan done is being written, so it writes over scan written, which
                        //is something that we don't want, so after taking the mutex we just make sure that no one wants anything
                        //The timer is timed slower than the drawing logic, so I am not afraid that the timer will send me messages
                        //here
                        xSemaphoreGive(xMutex);
                        break;
                    }
                    //This is to allow the box to be drawn every time with the fraction part removed, yet in the last time
                    // it will be complete and draw the full 80 pixels
                    u8g2_DrawBox(&u8g2, 48, 120, (uint32_t)((double)(++step_counter) * barIncrement), 8);
                    u8g2_SendBuffer(&u8g2);
                    xSemaphoreGive(xMutex);
                    if (step_counter < numSteps)
                    {
                        break;
                    }
                case OLED_PROGRESS_STOP:
                    xTimerStop(progress_timer, portMAX_DELAY);
                    //There maybe some messages in the queue due to the timer, just ignore them
                    xQueueReset(progress_queue);
                    step_counter = 0;
                    progressStarted = 0;
                    break;
            }
            ESP_LOGI(TAG, "Remaining stack from progress task %d bytes", uxTaskGetStackHighWaterMark(NULL));

        }
    }
}

_Noreturn void OLED_splashScreenTask(void *parameters) {
    ESP_LOGI(TAG, "Start OLED_splashScreen Task");
    xMutex = xSemaphoreCreateMutex();
    OLED_splashScreen();
    ESP_LOGI(TAG, "Remaining stack %d bytes", uxTaskGetStackHighWaterMark(NULL));
    ESP_LOGI(TAG, "End OLED_splashScreen Task");
    vTaskDelete(NULL);
    while (1);
}

void OLED_lightsOff(){
    OLED_message("Lights\nOff",MESSAGE_INFO);
}
