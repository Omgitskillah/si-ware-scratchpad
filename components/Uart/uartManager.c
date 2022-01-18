#include "uartManager_prv.h"
#include "esp_intr_alloc.h"

#define TAG     "uartManager"

static SemaphoreHandle_t s_uart_mutex = NULL;
static bool debug_mode_enabled = false;
#define UART_ONE_SECOND_TICKS  pdMS_TO_TICKS(1000)

static void uartManager_lock(const char * source)
{    
    if (!s_uart_mutex) {
        s_uart_mutex = xSemaphoreCreateMutex();
    }
    if(pdFALSE ==  xSemaphoreTake(s_uart_mutex, UART_ONE_SECOND_TICKS))
    {
        ESP_LOGE(TAG,"lock expired when trying to take from %s",source);
    }

}
static void uartManger_unlock(void)
{
    xSemaphoreGive(s_uart_mutex);
}
void uartManager_disableTimerForDebug(void)
{
    debug_mode_enabled = true;
}

_Noreturn static void uartManager_eventTask(void *parameters)
{
    uart_event_t event;
    UartRequestPacket_t uartReqPacket;
    while(1) 
    {
        //Waiting for UART event.
        xQueueReceive(_uartQueue, (void * )&event, (portTickType)portMAX_DELAY);

        switch(event.type) 
        {
            case UART_DATA:
                ESP_LOGI(TAG, "event.size = %d", event.size);
                if(event.size <= sizeof(uartReqPacket))
                {
                    uart_read_bytes(UART_NUM_0, (uint8_t*) &uartReqPacket, event.size, portMAX_DELAY);
                    //ESP_LOGI(TAG, "ID = %d", uartReqPacket.serviceIdentifier);
                    if(_connectionCB != NULL)
                    {
                        if(uartReqPacket.serviceIdentifier == CONNECTION_ID && !_connected)
                        {
                            uint8_t password_byte = 0xA5;
                            uartManager_sendData(UART_STARTUP_SERVICE_ID,&password_byte,1);
                            _connectionCB(true);
                            _connected = true;
                            xTimerReset(_uartTimer, 0);

                        }
                        else if(uartReqPacket.serviceIdentifier == CONNECTION_ID)
                            xTimerReset(_uartTimer, 0);
                        else if(uartReqPacket.serviceIdentifier == DISCONNECTION_ID && _connected)
                        {
                            _connectionCB(false);
                            _connected = false;
                        }
                    }

                    if(_connected)
                    {
                        CLService_t service = uartManager_identifyService(uartReqPacket.serviceIdentifier);
                        if(service == SERVICE_UNDEFINED)
                            break;
                        ESP_LOGI(TAG, "service = %d", service);
                        CL_rxHandler(service, uartReqPacket.data, sizeof(uartReqPacket.data));
                    }
                }
                break;
                
            case UART_FIFO_OVF:
                ESP_LOGW(TAG, "hw fifo overflow");
                uart_flush_input(UART_NUM_0);
                xQueueReset(_uartQueue);
                break;
                
            case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "ring buffer full");
                uart_flush_input(UART_NUM_0);
                xQueueReset(_uartQueue);
                break;
            default:
                ESP_LOGW(TAG, "uart event type: %d", event.type);
                break;
        }
    }
}

static CLService_t uartManager_identifyService(uint16_t identifier)
{
    switch(identifier)
    {
        case P3_SERVICE_ID:
            return SERVICE_P3;
        case SYS_SERVICE_ID:
            return SERVICE_SYSTEM;
        case MEM_SERVICE_ID:
            return SERVICE_MEMORY;
        case OTA_SERVICE_ID:
            return SERVICE_OTA;
        default:
            return SERVICE_UNDEFINED;
    }
}

static void uartManager_sendData(void* serviceID, uint8_t* data, uint16_t length)
{
    uart_response_packet_t response =  {
        .service_id = serviceID,
    };
    memcpy(response.data,data,length);
    memcpy(response.data + length,delimeter, 2* sizeof(uint8_t)); //if I just copy the delimeter to the delimeter
    //I will end up needing to send a larger message than I am needing, and than expected
    uartManager_lock("Send Data");
    //wait Tx done is actually faulty, as it sometimes hangs without reason
    // ESP_ERROR_CHECK_WITHOUT_ABORT(uart_wait_tx_done(UART_NUM_0,UART_ONE_SECOND_TICKS));
    uart_write_bytes(UART_NUM_0,&response,sizeof(uint16_t) + length + (2*sizeof(uint8_t)) );
    uartManger_unlock();
}

int uartManager_log(const char * format, va_list args)
{
    uartManager_lock("Send log");
    // ESP_ERROR_CHECK_WITHOUT_ABORT(uart_wait_tx_done(UART_NUM_0,UART_ONE_SECOND_TICKS));
    int result = vprintf(format,args);
    uartManger_unlock();
    return result;

}

static void uartManager_timerCB(TimerHandle_t pxTimer)
{
    if(_connected && !debug_mode_enabled)
    {
        _connectionCB(false);
        _connected = false;
    }
}

void uartManager_setup()
{
    /* Configure parameters of an UART driver */
    uart_config_t uart_config = {
        .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    //Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_0, UART_RX_BUFF_SIZE, 0, 10, &_uartQueue, ESP_INTR_FLAG_IRAM);
    uart_param_config(UART_NUM_0, &uart_config);

    //Create a task to handler UART event from ISR
    xTaskCreate(uartManager_eventTask, "uartManager_eventTask", 2048, NULL, 2, NULL);
    _uartTimer = xTimerCreate("uartTimer", UART_TIMER_PERIOD, pdFALSE, 0, uartManager_timerCB);
}

void uartManager_setConnectionCallback(CLConnectioncb_t callback)
{
    _connectionCB = callback;
}

void uartManager_responseTask(void* parameters)
{
    ResponsePacket_t respPacket;

    memcpy(&respPacket, parameters, sizeof(ResponsePacket_t));
    xEventGroupSetBits(taskReceivedData, UARTMANAGER_RESPONSE_TASK_BIT); //Set the bit after copying the parameters
    
    switch(respPacket.packetType)
    {
        case RESPONSEPACKET_P3:
            CL_p3ServiceTx(&respPacket, (CLSendHandler_t) uartManager_sendData, 
                (void*) P3_SERVICE_ID);
            break;

        case RESPONSEPACKET_SYSTEM:
            CL_systemServiceTx(&respPacket, (CLSendHandler_t) uartManager_sendData, 
                (void*) SYS_SERVICE_ID);
            break;

        case RESPONSEPACKET_MEMORY:
            CL_memoryServiceTx(&respPacket, (CLSendHandler_t) uartManager_sendData, 
                (void*) MEM_SERVICE_ID);   
            break;
        
        case RESPONSEPACKET_OTA:
            CL_OTAServiceTx(&respPacket, (CLSendHandler_t) uartManager_sendData, 
                (void*) OTA_SERVICE_ID);
            break;

        default:
            break;
    }

    ESP_LOGI(TAG, "Remaining stack %d bytes", uxTaskGetStackHighWaterMark(NULL));
    xEventGroupSetBits(availableTasks, UARTMANAGER_RESPONSE_TASK_BIT); //Set the bit before deleting the task
    vTaskDelete(NULL);  //Delete itself
    while(1);           //This task should be deleted after finishing
}