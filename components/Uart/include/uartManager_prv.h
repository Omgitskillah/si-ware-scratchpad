#include "uartManager.h"
#include "centralManager.h"

#include "driver/uart.h"

#include "sdkconfig.h"

#define UART_RX_BUFF_SIZE   (130)       //We require 22 bytes but should be more than 128 bytes (we may remove the whole buffer)

#define P3_SERVICE_ID       0xCA9E  	//P3 Service
#define SYS_SERVICE_ID      0xB100	    //System Service
#define MEM_SERVICE_ID      0xC100	    //Memory Service
#define OTA_SERVICE_ID      0xD100  	//OTA Service
#define UART_STARTUP_SERVICE_ID 0xF1AE  //Uart Service, used only on the first connection

#define CONNECTION_ID       0xA153  	//Connection ID
#define DISCONNECTION_ID    0xA150  	//Disconnection ID

#define UART_TIMER_PERIOD   (10000 / portTICK_PERIOD_MS) //10 seconds

typedef struct {
    uint16_t serviceIdentifier;
    uint8_t data[20];
} DATA_TYPE_ATTRIBUTE UartRequestPacket_t;
typedef struct{
    uint16_t service_id;
    uint8_t data[20];
    uint8_t delimeter[2];
} DATA_TYPE_ATTRIBUTE uart_response_packet_t;

static QueueHandle_t _uartQueue = NULL;
static TimerHandle_t _uartTimer = NULL;
static CLConnectioncb_t _connectionCB = NULL;
static bool _connected = false;
static const char delimeter [] = "\r\n";

static void uartManager_eventTask(void *parameters);
static CLService_t uartManager_identifyService(uint16_t identifier);
static void uartManager_timerCB(TimerHandle_t pxTimer);
static void uartManager_sendData(void* serviceID, uint8_t* data, uint16_t length);