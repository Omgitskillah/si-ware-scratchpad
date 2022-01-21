#include <sys/cdefs.h>
#include <string.h>
#include <math.h>

#include "oledInterface.h"
#include "icons.h"
#include "centralManager.h"
#include "centralManagerSystemIOs.h"

#include "u8g2.h"
#include "u8g2_hal.h"

#define OLED_LOGO_POSITION_X			0   //new logo is 4
#define OLED_NEW_LOGO_POSITION_X		4   //new logo is 4
#define OLED_LOGO_POSITION_Y			25
#define OLED_HLINE_POSITION_X			0
#define OLED_HLINE_POSITION_Y			14

#define OLED_BLUETOOTH_POSITION_X		85
#define OLED_BLUETOOTH_POSITION_Y		9

#define OLED_BATTERY_POSITION_X			12
#define OLED_BATTERY_POSITION_Y			70
#define OLED_BAT_TEXT_POSITION_X		9
#define OLED_BAT_TEXT_POSITION_Y		121
#define OLED_BAT_TEXT_CLR_POSITION_X	8
#define OLED_BAT_TEXT_CLR_POSITION_Y	110
#define OLED_BAT_TEXT_CLR_SIZE_X		30
#define OLED_BAT_TEXT_CLR_SIZE_Y		15
#define OLED_CHARGING_POSITION_X		20
#define OLED_CHARGING_POSITION_Y		110
#define OLED_BATTERY_FONT				u8g2_font_BBSesque_te

#define OLED_WIFI_POSITION_X			81

#define OLED_SDCARD_POSITION_X			19
#define OLED_SDCARD_POSITION_Y			7
#define OLED_DISK_POSITION_X			19
#define OLED_DISK_POSITION_Y			10
#define OLED_SDTEXT_POSITION_X			6
#define OLED_SDTEXT_POSITION_Y			52
#define OLED_SDTEXT_CLR_POSITION_X		5
#define OLED_SDTEXT_CLR_POSITION_Y		41
#define OLED_SDTEXT_CLR_SIZE_X			51
#define OLED_SDTEXT_CLR_SIZE_Y			12
#define OLED_SDTEXT_FONT				u8g2_font_BBSesque_te

#define OLED_MSG_ICON_POSITION_X		81
#define OLED_MSG_ICON_POSITION_Y		60
#define OLED_MSGTEXT_CLR_POSITION_X		49
#define OLED_MSGTEXT_CLR_POSITION_Y		63
#define OLED_MSGTEXT_CLR_SIZE_X			78
#define OLED_MSGTEXT_CLR_SIZE_Y			64
#define OLED_MSGTEXT_CENTER_X			87
#define OLED_MSGTEXT_CENTER_Y			95
#define OLED_MSGTEXT_NEWLINE_OFFSET		16
#define OLED_MSGTEXT_CHARACTER_OFFSET	8
#define OLED_MSGTEXT_FONT				u8g2_font_Born2bSportyV2_tf

#define OLED_MSGTEXT_READY				"Ready!"
#define OLED_MSGTEXT_DONE				"Done!"
#define OLED_MSGTEXT_LOADING			"Loading..."
#define OLED_MSGTEXT_PWROFF				"Power\nOff..."
#define OLED_MSGTEXT_EMERGENCY_PWROFF	"Battery Low\nPower Off..."
#define OLED_MSGTEXT_LOW_BATTERY	    "Battery Low\nPlug Charger"
#define OLED_MSGTEXT_REFSCAN			"Reference\nScan..."
#define OLED_MSGTEXT_MATSCAN			"Material\nScan..."
#define OLED_MSGTEXT_PSDSCAN			"PSD Material\nScan..."
#define OLED_MSGTEXT_GAINADJ			"Adjusting\nGain..."
#define OLED_MSGTEXT_SELFCORR			"WL\nCorrection..."
#define OLED_MSGTEXT_REFCORR_BG			"BG\nCorrection..."
#define OLED_MSGTEXT_REFCORR_S			"Ref Material\nCorrection..."
#define OLED_MSGTEXT_RSTR_DEFAULT		"Rstr Default\nCorrection..."
#define OLED_MSGTEXT_BURN_GAIN  		"Burn GAIN\nSettings..."
#define OLED_MSGTEXT_BURN_SELF          "Burn WVN\nCorrection..."
#define OLED_MSGTEXT_BURN_WLN           "Burn Ref\nCorrection..."
#define OLED_MSGTEXT_SCAN_COMPLETE		"Scan\nComplete"
#define OLED_MSGTEXT_UPDATEFW			"Update\nFirmware"
#define OLED_MSGTEXT_SCANREQ            "Retrieve\nScan"
#define OLED_MSGTEXT_MEMCLEAR           "Clear\nMemory"
#define OLED_MSGTEXT_SAVESETT           "Save\nSettings"
#define OLED_MSGTEXT_RESTORESETT         "Restore\nSettings"

#define OLED_PROGRESS_NUM_STEPS				16
#define OLED_PROGRESS_INTERVAL				500
#define OLED_PROGRESS_LOOP				5

#define OLED_FAST_PROGRESS_STEP			20
#define OLED_FAST_PROGRESS_LOOP			4

#define OLED_MAX_WIDTH					127

#define OLED_SPLASH_SCREEN_DURATION		(3000)
#define OLED_REFRESH_DELAY				(100 / portTICK_RATE_MS)
#define OLED_READY_PERIOD				(5000 / portTICK_RATE_MS)

#define OLED_SDCARD_MAXSIZE				"999"

typedef enum
{
    MESSAGE_INFO,
    MESSAGE_WARNING,
    MESSAGE_ERROR,
    MESSAGE_DEFAULT,
} OLED_messageType_t;

typedef enum
{
    CHARGING_DEFUALT = 0,
    CHARGING_SLOW = 1,
    CHARGING_FAST = 2,
} OLED_chargingType_t;


typedef struct {
#define OLED_PROGRESS_START 0
#define OLED_PROGRESS_STEP 1
#define OLED_PROGRESS_STOP 2
    uint8_t message_type;
    uint32_t time_in_millis : 24;
}OLED_progress_message_t;

static u8g2_t u8g2; //a structure which will contain all the data for the display

static SemaphoreHandle_t xMutex = NULL;
static QueueHandle_t progress_queue = NULL;
static TimerHandle_t readyTimer = NULL;

uint16_t _initialMemorySize;
uint8_t  _initialBatteryPercentage;
uint32_t _chargerStatus;

bool _oledStarted = false;

static void OLED_sleep();
static void OLED_splashScreen();
static void OLED_clearScreen();

static void OLED_batteryStatus(uint8_t chargesPercent, OLED_chargingType_t chargingType);

static void OLED_message(const char* str, OLED_messageType_t type);
static void OLED_printMessage(const char* str);

static void OLED_storageSize(int noOfWrittenFiles);



static void OLED_printP3Message(P3OperationID_t id);

static void OLED_readyTimerCB(TimerHandle_t pxTimer);

_Noreturn static void OLED_startingTask(void* parameters);

_Noreturn static void OLED_splashScreenTask(void * parameters);
static void progress_timer_callback(TimerHandle_t xTimer);
static void OLED_progressTask(void* parameters);