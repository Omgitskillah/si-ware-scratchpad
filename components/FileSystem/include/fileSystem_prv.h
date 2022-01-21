#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "esp_vfs.h"
#include "esp_system.h"
#include "esp_spiffs.h"
#include "esp_partition.h"



#include "centralManager.h"

#define TAG "fileSystem"  

#define BASE_PATH                   "/spiffs"
#define DEFAULT_FILE_NAME           BASE_PATH"/scanFile"
#define SCAN_NUM_FILE_NAME          BASE_PATH"/scanNum.bin"
#define BACKGROUND_PTR_FILE_NAME    BASE_PATH"/backgroundPTR.bin"
#define SCANNER_SETTINGS_FILE_NAME  BASE_PATH"/settings.bin"
#define SCANNER_ID_FILE_NAME        BASE_PATH"/scannerID.bin"
#define AMBIENT_TEMP_FILE_NAME      BASE_PATH"/ambientTemp.bin"
#define MAGIC_FILE_NAME             BASE_PATH"/magicFile.bin"
#define MOTHER_BOARD_FILE_NAME      BASE_PATH"/motherBoardVersion.bin"
#define GOLDEN_FILE_STATUS_FILE_NAME BASE_PATH"/goldenFileStatus.bin"
#define INACTIVE_TIMEOUT_FILE_NAME  BASE_PATH"/inactiveTimeout.bin"
#define TEMPERATURE_WINDOW_FILE_NAME  BASE_PATH"/temperatureWindow.bin"
#define BATTERY_LOG_FILE_NAME           BASE_PATH"/batteryLog.bin"
#define GAUGE_RESET_COUNT_FILE_NAME     BASE_PATH"/GaugeResets.bin"
#define CURRENT_FIRMWARE_MAGIC_ID   1
#define MAX_NUM_OF_FILES            999


#define OLD_FORMAT_REFERENCE_SCAN    0x0A
#define OLD_FORMAT_REFLECTANCE_SCAN  0x0B
#define OLD_FORMAT_RAW_SCAN          0x0D

#define MAXIMUM_BATTERY_LOG_ENTRIES 130

#define BATTERY_LOG_MEMORY  &sharedStaticMemory[5*1024]

typedef struct MagicMarker{
    uint8_t oldFirmwareId;
    uint8_t newFirmwareId;
}MagicMarker;

uint16_t _scanNum = 0;

static ScannerError_t clearMemory();

static ScannerError_t readFile(const char* fileName, void* data, int length);
static ScannerError_t writeFile(const char* fileName, void* data, int length);

static ScannerError_t readBackground(SpectralData_t* output);
static ScannerError_t writeBackground(SpectralData_t* input);

static ScannerError_t readScanFile(uint16_t fileNum, SpectralData_t* output);
static ScannerError_t readScanFileWithOldFormat(uint16_t fileNum,SpectralData_t * output);
static ScannerError_t writeScanFile(SpectralData_t* input);

static ScannerError_t readScanNum(uint16_t* scanNum);
static ScannerError_t writeScanNum(uint16_t scanNum);
static int openFileIfExist(FILE **file_to_open, const char *full_file_path);
static ScannerError_t readOldScanFile(FILE *scan_file, SpectralData_t *spectralData);
static ScannerError_t readOldConfigurationFile(ScannerSettings_t *scannerSettings, sourceSettings_t *sourceSettingsPtr);
static void convertFilesFromOldFirmware();
static ScannerError_t writeSpectralData(SpectralData_t* input);
static int getCommonWaveNumFromPSDLength(uint16_t psdLength);

static void listFiles();
static void format();