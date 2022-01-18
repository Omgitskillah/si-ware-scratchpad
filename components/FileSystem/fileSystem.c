#include <math.h>
#include "fileSystem_prv.h"
#include "fileSystem.h"
#include "sys/errno.h"
#include "NS_sensor.h"  //TODO : Replace the memory mappings in an additional component

static ScannerSettings_t _default_scanner_settings = {
        .apodizationSel = 0,    //Boxcar
        .commonWaveNum = 3,     //257-points
        .gainvalue = 0,         //ignored
        .mode = 0,              //single mode
        .opticalGain = 0,       //default
        .scanTime = 2000,       //2 seconds
        .zeroPadding = 3,       //32K padding
};
static ScannerSettings_t _current_scanner_settings;
static TemperatureReading _temperature_reading;
static const uint8_t magicIds[] = {0, CURRENT_FIRMWARE_MAGIC_ID};

const uint16_t fileSystem_gauge_log_entry_size = sizeof(GaugeInfo_t) + sizeof(event_type_t);
const uint16_t fileSystem_maximum_gauge_log_file_size = fileSystem_gauge_log_entry_size * MAXIMUM_BATTERY_LOG_ENTRIES;
static ScannerError_t readFile(const char *fileName, void *data, int length) {
    FILE *fp = fopen(fileName, "r");
    if (fp == NULL) {
        ESP_LOGE(TAG, "%s, OPEN_READ_FILE_ERR, Error : %s ,Errno : %d", fileName, strerror(errno), errno);
        return OPEN_READ_FILE_ERR;
    }

    fread(data, length, 1, fp);
    fclose(fp);
    return NoError;
}

static ScannerError_t writeFile(const char *fileName, void *data, int length) {
    FILE *fp = fopen(fileName, "w");
    if (fp == NULL) {
        ESP_LOGE(TAG, "%s, OPEN_WRITE_FILE_ERR, error is %s with error no %d", fileName, strerror(errno), errno);
        if (access(fileName, F_OK) == 0) {
            ESP_LOGE(TAG, "File Already Exists");
        } else {
            ESP_LOGE(TAG, "File Not Found");
        }
        return OPEN_WRITE_FILE_ERR;
    }

    fwrite(data, length, 1, fp);
    fclose(fp);
    return NoError;
}

static ScannerError_t readBackground(SpectralData_t *output) {
    uint16_t fileNum;

    ScannerError_t err =
            readFile(BACKGROUND_PTR_FILE_NAME, (void *) &fileNum, sizeof(uint16_t));
    if (err != NoError)
        return err;

    return readScanFile(fileNum, output);
}

static ScannerError_t writeBackground(SpectralData_t *input) {
    ScannerError_t err = writeScanFile(input);
    if (err != NoError)
        return err;

    return writeFile(BACKGROUND_PTR_FILE_NAME, &_scanNum, sizeof(_scanNum));
}

static ScannerError_t writeSpectralData(SpectralData_t *input) {
    ScannerError_t error = NoError;
    switch (input->type) {
        case SPECTRAL_TYPE_BCKGRND:
            if (NoError == (error = writeBackground(input))) {
                writeScanNum(++_scanNum);
            }
            break;
        case SPECTRAL_TYPE_PSD:
        case SPECTRAL_TYPE_SPECTRUM:
            if (NoError == (error = writeScanFile(input))) {
                writeScanNum(++_scanNum);
            }
    }
    return error;
}


static ScannerError_t readScanFile(uint16_t fileNum, SpectralData_t *output) {
    char fileName[50] = {0};
    snprintf(fileName, sizeof(fileName), DEFAULT_FILE_NAME "%d.bin", fileNum);

    ScannerError_t err = readFile(fileName, (void *) output, sizeof(SpectralData_t));
    if (err != NoError)
        return err;

    FILE *fp = fopen(fileName, "r"); //read spectral data
    if (fp == NULL) {
        ESP_LOGE(TAG, "%s, OPEN_READ_FILE_ERR", fileName);
        return OPEN_READ_FILE_ERR;
    }

    fseek(fp, sizeof(SpectralData_t), SEEK_SET); //Move seek to the start of spectralData
    switch (output->type) {
        case SPECTRAL_TYPE_BCKGRND:
            output->spectralPTR = (double *) NS_BACKGROUNDDATA;
            break;
        case SPECTRAL_TYPE_PSD:
        case SPECTRAL_TYPE_SPECTRUM:
            output->spectralPTR = (double *) NS_SPECTRALDATA;
            break;
        default:
            ESP_ERROR_CHECK(ESP_FAIL);
    }
    ESP_LOGI(TAG, "Spectral Ptr Address is %p, while shared memory access is %p", output->spectralPTR,
             NS_BACKGROUNDDATA);
    if (output->settings.commonWaveNum == 0)
        fread(output->spectralPTR, sizeof(double), output->psdLength * 2, fp);
    else
        fread(output->spectralPTR, sizeof(double), output->psdLength + 2, fp);
    fclose(fp);
//    ESP_LOGI(TAG,"[OUT]Quantized WVN data Start = %llu, Step = %llu",((uint64_t*)output->spectralPTR)[output->psdLength],((uint64_t*)output->spectralPTR)[output->psdLength+1]);

    return NoError;
}


ScannerError_t readScanFileWithOldFormat(uint16_t fileNum, SpectralData_t *output) {
    /*
     * Old format is
     * Type --> 1 byte
     * complete PSD in a quantized format
     * complete WVL in a quantized format
     */
    /**
     * To do this we actually have two options, since this will mostly be coming from the app
     * we can depend that at max there will be 257 points, which somewhat alleviates the memory constraints
     * However, to be in the safe side we can always put our data in the NS_Spectral_data ptr, so that even if we have
     * 4k points we can still generate the WVN no problem.
     * after putting the data we need to condition the output, either here or at the commLayer, as we need to put the type
     * and the operationID, actually it seems much more consistent to do it at the commLayer, but we need to set the type here
     * So we will adjust the type, populate the data properly into the spectralPtr field, and send it to the commLayer, so
     * the commLayer can properly format the response
     */
    ScannerError_t error = NoError;
    if (NoError != (error = readScanFile(fileNum, output))) {
        return error;
    }
    vTaskDelay(10);
    switch (output->type) {
        case SPECTRAL_TYPE_BCKGRND:
            output->type = OLD_FORMAT_REFERENCE_SCAN;
            break;
        case SPECTRAL_TYPE_SPECTRUM:
            output->type = OLD_FORMAT_REFLECTANCE_SCAN;
            break;
        case SPECTRAL_TYPE_PSD:
            output->type = OLD_FORMAT_RAW_SCAN;
            break;
    }
    /*
 * Now for the PSD part, we first need to make sure that all the data is at NS_SPECTRAL_DATA
 * then we need to quantize the PSD, finally we need to generate the WVN
 */
    if (output->spectralPTR != NS_SPECTRALDATA) {
        //This scan is a backGround, although it's tempting to keep it there because most probably we will never need the
        //additional space, but let's copy it to stream line the code
        //adding the 2 for the 2 WVN points
        if (output->settings.commonWaveNum != 0) {
            memcpy(NS_SPECTRALDATA, output->spectralPTR, sizeof(double) * (output->psdLength + 2));
        } else {
            memcpy(NS_SPECTRALDATA, output->spectralPTR, sizeof(double) * (output->psdLength * 2));
        }
    }
    output->spectralPTR = (double *) NS_SPECTRALDATA;
    uint64_t *psdPtr = (uint64_t *) output->spectralPTR;
    uint64_t *wvnPtr = (uint64_t *) (output->spectralPTR + output->psdLength);
    double *doubleWvnPtr = (double *) wvnPtr;
    for (int i = 0; i < output->psdLength; ++i) {
        psdPtr[i] = (uint64_t) (output->spectralPTR[i] * pow(2, 33));  //quantization of the PSD
    }
    vTaskDelay(10);
//    ESP_LOGI(TAG,"Common WaveNum is %d",output->settings.commonWaveNum);
    if (output->settings.commonWaveNum != 0) {
        //Linear interpleation is applied here, means we need to generate the wvn from start and step
//        ESP_LOGI(TAG,"Quantized WVN data Start = %llu, Step = %llu",wvnPtr[0],wvnPtr[1]);
        doubleWvnPtr[0] = (((double) ((wvnPtr[0] >> 3) * 10000)) / (1 << 30));
        double wvnStep = (((double) ((wvnPtr[1] >> 3) * 10000)) / (1 << 30));
//        ESP_LOGI(TAG,"First Values is %lf, and step is %lf",doubleWvnPtr[0],wvnStep);
        for (int i = 1; i < output->psdLength; ++i) {
            // wvnPtr[i] = wvnPtr[i - 1] + wvnStep;  //calculate wvn values as uint64_t
            doubleWvnPtr[i] = doubleWvnPtr[i - 1] + wvnStep;
        }
        vTaskDelay(10);
    }
    //quantize the WVN prior to sending
    for (int i = 0; i < output->psdLength; ++i) {
        // doubleWvnPtr[i] = (((double) (wvnPtr[i] >> 3) * 10000.0) / (1 << 30));
//            ESP_LOGI(TAG,"DoubleWVN[%d] = %lf",i,doubleWvnPtr[i]);
        //finally perform quantization so it can be sent over to the device
        wvnPtr[i] = (uint64_t) (doubleWvnPtr[i] * pow(2, 30));
        //ESP_LOGI(TAG,"WVN[%d] = %llu",i,wvnPtr[i]);
    }

    return NoError;
}

static ScannerError_t writeScanFile(SpectralData_t *input) {
    char fileName[50] = {0};
    snprintf(fileName, sizeof(fileName), DEFAULT_FILE_NAME "%hu.bin", _scanNum);


    ScannerError_t err = writeFile(fileName, (void *) input, sizeof(SpectralData_t));
    if (err != NoError)
        return err;

    FILE *fp = fopen(fileName, "a"); //append spectral data
    if (fp == NULL) {
        ESP_LOGE(TAG, "%s, OPEN_WRITE_FILE_ERR", fileName);
        return OPEN_WRITE_FILE_ERR;
    }
//    ESP_LOGI(TAG,"Quantized WVN data Start = %llu, Step = %llu",((uint64_t*)input->spectralPTR)[input->psdLength],((uint64_t*)input->spectralPTR)[input->psdLength+1]);
    if (input->settings.commonWaveNum == 0)
        fwrite(input->spectralPTR, sizeof(double), input->psdLength * 2, fp);
    else
        fwrite(input->spectralPTR, sizeof(double), input->psdLength + 2, fp);
    fclose(fp);

    return NoError;
}

static ScannerError_t readScanNum(uint16_t *scanNum) {
    FILE *fp = fopen(SCAN_NUM_FILE_NAME, "r");
    if (fp == NULL) {
        ESP_LOGE(TAG, "SCAN_NUM_FILE_NOT_EXIST,error is %s with error no %d", strerror(errno), errno);
        return SCAN_NUM_FILE_NOT_EXIST;
    }

    fread(scanNum, sizeof(scanNum), 1, fp);
    fclose(fp);
    return NoError;
}

static ScannerError_t writeScanNum(uint16_t scanNum) {
    FILE *fp = fopen(SCAN_NUM_FILE_NAME, "w");
    if (fp == NULL) {
        ESP_LOGE(TAG, "OPEN_WRITE_FILE_ERR");
        return OPEN_WRITE_FILE_ERR;
    }

    fwrite(&scanNum, sizeof(scanNum), 1, fp);
    fclose(fp);
    return NoError;
}

static ScannerError_t clearMemory() {
//    char fileName[50] = {0};
ScannerError_t err;
    err =  remove(BACKGROUND_PTR_FILE_NAME);
    if(NoError == err){
        return writeScanNum(_scanNum = 0);
    }else{
        return err;
    }
//    for (int16_t i = _scanNum - 1; i >= 0; i--) {
//        snprintf(fileName, sizeof(fileName), DEFAULT_FILE_NAME "%d.bin", i);
//        if (remove(fileName) != 0) {
//            ESP_LOGE(TAG, "SCAN_FILE_NOT_EXIST, File Name : %s, Error : %s, ErrorNumber : %d", fileName,
//                     strerror(errno), errno);
//            writeScanNum((_scanNum = i));
//            return SCAN_NUM_FILE_NOT_EXIST;
//        }
//        vTaskDelay(2);
//    }
//
//    return writeScanNum((_scanNum = 0));
}

/*
 * This function is used for debugging
 */
static void listFiles() {
    struct dirent *de;  // Pointer for directory entry
    unsigned long long used_space = 0;
    struct stat file_stat;
    char file_name[300];
    DIR *dr = opendir(BASE_PATH"/");

    if (dr == NULL) {
        ESP_LOGE(TAG, "Could not open current directory");
        return;
    }

    ESP_LOGW(TAG, "***SoF: Files in SPIFFS Directory***");
    while ((de = readdir(dr)) != NULL) {
        snprintf(file_name, 300, "%s/%s", BASE_PATH, de->d_name);
        stat(file_name, &file_stat);
        ESP_LOGW(TAG, "%s : %ld", de->d_name, file_stat.st_size);
        used_space += file_stat.st_size;

    }
    ESP_LOGW(TAG, "***EoF: Files in FAT Directory***");
    ESP_LOGW(TAG, "Total Size for files is %lld", used_space);

    closedir(dr);
    size_t total = 0, used = 0;
    esp_err_t ret = esp_spiffs_info("storage", &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        // ESP_LOGI(TAG, "Partition label: %s",conf.partition_label);
        ESP_LOGW(TAG, "Partition size: total: %d, used: %d", total, used);
    }
}


int openFileIfExist(FILE **file_to_open, const char *full_file_path) {
    (*file_to_open) = fopen(full_file_path, "r");
    return (*file_to_open) != NULL;
}

int getCommonWaveNumFromPSDLength(uint16_t psdLength) {
    switch (psdLength) {
        case 65:
            return 1;
        case 129:
            return 2;
        case 257:
            return 3;
        case 513:
            return 4;
        case 1024:
            return 5;
        case 2048:
            return 6;
        case 4096:
            return 7;
        default:
            return 0;

    }
}

ScannerError_t readOldScanFile(FILE *scan_file, SpectralData_t *spectralData) {
    uint8_t scan_tag;
    uint16_t y_bytes_len, x_bytes_len;
    uint16_t y_length;
    uint64_t *readPsd;
    double *converted_psd;
    fread(&scan_tag, sizeof(scan_tag), 1, scan_file);
    fread(&y_bytes_len, sizeof(uint16_t), 1, scan_file);
    fread(&x_bytes_len, sizeof(uint16_t), 1, scan_file);
    y_length = y_bytes_len / sizeof(uint64_t);
    //now we read the y_bytes_len bytes as uint64_t from the file
    switch (scan_tag) {
        case OLD_FORMAT_REFERENCE_SCAN:
            spectralData->type = SPECTRAL_TYPE_BCKGRND;
            readPsd = (uint64_t *) NS_BACKGROUNDDATA;
            break;
        case OLD_FORMAT_REFLECTANCE_SCAN:
            spectralData->type = SPECTRAL_TYPE_SPECTRUM;
            readPsd = (uint64_t *) NS_SPECTRALDATA;
            break;
        case OLD_FORMAT_RAW_SCAN:
            spectralData->type = SPECTRAL_TYPE_PSD;
            readPsd = (uint64_t *) NS_SPECTRALDATA;
            break;
        default:
            ESP_LOGE(TAG, "Illegal Scan tag %d", scan_tag);
            ESP_ERROR_CHECK(ESP_ERR_INVALID_ARG);
            return READ_FILE_ERR;
            break;
    }
    //now here's a clever trick, the maximum PSD length is actually 257 points, that means 257*8 bytes,
    //and since i have enough space to store  4098*8 bytes, I can get the PSD and the WVN in the same area, and just do
    //each conversion alone.
    //just a sanity check, if these 2 are not the same or the y contains more than 300 points
    // reset the controller and go in an infinite boot loop, this SHOULD never happen
    ESP_LOGI(TAG, "PSD length %d, WVL length %d", y_bytes_len, x_bytes_len);
    ESP_ERROR_CHECK((y_bytes_len == x_bytes_len) && (y_bytes_len < 2400) ? ESP_OK : ESP_ERR_INVALID_ARG);
    //read x_length+y_length of 8 byte long integers from the scan file
    if (2 * y_length != fread(readPsd, sizeof(uint64_t), 2 * y_length, scan_file)) {
        ESP_LOGE(TAG, "Couldn't read all bytes from the old scan file");
        fclose(scan_file);
        return READ_FILE_ERR;
    }
    fclose(scan_file);
    spectralData->settings.scanTime = _current_scanner_settings.scanTime;
    spectralData->settings.commonWaveNum = getCommonWaveNumFromPSDLength(y_length);
    spectralData->settings.opticalGain = _current_scanner_settings.opticalGain;
    spectralData->settings.apodizationSel = _current_scanner_settings.apodizationSel;
    spectralData->settings.zeroPadding = _current_scanner_settings.zeroPadding;
    spectralData->settings.mode = _current_scanner_settings.mode;
    //now for the conversion bit
    converted_psd = (double *) readPsd;   //now they should point to the same area in memory

    for (int i = 0; i < y_length; ++i) {
        converted_psd[i] = (double) (readPsd[i]) / ((uint64_t) 1 << 33);   //PSD conversion
    }

    if (spectralData->settings.commonWaveNum != 0) {
        uint64_t *wvnSourcePtr = &readPsd[y_length];
        uint64_t *wvnDestinationPtr = wvnSourcePtr;
        //Source is the same as destination because I don;t need to keep the source
        //data intact.
        //This sample is considered as initial value of WVN
        wvnDestinationPtr[0] = (wvnSourcePtr[0] / 10000) << 3;
        wvnDestinationPtr[1] = (wvnSourcePtr[8] / 10000) << 3;
        //This sample is considered as step value of WVN
        wvnDestinationPtr[1] = (wvnDestinationPtr[1] - wvnDestinationPtr[0]) / 8;

    } else {
        for (int i = 0; i < y_length; ++i) {
            //CommonWaveNum = 0, extract all wavelength and change to double
            converted_psd[y_length + i] = (double) (readPsd[y_length + i]) / ((uint64_t) 1 << 30);   //WVN conversion
        }
    }
    //now we have both the PSD and the WVN converted and in place, ready to be saved
    spectralData->spectralPTR = converted_psd;
    spectralData->psdLength = y_length;

    return NoError;
}

ScannerError_t readOldConfigurationFile(ScannerSettings_t *scannerSettings, sourceSettings_t *sourceSettingsPtr) {
    struct OldP3Configuration {
        uint32_t scan_time;                        //4 bytes
        uint8_t source_delta;
        uint8_t source_t1;
        uint8_t source_t2_c1;
        uint8_t source_t2_c2;
        uint8_t source_t2_tmax;
        uint8_t mode;
        uint8_t lampsNumber;
        uint8_t SourceLampSel;
        uint8_t absorbSel;
        uint8_t apodizationSel;
        uint8_t commonWaveNum;
        uint8_t zeroPadding;
        uint8_t points_number;
        uint8_t unit;
        uint8_t opticalGain;
        //till here we have 19 bytes of data, with no padding bytes in between
        //1 padding byte
        uint16_t gainvalue;  //2 bytes of data
        //2 padding bytes so the struct is aligned
    } oldP3Configuration;
    const uint8_t unPaddedP3ConfigSize = 19;
    FILE *configFilePtr;
    if (!openFileIfExist(&configFilePtr, BASE_PATH"/config_file2")) {
        return OPEN_READ_FILE_ERR;
    }
    if (unPaddedP3ConfigSize != fread(&oldP3Configuration, sizeof(uint8_t), unPaddedP3ConfigSize, configFilePtr)) {
        //I Could read most of the configuration, now to skip that 1 padding byte then read the last 2 bytes of the configuration
        fclose(configFilePtr);
        return READ_FILE_ERR;
    }
    fread(&(oldP3Configuration.gainvalue), sizeof(uint16_t), 1, configFilePtr);
    fclose(configFilePtr);
    //now let's put everything where it belongs
    scannerSettings->scanTime = oldP3Configuration.scan_time;
    //if old commonWaveNum =0 , then it's disabled, put as zero, else take the points number
    scannerSettings->commonWaveNum = oldP3Configuration.commonWaveNum != 0 ? oldP3Configuration.points_number : 0;
    scannerSettings->gainvalue = oldP3Configuration.gainvalue;
    scannerSettings->opticalGain = oldP3Configuration.opticalGain;
    scannerSettings->apodizationSel = oldP3Configuration.apodizationSel;
    scannerSettings->zeroPadding = oldP3Configuration.zeroPadding >> 5;
    //sourceSettings
    sourceSettingsPtr->lampsCount = oldP3Configuration.lampsNumber;
    sourceSettingsPtr->lampsSelect = oldP3Configuration.SourceLampSel;
    sourceSettingsPtr->t1 = oldP3Configuration.source_t1;
    sourceSettingsPtr->deltaT = oldP3Configuration.source_delta;
    sourceSettingsPtr->t2C1 = oldP3Configuration.source_t2_c1;
    sourceSettingsPtr->t2C2 = oldP3Configuration.source_t2_c2;
    sourceSettingsPtr->t2max = oldP3Configuration.source_t2_tmax;
    //remove now unneeded file
    remove(BASE_PATH"/config_file2");
    return NoError;
}

ScannerError_t readOldScannerId(uint64_t *id) {
    FILE *scannerIdFile;
    if (openFileIfExist(&scannerIdFile, BASE_PATH"/scanner_id_file")) {
        fread(id, sizeof(uint64_t), 1, scannerIdFile);
        fclose(scannerIdFile);
        remove(BASE_PATH"/scanner_id_file");
        return NoError;
    }
    return OPEN_READ_FILE_ERR;
}

ScannerError_t writeMagicMarker(FILE **magicFilePtr) {
    MagicMarker magicMarker = {
            .oldFirmwareId = magicIds[0],
            .newFirmwareId = magicIds[1],
    };
    /*
     * This file tells me that I have done the conversion process from an old firmware
     * version to a new firmware version, its format is one byte containing old firmware id
     * another byte containing the new firmware id;
     */
    ESP_LOGI(TAG, "Putting The magic marker");
    (*magicFilePtr) = fopen(MAGIC_FILE_NAME, "w");
    if ((*magicFilePtr) != NULL) {
        fwrite(&magicMarker, sizeof(MagicMarker), 1, *magicFilePtr);
        fclose(*magicFilePtr);
        return NoError;
    }
    return OPEN_WRITE_FILE_ERR;
}

ScannerError_t convertOldAmbientTempFile() {
    FILE *ambientTempFile;
    if (!openFileIfExist(&ambientTempFile, BASE_PATH"/ambient_temp")) {
        return OPEN_READ_FILE_ERR;
    }
    TemperatureReading temperatureReading;
    fread(&temperatureReading, sizeof(temperatureReading), 1, ambientTempFile);
    fclose(ambientTempFile);
    ambientTempFile = fopen(AMBIENT_TEMP_FILE_NAME, "w");
    if (ambientTempFile == NULL) {
        return OPEN_WRITE_FILE_ERR;
    }
    fwrite(&temperatureReading, sizeof(temperatureReading), 1, ambientTempFile);
    fclose(ambientTempFile);
//    _temperature_reading = temperatureReading;
    remove(BASE_PATH"/ambient_temp");
    return NoError;
}

ScannerError_t readOldMotherBoardVersionFile() {
    FILE *motherBoardVersionFile;
    uint32_t motherBoardVersion = 0;
    if (openFileIfExist(&motherBoardVersionFile, BASE_PATH"/MBVersion")) {
        ESP_LOGI(TAG, "Found Old Mother board version file. reading now");
        if (fread(&motherBoardVersion, sizeof(uint32_t), 1, motherBoardVersionFile) == 1) {
            fclose(motherBoardVersionFile);
            return writeFile(MOTHER_BOARD_FILE_NAME, &motherBoardVersion, sizeof(motherBoardVersion));
        } else {
            return READ_FILE_ERR;
        }
    } else {
        return OPEN_READ_FILE_ERR;
    }

}

void convertFilesFromOldFirmware(sourceSettings_t *sourceSettingsObject) {
    /**
     * This function is used to convert from the old firmware file format to the new firmware file format.
     * However, it needs to be done in an intelligent manner, as doing all these actions will surely cause a watchdog interrupt
     * So I think we need firs to list the files, then for each file that matches the pattern scan_file_%d, read the data from that scan file
     * the data in the scan file is either a RAW (PSD) or an REFLECTANCE(Absorabce) or a REFERENCE(BackGrnd), and it's in quantized format that means I need first
     * to divide it by 2^33 (not just shift as I need the output to be double) then send a request to the file system to save this data.
     * The request will then get executed by the file system task, which will read a file, save it then delete the old file and notify this task with completion
     */

    /**
     * first get the list of files that are generated by the old firmware:
     * 1) scan_file_/d : The file which contains the actual scan data
     * 2) config_file2 : Scanner Settings
     * 3) bg_index : BackGround scan num
     * 4) scans_num : total scans taken
     **/
    //First Make sure that we didn't convert before
//    return; //TODO : The issue is in the magic marker, find and fix it
    MagicMarker magicMarker;
    FILE *magicFilePtr = fopen(MAGIC_FILE_NAME, "r");
    if (magicFilePtr != NULL) {
        size_t read_bytes = fread(&magicMarker, sizeof(MagicMarker), 1, magicFilePtr);
        fclose(magicFilePtr);
        ESP_LOGI(TAG, "Read %d bytes from magic marker", read_bytes);
        if (magicMarker.newFirmwareId == CURRENT_FIRMWARE_MAGIC_ID) {
            //If the magic file exists, you have already converted the files, no need to do anything
            ESP_LOGI(TAG, "Magic file found, continue to startup");
            return;
        }
    }
    ESP_LOGI(TAG, "No Magic file found, continue to restore");
    //Configuration file
    //TODO : Handle source settings in centralManager

    if (NoError == readOldConfigurationFile(&_current_scanner_settings, sourceSettingsObject)) {
        ESP_LOGI(TAG, "Config File Found, writing data now");
        writeFile(SCANNER_SETTINGS_FILE_NAME, (void *) &_current_scanner_settings, sizeof(ScannerSettings_t));
    }
    readOldMotherBoardVersionFile();

    vTaskDelay(10);

    //read scanner id
    uint64_t scannerId;
    if (NoError == readOldScannerId(&scannerId)) {
        writeFile(SCANNER_ID_FILE_NAME, (void *) &scannerId, sizeof(scannerId));
        ESP_LOGI(TAG, "Scanner Id File Found, id is %llu", scannerId);

    }
    vTaskDelay(10);
    convertOldAmbientTempFile();
    vTaskDelay(10);

    //read scans num
    FILE *scan_num_file;
    int numOldScans = 0;
    if (openFileIfExist(&scan_num_file, BASE_PATH"/scans_num")) {
        fread(&numOldScans, sizeof(numOldScans), 1, scan_num_file);
        fclose(scan_num_file);
        ESP_LOGI(TAG, "found %d old scans", numOldScans);
    } else {
        //no scans taken, just return;
        ESP_LOGI(TAG, "No Scan num file found");
        writeMagicMarker(&magicFilePtr);
        return;
    }
    //delay a little bit to avoid triggering the Watchdog
    vTaskDelay(10);
    //now that we have the old scan numbers, we read the file from 1 to scan_num
    char scan_file_name[80];
    FILE *scan_file_ptr;
    SpectralData_t spectralData;
    for (int i = 1; i < numOldScans; i++) {
        snprintf(scan_file_name, sizeof(scan_file_name), BASE_PATH"/scan_file_%d", i);
        if (openFileIfExist(&scan_file_ptr, scan_file_name)) {
            ESP_LOGI(TAG, "found %s reading it now", scan_file_name);
            //The file exists, now we need to read it
            if (NoError == readOldScanFile(scan_file_ptr, &spectralData)) {
                //now we need to save this spectralData while checking if any error happened while saving, report the error
                //This will also handle the background ptr file, and handle the scanNum file
                ESP_ERROR_CHECK_WITHOUT_ABORT(writeSpectralData(&spectralData));
                //delete the scan_file_name
                remove(scan_file_name);
                //delay a little to avoid triggering the watchdog
            }
            vTaskDelay(100);
        }
    }
    //now to delete the background_ptr and the scan_num files
    remove(BASE_PATH"/bg_index");
    remove(BASE_PATH"/scans_num");
    //put as a marker that we have finished conversion, no need to do it again
    writeMagicMarker(&magicFilePtr);
}

/*
 * This function is used for debugging
 */
static void format() {
    const esp_partition_t *partition =
            esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "storage");

    if (partition != NULL)
        ESP_ERROR_CHECK(esp_partition_erase_range(partition, 0, 0x600000));
    else
        ESP_LOGE(TAG, "Error formatting fat partition");
}

void fileSystem_init_flash() {
    // format(); //just for debugging

    // To mount device we need name of device partition, define base_path
    // and allow format partition in case if it is new one and was not formated before
    esp_vfs_spiffs_conf_t conf = {
            .base_path = "/spiffs",
            .partition_label = "storage",
            /*
             * This max_files parameter actually has a very interesting side effect, you see, to save some memory it's done
             * that way to save some memory as we only need one open file at any given time, but what happens if you forget
             * to close a file, wellllll, the SPIFFS simply won't open ANY OTHER FILE, and it won't even give you a meaningful
             * error, yes I know that file doesn't exist, that's why I am creating it you imbecile. Soo, if you find the FS acting up
             * just make sure you haven't opened a file and didn't close it
             */
            .max_files = 1,
            .format_if_mount_failed = true
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

//    listFiles(); //just for Debugging


}

void fileSystem_init(ScannerSettings_t *outSettings, ScannerID_t *scannerID, uint16_t *memorySize,
                     SpectralData_t *background, sourceSettings_t *outSourceSettings,
                     TemperatureReading *temperature_reading, uint32_t *mother_board_version,
                     uint32_t *inactive_timeout, uint8_t *saved_temperature_window) {


//    listFiles(); //just for Debugging
//    remove(GOLDEN_FILE_STATUS_FILE_NAME);

    if (readScanNum(&_scanNum) != NoError)
        writeScanNum(_scanNum);

    convertFilesFromOldFirmware(outSourceSettings);
    readFile(SCANNER_SETTINGS_FILE_NAME, (void *) outSettings, sizeof(ScannerSettings_t));
    _current_scanner_settings = *outSettings;

    uint64_t id = 0x0;
    readFile(SCANNER_ID_FILE_NAME, (void *) &id, sizeof(id));
    if (id == 0)
        scannerID->id[0] = 0;
    else
        snprintf((char *) &scannerID->id, sizeof(ScannerID_t), "NS-Scanner_%lld", id);

    if (readBackground(background) != NoError)
        background->spectralPTR = NULL;

    *memorySize = _scanNum;
    //Now this part requires some logic, first we check if the temperature file exists, because if it exists then read it
    // and put it in the temperature_reading variable so that the central manager can send it to the NS module.
    //Either way we need to save the initial TAIF temperature internally in case we needed to rewrite the ambient temp file

    //first save the ambient temp internally
    _temperature_reading = *temperature_reading;
    //That will overwrite the temperature reading if it finds flash stored data, to overwrite the flash stored data you need to send
    //another request from the mobile
    readFile(AMBIENT_TEMP_FILE_NAME, (void *) temperature_reading, sizeof(_temperature_reading));
    readFile(MOTHER_BOARD_FILE_NAME, (void *) mother_board_version, sizeof(*mother_board_version));
    readFile(INACTIVE_TIMEOUT_FILE_NAME, (void *) inactive_timeout, sizeof(*inactive_timeout));
    readFile(TEMPERATURE_WINDOW_FILE_NAME, (void *) saved_temperature_window, sizeof(*saved_temperature_window));

}

void fileSystem_requestTask(void *parameters) {
    MemRequestPacket_t rp;

    memcpy(&rp, parameters, sizeof(MemRequestPacket_t));
    xEventGroupSetBits(taskReceivedData, FILESYSTEM_REQUEST_TASK_BIT); //Set the bit after copying the parameters

    ESP_LOGI(TAG, "Request came with operation ID (%d)", rp.operationID);

    ResponsePacket_t resp = {
            .packetType = RESPONSEPACKET_MEMORY,
            .statusCode = NoError,
            .packet.memResponse = {
                    .operationID = (uint32_t) rp.operationID
            },
    };

    switch (rp.operationID) {
        case OPID_MEM_STATS:
            resp.packet.memResponse.U.stat.usedMemory = _scanNum;
            resp.packet.memResponse.U.stat.version = FW_VERSION;
            break;

        case OPID_MEM_CLEAR:
//            writeFile(BATTERY_LOG_FILE_NAME,BATTERY_LOG_MEMORY,fileSystem_maximum_gauge_log_file_size *2 ); //Just for testing
            if (NoError == (resp.statusCode = clearMemory())) {
                resp.packet.memResponse.U.stat.usedMemory = _scanNum;
            }
            break;

        case OPID_SCAN_REQ_WITH_SETTINGS:
            resp.statusCode = readScanFile(rp.U.scanNum - 1,    //scanNum -1 because counting starts from zero
                                           &resp.packet.memResponse.U.spectralData);
            break;
        case OPID_SCAN_REQ:
            resp.statusCode = readScanFileWithOldFormat(rp.U.scanNum - 1, &resp.packet.memResponse.U.spectralData);
            break;
        case OPID_SAVE_SETTINGS:
            _current_scanner_settings = rp.U.scannerSettings;
            resp.statusCode = writeFile(SCANNER_SETTINGS_FILE_NAME,
                                        (void *) &rp.U.scannerSettings, sizeof(ScannerSettings_t));
            break;

        case OPID_RESTORE_DEFAULT_SETTINGS:
            _current_scanner_settings = _default_scanner_settings;
            resp.statusCode = writeFile(SCANNER_SETTINGS_FILE_NAME,
                                        (void *) &_default_scanner_settings, sizeof(ScannerSettings_t));
            break;

        case OPID_SET_SCANNER_ID:
            resp.statusCode = writeFile(SCANNER_ID_FILE_NAME,
                                        (void *) &rp.U.scannerID, sizeof(rp.U.scannerID));
            break;
        case OPID_SAVE_AMBIENT_TEMP_FILE:
            _temperature_reading.ambientTemp = rp.U.temperature;
            //Save temperature read from mobile with the TAIF temperature read at startup
            resp.statusCode = writeFile(AMBIENT_TEMP_FILE_NAME, (void *) &_temperature_reading,
                                        sizeof(_temperature_reading));
            //send it to the response so the NS_Sensor module gets the data
            resp.packet.memResponse.U.temperature_reading = _temperature_reading;
            break;
        case OPID_READ_AMBIENT_TEMP_FILE:
            resp.statusCode = readFile(AMBIENT_TEMP_FILE_NAME, (void *) &resp.packet.memResponse.U.temperature_reading,
                                       sizeof(TemperatureReading));
            break;
        case OPID_SAVE_MOTHER_BOARD_VERSION_FILE:
            ESP_LOGI(TAG, "Saving Mother Board Version %d", rp.U.mother_board_version);
            resp.statusCode = writeFile(MOTHER_BOARD_FILE_NAME, (void *) &rp.U.mother_board_version,
                                        sizeof(rp.U.mother_board_version));
            break;
        case OPID_READ_MOTHER_BOARD_VERSION_FILE:
            resp.statusCode = readFile(MOTHER_BOARD_FILE_NAME, (void *) &resp.packet.memResponse.U.mother_board_version,
                                       sizeof(resp.packet.memResponse.U.mother_board_version));
            ESP_LOGI(TAG, "Reading Mother Board Version %d", resp.packet.memResponse.U.mother_board_version);
            break;
        case OPID_SAVE_INACTIVE_TIMEOUT:
            resp.statusCode = writeFile(INACTIVE_TIMEOUT_FILE_NAME, (void *) &rp.U.inactive_timeout,
                                        sizeof(rp.U.inactive_timeout));
            if (NoError == resp.statusCode) {
                resp.statusCode = (pdTRUE == centralManager_set_inactive_timeout(rp.U.inactive_timeout)) ? NoError
                                                                                                         : WRITE_FILE_ERR;
            }
            break;
        case OPID_SAVE_TEMPERATURE_WINDOW:
            resp.statusCode = writeFile(TEMPERATURE_WINDOW_FILE_NAME, (void *) &rp.U.temperature_window,
                                        sizeof(rp.U.temperature_window));
            if (NoError == resp.statusCode) {
                centralManager_set_scan_temperature_window(rp.U.temperature_window);
            }
            break;
        case OPID_GET_BATTERY_LOG:{
            struct stat * file_stats = (struct stat *) BATTERY_LOG_MEMORY;
            stat(BATTERY_LOG_FILE_NAME,file_stats);
            resp.packet.memResponse.U.battery_log_data.length = file_stats->st_size;
            resp.statusCode = readFile(BATTERY_LOG_FILE_NAME,BATTERY_LOG_MEMORY,resp.packet.memResponse.U.battery_log_data.length);
            ESP_LOGI(TAG,"Getting Battery log with size %d",resp.packet.memResponse.U.battery_log_data.length);
            resp.packet.memResponse.U.battery_log_data.data_bytes = BATTERY_LOG_MEMORY;
            resp.packet.memResponse.operationID = OPID_GET_BATTERY_LOG;
            for (int i = 0; i < resp.packet.memResponse.U.battery_log_data.length; i+=fileSystem_gauge_log_entry_size) {
                ESP_LOGI(TAG,"Event type is %d while battery percentage is %d",*((uint32_t*)(resp.packet.memResponse.U.battery_log_data.data_bytes+i)),((GaugeInfo_t*)(resp.packet.memResponse.U.battery_log_data.data_bytes+i +4))->batteryPercentage);
            }
        }
            break;
        default:
            break;
    }

    xQueueSend(responseQueue, (void *) &resp, portMAX_DELAY);

    ESP_LOGI(TAG, "Remaining stack %d bytes", uxTaskGetStackHighWaterMark(NULL));
    xEventGroupSetBits(availableTasks, FILESYSTEM_REQUEST_TASK_BIT); //Set the bit before deleting the task
    vTaskDelete(NULL);  //Delete itself
    while (1);           //This task should be deleted after finishing
}

void fileSystem_responseTask(void *parameters) {
    RequestPacket_t rp = {
            //This request is used to send mem stats after saving new a reading.
            .packetType = REQUESTPACKET_MEMORY,
            .packet.memPacket = {
                    .operationID = OPID_MEM_STATS,
            },
    };
    P3Response_t resp;

    memcpy(&resp, parameters, sizeof(P3Response_t));
    xEventGroupSetBits(taskReceivedData, FILESYSTEM_RESPONSE_TASK_BIT); //Set the bit after copying the parameters

    ESP_LOGI(TAG, "Response came with operation ID (%d)", resp.operationID);

    switch (resp.operationID) {
        case OPID_RUN_BKGND:
            if (NoError == writeBackground(&resp.data.spectralData)) {
                writeScanNum(++_scanNum);
            }

            xQueueSend(requestQueue, (void *) &rp, portMAX_DELAY);
            break;

        case OPID_RUN_PSD:
        case OPID_RUN_ABSRB:
            //save only if I am not connected
            if (!centralManager_isScannerConnected() && NoError == writeScanFile(&resp.data.spectralData)) {
                writeScanNum(++_scanNum);
                //Send the request only if I tried to save something in the memory
                xQueueSend(requestQueue, (void *) &rp, portMAX_DELAY);
            }
            break;

        default:
            break;
    }

    ESP_LOGI(TAG, "Remaining stack %d bytes", uxTaskGetStackHighWaterMark(NULL));
    xEventGroupSetBits(availableTasks, FILESYSTEM_RESPONSE_TASK_BIT); //Set the bit before deleting the task
    vTaskDelete(NULL);  //Delete itself
    while (1);           //This task should be deleted after finishing
}
void fileSystem_save_scan_data(P3Response_t *resp){
    RequestPacket_t rp = {
            //This request is used to send mem stats after saving new a reading.
            .packetType = REQUESTPACKET_MEMORY,
            .packet.memPacket = {
                    .operationID = OPID_MEM_STATS,
            },
    };
    ESP_LOGI(TAG, "Response came with operation ID (%d)", resp->operationID);

    switch (resp->operationID) {
        case OPID_RUN_BKGND:
            if (NoError == writeBackground(&resp->data.spectralData)) {
                writeScanNum(++_scanNum);
            }

            xQueueSend(requestQueue, (void *) &rp, portMAX_DELAY);
            break;

        case OPID_RUN_PSD:
        case OPID_RUN_ABSRB:
            //save only if I am not connected
            if (!centralManager_isScannerConnected() && NoError == writeScanFile(&resp->data.spectralData)) {
                writeScanNum(++_scanNum);
                //Send the request only if I tried to save something in the memory
                xQueueSend(requestQueue, (void *) &rp, portMAX_DELAY);
            }
            break;

        default:
            break;
    }
    xEventGroupSetBits(availableTasks, FILESYSTEM_RESPONSE_TASK_BIT); //Set the bit before deleting the task
}
esp_err_t fileSystem_save_gauge_debug_log(battery_log_data_t * battery_log_data){
    ESP_LOGI(TAG,"gauge Log entry size is %d",fileSystem_gauge_log_entry_size);
    esp_err_t ret_val = ESP_OK;
    uint8_t * memory_region = BATTERY_LOG_MEMORY;  //5 kilo after the beginning of the shared memory, will change later
    struct stat *file_stats = (struct stat *) memory_region;
    uint16_t file_size;
    //setting all bytes to zero in case of failed stat call because of file doesn't exist, it will go with the path that creates the file and writes to it
    //no difference in behaviour
    memset(file_stats,0,sizeof(struct stat));
    if(stat(BATTERY_LOG_FILE_NAME,file_stats) != 0){
        ESP_LOGI(TAG,"Failed to get %s stats, with error %s",BATTERY_LOG_FILE_NAME, strerror(errno));
        if(errno != ENOENT){
            return ESP_FAIL;
        }
    }
    file_size = file_stats->st_size;
    ESP_LOGI(TAG,"%s size is now %d",BATTERY_LOG_FILE_NAME,file_size);
    if(file_size == fileSystem_maximum_gauge_log_file_size){
        //This file is larger that the maximum, need to read it and switch all the data above by one gaugeLog size
        //open the file for reading
        FILE * fptr = fopen(BATTERY_LOG_FILE_NAME,"r+");
        configASSERT(fptr); //check that the fptr is not null, it should never be null here as I have read its stats, and it has a size
        fread(memory_region,1,file_size,fptr);
        //need to shift the whole region up by fileSystem_gauge_log_entry_size
        memmove(memory_region,memory_region + fileSystem_gauge_log_entry_size,file_size - fileSystem_gauge_log_entry_size);
        uint8_t * to_edit_region = memory_region + file_size - fileSystem_gauge_log_entry_size;
        memcpy(to_edit_region,&battery_log_data->event_type,sizeof(battery_log_data->event_type));
        memcpy(to_edit_region + sizeof(battery_log_data->event_type),battery_log_data->gauge_info_ptr,sizeof(GaugeInfo_t));
        //finally, write to the file
        rewind(fptr);
        if(fwrite(memory_region,1,file_size,fptr) == file_size){
            ret_val = ESP_OK;
        }else{
            ESP_LOGI(TAG,"Invalid write size while writing %d to the file %s",file_size,BATTERY_LOG_FILE_NAME);
            ret_val = ESP_FAIL;
        }
        fclose(fptr);
    }else if(file_size < fileSystem_maximum_gauge_log_file_size){
        //just append your data to the file
        uint8_t * to_edit_region = memory_region;
        memcpy(to_edit_region,&battery_log_data->event_type,sizeof(battery_log_data->event_type));
        configASSERT(battery_log_data->gauge_info_ptr);
        memcpy(to_edit_region + sizeof(battery_log_data->event_type),battery_log_data->gauge_info_ptr,sizeof(GaugeInfo_t));
        FILE * fptr = fopen(BATTERY_LOG_FILE_NAME,"a"); //open the file to append to it
        configASSERT(fptr); //check that the fptr is not null, it should never be null here as I have read its stats, and it has a size
        if(fwrite(to_edit_region,fileSystem_gauge_log_entry_size,1,fptr) == 1){
            ret_val = ESP_OK;
        }else{
            ESP_LOGI(TAG,"Invalid write size while writing %d to the file %s",fileSystem_gauge_log_entry_size,BATTERY_LOG_FILE_NAME);
            ret_val = ESP_FAIL;
        }
        fclose(fptr);
    }else{
        ESP_LOGI(TAG,"%s file is too big, something wrong happened",BATTERY_LOG_FILE_NAME);
        //the something wrong that happened is that I put fileSystem_maximum_gauge_log_file_size of bytes after the original bytes of the file, which
        // in this case is already at fileSystem_maximum_gauge_log_file_size, so I need to read the last fileSystem_maximum_gauge_log_file_size and write them only to
        // the file instead of removing it.
        FILE * fptr = fopen(BATTERY_LOG_FILE_NAME,"r");
        configASSERT(fptr);
        //move to the second 6500 bytes of the files
        fseek(fptr,fileSystem_maximum_gauge_log_file_size,SEEK_SET);
        if(fread(BATTERY_LOG_MEMORY,fileSystem_maximum_gauge_log_file_size,1,fptr) == 1){
            fclose(fptr);
            fptr = fopen(BATTERY_LOG_FILE_NAME,"w");//open in write mode to clear the file
            configASSERT(fptr);
            if(fwrite(BATTERY_LOG_MEMORY,fileSystem_maximum_gauge_log_file_size,1,fptr) == 1) {
                ESP_LOGI(TAG,"Done Fixing %s",BATTERY_LOG_FILE_NAME);
                ret_val = ESP_OK;
            }else{
                remove(BATTERY_LOG_FILE_NAME);
                ret_val = ESP_FAIL;
            }
        }else{
            remove(BATTERY_LOG_FILE_NAME);
            ret_val = ESP_FAIL;
        }
        fclose(fptr);
//        remove(BATTERY_LOG_FILE_NAME);
//        ESP_ERROR_CHECK(ESP_FAIL);  //reset here
    }

    return ret_val;
}

ScannerSettings_t *fileSystem_get_scanner_settings() {
    return &_current_scanner_settings;

}

bool fileSystem_is_gauge_golden_file_burnt() {
    FILE *golden_file_ptr;
    bool retVal = false;
    if (openFileIfExist(&golden_file_ptr, GOLDEN_FILE_STATUS_FILE_NAME)) {
        retVal = true;
        fclose(golden_file_ptr);
    }
    return retVal;
}

void fileSystem_register_golden_file_burnt() {
    FILE *golden_file_ptr;
    golden_file_ptr = fopen(GOLDEN_FILE_STATUS_FILE_NAME, "w");
    if (golden_file_ptr != NULL) {
        ESP_LOGI(TAG, "Burnt Golden file and wrote the status file");
        fclose(golden_file_ptr);
    } else {
        ESP_LOGE(TAG, "Couldn't Burn Golden File with err %d : %s", errno, strerror(errno));
    }
}
