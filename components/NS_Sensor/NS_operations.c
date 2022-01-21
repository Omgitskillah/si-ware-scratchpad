#include <string.h>
#include <math.h>
#include "NS_operations.h"
#include "NS_sensor.h"
#include "driver/gpio.h"
#include "centralManagerErrorCodes.h"
#include "centralManagerSystemIOs.h"

#define TAG "NS_Operations"
static const double HYSTERESIS_WINDOW_C = 5.;
static const double MAX_ALLOWED_TEMP_C = 55.;
static const double C_TO_LSB_FACTOR = 5000.;
uint8_t NS_temperature_window = DEFAULT_TEMPERATURE_WINDOW;

static uint32_t calibration_wells[5] = {0};
sourceSettings_t persistant_source_settings;
TemperatureReading NS_temperature = {
        .tAIFReading = -1,
        .ambientTemp = -1,
};


static uint32_t calculate_lights_off_time(uint32_t timeout_ms) {
    return timeout_ms +
           ((persistant_source_settings.deltaT + persistant_source_settings.t1 + persistant_source_settings.deltaT) *
            50) + 500;
}

ScannerError_t check_temperature() {
    static ScannerError_t last_check_temp_status = NoError;
    P3RequestPacket_t rp = {
            .operationID = OPID_RUN_Temp,
    };
    P3Response_t response;
    ScannerError_t status;

    if (NS_temperature.ambientTemp == -1) {
        return TEMP_NOT_STORED_ERROR;
    }
    if ((status = runTemperature(&rp, &response))) {
        return status;
    }

    uint32_t current_taif_temperature = response.data.temperature;
    double ambient_temp_celsius = NS_temperature.ambientTemp / pow(2, 26);
    double current_temp_celsius = ambient_temp_celsius +
                                  (double) ((int) current_taif_temperature - (int) NS_temperature.tAIFReading) /
                                  C_TO_LSB_FACTOR;
    double threshold =
            last_check_temp_status == NoError ? MAX_ALLOWED_TEMP_C : MAX_ALLOWED_TEMP_C - HYSTERESIS_WINDOW_C;
    ESP_LOGI(TAG, "Current Temp is %lf, Threshold is %lf", current_temp_celsius, threshold);
    ESP_LOGI(TAG, "Current TAIF Temp is %u, Saved TAIF temp is is %u", current_taif_temperature,
             NS_temperature.tAIFReading);
    return current_temp_celsius >= threshold ? (last_check_temp_status = HIGH_TEMP_ERROR)
                                             : (last_check_temp_status = NoError);
}
static ScannerError_t checkLightSourceRelatedErrors(){
    ScannerError_t err;
    if ((err = check_temperature()) == HIGH_TEMP_ERROR) {
        ESP_LOGE(TAG, "Error checking temperature. code =  %u", err);
    }
    if ((err = centralManager_check_battery_level(1)) == LOW_BATTERY_SCAN_ERROR) {
        ESP_LOGE(TAG, "Error checking battery. code =  %u", err);
    }
    return NoError;
}

/* This function stream PSD and WVN data from sensor. */
static void streamSpectralData(P3Response_t *resp) {
    char sbuf[25];

    NS_sendSpi("12, 1", NULL); //AutoincB = 1

    if (resp->data.spectralData.settings.commonWaveNum != 0) {
        uint64_t *wvn_srcptr = (uint64_t *) resp->data.spectralData.spectralPTR;
        uint64_t *wvn_destptr = (uint64_t *) &resp->data.spectralData.spectralPTR[resp->data.spectralData.psdLength];

        //Read 8 samples from WVN to use first and 8th samples
        NS_sendSpi("168, z72", (uint8_t *) wvn_srcptr);
        //Convert the 1st sample and put it at spectralData[psdLength]
        //This sample is considered as initial value of WVN
        wvn_destptr[0] = (wvn_srcptr[0] / 10000) << 3;

        //Convert the 8th sample and put it at spectralData[psdLength + 1]
        wvn_destptr[1] = (wvn_srcptr[8] / 10000) << 3;
        //This sample is considered as step value of WVN
        wvn_destptr[1] = (wvn_destptr[1] - wvn_destptr[0]) / 8;
//        ESP_LOGI(TAG,"Quantized WVN data Start = %llu, Step = %llu",wvn_destptr[0],wvn_destptr[1]);
    } else {
        sprintf(sbuf, "168, z%d", resp->data.spectralData.psdLength * sizeof(uint64_t));
        //Read wvn vector and place it after the PSD vector
        //Note this command should be executed before the psd streaming command
        //otherwise it will ruin last psd sample
        NS_sendSpi(sbuf, (uint8_t *) &resp->data.spectralData.spectralPTR[resp->data.spectralData.psdLength]);

        //Convert wvn fixedpoint to floating point
        uint64_t *wvn_ptr = (uint64_t *) &resp->data.spectralData.spectralPTR[resp->data.spectralData.psdLength];
        for (uint16_t i = 0; i < resp->data.spectralData.psdLength; i++)
            resp->data.spectralData.spectralPTR[resp->data.spectralData.psdLength + i] =
                    ((double) wvn_ptr[i]) / ((uint64_t) 1 << 30);
    }

    {   //Stream PSD vector
        sprintf(sbuf, "160, z%d", resp->data.spectralData.psdLength * sizeof(uint64_t));
        NS_sendSpi(sbuf, (uint8_t *) resp->data.spectralData.spectralPTR);

        //Convert psd fixedpoint to floating point
        uint64_t *psd_ptr = (uint64_t *) resp->data.spectralData.spectralPTR;
        for (uint16_t i = 0; i < resp->data.spectralData.psdLength; i++)
            resp->data.spectralData.spectralPTR[i] = ((double) psd_ptr[i]) / ((uint64_t) 1 << 33);
    }
//    ESP_LOGI(TAG,"Quantized WVN data Start = %llu, Step = %llu",((uint64_t*)resp->data.spectralData.spectralPTR)[resp->data.spectralData.psdLength],((uint64_t*)resp->data.spectralData.spectralPTR)[resp->data.spectralData.psdLength+1]);
    NS_sendSpi("12, 0", NULL); //AutoincB = 0
}

/* Prepare spectral data in case of reflectance requested and 
 * save background data in case of background reading requested.
 */
static void postSpectralProcessing(P3Response_t *resp) {
    switch (resp->operationID) {
        case OPID_RUN_BKGND:
            memcpy(&background, &resp->data.spectralData, sizeof(SpectralData_t));
            break;

        case OPID_RUN_ABSRB: {
            for (uint16_t i = 0; i < resp->data.spectralData.psdLength; i++)
                resp->data.spectralData.spectralPTR[i] =
                        resp->data.spectralData.spectralPTR[i] / background.spectralPTR[i];

            break;
        }

        case OPID_RUN_PSD:
        default:
            return;
    }
}

static bool checkBackgroundValidity(P3Response_t *resp) {
//    ESP_LOGI(TAG,"Background temperature is %d, while current temperature is %d",background.temperature_during_scan,resp->data.spectralData.temperature_during_scan);
//    ESP_LOGI(TAG,"Temperature Difference is %d",temperature_difference);
    bool valid = (background.settings.scanTime >= resp->data.spectralData.settings.scanTime) &&
                 (background.settings.apodizationSel == resp->data.spectralData.settings.apodizationSel) &&
                 (background.settings.commonWaveNum == resp->data.spectralData.settings.commonWaveNum) &&
                 (background.settings.zeroPadding == resp->data.spectralData.settings.zeroPadding);
    return valid;
}
static bool checkBackgroundTemperature(P3Response_t * resp){
    uint32_t temperature_difference = abs((int32_t)background.temperature_during_scan - (int32_t)resp->data.spectralData.temperature_during_scan);
    return (temperature_difference < (NS_temperature_window * C_TO_LSB_FACTOR));

}

/* This function writes the common wavenumber value to the corresponding register in DVK. */
static void commonWaveNumSelect(int commonWaveNum) {
    switch (commonWaveNum) {
        case 0:
            NS_sendSpi("20, 0, 0", NULL); //no. of points = 0
            return;

        case 1:
            NS_sendSpi("20, 65, 0", NULL); //no. of points = 65
            return;

        case 2:
            NS_sendSpi("20, 129, 0", NULL); //no. of points = 129
            return;

        case 3:
            NS_sendSpi("20, 1, 1", NULL); //no. of points = 257
            return;

        case 4:
            NS_sendSpi("20, 1, 2", NULL); //no. of points = 513
            return;

        case 5:
            NS_sendSpi("20, 0, 4", NULL); //no. of points = 1024
            return;

        case 6:
            NS_sendSpi("20, 0, 8", NULL); //no. of points = 2048
            return;

        case 7:
            NS_sendSpi("20, 0, 16", NULL); //no. of points = 4096
            return;
    }
}

/* This function blocks the flow till the data ready pin becomes "1" or it time out. */
ScannerError_t readDataReady(uint32_t timeout_ms) {

    TickType_t tasktime_Start = xTaskGetTickCount();
    int source_delay;

    if (timeout_ms < (persistant_source_settings.t2max * 100)) {
        source_delay = (persistant_source_settings.t2C1 * 50) + (persistant_source_settings.t1 * 50);
    } else {
        source_delay =
                (int) (((double) ((double) persistant_source_settings.t2C2 / (double) 100)) * (double) timeout_ms) +
                (persistant_source_settings.t1 * 50);
    }
    timeout_ms += source_delay;
    while ((xTaskGetTickCount() - tasktime_Start) < ((MINIMUM_TIMEOUT_MS + timeout_ms) / portTICK_RATE_MS)) {
        if (gpio_get_level((gpio_num_t) IO_NS_DRDY)) {
            ScannerError_t statusCode;
            NS_DEFAULT_SPI_RX[0] = 0;
            NS_sendSpi("187, 0", NS_DEFAULT_SPI_RX);            // read statusCode
            statusCode = NS_DEFAULT_SPI_RX[0];
            ESP_LOGI(TAG,"Status Code %u",statusCode);
            return statusCode;
        }
        vTaskDelay(100 / portTICK_RATE_MS); //delay 100 ms to release the task
    }
    return NS_DATA_READY_TIMEOUT;
}

uint32_t calculateTimeOutUsingScanTime(uint32_t timeout_ms) {
    int source_delay;

    if (timeout_ms < (persistant_source_settings.t2max * 100)) {
        source_delay = (persistant_source_settings.t2C1 * 50) + (persistant_source_settings.t1 * 50);
    } else {
        source_delay =
                (int) (((double) ((double) persistant_source_settings.t2C2 / (double) 100)) * (double) timeout_ms) +
                (persistant_source_settings.t1 * 50);
    }
    timeout_ms += source_delay;
    return timeout_ms;
}


/* Read Module ID flow handler. */
ScannerError_t readModuleID(P3RequestPacket_t *rp, P3Response_t *resp) {
    ESP_LOGI(TAG, "readModuleID (operation ID = %d)", rp->operationID);

    NS_sendSpi("12, 0", NULL); //AutoincB = 0
    NS_sendSpi("128, z8", NS_DEFAULT_SPI_RX);//module ID

    memcpy(&resp->data.moduleID, NS_DEFAULT_SPI_RX, sizeof(resp->data.moduleID));
    return NoError;
}

/* Run PSD flow handler. */
ScannerError_t runPSD(P3RequestPacket_t *rp, P3Response_t *resp) {
    char sbuf[25];
    ScannerError_t err;
    uint16_t trial_count = 0;
    ESP_LOGI(TAG, "runPSD (operation ID = %d)", rp->operationID);
    if((err = checkLightSourceRelatedErrors()) != NoError){
        return err;
    }
    if (rp->operationID == OPID_RUN_ABSRB) {
        if(background.type == SPECTRAL_TYPE_INVALID){
            return NS_BACKGROUND_NOT_FOUND;
        }
        if(!checkBackgroundValidity(resp)){
            return NS_BACKGROUND_NOT_VALID;
        }
        if(!checkBackgroundTemperature(resp)){
            return NS_BACKGROUND_TEMPERATURE_EXCEEDED;
        }
    }
    ESP_LOGI(TAG, "Run PSD commonWaveNum %d", rp->U.measurement.commonWaveNum);
    ESP_LOGI(TAG, "Run PSD zeroPadding %d", rp->U.measurement.zeroPadding);
    ESP_LOGI(TAG, "Run PSD mode %d", rp->U.measurement.mode);
    ESP_LOGI(TAG, "Run PSD apodizationSel %d", rp->U.measurement.apodizationSel);
    ESP_LOGI(TAG, "Run PSD opticalGain %d", rp->U.measurement.opticalGain);
    ESP_LOGI(TAG, "Run PSD scanTime %d", rp->U.measurement.scanTime);

    NS_sendSpi("12, 0", NULL);//AutoincB = 0

    commonWaveNumSelect(rp->U.measurement.commonWaveNum);

    sprintf(sbuf, "13, %d", ((rp->U.measurement.commonWaveNum > 0) << 7)
                            + (rp->U.measurement.zeroPadding << 5)
                            + (rp->U.measurement.mode << 1));
    //Common wave(bit7) - 1xPadding(bit6-5) - mode(bit4-1) - resolution(bit0) (Note: resolution not supported)
    NS_sendSpi(sbuf, NULL);

    sprintf(sbuf, "14, %d", (rp->U.measurement.apodizationSel << 3) + (rp->U.measurement.opticalGain << 1));
    //Absorbance(bit6) - WinSel(bit5-3) - GainSel(bit2-1) - unit(bit0) (Note: Absorbance and Unit aren't supported)
    NS_sendSpi(sbuf, NULL);

    sprintf(sbuf, "16, %d, %d, %d", rp->U.measurement.scanTime & 0xFF,
            (rp->U.measurement.scanTime >> 8) & 0xFF, (rp->U.measurement.scanTime >> 16) & 0xFF);
    NS_sendSpi(sbuf, NULL); //scanTime
    do{
        NS_sendSpi("24, 1", NULL);//run psd
        // ESP_LOGI(TAG,"Waiting Data Ready");
        centralManager_oled_notify_lightsOff(calculate_lights_off_time(rp->U.measurement.scanTime));
        err = readDataReady(rp->U.measurement.scanTime);
        ESP_LOGI(TAG, "Run PSD No. %d with Status %d",trial_count, err);
    }while((err != NoError && err < NS_BACKGROUND_NOT_VALID) && ++trial_count < MAX_TRIAL_COUNT);

    ESP_LOGI(TAG, "Run PSD Status %d", err);
    if (err != NoError)
        return err;

    NS_sendSpi("150, 0, 0", NS_DEFAULT_SPI_RX);//psd length
    memcpy(&resp->data.spectralData.psdLength, NS_DEFAULT_SPI_RX, sizeof(resp->data.spectralData.psdLength));
    ESP_LOGI(TAG, "PSD length = %d", resp->data.spectralData.psdLength);
    if(resp->data.spectralData.psdLength == 0){
        return NS_PSD_LENGTH_INVALID;
    }

    streamSpectralData(resp);
    postSpectralProcessing(resp);
    ESP_LOGI(TAG, "runPSD Done");
    return NoError;
}

/* Run Gain Adjustment flow handler. */
ScannerError_t runGainAdj(P3RequestPacket_t *rp, P3Response_t *resp) {
    ESP_LOGI(TAG, "runGainAdj (operation ID = %d)", rp->operationID);

    NS_sendSpi("12, 0", NULL);//AutoincB = 0
    NS_sendSpi("24, 5", NULL);//run gain Adjusment

    ScannerError_t err = readDataReady(0);
    if (err != NoError)
        return err;

    NS_sendSpi("222, 0, 0", NS_DEFAULT_SPI_RX);//Read gain value
    memcpy(&resp->data.gainData, NS_DEFAULT_SPI_RX, sizeof(resp->data.gainData));

    return NoError;
}

/* Burn Gain Adjusment Settings flow handler. */
ScannerError_t BurnGain(P3RequestPacket_t *rp, P3Response_t *resp) {
    ESP_LOGI(TAG, "BurnGain (operation ID = %d)", rp->operationID);

    NS_sendSpi("12, 0", NULL);//AutoincB = 0
    NS_sendSpi("24, 13", NULL);//burn gain settings

    return readDataReady(0);
}

/* Burn Self Correction Parameters flow handler. */
ScannerError_t BurnSelf(P3RequestPacket_t *rp, P3Response_t *resp) {
    ESP_LOGI(TAG, "BurnSelf (operation ID = %d)", rp->operationID);

    NS_sendSpi("12, 0", NULL);//AutoincB = 0
    NS_sendSpi("24, 11", NULL);//burn self corr

    return readDataReady(0);
}

/* Burn Wavelength Correction Parameters flow handler. */
ScannerError_t BurnWLN(P3RequestPacket_t *rp, P3Response_t *resp) {
    ESP_LOGI(TAG, "BurnWLN (operation ID = %d)", rp->operationID);

    NS_sendSpi("12, 0", NULL);//AutoincB = 0
    NS_sendSpi("24, 12", NULL);//burn WLN corr

    return readDataReady(0);
}

/* Run Self Correction flow handler. */
ScannerError_t runSelfCorr(P3RequestPacket_t *rp, P3Response_t *resp) {
    char sbuf[25];

    ESP_LOGI(TAG, "runSelfCorr (operation ID = %d)", rp->operationID);

    NS_sendSpi("12, 0", NULL);//AutoincB = 0

    commonWaveNumSelect(rp->U.measurement.commonWaveNum);

    sprintf(sbuf, "13, %d", ((rp->U.measurement.commonWaveNum > 0) << 7)
                            + (rp->U.measurement.zeroPadding << 5)
                            + (rp->U.measurement.mode << 1));
    //Common wave(bit7) - 1xPadding(bit6-5) - mode(bit4-1) - resolution(bit0) (Note: resolution not supported)
    NS_sendSpi(sbuf, NULL);

    sprintf(sbuf, "14, %d", (rp->U.measurement.apodizationSel << 3) + (rp->U.measurement.opticalGain << 1));
    //Absorbance(bit6) - WinSel(bit5-3) - GainSel(bit2-1) - unit(bit0) (Note: Absorbance and Unit aren't supported)
    NS_sendSpi(sbuf, NULL);

    sprintf(sbuf, "16, %d, %d, %d", rp->U.measurement.scanTime & 0xFF,
            (rp->U.measurement.scanTime >> 8) & 0xFF, (rp->U.measurement.scanTime >> 16) & 0xFF);
    NS_sendSpi(sbuf, NULL); //scanTime

    NS_sendSpi("24, 2", NULL);//run Self correction

    return readDataReady(rp->U.measurement.scanTime);
}

/* Run Wavelength Correction background flow handler. */
ScannerError_t runWavelengthCorrBG(P3RequestPacket_t *rp, P3Response_t *resp) {
    char sbuf[25];

    ESP_LOGI(TAG, "runWavelengthCorrBG (operation ID = %d)", rp->operationID);
    ESP_LOGI(TAG, "Run PSD commonWaveNum %d", rp->U.measurement.commonWaveNum);
    ESP_LOGI(TAG, "Run PSD zeroPadding %d", rp->U.measurement.zeroPadding);
    ESP_LOGI(TAG, "Run PSD mode %d", rp->U.measurement.mode);
    ESP_LOGI(TAG, "Run PSD apodizationSel %d", rp->U.measurement.apodizationSel);
    ESP_LOGI(TAG, "Run PSD opticalGain %d", rp->U.measurement.opticalGain);
    ESP_LOGI(TAG, "Run PSD scanTime %d", rp->U.measurement.scanTime);

    NS_sendSpi("12, 0", NULL);//AutoincB = 0

    commonWaveNumSelect(rp->U.measurement.commonWaveNum);

    sprintf(sbuf, "13, %d", ((rp->U.measurement.commonWaveNum > 0) << 7)
                            + (rp->U.measurement.zeroPadding << 5)
                            + (rp->U.measurement.mode << 1));
    //Common wave(bit7) - 1xPadding(bit6-5) - mode(bit4-1) - resolution(bit0) (Note: resolution not supported)
    NS_sendSpi(sbuf, NULL);

    sprintf(sbuf, "14, %d", (rp->U.measurement.apodizationSel << 3) + (rp->U.measurement.opticalGain << 1));
    //Absorbance(bit6) - WinSel(bit5-3) - GainSel(bit2-1) - unit(bit0) (Note: Absorbance and Unit aren't supported)
    NS_sendSpi(sbuf, NULL);

    sprintf(sbuf, "16, %d, %d, %d", rp->U.measurement.scanTime & 0xFF,
            (rp->U.measurement.scanTime >> 8) & 0xFF, (rp->U.measurement.scanTime >> 16) & 0xFF);
    NS_sendSpi(sbuf, NULL); //scanTime

    NS_sendSpi("24, 3", NULL);//run WVL correction BG

    return readDataReady(rp->U.measurement.scanTime);
}

/* Run Wavelength Correction Sample flow handler. */
ScannerError_t runWavelengthCorr(P3RequestPacket_t *rp, P3Response_t *resp) {
    char sbuf[25];

    ESP_LOGI(TAG, "runWavelengthCorr (operation ID = %d)", rp->operationID);
    for (size_t i = 0; i < 5; i++) {
        ESP_LOGI(TAG, "Calibration Wells [%d] = %d", i, calibration_wells[i]);
    }

    NS_sendSpi("12, 0", NULL);//AutoincB = 0

    commonWaveNumSelect(rp->U.measurement.commonWaveNum);

    sprintf(sbuf, "13, %d", ((rp->U.measurement.commonWaveNum > 0) << 7)
                            + (rp->U.measurement.zeroPadding << 5)
                            + (rp->U.measurement.mode << 1));
    //Common wave(bit7) - 1xPadding(bit6-5) - mode(bit4-1) - resolution(bit0) (Note: resolution not supported)
    NS_sendSpi(sbuf, NULL);

    sprintf(sbuf, "14, %d", (rp->U.measurement.apodizationSel << 3) + (rp->U.measurement.opticalGain << 1));
    //Absorbance(bit6) - WinSel(bit5-3) - GainSel(bit2-1) - unit(bit0) (Note: Absorbance and Unit aren't supported)
    NS_sendSpi(sbuf, NULL);

    sprintf(sbuf, "16, %d, %d, %d", rp->U.measurement.scanTime & 0xFF,
            (rp->U.measurement.scanTime >> 8) & 0xFF, (rp->U.measurement.scanTime >> 16) & 0xFF);
    NS_sendSpi(sbuf, NULL); //scanTime

    for (size_t i = 0; i < WELLS_SIZE; i++) {
        sprintf(sbuf, "%d, %d, %d, %d, %d", (64 + i * 4), calibration_wells[i] & 0xFF,
                (calibration_wells[i] >> 8) & 0xFF, (calibration_wells[i] >> 16) & 0xFF,
                (calibration_wells[i] >> 24) & 0xFF);
        NS_sendSpi(sbuf, NULL); //calibrationWells
        ESP_LOGI(TAG, "%s", sbuf);
    }

    NS_sendSpi("24, 4", NULL);//run WVL correction sample

    return readDataReady(rp->U.measurement.scanTime);
}

/* Restore Factory Settings flow handler. */
ScannerError_t restoreDefault(P3RequestPacket_t *rp, P3Response_t *resp) {
    ESP_LOGI(TAG, "restoreDefault (operation ID = %d)", rp->operationID);

    NS_sendSpi("12, 0", NULL);//AutoincB = 0
    NS_sendSpi("24, 15", NULL);//restore to default

    return readDataReady(rp->U.measurement.scanTime);
}

/* Set Light Source Settings flow handler. */
ScannerError_t sourceSettings(P3RequestPacket_t *rp, P3Response_t *resp) {
    char sbuf[25];

    ESP_LOGI(TAG, "sourceSettings (lampsCount = %d)", rp->U.sourceSettings.lampsCount);
    ESP_LOGI(TAG, "sourceSettings (lampsSelect = %d)", rp->U.sourceSettings.lampsSelect);
    ESP_LOGI(TAG, "sourceSettings (deltaT = %d)", rp->U.sourceSettings.deltaT);
    ESP_LOGI(TAG, "sourceSettings (t1 = %d)", rp->U.sourceSettings.t1);
    ESP_LOGI(TAG, "sourceSettings (t2C1 = %d)", rp->U.sourceSettings.t2C1);
    ESP_LOGI(TAG, "sourceSettings (t2C2 = %d)", rp->U.sourceSettings.t2C2);
    ESP_LOGI(TAG, "sourceSettings (t2max = %d)", rp->U.sourceSettings.t2max);

    NS_sendSpi("12, 0", NULL);//AutoincB = 0

    sprintf(sbuf, "41, %d", rp->U.sourceSettings.lampsCount);
    NS_sendSpi(sbuf, NULL);// lamps count

    sprintf(sbuf, "42, %d", rp->U.sourceSettings.lampsSelect);
    NS_sendSpi(sbuf, NULL);// lamp select

    sprintf(sbuf, "43, %d", rp->U.sourceSettings.deltaT);
    NS_sendSpi(sbuf, NULL);// lamp delta t

    sprintf(sbuf, "44, %d", rp->U.sourceSettings.t1);
    NS_sendSpi(sbuf, NULL);//lamp t1

    sprintf(sbuf, "45, %d", rp->U.sourceSettings.t2C1);
    NS_sendSpi(sbuf, NULL);//lamp t2_c1

    sprintf(sbuf, "46, %d", rp->U.sourceSettings.t2C2);
    NS_sendSpi(sbuf, NULL);//lamp t2_c2

    sprintf(sbuf, "47, %d", rp->U.sourceSettings.t2max);
    NS_sendSpi(sbuf, NULL);//lamp t2_tmax
    persistant_source_settings = rp->U.sourceSettings;

    return NoError;
}

/* Read DVK Software version flow handler. */
ScannerError_t readSoftwareVersion(P3RequestPacket_t *rp, P3Response_t *resp) {
    ESP_LOGI(TAG, "readSoftwareVersion (operation ID = %d)", rp->operationID);

    NS_sendSpi("12, 0", NULL); //AutoincB = 0
    NS_sendSpi("164, z4", NS_DEFAULT_SPI_RX); //FW version

    memcpy(&resp->data.fwVersion, NS_DEFAULT_SPI_RX, sizeof(resp->data.fwVersion));

    return NoError;
}

/* Run Sleep Action flow handler. */
ScannerError_t runSleepAction(P3RequestPacket_t *rp, P3Response_t *resp) {
    NS_sendSpi("12, 0", NULL);//AutoincB = 0
    NS_sendSpi("24, 6", NULL);//run sleep action

    vTaskDelay(100 / portTICK_RATE_MS); //delay 100 ms to ensure entering sleep mode

    return NoError;
}

/* Run Wake up Action flow handler. */
ScannerError_t
runWakeUpAction(P3RequestPacket_t *rp, P3Response_t *resp) {    //This function requires WAKE_UP_PIN to wake up

    // io_write(WAKE_UP_PIN, 1);

    // usleep(1000);

    // io_write(WAKE_UP_PIN, 0);

    // bufLength = 1;
    // buffer[0] = readDataReady(0);
    // printf("Status = %d\n", buffer[0]);

    return NoError;
}

/* Run Power off flow handler. */
ScannerError_t runPowerOff(P3RequestPacket_t *rp,
                           P3Response_t *resp) {   //This function requires LDO_EN and DCDC_EN pins to be able to power off

    // io_write(UI_CSB, 0);
    // io_write(LDO_EN, 0);
    // io_write(DCDC_EN, 0);

    // buffer[0] = 0;
    // bufLength = 1;
    // printf("Status = %d\n", buffer[0]);

    return NoError;
}

/* Run Power on flow handler. */
ScannerError_t runPowerOn(P3RequestPacket_t *rp,
                          P3Response_t *resp) {    //This function requires LDO_EN and DCDC_EN pins to be able to power on

    // io_write(LDO_EN, 1);
    // io_write(DCDC_EN, 1);
    // io_write(UI_CSB, 1);

    // bufLength = 1;
    // buffer[0] = readDataReady(0);
    // printf("Status = %d\n", buffer[0]);

    return NoError;
}

/* Set Gain Settings flow handler. */
ScannerError_t setGainSettings(P3RequestPacket_t *rp, P3Response_t *resp) {
    char sbuf[25];

    ESP_LOGI(TAG, "setGainSettings (operation ID = %d)", rp->operationID);

    NS_sendSpi("12, 0", NULL);//AutoincB = 0

    sprintf(sbuf, "92, %d, %d", rp->U.opticalGainVal & 0xFF,
            (rp->U.opticalGainVal >> 8) & 0xFF);
    NS_sendSpi(sbuf, NULL); //write gain value

    return NoError;
}

/* Inject External Apodization Window flow handler. */
ScannerError_t injectExternalWindow(P3RequestPacket_t *rp, P3Response_t *resp) {
    ESP_LOGI(TAG, "injectExternalWindow (operation ID = %d)", rp->operationID);

    NS_sendSpi("12, 0", NULL); //AutoincB = 0
    NS_sendSpi("84, 20, 0", NULL); //write length = 20 samples

    NS_sendSpi("24, 7", NULL); //write window request
    ScannerError_t err = readDataReady(0);
    if (err != NoError)
        return err;

    NS_sendSpi("12, 1", NULL); //Auto_inc_B = 1
    {
        //write window
        // tx[0] = 86;
        // memcpy(&tx[1], calibration_wells, sizeof(calibration_wells));
        // spi_send_receive(tx, sizeof(calibration_wells) + 1);
    }
    err = readDataReady(0);
    if (err != NoError)
        return err;

    NS_sendSpi("12, 0", NULL); //Auto_inc_B = 0

    return NoError;
}

ScannerError_t runTemperature(P3RequestPacket_t *rp, P3Response_t *resp) {
    ScannerError_t err = NoError;

    ESP_LOGI(TAG, "runTemperature (operation ID = %d)", rp->operationID);

    NS_sendSpi("12, 0", NULL); //AutoincB = 0

    NS_sendSpi("100, 255, 0, 255, 0, 255, 0, 255, 0", NULL); //Unlock Code
    NS_sendSpi("24, 165", NULL); //Run Unlock
    err = readDataReady(0);
    if (err != NoError) {
        NS_sendSpi("108, 8", NULL); //Run Lock
        readDataReady(0);
        return err;
    }

    NS_sendSpi("108, 17", NULL); //run temp
    err = readDataReady(0);
    if (err != NoError) {
        NS_sendSpi("108, 8", NULL); //Run Lock
        readDataReady(0);
        return err;
    }

    NS_sendSpi("108, 8", NULL); //Run Lock
    err = readDataReady(0);

    NS_sendSpi("161, z3", NS_DEFAULT_SPI_RX); //FW version
    resp->data.temperature = 0;
    memcpy(&resp->data.temperature, NS_DEFAULT_SPI_RX, 3);
    ESP_LOGI(TAG, "Temperature is %d", resp->data.temperature);


    return err;
}

ScannerError_t injectProgram(P3RequestPacket_t *rp, P3Response_t *resp) {
    // RequestPacket* rp = (RequestPacket*)vrp;
    // char sbuf[25];
    // int trials;

    // printf("%d\n",rp->U.operation);
    // printf("%d\n",rp->U.resolution);
    // printf("%d\n",rp->U.mode);
    // printf("%d\n",rp->U.zeroPadding);
    // printf("%d\n",rp->U.scanTime);
    // printf("%d\n",rp->U.measurement.commonWaveNum);
    // printf("%d\n",rp->U.opticalGain);
    // printf("%d\n",rp->U.apodizationSel);


    // bufLength = 1;

    // NS_sendSpi("12, 0");//AutoincB = 0

    // for(trials = 0; trials < 10; trials++)
    // {
    // 	NS_sendSpi("12, 0");//AutoincB = 0
    // 	sprintf(sbuf,"84, %d, %d", PROGRAM_LENGTH & 0xFF, (PROGRAM_LENGTH >> 8)& 0xFF);
    // 	NS_sendSpi(sbuf); //write length
    // 	NS_sendSpi("24, 22");//write program request
    // 	buffer[0] = readDataReady(0);
    // 	if(buffer[0] != 0)
    // 		return 0;

    // 	NS_sendSpi("12, 1"); //Auto_inc_B = 1
    // 	{//write program
    // 		tx[0] = 86;
    // 		memcpy(&tx[1], rp->U.sampleData, PROGRAM_LENGTH * 4);
    // 		spi_send_receive(tx, (PROGRAM_LENGTH * 4) + 1);
    // 	}

    // 	buffer[0] = readDataReady(0);
    // 	printf("Status = %d\n", buffer[0]);

    // 	if(buffer[0] == 49)  // repeat if CRC not matched
    // 		continue;
    // 	else
    // 		break;
    // }

    // NS_sendSpi("12, 0"); //Auto_inc_B = 0
    // if(buffer[0] != 0)
    // 	return 0;

    return NoError;
}

ScannerError_t programM7(P3RequestPacket_t *rp, P3Response_t *resp) {
    // RequestPacket* rp = (RequestPacket*)vrp;
    // char sbuf[25];

    // printf("%d\n",rp->U.operation);
    // printf("%d\n",rp->U.resolution);
    // printf("%d\n",rp->U.mode);
    // printf("%d\n",rp->U.zeroPadding);
    // printf("%d\n",rp->U.scanTime);
    // printf("%d\n",rp->U.measurement.commonWaveNum);
    // printf("%d\n",rp->U.opticalGain);
    // printf("%d\n",rp->U.apodizationSel);
    // printf("%d\n",calibration_wells[0]);


    // NS_sendSpi("12, 0");//AutoincB = 0

    // sprintf(sbuf,"64, %d, %d, %d, %d", calibration_wells[0] & 0xFF,
    // 	(calibration_wells[0] >> 8)& 0xFF,
    // 	(calibration_wells[0] >> 16)& 0xFF,
    // 	(calibration_wells[0] >> 24)& 0xFF);
    // NS_sendSpi(sbuf); //calibrationWells

    // bufLength = 1;

    // NS_sendSpi("24, 21");//Run flash program

    // int pinStatus = 0;
    // int intrpStatus = 0;
    // int timeOut = 0;
    // do{
    // 	timeOut++;
    // 	io_read(DATA_READY, &pinStatus);
    // 	io_read(INTERRUPT, &intrpStatus);
    // 	if(timeOut >= 60)
    // 		break;
    // 	sleep(1);
    // }while(!pinStatus && !intrpStatus);


    // if(intrpStatus)
    // {
    // 	printf("interrupt\n");
    // 	buffer[0] = readDataReady(0);
    // 	return 0;
    // }
    // else if(pinStatus)
    // {
    // 	buffer[0] = 0;
    // 	printf("Done\n");
    // }
    // else
    // {
    // 	printf("timeOut\n");
    // 	buffer[0] = -1;
    // }

    return NoError;
}

ScannerError_t writeCalibrationWells1(P3RequestPacket_t *rp, P3Response_t *resp) {
    ESP_LOGI(TAG, "writeCalibrationWells1 (operation ID = %d)", rp->operationID);
    memcpy(&calibration_wells[0], rp->U.calibWells_1, sizeof(rp->U.calibWells_1));
    return NoError;
}

ScannerError_t writeCalibrationWells2(P3RequestPacket_t *rp, P3Response_t *resp) {
    ESP_LOGI(TAG, "writeCalibrationWells2 (operation ID = %d)", rp->operationID);
    memcpy(&calibration_wells[3], rp->U.calibWells_2, sizeof(rp->U.calibWells_2));
    return NoError;
}