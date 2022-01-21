#include <string.h>
#include "NS_sensor.h"
#include "centralManager.h"
#include "centralManagerSystemIOs.h"
#include "NS_operations.h"

#include "driver/gpio.h"

#define TAG "NS_SENSOR"

SpectralData_t background = {
        .spectralPTR = (double *) NS_BACKGROUNDDATA,
        .type = SPECTRAL_TYPE_INVALID,
};
static uint64_t _moduleID = 0;

/*
 * Array of all NS_operations sorted according to operation IDs 
 */
static ScannerError_t (*const NS_operationFlows[])(P3RequestPacket_t *, P3Response_t *) = {
        0, 0, 0,
        runPSD,                         //3
        runPSD,                         //4
        runPSD,                         //5
        runGainAdj,                     //6
        BurnGain,                       //7
        BurnSelf,                       //8
        BurnWLN,                        //9
        runSelfCorr,                    //10
        runWavelengthCorrBG,            //11
        runWavelengthCorr,              //12
        restoreDefault,                 //13
        0, 0, 0, 0, 0, 0, 0, 0,
        sourceSettings,                 //22
        0, 0, 0, 0,
        setGainSettings,                //27
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0,
        writeCalibrationWells1,          //90
        writeCalibrationWells2,          //91
        0, 0, 0, 0,
        runTemperature,                 //96
};

void NS_sensorInitIOs() {

    gpio_config_t conf = {
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_DISABLE,
            .pin_bit_mask = (1ULL << IO_NS_DRDY),
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&conf));
    gpio_config_t conf2 = {
            .mode = GPIO_MODE_OUTPUT,
            .intr_type = GPIO_INTR_DISABLE,
            .pin_bit_mask = (1ULL << IO_NS_ENABLE),
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&conf2));
    gpio_set_level(IO_NS_ENABLE, 1);


//    gpio_pad_select_gpio(IO_NS_DRDY);
//    gpio_pad_select_gpio(IO_NS_ENABLE);
//    gpio_set_direction(IO_NS_DRDY, GPIO_MODE_INPUT);
//    gpio_set_direction(IO_NS_ENABLE, GPIO_MODE_OUTPUT);

}

void NS_sensorInit(uint8_t barq_installed) {
    P3Response_t resp;
    P3RequestPacket_t rp = {
            .operationID = 0,
    };

    // NS_sensorInitIOs();
    NS_spiSetup(barq_installed);

    uint8_t status = readDataReady(0);
    ESP_LOGI(TAG, "NS_sensorInit status = %d", status);

    vTaskDelay(500 / portTICK_RATE_MS); // 500 milliseconds delay
    //Read Temperature
    rp.operationID = OPID_RUN_Temp;
    runTemperature(&rp, &resp);
    NS_temperature.tAIFReading = resp.data.temperature;
    readModuleID(&rp, &resp);
    _moduleID = resp.data.moduleID;
    ESP_LOGI(TAG, "Module id is %llu, Temperature is %d", _moduleID, NS_temperature.tAIFReading);
}

void NS_setupSettings(ScannerSettings_t *scannerSettings, sourceSettings_t *sourceSettingsObject) {
    P3RequestPacket_t rp = {
            .operationID = OPID_SET_SRC_SETT,
            .U.sourceSettings = *sourceSettingsObject,
    };
    sourceSettings(&rp, NULL);

    rp.operationID = OPID_SET_OPTCL_SET;
    rp.U.opticalGainVal = scannerSettings->gainvalue;
    setGainSettings(&rp, NULL);
}

void NS_getScannerID(ScannerID_t *scannerID) {
    snprintf((char *) &scannerID->id, sizeof(ScannerID_t), "NS-Scanner_%lld", _moduleID);
}

TemperatureReading NS_getTemperature() {
    return NS_temperature;
}

void NS_set_temperature(TemperatureReading temperature_reading) {
    NS_temperature = temperature_reading;
}

void NS_getSensorID(uint64_t *sensorID) {
    *sensorID = _moduleID;
}

void NS_setBackground(SpectralData_t *bckgrnd) {
    memcpy(&background, bckgrnd, sizeof(SpectralData_t));
}

//Never Call this from the context of any NS operation, it will stuck waiting for the available task bit
uint32_t NS_run_temperature() {
    P3RequestPacket_t temperature_request = {
            .operationID = OPID_RUN_Temp,
    };
    ResponsePacket_t response;
    xEventGroupWaitBits(availableTasks, NS_REQEUST_TASK_BIT,
                        pdTRUE,  //bit should be cleared before returning.
                        pdFALSE, portMAX_DELAY);
    NS_run_operation(&temperature_request, &response);
    xEventGroupSetBits(availableTasks,
                       NS_REQEUST_TASK_BIT); //Set the bit to signal that we can call NS_Tasks again
    return response.packet.p3Response.data.temperature;
}

void NS_run_operation(P3RequestPacket_t *rp, ResponsePacket_t *responsePacket) {

    P3Response_t *resp = &responsePacket->packet.p3Response;
    // The spectral settings in response can be overwritten in case the reading is not spectral reading
    memcpy(&resp->data.spectralData.settings, &rp->U.measurement, sizeof(MeasurementSettings_t));
    ESP_LOGI(TAG, "Running Operation %d", rp->operationID);
    //Run temperature does nothing with rp, I just added it for consistency
    runTemperature(rp, resp);
    resp->data.spectralData.temperature_during_scan = resp->data.temperature;

    switch (rp->operationID) {
        case OPID_RUN_PSD:
            resp->data.spectralData.spectralPTR = (double *) NS_SPECTRALDATA;
            resp->data.spectralData.type = SPECTRAL_TYPE_PSD;
            break;

        case OPID_RUN_BKGND:
            resp->data.spectralData.spectralPTR = (double *) NS_BACKGROUNDDATA;
            resp->data.spectralData.type = SPECTRAL_TYPE_BCKGRND;
            break;

        case OPID_RUN_ABSRB:
            resp->data.spectralData.spectralPTR = (double *) NS_SPECTRALDATA;
            resp->data.spectralData.type = SPECTRAL_TYPE_SPECTRUM;
            break;

        default:
            break;
    }
    ESP_LOGI(TAG, "Operation Id %d", rp->operationID);

    if (rp->operationID < sizeof(NS_operationFlows) / sizeof(void *)) {
        responsePacket->statusCode = NS_operationFlows[rp->operationID](rp, resp);
    } else if (rp->operationID == OPID_CHECK_TEMP) {
        responsePacket->statusCode = check_temperature();
    }
}

void NS_requestTask(void *parameters) {
    P3RequestPacket_t rp;
    ESP_LOGI(TAG, "NS_RequestTask");

    memcpy(&rp, parameters, sizeof(rp));
    xEventGroupSetBits(taskReceivedData, NS_REQEUST_TASK_BIT); //Set the bit after copying the parameters

    ResponsePacket_t responsePacket = {
            .packetType = RESPONSEPACKET_P3,
            .statusCode = NoError,
            .packet.p3Response = {
                    .operationID = rp.operationID
            },
    };
    NS_run_operation(&rp, &responsePacket);
//    P3Response_t *resp = &responsePacket.packet.p3Response;
//    // The spectral settings in response can be overwritten in case the reading is not spectral reading
//    memcpy(&resp->data.spectralData.settings, &rp.U.measurement, sizeof(MeasurementSettings_t));
//
//    switch (rp.operationID) {
//        case OPID_RUN_PSD:
//            resp->data.spectralData.spectralPTR = (double *) NS_SPECTRALDATA;
//            resp->data.spectralData.type = SPECTRAL_TYPE_PSD;
//            break;
//
//        case OPID_RUN_BKGND:
//            resp->data.spectralData.spectralPTR = (double *) NS_BACKGROUNDDATA;
//            resp->data.spectralData.type = SPECTRAL_TYPE_BCKGRND;
//            break;
//
//        case OPID_RUN_ABSRB:
//            resp->data.spectralData.spectralPTR = (double *) NS_SPECTRALDATA;
//            resp->data.spectralData.type = SPECTRAL_TYPE_SPECTRUM;
//            break;
//
//        default:
//            break;
//    }
//    ESP_LOGI(TAG, "Operation Id %d", rp.operationID);
//
//    if (rp.operationID < sizeof(NS_operationFlows) / sizeof(void *))
//        responsePacket.statusCode = NS_operationFlows[rp.operationID](&rp, resp);
    xQueueSend(responseQueue, (void *) &responsePacket, portMAX_DELAY);
    ESP_LOGI(TAG, "Remaining stack %d bytes", uxTaskGetStackHighWaterMark(NULL));
    xEventGroupSetBits(availableTasks, NS_REQEUST_TASK_BIT); //Set the bit before deleting the task
    vTaskDelete(NULL);  //Delete itself
    while (1);           //This task should be deleted after finishing
}

uint32_t NS_CalculateRequiredTime(P3RequestPacket_t *requestPacket) {
    switch (requestPacket->operationID) {
        case OPID_RUN_PSD:
        case OPID_RUN_ABSRB:
        case OPID_RUN_BKGND:
            return calculateTimeOutUsingScanTime(requestPacket->U.measurement.scanTime);
        default:
            return 0;
    }
}