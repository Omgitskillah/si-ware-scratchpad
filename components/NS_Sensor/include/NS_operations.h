#ifndef NS_OPERATIONS_H
#define NS_OPERATIONS_H

#include "centralManager.h"

#define MINIMUM_TIMEOUT_MS    10000         //10 seconds  
#define WELLS_SIZE 5
#define MAX_TRIAL_COUNT					3
#define DEFAULT_TEMPERATURE_WINDOW      10

extern TemperatureReading NS_temperature;


ScannerError_t readDataReady(uint32_t timeout_ms);

ScannerError_t readModuleID(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t runPSD(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t runBackground(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t runAbsorbance(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t runGainAdj(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t BurnGain(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t BurnSelf(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t BurnWLN(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t runSelfCorr(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t runWavelengthCorrBG(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t runWavelengthCorr(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t restoreDefault(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t sourceSettings(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t readSoftwareVersion(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t setGainSettings(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t injectExternalWindow(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t runTemperature(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t runSleepAction(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t runWakeUpAction(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t runPowerOff(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t runPowerOn(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t injectProgram(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t programM7(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t writeCalibrationWells1(P3RequestPacket_t* rp, P3Response_t* resp);
ScannerError_t writeCalibrationWells2(P3RequestPacket_t* rp, P3Response_t* resp);
uint32_t calculateTimeOutUsingScanTime(uint32_t timeout_ms);
ScannerError_t check_temperature();

#endif