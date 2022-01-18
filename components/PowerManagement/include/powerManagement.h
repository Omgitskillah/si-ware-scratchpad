#ifndef POWERMANAGEMENT_H
#define POWERMANAGEMENT_H

#include "centralManager.h"

#define PM_I2C_NUM          I2C_NUM_0
#define PM_I2C_FREQ_HZ      20000

#define PM_TASK_PERIOD      (30000 / portTICK_PERIOD_MS)    //30 Seconds

void PM_init(bool initializeGauge, bool writeGoldenFile);
void PM_wakeup_reason_check();
void PM_sleep();
void PM_getBatteryStatus(uint8_t* batteryPercentage, uint32_t* chargerState);
void PM_getGaugeInfo(GaugeInfo_t** infoPTR);
life_time_data_t * power_management_get_life_time_data();
void PM_selfTest(uint32_t *charger_status, uint32_t *gauge_status);

#endif