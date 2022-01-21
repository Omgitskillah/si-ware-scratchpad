#ifndef FILESYSTEM_H
#define FILESYSTEM_H

// #include "centralManager.h"

void fileSystem_init(ScannerSettings_t *outSettings, ScannerID_t *scannerID, uint16_t *memorySize,
                     SpectralData_t *background, sourceSettings_t *outSourceSettings,
                     TemperatureReading *temperature_reading, uint32_t *mother_board_version,
                     uint32_t *inactive_timeout, uint8_t *saved_temperature_window);
void fileSystem_init_flash();
bool fileSystem_is_gauge_golden_file_burnt();
void fileSystem_register_gauge_reset();
void fileSystem_save_reset_and_wakeup_reason(esp_reset_reason_t reset_reason,esp_sleep_wakeup_cause_t wakeup_reason);
void fileSystem_register_golden_file_burnt();
void fileSystem_requestTask(void* parameters);
void fileSystem_responseTask(void* parameters);
void fileSystem_save_scan_data(P3Response_t *resp);
ScannerSettings_t * fileSystem_get_scanner_settings();
esp_err_t fileSystem_save_gauge_debug_log(battery_log_data_t * battery_log_data);

#endif