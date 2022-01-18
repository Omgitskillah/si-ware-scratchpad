#ifndef BLUETOOTHMANAGER_H
#define BLUETOOTHMANAGER_H

#include <stdbool.h>
#include "centralManagerDefs.h"
#include "commLayer.h"

/**
 * @brief           Initialize bluetooth module and bluetoothManager.This function 
 *                  should be called before any other function from bluetoothManager.
 *                  
 *
 * @param[in]       scannerID: The name used for advertising.
 * 
 */
void bluetoothManager_init(ScannerID_t* scannerID);

/**
 * @brief           Set bluetooth connection callback.The callback function will be
 *                  triggered upon any connection or disconnection via bluetooth. The
 *                  callback function should accept a boolean input to indication whether
 *                  the event was connection or disconnection.
 *                  
 *
 * @param[in]       callback: Pointer to the callback function
 * 
 */
void bluetoothManager_setConnectionCallback(CLConnectioncb_t callback);

/**
 * @brief           Start or stop the adversement of the bluetooth.
 *                  
 *
 * @param[in]       start: indicate whether to start or stop.
 * 
 */
void bluetoothManager_advertisement(bool start);


void bluetoothManager_responseTask(void* parameters);

#endif