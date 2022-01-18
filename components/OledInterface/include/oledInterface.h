/*
 * OLED_Interface.h
 *
 *  Created on: Aug 28, 2019
 *      Author: Hossam
 */

#ifndef OLED_INTERFACE_H_
#define OLED_INTERFACE_H_
#include <stdint.h>

typedef enum
{
    BLUETOOTH_ADVERTISEMENT,
    BLUETOOTH_CONNECTED,
    BLUETOOTH_OFF,
} OLED_bluetoothMode_t;

typedef enum
{
    UART_CONNECTED,
    UART_DISCONNECTED,
} OLED_uartStatus_t;

void OLED_setupFirstStage();
void OLED_setupSecondStage(uint16_t initialMemorySize, uint8_t initialBatteryPercentage, uint32_t initialChargerStatus);
void OLED_shutdown(uint8_t emergencyShutdown);

void OLED_bluetooth(OLED_bluetoothMode_t bluetoothMode);
void OLED_uart(OLED_uartStatus_t uartStatus);
void OLED_requestTask(void* parameters);
void OLED_responseTask(void* parameters);
void OLED_progress(uint32_t millis);
void OLED_lightsOff();

#endif /* OLED_INTERFACE_H_ */
