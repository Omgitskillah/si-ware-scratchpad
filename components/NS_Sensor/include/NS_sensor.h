#ifndef NS_SENSOR_H
#define NS_SENSOR_H

#include "NS_spiInterface.h"
#include "centralManager.h"


/**
 * A little note for any one wondering what this arrangement is
 * the sharedStaticMemory is a 64 kb of statically allocated memory,
 * The NUM_DUMMY_BYTES is the amount of bytes between the first SPI byte sent and received,
 * on M7 this is 1 byte, which means I send out the first byte, the second byte and the response comes from the third byte
 * but since this is a hardware SPI on the ESP, we need to discard the first byte received since the m7 didn't actually send anything
 * hence it's a "DUMMY" byte, so if you want your data at address 0x02 you need to tell the ESP to use
 * 0x02 - (1 --> First SPI byte) - (DUMMY_BYTE) which in M7 equates to 1, so I tell ESP that my rx buffer starts @ 0x00 and the
 * data should be available from 0x02, in BARQ however there is no DUMMY BYTES, there is only that first byte, so for rx buffer @ 0x02
 * the data starts at 0x01.
 * So each +1 in these macros is to compensate for that, as each of these addresses is the beginning where I need the data
 * so for NS_SPECTRAData I add 1 and DUMMY Byte, then the SPI_Interface subtracts those before setting it to SPI rx buffer, eventually
 * puts the data in the NS_SPECTRALData address
 * Now I do have a choice, either leave the NUM_DUMMY_BYTES as 1 and let the addresses be defined @ compile time, and handle the
 * transaction at the driver level. or let the NUM_DUMMY_BYTES be a runtime decision and make the addresses not constant
 *
 * I chose the first, because this memory is already reserved, and it will save me from much complication in the code
 *
 */
#define NS_DEFAULT_SPI_RX           (&sharedStaticMemory[NS_MAX_NUM_DUMMY_BYTES + 1])
#define NS_DEFAULT_SPI_RX_MAXLEN    (63 - NS_MAX_NUM_DUMMY_BYTES) //in bytes

#define NS_SPECTRALDATA             (&NS_DEFAULT_SPI_RX[NS_DEFAULT_SPI_RX_MAXLEN + NS_MAX_NUM_DUMMY_BYTES + 1])
#define NS_SPECTRALDATA_MAXLEN      (4098 * 8) // in bytes

#define NS_BACKGROUNDDATA           (&NS_SPECTRALDATA[NS_SPECTRALDATA_MAXLEN + NS_MAX_NUM_DUMMY_BYTES + 1])
#define NS_BACKGROUNDDATA_MAXLEN    (4098 * 8) // in bytes

extern SpectralData_t background;
extern uint8_t NS_temperature_window;

void NS_sensorInitIOs();
void NS_sensorInit(uint8_t barq_installed);

void NS_setupSettings(ScannerSettings_t *scannerSettings, sourceSettings_t *sourceSettingsObject);
void NS_getScannerID(ScannerID_t* scannerID);
TemperatureReading NS_getTemperature();
void NS_set_temperature(TemperatureReading temperature_reading);
void NS_getSensorID(uint64_t* sensorID);
void NS_setBackground(SpectralData_t* bckgrnd);
void NS_run_operation(P3RequestPacket_t *rp, ResponsePacket_t *responsePacket);
uint32_t NS_run_temperature();
void NS_requestTask(void* parameters);
uint32_t NS_CalculateRequiredTime(P3RequestPacket_t * requestPacket);

#endif