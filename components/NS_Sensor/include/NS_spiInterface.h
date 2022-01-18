#ifndef NS_SPIINTERFACE_H
#define NS_SPIINTERFACE_H

#include <stdint.h>

#define MAX_SPI_BYTES       (32 * 1024 + 2)     //Size of maximum length of PSD plus its spi address and dummy byte
#define NS_SPI_CLK_SPEED    (500 * 1000)        //SPI CLK speed = 500KHz
#define NS_MAX_NUM_DUMMY_BYTES  (1)                 //0 or 1

/**
 * @brief           This function used to setup SPI interface
 *                  to communicate with NS_sensor.
 *
 * @param[in]       
 * 
 */
void NS_spiSetup(uint8_t barq_installed);

/**
 * @brief           This function used to send data over SPI 
 *                  and get the response.
 *
 * @param[in]       tx_rx: Pointer to tx and rx buffer
 *                  length: length of the data 
 * 
 */
void NS_spiSendReceive(uint8_t* tx_rx, int length);

/**
 * @brief           This function takes a char array describing 
 *                  what to send over SPI. It decodes the array,
 *                  then creates a corresponding SPI frame.
 *
 * @param[in]       str: Pointer to the string
 * @param[in]       rx: Pointer to a rx buffer. The buffer should has 
 *                  one extra byte plus number of extra bytes equals to 
 *                  NS_MAX_NUM_DUMMY_BYTES to be used in the tx buffer, and
 *                  the extra bytes should be before the rx pointer. The
 *                  rx pointer can be NULL in case of the receive data is 
 *                  unimportant.
 * 
 * @return          length of data sent.
 */
uint16_t NS_sendSpi(char* str, uint8_t* rx);

#endif