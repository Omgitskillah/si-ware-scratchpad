#ifndef BLUETOOTHMANAGERUTILS_H
#define BLUETOOTHMANAGERUTILS_H

#include <stdint.h>

/**
 * @brief           Set a 16-bytes uuid array using values provided from a 
 *                  string contains the hexdecimal uuid value. This function 
 *                  used only with UUID128.
 *
 * @param[out]      uuid: constant pointer to the uuid array
 * @param[in]       hex_str: pointer to constant string of the hexdecimal uuid  
 *
 * @return
 *
 */
void bluetoothManagerUtils_setUUID128(uint8_t *const uuid, const char *hex_str);


/**
 * @brief           Compare two arrays of UUID128. Each array should contain 16 
 *                  bytes.
 *
 * @param[in]       uuid1: Pointer to the first uuid
 * @param[in]       uuid2: Pointer to the second uuid.
 *
 * @return
 *                  - 0 : uuid1 == uuid2
 *                  - 1 : not equal
 */
uint8_t bluetoothManagerUtils_compareUUID128(uint8_t *const uuid1, uint8_t *const uuid2);


#endif