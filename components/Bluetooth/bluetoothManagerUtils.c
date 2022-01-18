#include <string.h>

#include "bluetoothManagerUtils.h"
#include "bluetoothManagerDefs.h"


void bluetoothManagerUtils_setUUID128(uint8_t *const uuid, const char *hex_str)
{
    char sub_str[3] = {0};
    for (int i = 0; i < ESP_UUID_LEN_128; i++)
    {
        strncpy(sub_str, &hex_str[(ESP_UUID_LEN_128 - (i + 1)) * 2], 2);
        uuid[i] = (uint8_t)strtol(sub_str, NULL, 16);
    }
}

uint8_t bluetoothManagerUtils_compareUUID128(uint8_t *const uuid1, uint8_t *const uuid2)
{
    for (int i = 0; i < ESP_UUID_LEN_128; i++)
    {
        if(uuid1[i] != uuid2[i])
            return 1;
    }

    return 0;
}