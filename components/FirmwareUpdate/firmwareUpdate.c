#include <string.h>
#include "firmwareUpdate.h"

#include "centralManager.h"
#include "esp_ota_ops.h"

#define TAG     "FirmwareUpdate"

static const esp_partition_t *update_partition;
static esp_ota_handle_t update_handle = 0;
static uint32_t binary_file_length = 0;

static int getNoOfWrittenBytes()
{
    return binary_file_length;
}

static ScannerError_t FU_start()
{
    ESP_LOGI(TAG, "Setup firmware update");

    binary_file_length = 0;
    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();

    if (configured != running)
    {
        ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
                 configured->address, running->address);
        ESP_LOGW(TAG,
                 "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
             running->type, running->subtype, running->address);

    update_partition = esp_ota_get_next_update_partition(NULL);
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
             update_partition->subtype, update_partition->address);
    assert(update_partition != NULL);

    esp_err_t err_ota = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
    if (err_ota != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err_ota));
        return OTA_INIT_FAILED;
    }

    ESP_LOGI(TAG, "Setup firmware update succeeded");
    return NoError;
}

static ScannerError_t FU_step(uint8_t *data, uint16_t data_len, uint8_t memory_reference)
{
    if (esp_ota_write(update_handle, (const void *) data, data_len) != ESP_OK)
        return OTA_STEP_FAILED;

    binary_file_length += data_len;
    ESP_LOGI(TAG, "Written length %d\t Total length = %d", data_len, binary_file_length);
    if(memory_reference != MEMORY_IDX_INVALID)
    {
        //really should never wait, the queue can hold the whole memory being sent
        xQueueSend(memoryAllocationQueue,&memory_reference,portMAX_DELAY);
    }

    return NoError;
}

static ScannerError_t FU_end()
{
    ESP_LOGI(TAG, "Total Write binary data length : %d", binary_file_length);
    binary_file_length = 0;

    if (esp_ota_end(update_handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_end failed!");
        return OTA_END_FAILED;
    }

    esp_err_t err_ota = esp_ota_set_boot_partition(update_partition);
    if (err_ota != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err_ota));
        return OTA_END_FAILED;
    }

    ESP_LOGI(TAG, "Restart system!");
#ifdef SUPPORT_OLD_OTA_WRITE_PACKET
    esp_restart();
#endif
    // ota_restartScanner();
    return NoError;
}

static ScannerError_t FU_backToFactory()
{
    esp_partition_iterator_t iter = esp_partition_find(ESP_PARTITION_TYPE_APP, // Get partition iterator for
                                                       ESP_PARTITION_SUBTYPE_APP_FACTORY,                 // factory partition
                                                       "factory");
    if (iter == NULL)
    {
        ESP_LOGE(TAG, "Failed to find factory partition");
        return OTA_RESTORE_FAILED;
    }

    const esp_partition_t *factory = esp_partition_get(iter);
    esp_partition_iterator_release(iter);
    if (esp_ota_set_boot_partition(factory) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set boot partition");
        return OTA_RESTORE_FAILED;
    }

    ESP_LOGI(TAG, "Back to Factory Partition");
    // ota_restartScanner();
#ifdef SUPPORT_OLD_OTA_WRITE_PACKET
    esp_restart();
#endif
    return NoError;
}

void FU_requestTask(void *parameters)
{
    static uint8_t first_data_packet_received = false;
    OTARequestPacket_t rp;

    memcpy(&rp, parameters, sizeof(rp));
    xEventGroupSetBits(taskReceivedData, FIRMWARE_UPDATE_REQUEST_TASK_BIT); //Set the bit after copying the parameters

    ResponsePacket_t responsePacket = {
            .packetType = RESPONSEPACKET_OTA,
            .statusCode = NoError,
            .packet.otaResponse = {
                    .operationID = OPID_OTA_OPERATION_DONE
            },
    };

    switch (rp.operationID)
    {
        case OPID_RSTR_FCTRY_FRMWR:
            responsePacket.statusCode = FU_backToFactory();
            break;

        case OPID_OTA_START:
            responsePacket.statusCode = FU_start();
            break;

        case OPID_OTA_END:
            responsePacket.statusCode = FU_end();
            break;

        case OPID_OTA_DATA:
            responsePacket.statusCode = FU_step(rp.data_ptr, rp.len,rp.memory_index);
            responsePacket.packet.otaResponse.operationID = OPID_OTA_DATA_WRITTEN;
            responsePacket.packet.otaResponse.writtenBytes = getNoOfWrittenBytes();
            break;

        default:
#ifdef SUPPORT_OLD_OTA_WRITE_PACKET
            /*
             * This code is written as a backward compatibility step to be compatible with old IOS apps
             * The ios apps doesn't send the opCode in the beginning of the request, which means the request is all data
             * but as a safety measure I will just make sure of some magic byte (that's special to the IDF itself not to our firmware)
             * that's in the first packet, then will consider all the other packets as data
             */
            if ((((uint8_t *) &rp.operationID)[0] == ESP_IMAGE_HEADER_MAGIC && !first_data_packet_received) ||
                first_data_packet_received)
            {
                ESP_LOGI(TAG, "Writing Data without write op code");
                first_data_packet_received = true;
                if (rp.len)
                {
                    memmove(rp.data_ptr + sizeof(rp.operationID), rp.data_ptr, rp.len);
                    memcpy(rp.data_ptr, &rp.operationID, sizeof(rp.operationID));
                    rp.len += sizeof(rp.operationID);
                    responsePacket.statusCode = FU_step(rp.data_ptr, rp.len,rp.memory_index);
                } else
                {
                    //This is to handle if we only have 4 incoming bytes
                    responsePacket.statusCode = FU_step((uint8_t *) &rp.operationID, sizeof(rp.operationID),0xFF);
                }
                responsePacket.packet.otaResponse.operationID = OPID_OTA_DATA_WRITTEN;
                responsePacket.packet.otaResponse.writtenBytes = getNoOfWrittenBytes();
            }
#endif
            break;
    }

    xQueueSend(responseQueue, (void *) &responsePacket, portMAX_DELAY);

    ESP_LOGI(TAG, "Remaining stack %d bytes", uxTaskGetStackHighWaterMark(NULL));
    xEventGroupSetBits(availableTasks, FIRMWARE_UPDATE_REQUEST_TASK_BIT); //Set the bit before deleting the task
    vTaskDelete(NULL);  //Delete itself
    while (1);           //This task should be deleted after finishing
}

void FU_getCurrentAppVersion(AppVersion_t *version)
{
    esp_app_desc_t app_description;

    const esp_partition_t *runningPartition = esp_ota_get_running_partition();

    esp_ota_get_partition_description(runningPartition, &app_description);
    memcpy(version, app_description.version, sizeof(app_description.version));
}