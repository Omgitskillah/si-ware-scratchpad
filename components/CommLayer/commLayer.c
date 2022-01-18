#include "commLayer.h"

#define TAG     "CommLayer"
static uint64_t static_shared_memory_allocation_trackers[MEMORY_ALLOCATION_QUEUE_SIZE / (sizeof(uint64_t) * 8)] = {0ll};
#define get_memory_section_from_idx(idx) (idx > (8* sizeof(uint64_t)) ? (idx-8*sizeof(uint64_t)) : idx)
#define get_tracker_idx_from_idx(idx) (idx / (8 * sizeof(uint64_t)))

BaseType_t CL_rxHandler(CLService_t serviceType, uint8_t *data, uint16_t len) {
    if (data == NULL)
        return pdFALSE;
    RequestPacket_t requestPacket;
    uint8_t last_freed_memory = 0;
    while (xQueueReceive(memoryAllocationQueue, &last_freed_memory, 0)) {
        static_shared_memory_allocation_trackers[get_tracker_idx_from_idx(last_freed_memory)] &= ~(1ll
                << get_memory_section_from_idx(last_freed_memory));
    }

    switch (serviceType) {
        case SERVICE_P3:
            requestPacket.packetType = REQUESTPACKET_P3;
            break;
        case SERVICE_SYSTEM:
            requestPacket.packetType = REQUESTPACKET_SYSTEM;
            break;
        case SERVICE_MEMORY:
            requestPacket.packetType = REQUESTPACKET_MEMORY;
            break;
        case SERVICE_OTA:
            requestPacket.packetType = REQUESTPACKET_OTA;
            break;

        case SERVICE_UNDEFINED:
        default:
            return pdFALSE;
    }

    if (serviceType != SERVICE_OTA)
        memcpy((void *) &requestPacket.packet, (void *) data,
               (len < sizeof(requestPacket.packet)) ? len : sizeof(requestPacket.packet));
    else {
        memcpy((void *) &requestPacket.packet, (void *) data, sizeof(requestPacket.packet.otaPacket.operationID));
        requestPacket.packet.otaPacket.len = 0;
        uint8_t request_allocated_memory_idx = MEMORY_IDX_INVALID;
        if (len > sizeof(requestPacket.packet.otaPacket.operationID)) {
            for (uint8_t i = 0; i < MEMORY_ALLOCATION_QUEUE_SIZE; i++) {
                uint8_t tracker_idx = get_tracker_idx_from_idx(i);
                uint8_t memory_section_bit = get_memory_section_from_idx(i);
                if ((static_shared_memory_allocation_trackers[tracker_idx] & (1ll << memory_section_bit)) == 0) {
                    //we found empty memory, need to allocate it
                    request_allocated_memory_idx = i;
                    static_shared_memory_allocation_trackers[tracker_idx] |= (1ll << memory_section_bit);
                    break;
                }
            }
            if (request_allocated_memory_idx == MEMORY_IDX_INVALID) {
                //didn't find any memory, wait for one to become free
                xQueueReceive(memoryAllocationQueue, &request_allocated_memory_idx, portMAX_DELAY);
            }
            assert(request_allocated_memory_idx != MEMORY_IDX_INVALID);
            ESP_LOGI(TAG, "Allocated Memory With address %d", request_allocated_memory_idx);
            memcpy((void *) (&sharedStaticMemory[request_allocated_memory_idx * SHARED_MEMORY_CHUNCK]),
                   (void *) (data + sizeof(requestPacket.packet.otaPacket.operationID)),
                   len - sizeof(requestPacket.packet.otaPacket.operationID));
            requestPacket.packet.otaPacket.data_ptr = (void *) (&sharedStaticMemory[request_allocated_memory_idx *
                                                                                    SHARED_MEMORY_CHUNCK]);
            requestPacket.packet.otaPacket.memory_index = request_allocated_memory_idx;
            requestPacket.packet.otaPacket.len = len - sizeof(requestPacket.packet.otaPacket.operationID);
        }
    }

    return xQueueSend(requestQueue, (void *) &requestPacket, (TickType_t) 500);
}

void CL_p3ServiceTx(ResponsePacket_t *respPacket, CLSendHandler_t sendHandler, void *parameters) {
    P3Response_t *resp = &respPacket->packet.p3Response;
    CLResponse_t clPreamble = {
            .U.preamble.status = respPacket->statusCode,
            .U.preamble.operation_Id = resp->operationID,
    };
    CLResponse_t clCoreData;

    switch (resp->operationID) {
        case OPID_RUN_PSD:
        case OPID_RUN_ABSRB:
            clPreamble.U.preamble.length = resp->data.spectralData.psdLength;
            clCoreData.U.core.dataPTR = (CLData_t *) resp->data.spectralData.spectralPTR;
            if (resp->data.spectralData.settings.commonWaveNum)
                clCoreData.U.core.numOfPackets = ceil((resp->data.spectralData.psdLength + 2) * 8.0 / sizeof(CLData_t));
            else
                clCoreData.U.core.numOfPackets = ceil((resp->data.spectralData.psdLength * 2) * 8.0 / sizeof(CLData_t));

            break;

        case OPID_RUN_GNADJ:
            clPreamble.U.preamble.length = 2;
            clCoreData.U.core.dataPTR = (CLData_t *) &resp->data.gainData;
            clCoreData.U.core.numOfPackets = 0;
            break;

        case OPID_RUN_Temp:
            clPreamble.U.preamble.length = 4;
            clCoreData.U.core.dataPTR = (CLData_t *) &resp->data.temperature;
            clCoreData.U.core.numOfPackets = 0;
            break;

        default:
            clPreamble.U.preamble.length = 1;
            clCoreData.U.core.numOfPackets = 0;
            clCoreData.U.core.dataPTR = (CLData_t *) &clCoreData; //Just pointer to a dummy value
            break;
    }
    ESP_LOGI(TAG, "Sending Preamble for operation %d with status %d", clPreamble.U.preamble.operation_Id,
             respPacket->statusCode);
    ESP_LOGI(TAG, "%d packets for operation %d", clCoreData.U.core.numOfPackets, clPreamble.U.preamble.operation_Id);
    sendHandler(parameters, (uint8_t *) &clPreamble, sizeof(CLData_t));

    if (respPacket->statusCode != NoError)
        return;

    if (clCoreData.U.core.numOfPackets == 0) {
        sendHandler(parameters, (uint8_t *) clCoreData.U.core.dataPTR, clPreamble.U.preamble.length);
        //This is done so all packets sent are of size 20 bytes
//        sendHandler(parameters, (uint8_t*) clCoreData.U.core.dataPTR, sizeof(CLData_t));
    } else {
        for (uint16_t i = 0; i < clCoreData.U.core.numOfPackets; i++) {
            ESP_LOGI(TAG, "sending packet %i", i);
            sendHandler(parameters, (uint8_t *) &clCoreData.U.core.dataPTR[i], sizeof(CLData_t));
        }
    }
}

void CL_systemServiceTx(ResponsePacket_t *respPacket, CLSendHandler_t sendHandler, void *parameters) {
    SysResponse_t *resp = &respPacket->packet.sysResponse;
    CLResponse_t clPreamble = {
            .U.preamble.status = respPacket->statusCode,
            .U.preamble.operation_Id = resp->operationID,
    };
    CLResponse_t clCoreData;
    CLData_t firstPacket;

    switch (resp->operationID) {
        case OPID_GAUGE_INFO_REQ:
            clPreamble.U.preamble.length = sizeof(resp->operationID) + sizeof(oldFormatGaugeInfo_t);
            clCoreData.U.core.numOfPackets = ceil(clPreamble.U.preamble.length * 1.0 / sizeof(CLData_t));


            memcpy(((uint8_t *) &firstPacket) + 0, &resp->operationID, sizeof(resp->operationID)); //copy operationID
            memcpy(((uint8_t *) &firstPacket) + sizeof(resp->operationID), resp->U.oldGaugeInfoPtr,sizeof(CLData_t) - sizeof(resp->operationID));

            clCoreData.U.core.dataPTR = (CLData_t *) ((uint8_t *) resp->U.oldGaugeInfoPtr + sizeof(CLData_t) -
                                                      sizeof(resp->operationID)); //point to the data after the firstPacket
//            ESP_LOGI(TAG,"Size of Packets for GAUGE_INFO_REQ is %d and operation id is %d",clCoreData.U.core.numOfPackets,resp->U.oldGaugeInfoPtr->operation_id1);
//            ESP_LOG_BUFFER_HEX(TAG,&firstPacket,sizeof(CLData_t));
//            ESP_LOG_BUFFER_HEX(TAG,clCoreData.U.core.dataPTR,sizeof(oldFormatGaugeInfo_t) - (sizeof(CLData_t)-sizeof(resp->operationID)));

            break;
        case OPID_GET_LIFE_TIME_DATA: {
            uint16_t life_time_data_length = resp->U.life_time_data_packet.life_time_data_length;
            clPreamble.U.preamble.length = sizeof(resp->operationID) + sizeof(life_time_data_t);
            clCoreData.U.core.numOfPackets = ceil(clPreamble.U.preamble.length * 1.0 / sizeof(CLData_t));
            memcpy(((uint8_t *) &firstPacket), &resp->operationID, sizeof(resp->operationID)); //copy operationID
            memcpy(((uint8_t *) &firstPacket) + sizeof(resp->operationID), &life_time_data_length,sizeof(life_time_data_length));
            memcpy(((uint8_t *) &firstPacket) + sizeof(resp->operationID) + sizeof(life_time_data_length), (const void *)resp->U.life_time_data_packet.life_time_data_ptr,sizeof(CLData_t) - sizeof(resp->operationID) - sizeof(life_time_data_length));
            clCoreData.U.core.dataPTR = (CLData_t *) ((uint8_t *) resp->U.life_time_data_packet.life_time_data_ptr + sizeof(CLData_t) - sizeof(resp->operationID) - sizeof(life_time_data_length));
        }
            break;
        case OPID_VERSION_REQ:
            clPreamble.U.preamble.length = sizeof(resp->operationID) + sizeof(resp->U.version);
            clCoreData.U.core.numOfPackets = ceil(clPreamble.U.preamble.length * 1.0 / sizeof(CLData_t));

            memcpy(((uint8_t *) &firstPacket) + 0, &resp->operationID, sizeof(resp->operationID)); //copy operationID
            memcpy(((uint8_t *) &firstPacket) + sizeof(resp->operationID), &resp->U.version,
                   sizeof(CLData_t) - sizeof(resp->operationID));

            clCoreData.U.core.dataPTR = (CLData_t *) ((uint8_t *) &resp->U.version + sizeof(CLData_t) -
                                                      sizeof(resp->operationID)); //point to the data after the firstPacket
            break;

        case OPID_BATT_REQ:
            clPreamble.U.preamble.length = sizeof(resp->operationID) + sizeof(resp->U.batReq);
            clCoreData.U.core.dataPTR = (CLData_t *) resp;
            clCoreData.U.core.numOfPackets = 0;
            break;

        case OPID_SID_REQ:
            clPreamble.U.preamble.length = sizeof(resp->operationID) + sizeof(resp->U.sensorID);
            clCoreData.U.core.dataPTR = (CLData_t *) resp;
            clCoreData.U.core.numOfPackets = 0;
            break;

        case OPID_POWER_SELFTEST_REQ:
            clPreamble.U.preamble.length = sizeof(resp->operationID) + sizeof(resp->U.selfTestStatus);
            clCoreData.U.core.dataPTR = (CLData_t *) resp;
            clCoreData.U.core.numOfPackets = 0;
            break;
        case OPID_READ_TEMPERATURE_REQ:
            clPreamble.U.preamble.length = sizeof(resp->operationID) + sizeof(resp->U.temperature);
            clCoreData.U.core.dataPTR = (CLData_t *) resp;
            clCoreData.U.core.numOfPackets = 0;
            break;
        case OPID_CHECK_BOARD:
        case OPID_ENTER_DEBUG_MODE:
            clPreamble.U.preamble.length = 4;
            clCoreData.U.core.dataPTR = (CLData_t *) resp;
            clCoreData.U.core.numOfPackets = 0;
            break;
        default:
            return;
    }

    sendHandler(parameters, (uint8_t *) &clPreamble, sizeof(CLData_t));

    if (respPacket->statusCode != NoError) {
        ESP_LOGI(TAG, "Bad Status %d", respPacket->statusCode);
        return;
    }

    ESP_LOGI(TAG, "First Data byte %d", *((uint8_t *) clCoreData.U.core.dataPTR));

    if (clCoreData.U.core.numOfPackets == 0) {
        sendHandler(parameters, (uint8_t *) clCoreData.U.core.dataPTR, clPreamble.U.preamble.length);
//        sendHandler(parameters, (uint8_t*) clCoreData.U.core.dataPTR, sizeof(CLData_t));

    } else {
        for (uint16_t i = 0; i < clCoreData.U.core.numOfPackets; i++) {
            if (i == 0)
                sendHandler(parameters, (uint8_t *) &firstPacket, sizeof(CLData_t));
            else
                sendHandler(parameters, (uint8_t *) &clCoreData.U.core.dataPTR[i - 1], sizeof(CLData_t));
        }
    }
}

void CL_memoryServiceTx(ResponsePacket_t *respPacket, CLSendHandler_t sendHandler, void *parameters) {
    MemResponse_t *resp = &respPacket->packet.memResponse;
    CLResponse_t clPreamble = {
            .U.preamble.status = respPacket->statusCode,
            .U.preamble.operation_Id = resp->operationID,
    };
    CLResponse_t clCoreData;
    CLData_t firstPacket;

    switch (resp->operationID) {
        case OPID_SCAN_REQ_WITH_SETTINGS:
            if (clPreamble.U.preamble.status != NoError) {
                clPreamble.U.preamble.length = 1;
                break;
            }

            if (resp->U.spectralData.settings.commonWaveNum)
                clPreamble.U.preamble.length = sizeof(resp->operationID) + sizeof(resp->U.spectralData.type)
                                               + sizeof(resp->U.spectralData.settings) +
                                               ((resp->U.spectralData.psdLength + 2) * sizeof(double));
            else
                clPreamble.U.preamble.length = sizeof(resp->operationID) + sizeof(resp->U.spectralData.type)
                                               + sizeof(resp->U.spectralData.settings) +
                                               (resp->U.spectralData.psdLength * 2 * sizeof(double));

            clCoreData.U.core.numOfPackets = ceil(clPreamble.U.preamble.length * 1.0 / sizeof(CLData_t));

            memcpy(((uint8_t *) &firstPacket) + 0, &resp->operationID, sizeof(resp->operationID)); //copy operationID
            memcpy(((uint8_t *) &firstPacket) + sizeof(resp->operationID),
                   &resp->U.spectralData.type, sizeof(resp->U.spectralData.type)); //append spectral type
            memcpy(((uint8_t *) &firstPacket) + sizeof(resp->operationID) + sizeof(resp->U.spectralData.type),
                   &resp->U.spectralData.settings, sizeof(resp->U.spectralData.settings)); //append measurement settings
            memcpy(((uint8_t *) &firstPacket) + sizeof(resp->operationID) + sizeof(resp->U.spectralData.type) + sizeof(resp->U.spectralData.settings),
            &resp->U.spectralData.temperature_during_scan,sizeof(resp->U.spectralData.temperature_during_scan));    //copy the temperature

            memcpy(((uint8_t *) &firstPacket) + sizeof(resp->operationID) + sizeof(resp->U.spectralData.type) + //4+1
                   sizeof(resp->U.spectralData.settings) +sizeof(resp->U.spectralData.temperature_during_scan) ,   //+8+4
                   resp->U.spectralData.spectralPTR,
                   sizeof(CLData_t) - sizeof(resp->operationID) - sizeof(resp->U.spectralData.type) -
                   sizeof(resp->U.spectralData.settings) -sizeof(resp->U.spectralData.temperature_during_scan) ); //append spectral data

            clCoreData.U.core.dataPTR = (CLData_t *) ((uint8_t *) resp->U.spectralData.spectralPTR + sizeof(CLData_t)
                                                      - sizeof(resp->operationID) - sizeof(resp->U.spectralData.type) -
                                                      sizeof(resp->U.spectralData.settings) - sizeof(resp->U.spectralData.temperature_during_scan)); //point to the data after the firstPacket
            break;
        case OPID_SCAN_REQ:
            if (clPreamble.U.preamble.status != NoError) {
                clPreamble.U.preamble.length = 1;
                break;
            }
            clPreamble.U.preamble.length = sizeof(resp->operationID) + sizeof(resp->U.spectralData.type) +
                                           resp->U.spectralData.psdLength * 2 * sizeof(uint64_t);
            clCoreData.U.core.numOfPackets = ceil(clPreamble.U.preamble.length * 1.0 / sizeof(CLData_t));

            //This might cause unaligned access, need to copy the operationID to some temp location then use it
            memcpy((uint8_t *) (&firstPacket), &resp->operationID, sizeof(resp->operationID));
            memcpy((uint8_t *) (&firstPacket) + sizeof(resp->operationID), &resp->U.spectralData.type,
                   sizeof(resp->U.spectralData.type));
            memcpy((uint8_t *) (&firstPacket) + sizeof(resp->operationID) + sizeof(resp->U.spectralData.type),
                   resp->U.spectralData.spectralPTR,
                   sizeof(CLData_t) - sizeof(resp->operationID) - sizeof(resp->U.spectralData.type));
            clCoreData.U.core.dataPTR = (CLData_t *) ((uint8_t *) resp->U.spectralData.spectralPTR + sizeof(CLData_t) -
                                                      sizeof(resp->operationID) - sizeof(resp->U.spectralData.type));
//            ESP_LOG_BUFFER_HEX(TAG,&firstPacket,sizeof(CLData_t));
//            ESP_LOG_BUFFER_HEX(TAG,clCoreData.U.core.dataPTR,clPreamble.U.preamble.length - sizeof(CLData_t));
            break;

        case OPID_MEM_STATS:
            clPreamble.U.preamble.length = sizeof(resp->operationID) + sizeof(resp->U.stat);
            clCoreData.U.core.dataPTR = (CLData_t *) resp;
            clCoreData.U.core.numOfPackets = 0;
            break;

        case OPID_SET_SCANNER_ID:
            clPreamble.U.preamble.length = sizeof(resp->operationID);
            clCoreData.U.core.dataPTR = (CLData_t *) resp;
            clCoreData.U.core.numOfPackets = 0;
            break;
        case OPID_READ_AMBIENT_TEMP_FILE:
            clPreamble.U.preamble.length = sizeof(resp->operationID) + sizeof(resp->U.temperature_reading);
            clCoreData.U.core.dataPTR = (CLData_t *) resp;
            clCoreData.U.core.numOfPackets = 0;
            break;
        case OPID_READ_MOTHER_BOARD_VERSION_FILE:
            clPreamble.U.preamble.length = sizeof(resp->operationID) + sizeof(resp->U.mother_board_version);
            clCoreData.U.core.dataPTR = (CLData_t *) resp;
            clCoreData.U.core.numOfPackets = 0;
            break;
        case OPID_GET_BATTERY_LOG:
            clPreamble.U.preamble.length = sizeof(resp->operationID) + sizeof(resp->U.battery_log_data.length) + resp->U.battery_log_data.length;
            clCoreData.U.core.numOfPackets =  ceil(clPreamble.U.preamble.length * 1.0 / sizeof(CLData_t));
            memcpy((uint8_t *) (&firstPacket), &resp->operationID, sizeof(resp->operationID));
            memcpy((uint8_t *) (&firstPacket) + sizeof(resp->operationID), &resp->U.battery_log_data.length,sizeof(resp->U.battery_log_data.length));
            memcpy((uint8_t *) (&firstPacket) + sizeof(resp->operationID) + sizeof(resp->U.battery_log_data.length), resp->U.battery_log_data.data_bytes,sizeof(CLData_t) -sizeof(resp->operationID) - sizeof(resp->U.battery_log_data.length));

            clCoreData.U.core.dataPTR = (CLData_t * )(resp->U.battery_log_data.data_bytes +sizeof(CLData_t) -sizeof(resp->operationID) - sizeof(resp->U.battery_log_data.length));
            break;
        default:
            clPreamble.U.preamble.length = 1;
            clCoreData.U.core.numOfPackets = 0;
            clCoreData.U.core.dataPTR = (CLData_t *) &clCoreData; //Just pointer to a dummy value
            break;
    }

    sendHandler(parameters, (uint8_t *) &clPreamble, sizeof(CLData_t));

    if (respPacket->statusCode != NoError)
        return;

    if (clCoreData.U.core.numOfPackets == 0) {
        sendHandler(parameters, (uint8_t *) clCoreData.U.core.dataPTR, clPreamble.U.preamble.length);
//        sendHandler(parameters, (uint8_t*) clCoreData.U.core.dataPTR, sizeof(CLData_t));

    } else {
        for (uint16_t i = 0; i < clCoreData.U.core.numOfPackets; i++) {
            if (i == 0)
                sendHandler(parameters, (uint8_t *) &firstPacket, sizeof(CLData_t));
            else
                sendHandler(parameters, (uint8_t *) &clCoreData.U.core.dataPTR[i - 1], sizeof(CLData_t));
        }
    }
}

void CL_OTAServiceTx(ResponsePacket_t *respPacket, CLSendHandler_t sendHandler, void *parameters) {
    OTAResponse_t *resp = &respPacket->packet.otaResponse;
    CLResponse_t clPreamble = {
            .U.preamble.status = respPacket->statusCode,
            .U.preamble.operation_Id = resp->operationID,
    };
    CLResponse_t clCoreData;

    switch (resp->operationID) {
        case OPID_OTA_OPERATION_DONE:
            clPreamble.U.preamble.length = 1;
            clCoreData.U.core.dataPTR = (CLData_t *) &resp->operationID;
            clCoreData.U.core.numOfPackets = 0;
            break;

        case OPID_OTA_DATA_WRITTEN:
            clPreamble.U.preamble.length = sizeof(OTAResponse_t);
            clCoreData.U.core.dataPTR = (CLData_t *) resp;
            clCoreData.U.core.numOfPackets = 0;
            break;

        default:
            return;
    }

    sendHandler(parameters, (uint8_t *) &clPreamble, sizeof(CLData_t));
    if (respPacket->statusCode != NoError)
        return;

    sendHandler(parameters, (uint8_t *) clCoreData.U.core.dataPTR, clPreamble.U.preamble.length);
}