#ifndef COMMLAYER_H
#define COMMLAYER_H

#include <string.h>
#include <math.h>

#include "centralManager.h"

typedef void (*CLSendHandler_t) (void* parameter, uint8_t* data, uint16_t len);
typedef void (*CLConnectioncb_t) (bool);

typedef enum {
    SERVICE_P3 = 0,
    SERVICE_SYSTEM = 1,
    SERVICE_MEMORY = 2,
    SERVICE_OTA = 3,
    SERVICE_MAX = 4,
    SERVICE_UNDEFINED = 0xFF
} CLService_t;

typedef struct {
    uint8_t  dummies[20];
} CLData_t;

typedef struct {
    union {
        struct {
            uint8_t  status;
            uint16_t length;
            uint32_t operation_Id;
            uint64_t failure_reason;
            uint8_t  dummies[5];
        } __attribute__((packed)) preamble;

        struct {
            uint16_t  numOfPackets;
            CLData_t* dataPTR;
        } __attribute__((packed)) core;
    } U;
} __attribute__((packed)) CLResponse_t;

BaseType_t CL_rxHandler(CLService_t serviceType, uint8_t* data, uint16_t len);

void CL_p3ServiceTx(ResponsePacket_t* respPacket, CLSendHandler_t sendHandler, void* parameters);
void CL_systemServiceTx(ResponsePacket_t* respPacket, CLSendHandler_t sendHandler, void* parameters);
void CL_memoryServiceTx(ResponsePacket_t* respPacket, CLSendHandler_t sendHandler, void* parameters);
void CL_OTAServiceTx(ResponsePacket_t* respPacket, CLSendHandler_t sendHandler, void* parameters);

#endif