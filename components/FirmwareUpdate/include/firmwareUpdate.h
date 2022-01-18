#ifndef FIRMWARE_UPDATE_H
#define FIRMWARE_UPDATE_H

#include "centralManagerDefs.h"

void FU_requestTask(void* parameters);

void FU_getCurrentAppVersion(AppVersion_t* version);

#endif