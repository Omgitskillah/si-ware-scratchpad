#ifndef UARTMANAGER_H
#define UARTMANAGER_H

#include "commLayer.h"

void uartManager_setup();
void uartManager_setConnectionCallback(CLConnectioncb_t callback);
void uartManager_responseTask(void* parameters);
int uartManager_log(const char *, va_list);
void uartManager_disableTimerForDebug(void);

#endif