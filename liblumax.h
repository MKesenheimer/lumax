#pragma once

#include <stdint.h>

// internal helpers (see liblumax.c)

#ifndef WINDOWS
uint32_t timeGetTime(void);
#endif

int openDev(int numDev, void **handle);

int clearBuffer(void *handle);

void checkIfBusy(void *handle, uint32_t ftStatus);

int isOpen(void **handle);

int sendInitKey(void *handle);

int sendKey(void *handle, int flavor);

int writeToDev(void *handle, uint8_t *buffer, uint32_t bytesToWrite);

int readFromDev(void *handle, uint8_t *buffer, uint32_t bytesToRead);

int writeFrameBuffer(void *handle, uint8_t *frameBuffer, uint16_t frameBufferSize, uint16_t numberOfBytes, uint8_t counter, int flag);

int readID(void *handle, uint8_t *arr, uint16_t size);

int readMemory(void *handle, uint8_t *arr, uint16_t start, uint16_t end);

// Set the blanking delay in milliseconds. At each off->on light transition
// (laser turned on), this many ms worth of blank (laser-off) points are
// inserted at the same position. At each on->off transition, extra "on"
// points are added before the laser turns off.
// delay_ms: blanking delay in milliseconds (0 = disabled, default).
void Lumax_SetBlankingDelay(int delay_ms);

int dongleCom(void *handle, uint8_t flag, uint32_t address, uint8_t *writeBuffer, uint8_t *readBuffer);
