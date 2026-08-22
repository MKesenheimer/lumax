#pragma once

#include <stdint.h>

// internal helpers (see liblumax.c)

#ifndef WINDOWS
uint32_t timeGetTime(void);
#endif

int openDev(int numDev, void **handle);

int clearBuffer(void *handle);

void checkIfBusy(void *handle, uint32_t ftStatus);

int isOpen(void *handle);

int writeToDev(void *handle, uint8_t *buffer, uint32_t bytesToWrite);

int readFromDev(void *handle, uint8_t *buffer, uint32_t bytesToRead);

int writeFrameBuffer(void *handle, uint8_t *frameBuffer, uint16_t frameBufferSize, uint16_t numberOfBytes, uint8_t counter, int flag);

int readID(void *handle, uint8_t *arr, uint16_t size);

int readMemory(void *handle, uint8_t *arr, uint16_t start, uint16_t end);

int dongleCom(void *handle, uint8_t flag, uint32_t address, uint8_t *writeBuffer, uint8_t *readBuffer);
