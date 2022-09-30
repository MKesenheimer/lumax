#pragma once

#ifndef WINDOWS
uint32_t timeGetTime();
#endif

int ftHandle(int numDev, void **ftHandle);

int clearBuffer(void *handle);

void setBusy_uncertain(void *handle, FT_STATUS ftStatus);

int isBusy_uncertain(void *handle);

int writeToDev(void *handle, uint8_t *buffer, uint32_t bytesToWrite);

int readFromDev(void *handle, uint8_t *buffer, uint32_t bytesToRead);

int getAmountInQueue(void* handle);

int writeFrameBuffer(void *handle, uint8_t *frameBuffer, uint16_t frameBufferSize, uint16_t numberOfBytes, uint8_t counter, int check, int flag);

int readMemory(void *handle, uint8_t *arr, uint16_t start, uint16_t end);