// generate object:
// gcc -c -I/opt/local/include liblumax.c
// generate shared library
// gcc -shared -o liblumax.so liblumax.o -L/opt/local/lib/ -lftd2xx

#include <stdio.h>
#include <stdlib.h>
#include <ftdi.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <stdint.h>

#include "lumax.h"
#include "liblumax.h"


// Falls die Ausgabekarte nicht angesprochen wird, sollten zuerst folgende Punkte überprüft werden:
// - Stimmt der Flavor?
// - Ist der BufferLayout korrekt gesetzt?
// - Wird die DongleCom-Funktion aufgerufen? (bis jetzt noch nicht vollständig verstanden und implementiert)

// constants
// TODO: in Struktur oder globale Klasse anordnen
const int MaxPoints = 4500; // flags[36], pos = 1456
const int MinScanSpeed = 250;
const int MaxScanSpeed = 70000; // flags[51]
// Danach richtet sich die Anzahl der Farbkanäle der Lumax
const int Flavor = 1; // flags[49], LaserWorld Lumax = 1, Bare-Lumax = 4
const uint8_t BytesPerFrame = 7; // flags[34], wie viele Bytes können mit einem Sendevorgang gesendert werden
const uint32_t ClockSpeed = 16000000; // flags[324144]

// vid / pid
const uint32_t vid = 0x403;
const uint32_t pid = 0xc88a;

// globals
// TTL buffer
// hier können globale TTL Werte gesetzt werden, die bei jedem SendFrame
// mitgesendet werden.
uint8_t BufferLayout = 1; // flags[324143], flags[16438], flags[1308] (?), LaserWorld Lumax = 1, Bare Lumax = 5
char SerialNumber[16];
uint8_t TTLBuffer = 0; // flags[32], pos = 1440
uint8_t TTLAvailable = 0; // flags[31], pos = 1436
uint32_t NextLoopCounts = 0; // flags[42]
uint32_t TimeUntilFree = 0; // flags[44]
uint32_t TimeOffset = 250; // flags[43], TODO Mindestzeit, die bei Aufruf von Lumax_WaitForBuffer auf jeden Fall gewartet werden muss
uint32_t NumberOfFramesSend = 0; // flags[2790]
uint32_t ScanSpeed = 1000; // flags[46]
uint32_t NumberOfPoints = 0; // flags[47]
//uint32_t Handle = 0; // flags[1], pos = 1316
uint32_t IsBusy = 0; // flags[30], pos = 1432
uint32_t BusyTime; // flags[37], pos = 1460

// DEBUG Flags
uint32_t lumax_verbosity = 0; //DBG_ALL;

// Done
#ifndef WINDOWS
uint32_t timeGetTime() {
    struct timespec _t;
    clock_gettime(CLOCK_REALTIME, &_t);
    return _t.tv_sec*1000 + lround(_t.tv_nsec/1.0e6);
}
#endif

// Done
int openDev(int numDev, void **handle) {
    uint32_t ret;
    struct ftdi_context *ftHandle;

    if ((ftHandle = ftdi_new()) == 0) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_ERROR || lumax_verbosity & DBG_ALL)
            fprintf(stderr, "[ERROR] openDev: ftdi_new failed.\n");
#endif
        return 1;
    }

    // TODO: check if device is already open
    if ((ret = ftdi_usb_open_desc(ftHandle, vid, pid, NULL, SerialNumber)) < 0) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_ERROR || lumax_verbosity & DBG_ALL)
            fprintf(stderr, "[ERROR] openDev: ftdi_usb_open_desc failed: %d\n", ret);
#endif
        return 1;
    }

    ftdi_set_latency_timer(ftHandle, 2);
    ftHandle->usb_read_timeout = 1000;
    ftHandle->usb_write_timeout = 1000;
    *handle = ftHandle;
    return 0;
}

// Done
int clearBuffer(void *handle) {
    struct ftdi_context *ftHandle = (struct ftdi_context*)handle;
    uint8_t buffer[256];
    uint32_t bytesWritten;
    for (int i = 0; i <= 255; ++i)
        buffer[i] = 0;
    for (int i = 0; i <= 255; ++i)
        ftdi_write_data(ftHandle, buffer, 255);
  usleep(200000u);
  ftdi_tciflush(ftHandle);
  ftdi_tcoflush(ftHandle);
  return 0;
}

// Done
void checkIfBusy(void *handle, uint32_t ftStatus) {
    if (ftStatus == -1 && IsBusy != 1) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_CHECKIFBUSY || lumax_verbosity & DBG_ALL)
            printf("[DEBUG] checkIfBusy: Device with handle 0x%x is busy.\n", (unsigned int)(uintptr_t)handle);
#endif
        IsBusy = 1;
        BusyTime = timeGetTime();
    }
}

// Done
int isOpen(void *handle) {
    if (handle == 0) 
        return 1; // 1 means error, device is closed!

    if (IsBusy) {
        uint32_t time = timeGetTime();
        uint32_t diff = time - BusyTime;
        // if device is busy for too long:
        if (diff > 2000 && IsBusy == 1) {
            Lumax_CloseDevice(handle);
            // open new device
            if (openDev(0, handle)) { // TODO: handle multiple devices
                // opening the device failed
                BusyTime = time;
                return 1;
            }
        }
    }

    // all good
    IsBusy = 0;
    return 0;
}

// Done
int writeToDev(void *handle, uint8_t *buffer, uint32_t bytesToWrite) {
    struct ftdi_context *ftHandle = (struct ftdi_context*)handle;
    uint32_t bytesWritten;
    uint32_t ftStatus = ftdi_write_data(ftHandle, buffer, bytesToWrite);
    bytesWritten = ftStatus;

#ifdef DEBUG_POSSIBLE
    if (lumax_verbosity & DBG_WRITETODEV || lumax_verbosity & DBG_ALL) {
        for (uint32_t i = 0; i < bytesToWrite; ++i)
            printf("[DEBUG] writeToDev: buffer[%d] = 0x%x\n", i, buffer[i]);
        printf("[DEBUG] writeToDev: ftStatus = %d, bytesToWrite = %d, bytesWritten = %d\n", ftStatus, bytesToWrite, bytesWritten);
    }
#endif

    if (bytesToWrite != bytesWritten) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_WRITETODEV || lumax_verbosity & DBG_ALL) {
            printf("[DEBUG] writeToDev: bytesToWrite != bytesWritten.\n");
        }
#endif
        checkIfBusy(handle, ftStatus);
        return 1;
    }
    
    return 0;
}

int readFromDev(void *handle, uint8_t *buffer, uint32_t bytesToRead) {
    struct ftdi_context *ftHandle = (struct ftdi_context*)handle;
    int result = 0;
    uint32_t received;

    if ((result = ftdi_read_data(ftHandle, buffer, bytesToRead)) < 0) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_WARN || lumax_verbosity & DBG_ALL) {
            printf("[WARN] readFromDev: ftdi_read_data failed with error code %d (%s).\n", result, ftdi_get_error_string(ftHandle));
        }
        return 1;
    }
#endif

    received = result;

#ifdef DEBUG_POSSIBLE
    if (lumax_verbosity & DBG_READFROMDEV || lumax_verbosity & DBG_ALL)
        printf("[DEBUG] readFromDev: ftStatus = %d, bytesToRead = %d, bytesReceived = %d.\n", result, bytesToRead, received);
#endif

    if (bytesToRead != received)
        return 1;

    return 0;
}

// Done
int writeFrameBuffer(void *handle, uint8_t *frameBuffer, uint16_t frameBufferSize, uint16_t numberOfBytes, uint8_t counter, int flag) {

#ifdef DEBUG_POSSIBLE
    if (lumax_verbosity & DBG_WRITEFRAMEBUFFER || lumax_verbosity & DBG_ALL)
        printf("[DEBUG] writeFrameBuffer: frameBufferSize = %d, numberOfBytes = %d, counter = %d, flag = %d.\n", frameBufferSize, numberOfBytes, counter, flag);
#endif

    if (!numberOfBytes || numberOfBytes + frameBufferSize > 0x10000)
        return 1;

    uint8_t writeb[7];
    writeb[0] = flag ? 1 : BufferLayout;
    writeb[1] = frameBufferSize;
    writeb[2] = frameBufferSize / 256;
    writeb[3] = numberOfBytes;
    writeb[4] = numberOfBytes / 256;
    writeb[5] = counter;
    writeb[6] = writeb[5] ^ writeb[4] ^ writeb[3] ^ writeb[2] ^ writeb[1] ^ writeb[0];
    
    if (writeToDev(handle, writeb, 7) || writeToDev(handle, frameBuffer, numberOfBytes)) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_WRITEFRAMEBUFFER || lumax_verbosity & DBG_ALL)
            fprintf(stderr,"[Error] writeFrameBuffer: writeToDev failed.\n");
#endif
        return 1;
    }

    uint8_t readb[2];
    if (readFromDev(handle, readb, 2u) || readb[0] != writeb[6] || readb[1] != 0x55) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_WRITEFRAMEBUFFER || lumax_verbosity & DBG_ALL)
            fprintf(stderr,"[Error] writeFrameBuffer: readFromDev failed.\n");
#endif
        return 1;
    }

    return 0;
}

// Done
int readID(void *handle, uint8_t *arr, uint16_t size) {
    if (!arr) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_ERROR || lumax_verbosity & DBG_ALL)
            fprintf(stderr, "[ERROR] readID: array not initialized.\n");
#endif
        return 1;
    }

    uint8_t buffer[7] = {0xc9, 0, 0, 0, 0, 0, 0xc9};
    if (writeToDev(handle, buffer, 7)) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_ERROR || lumax_verbosity & DBG_ALL)
            fprintf(stderr, "[ERROR] readID: write to device failed.\n");
#endif
        return 1;
    }

    usleep(50000u);
    uint8_t lastByte;
    // read back one byte and check start of frame
    if (readFromDev(handle, &lastByte, 1u) || lastByte != buffer[6]) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_ERROR || lumax_verbosity & DBG_ALL)
            fprintf(stderr, "[ERROR] readID: check failed.\n");
#endif
        return 1;
    }
            
    // read id
    if (readFromDev(handle, arr, size)) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_ERROR || lumax_verbosity & DBG_ALL)
            fprintf(stderr, "[INFO] readID: no ID available.\n");
#endif
        for (int i = 0; i < size; ++i) 
            arr[i] = 0;
        return 1;
    }

    // read checksum
    if (readFromDev(handle, &lastByte, 1u)) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_ERROR || lumax_verbosity & DBG_ALL)
            fprintf(stderr, "[ERROR] readID: read checksum failed.\n");
#endif
        return 1;
    }

    // calculate checksum
    uint8_t check = 0;
    for (int i = 0; i < size; ++i) {
        check ^= arr[i];
    }
    if (lastByte != check) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_ERROR || lumax_verbosity & DBG_ALL)
            fprintf(stderr, "[ERROR] readID: checksum invalid.\n");
#endif
        return 1;
    }

    return 0;
}

// Done
int readMemory(void *handle, uint8_t *arr, uint16_t start, uint16_t end) {
    if (!arr) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_ERROR || lumax_verbosity & DBG_ALL)
            fprintf(stderr, "[ERROR] readMemory: array not initialized.\n");
#endif
        return 1;
    }

    if (start < 0 || end <= 0 || start + end > 463) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_READMEMORY || lumax_verbosity & DBG_ALL)
            fprintf(stderr,"[ERROR] readMemory: boundaries not ok.\n");
#endif
        return 1;
    }

    uint8_t writeb[7];
    writeb[0] = 0xCA;
    writeb[1] = start;
    writeb[2] = start / 256; // HIBYTE
    writeb[3] = end;
    writeb[4] = end / 256;
    writeb[5] = 0;
    writeb[6] = writeb[4] ^ writeb[3] ^ writeb[2] ^ writeb[1] ^ writeb[0];
    uint8_t readb[522];

    if (writeToDev(handle, writeb, 7) || readFromDev(handle, readb, end + 2)) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_READMEMORY || lumax_verbosity & DBG_ALL)
            printf("[DEBUG] readMemory: read or write error.\n");
#endif
        return 1;
    }
    
    uint8_t check = 0;
    for (int i = 0; i < end; ++i) {
        arr[i] = readb[i + 1];
        //printf("[DEBUG] readMemory: arr[%d] = %c\n", i, arr[i]); // LXPVB77T
        check ^= arr[i];
    }
    if (readb[0] != writeb[6] || readb[end + 1] != check) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_WRITEFRAMEBUFFER || lumax_verbosity & DBG_ALL)
            fprintf(stderr,"[Error] readMemory: checksum failed.\n");
#endif
        return 1;
    }
    
    return 0;
}

// TODO
int dongleCom(void *handle, uint8_t flag, uint32_t address, uint8_t *writeBuffer, uint8_t *readBuffer) {
    // TODO: Es ist noch nicht klar wie der writeBuffer befüllt wird.
    // Vermutlich gibt es hier Schutzmechanismen gegen Debugging (Stack Obfuscation)
    uint8_t writeb[24];
    uint8_t readb[14];
    int result = 1;
    writeb[0] = 0xcd;
    writeb[1] = address;
    writeb[2] = address >> 8;
    writeb[3] = address >> 16;
    writeb[4] = address >> 24;
    writeb[5] = flag;
    writeb[6] = writeb[5] ^ writeb[4] ^ writeb[3] ^ writeb[2] ^ writeb[1] ^ writeb[0];
    int j = 7;
    uint8_t check = 0;
    for (int i = 0; i < 16; ++i) {
        writeb[j++] = writeBuffer[i];
        check ^= writeBuffer[i];
    }
    writeb[23] = check;
    if (!writeToDev(handle, writeb, 24)
        && !readFromDev(handle, readb, 1) // TODO: war in IDA readFromDev(handle, readb, 1, 0)
        && readb[0] == writeb[6]
        && !readFromDev(handle, readb, 14)) {
        check = 0;
        for (int i = 0; i < 13; ++i)
            check ^= readb[i];
        if (readb[13] == check) {
            for (int i = 0; i < 13; ++i)
                readBuffer[i] = readb[i];
            result = 0;
        }
    }
    return result;
}

// Done
float Lumax_GetApiVersion() { return 0.9; }

// TODO
int Lumax_GetDeviceInfo(int physicalDevice, int infoID, uint8_t *inBuffer, uint16_t inLength, uint8_t *outBuffer, uint16_t outLength) { return 0; }

// Done
int Lumax_GetPhysicalDevices() {
    uint32_t ret, numberOfDevices;
    struct ftdi_context *ftHandle;
    struct ftdi_device_list *devlist, *curdev;
    char manufacturer[128], description[128], serialnumber[16];

    if ((ftHandle = ftdi_new()) == 0) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_ERROR || lumax_verbosity & DBG_ALL)
            fprintf(stderr, "[ERROR] Lumax_GetPhysicalDevices: ftdi_new failed.\n");
#endif
        Lumax_CloseDevice(ftHandle);
        return 0;
    }

    if ((ret = ftdi_usb_find_all(ftHandle, &devlist, vid, pid)) < 0) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_ERROR || lumax_verbosity & DBG_ALL)
            fprintf(stderr, "[ERROR] Lumax_GetPhysicalDevices: ftdi_usb_find_all failed: %d (%s)\n", ret, ftdi_get_error_string(ftHandle));
#endif
        Lumax_CloseDevice(ftHandle);
        return 0;
    }
    numberOfDevices = ret;

    if (numberOfDevices < 1) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_ERROR || lumax_verbosity & DBG_ALL)
            fprintf(stderr, "[ERROR] Lumax_GetPhysicalDevices: No device detected.\n");
#endif
        Lumax_CloseDevice(ftHandle);
        return 0;
    }

#ifdef DEBUG_POSSIBLE
    if (lumax_verbosity & DBG_INFO || lumax_verbosity & DBG_ALL)
        printf("[INFO] Lumax_GetPhysicalDevices: Number of devices is %d\n", numberOfDevices);
#endif

    int i = 0;
    for (curdev = devlist; curdev != NULL; ++i) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_INFO || lumax_verbosity & DBG_ALL)
            printf("[INFO] Lumax_GetPhysicalDevices: Checking device: %d\n", i);
#endif
        if ((ret = ftdi_usb_get_strings(ftHandle, curdev->dev, manufacturer, 128, description, 128, serialnumber, 16)) < 0) {
#ifdef DEBUG_POSSIBLE
            if (lumax_verbosity & DBG_ERROR || lumax_verbosity & DBG_ALL)
                fprintf(stderr, "[ERROR] Lumax_GetPhysicalDevices: ftdi_usb_get_strings failed: %d (%s)\n", ret, ftdi_get_error_string(ftHandle));
#endif
            Lumax_CloseDevice(ftHandle);
            return 0;
        }

        if (serialnumber[0] == '\0') {
#ifdef DEBUG_POSSIBLE
            if (lumax_verbosity & DBG_ERROR || lumax_verbosity & DBG_ALL)
                fprintf(stderr, "[ERROR] Lumax_GetPhysicalDevices: serialnumber empty.\n");
#endif
            Lumax_CloseDevice(ftHandle);
            return 0;
        }

#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_GENERAL || lumax_verbosity & DBG_ALL)
            printf("[DEBUG] Lumax_GetPhysicalDevices: *serialnumber 0x%x.\n", (unsigned int)(uintptr_t)serialnumber);
#endif

#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_INFO || lumax_verbosity & DBG_ALL) {
            printf("[INFO] Lumax_GetPhysicalDevices: Manufacturer: %s, Description: %s\n", manufacturer, description);
            printf("[INFO] Lumax_GetPhysicalDevices: Serialnumber: %s\n", serialnumber);
        }
#endif
        // TODO: handle multiple Devices. use first device for now
        memcpy(SerialNumber, serialnumber, 16);
        // handle next device
        curdev = curdev->next;
    }

    ftdi_list_free(&devlist);
    Lumax_CloseDevice(ftHandle);
    return numberOfDevices;
}

// Done
int Lumax_SetTTL(void *handle, uint8_t TTL) {
    if (isOpen(handle))
        return 1; // device is not open

    TTLBuffer = TTL;
    TTLAvailable |= 2u;
    if (!TTLAvailable)
        return 0; // nothing to handle, continue without error

    uint8_t buffer[7];
    buffer[0] = 0x1E;
    buffer[1] = TTL;
    buffer[2] = 0;
    buffer[3] = 0;
    buffer[4] = 0;
    buffer[5] = 0;
    buffer[6] = buffer[1] ^ buffer[0];
    uint8_t lastByte;
    if (writeToDev(handle, buffer, 7) || readFromDev(handle, &lastByte, 1u) || lastByte != buffer[6])
        return 1;

    return 0;
}

// Done
int Lumax_WaitForBuffer(void* handle, int timeOut, int *timeToWait, int *bufferChanged) {
    // with libftdi, the buffer is handled differently (without threading)
    // Lumax_WaitForBuffer is therefore not necessary.
    *timeToWait = 0;
    *bufferChanged = 0;
    return 0;
}

// Done
int Lumax_CloseDevice(void* handle) {
    struct ftdi_context *ftHandle = (struct ftdi_context*)handle;
#ifdef DEBUG_POSSIBLE
    if (lumax_verbosity & DBG_GENERAL || lumax_verbosity & DBG_ALL)
        printf("[DEBUG] Lumax_CloseDevice: Closing device with handle 0x%x.\n", (unsigned int)(uintptr_t)ftHandle);
#endif
    clearBuffer(ftHandle);
    int ret = ftdi_usb_close(ftHandle);
    if (ftHandle) ftdi_free(ftHandle);
    return ret;
}

// Done
int Lumax_SendFrame(void *handle, TLumax_Point *points, int numOfPoints, int scanSpeed, int updateMode, int *timeToWait) {
    // variables
    uint8_t writeb[32768];
    TLumax_Point point;
    TLumax_Point *lpoints;
    lpoints = points;
    int result = 1;

#ifdef DEBUG_POSSIBLE
    if (lumax_verbosity & DBG_SENDFRAME || lumax_verbosity & DBG_ALL) {
        printf("[DEBUG] Lumax_SendFrame: Device handle = 0x%x.\n", (unsigned int)(uintptr_t)handle);
        printf("[DEBUG] Lumax_SendFrame: numOfPoints = %d.\n", numOfPoints);
        for (int i = 0; i < numOfPoints; ++i)
            printf("[DEBUG] Lumax_SendFrame: point%d: CH1 = %d, CH2 = %d, CH3 = %d, CH4 = %d, CH5 = %d.\n", i, lpoints[i].Ch1, lpoints[i].Ch2, lpoints[i].Ch3, lpoints[i].Ch4, lpoints[i].Ch5);
    }
#endif

    // if buffer is empty (initialization)
    if (!numOfPoints) {
        point.Ch1 = 0x8000;
        point.Ch2 = 0x8000;
        point.Ch3 = 0;
        point.Ch4 = 0;
        point.Ch5 = 0;
        point.Ch6 = 0;
        point.Ch7 = 0;
        point.Ch8 = 0;
        point.TTL = 0;
        scanSpeed = 1000;
        lpoints = &point;
        numOfPoints = 1;
    }

    // fill the buffer and send to device
    if (!isOpen(handle)) { // device is open
        if (numOfPoints > 0 && 16 * MaxPoints / 2 >= numOfPoints) {
            if (scanSpeed < MinScanSpeed) scanSpeed = MinScanSpeed;
            else if (scanSpeed > MaxScanSpeed) scanSpeed = MaxScanSpeed;

            ++NumberOfFramesSend;
            NumberOfPoints = numOfPoints;
            ScanSpeed = scanSpeed;
            TimeOffset = 0;
            uint16_t maxLoops = numOfPoints / MaxPoints;
            uint16_t residual = numOfPoints % MaxPoints;
            if (residual) ++maxLoops;
#ifdef DEBUG_POSSIBLE
            if (lumax_verbosity & DBG_SENDFRAME || lumax_verbosity & DBG_ALL) {
                printf("[DEBUG] Lumax_SendFrame: maxLoops = %d, residual = %d, TTLAvailable = %d.\n", maxLoops, residual, TTLAvailable);
            }
#endif
            // big loop to fill the buffer
            int readOK;
            int npoint = 0;
            for (int i = 0; i < maxLoops; ++i) {
                int pointsPerLoop;
                if (i != 0)
                    pointsPerLoop = MaxPoints;
                else
                    pointsPerLoop = residual;

                int k = 0;
                if (Flavor == 2) {
                    for (int j = 0; j < pointsPerLoop; ++j) {
                        uint16_t l0 = lpoints[npoint].Ch1 >> 4;
                        uint16_t l1 = lpoints[npoint].Ch2 >> 4;
                        writeb[k++] = l0 / 256 + 16 * (l1 / 256);
                        writeb[k++] = l0;
                        writeb[k++] = l1;
                        writeb[k++] = lpoints[npoint].Ch3 >> 8;
                        writeb[k++] = lpoints[npoint].Ch4 >> 8;
                        writeb[k++] = lpoints[npoint++].Ch5 >> 8;
                    }
                }

                if (Flavor == 4) {
                    for (int j = 0; j < pointsPerLoop; ++j) {
                        uint16_t l0 = lpoints[npoint].Ch1 >> 4;
                        uint16_t l1 = lpoints[npoint].Ch2 >> 4;
                        //if ((l1 & ))
                        writeb[k++] = l0 / 256 + 16 * (l1 / 256);
                        writeb[k++] = l0;
                        writeb[k++] = l1;
                        writeb[k++] = lpoints[npoint].Ch3 >> 8;
                        writeb[k++] = lpoints[npoint].Ch4 >> 8;
                        writeb[k++] = lpoints[npoint].Ch5 >> 8;
                        writeb[k++] = lpoints[npoint++].Ch8 >> 8;
                    }
                }

                if (Flavor == 8) {
                    for (int j = 0; j < pointsPerLoop; ++j) {
                        uint8_t l0 = lpoints[npoint].Ch1 >> 4;
                        uint8_t l1 = lpoints[npoint].Ch2 >> 4;
                        writeb[k++] = l0 / 256 + 16 * (l1 / 256);
                        writeb[k++] = l0;
                        writeb[k++] = l1;
                        writeb[k++] = lpoints[npoint].Ch3 >> 8;
                        writeb[k++] = lpoints[npoint].Ch4 >> 8;
                        writeb[k++] = lpoints[npoint].Ch5 >> 8;
                        writeb[k++] = lpoints[npoint].Ch8 >> 8;
                        writeb[k++] = lpoints[npoint++].Ch6 >> 8;
                    }
                }

                if (Flavor == 16) {
                    for (int j = 0; j < pointsPerLoop; ++j) {
                        uint16_t l0 = lpoints[npoint].Ch1 >> 4;
                        uint16_t l1 = lpoints[npoint].Ch2 >> 4;
                        writeb[k++] = l0 / 256 + 16 * (l1 / 256);
                        writeb[k++] = l0;
                        writeb[k++] = l1;
                        writeb[k++] = lpoints[npoint].Ch3 >> 8;
                        writeb[k++] = lpoints[npoint].Ch4 >> 8;
                        writeb[k++] = lpoints[npoint].Ch5 >> 8;
                        writeb[k++] = lpoints[npoint].Ch8 >> 8;
                        writeb[k++] = lpoints[npoint].Ch6 >> 8;
                        writeb[k++] = lpoints[npoint++].Ch7 >> 8;
                    }
                }

                if (Flavor == 1) {
                    if (TTLAvailable & 1) {
                        for (int j = 0; j < pointsPerLoop; ++j) {
                            uint16_t l0 = lpoints[npoint].Ch1 >> 4;
                            uint16_t l1 = lpoints[npoint].Ch2 >> 4;
                            writeb[k++] = l0 / 256 + 16 * (l1 / 256);
                            writeb[k++] = l0;
                            writeb[k++] = l1;
                            writeb[k++] = lpoints[npoint].Ch3 >> 8;
                            writeb[k++] = lpoints[npoint].Ch4 >> 8;
                            writeb[k++] = lpoints[npoint++].Ch5 >> 8;
                        }
                    } else if (TTLAvailable & 2) {
                        for (int j = 0; j < pointsPerLoop; ++j) {
                            uint16_t l0 = lpoints[npoint].Ch1 >> 4;
                            uint16_t l1 = lpoints[npoint].Ch2 >> 4;
                            writeb[k++] = l0 / 256 + 16 * (l1 / 256);
                            writeb[k++] = l0;
                            writeb[k++] = l1;
                            writeb[k++] = lpoints[npoint].Ch3 >> 8;
                            writeb[k++] = lpoints[npoint].Ch4 >> 8;
                            writeb[k++] = lpoints[npoint].Ch5 >> 8;
                            writeb[k++] = TTLBuffer;
                            ++npoint;
                        }
                    } else {
                        for (int j = 0; j < pointsPerLoop; ++j) {
                            uint16_t l0 = lpoints[npoint].Ch1 >> 4;
                            uint16_t l1 = lpoints[npoint].Ch2 >> 4;
                            writeb[k++] = l0 / 256 + 16 * (l1 / 256);
                            writeb[k++] = l0;
                            writeb[k++] = l1;
                            writeb[k++] = lpoints[npoint].Ch3 >> 8;
                            writeb[k++] = lpoints[npoint].Ch4 >> 8;
                            writeb[k++] = lpoints[npoint].Ch5 >> 8;
                            writeb[k++] = lpoints[npoint++].TTL;
                        }
                    }
                }

                // write to Device
                readOK = writeFrameBuffer(handle, writeb, 32768, pointsPerLoop * BytesPerFrame, NextLoopCounts + i, 0);
                if (readOK) break;
            }

             // terminate frame, send scanspeed
            if (!readOK) {
                uint16_t cycles = ClockSpeed / scanSpeed;
                writeb[0] = 3;
                writeb[1] = residual;
                writeb[2] = residual / 256;
                writeb[3] = cycles;
                writeb[4] = cycles / 256;
                writeb[5] = NextLoopCounts + 16 * (maxLoops - 1);
                writeb[6] = writeb[5] ^ writeb[4] ^ writeb[3] ^ writeb[2] ^ writeb[1] ^ writeb[0];
                readOK = writeToDev(handle, writeb, 7u);
                if (!readOK) {
                    uint8_t readb[4];
                    readOK = readFromDev(handle, readb, 4u);
                    if (!readOK && readb[0] == writeb[6]) {
                        if (readb[3] >= 0) {
                            TimeUntilFree = 0;
                        } else {
                            readb[3] &= 127;
                            TimeUntilFree = readb[1] + (readb[2] << 8) + MaxPoints * (readb[3] - 1);
                            TimeUntilFree = 1000 * TimeUntilFree / ScanSpeed;
                        }
                        TimeOffset = 1000 * numOfPoints / scanSpeed; // die zu erwartende Zeit, bis der nächste Frame gesendet werden kann
#ifdef DEBUG_POSSIBLE
                        if (lumax_verbosity & DBG_SENDFRAME || lumax_verbosity & DBG_ALL)
                            printf("[DEBUG] Lumax_SendFrame: TimeOffset = %d\n", TimeOffset);
#endif
                        if (timeToWait)
                            *timeToWait = TimeUntilFree;
                        result = 0;
                    }
                }
                if (NextLoopCounts)
                    NextLoopCounts = 0;
                else
                    NextLoopCounts = 8;
            }
        }
    }
#ifdef DEBUG_POSSIBLE
    if (lumax_verbosity & DBG_SENDFRAME || lumax_verbosity & DBG_ALL)
        printf("[DEBUG] Lumax_SendFrame: result = %d\n", result);
#endif
    return result;
}

// Done
int Lumax_StopFrame(void* handle) {
    return Lumax_SendFrame(handle, NULL, 0, 0, 0, NULL);;
}

// TODO
int Lumax_DongleCom(void* handle, int flag, int address, int writeVar, int *readVar) {
    /*// DEBUG
    int result = 0;
    uint8_t writeb[24];
    uint8_t readb;

    writeb[0] = 0xcd;
    writeb[1] = 0x3a;
    writeb[2] = 0xb1;
    writeb[3] = 0xd4;
    writeb[4] = 0x49;
    writeb[5] = 0x01;
    writeb[6] = writeb[5] ^ writeb[4] ^ writeb[3] ^ writeb[2] ^ writeb[1] ^ writeb[0];
    writeb[7] = 0x02; // Beispiel eines Schlüssels writeb[7:23]
    writeb[8] = 0x0;
    writeb[9] = 0x0;
    writeb[10] = 0x0;
    writeb[11] = 0x01;
    writeb[12] = 0x0;
    writeb[13] = 0x0;
    writeb[14] = 0x0;
    writeb[15] = 0x14;
    writeb[16] = 0x16;
    writeb[17] = 0x1c;
    writeb[18] = 0x03;
    writeb[19] = 0x0;
    writeb[20] = 0x0;
    writeb[21] = 0x0;
    writeb[22] = 0x0;
    writeb[23] = 0x1e;
    writeToDev(handle, writeb, 24u);
    if (readFromDev(handle, &readb, 1u) || readb != writeb[6])
        result = 1;*/

    // TODO: dieser Buffer muss mit ganz bestimmten Werten befüllt werden (Schlüssel, etc, siehe Beispiel oben)
    // Siehe IDA-Projekt: test2\lumax.dll.idb
    uint8_t writeb[16];
    for (int i = 0; i < 16; ++i)
        writeb[i] = 0;
    uint8_t readb[14];
    int result = 1;
    if (!dongleCom(handle, 1, 0x49D4B13Au, writeb, readb)) {
        if (readb[12] == 1)
            result = 205;
        if (!readb[12]) {
            *readVar = readb[0];
            result = 0;
        }
    }

    // diese beiden Frames werden geschickt, nachdem die dongleCom abgeschlossen ist.
    // zu finden in sendInitKey():
    uint8_t byte;
    writeb[0] = 0x09;
    writeb[1] = 0;
    writeb[2] = 0;
    writeb[3] = 0;
    writeb[4] = 0;
    writeb[5] = 0;
    writeb[6] = writeb[5] ^ writeb[4] ^ writeb[3] ^ writeb[2] ^ writeb[1] ^ writeb[0];
    writeToDev(handle, writeb, 7u);
    if (readFromDev(handle, &byte, 1u) || byte != writeb[6])
        result = 1;

    // zu finden in sendKey():
    writeb[0] = 0x07;
    writeb[1] = 0x04;
    writeb[2] = 0;
    writeb[3] = 0x94;
    writeb[4] = 0x11;
    writeb[5] = 0;
    writeb[6] = writeb[5] ^ writeb[4] ^ writeb[3] ^ writeb[2] ^ writeb[1] ^ writeb[0];
    writeToDev(handle, writeb, 7);
    if (readFromDev(handle, &byte, 1u) || byte != writeb[6])
        result = 1;
    
#ifdef DEBUG_POSSIBLE
    if (lumax_verbosity & DBG_SETDMXMODE || lumax_verbosity & DBG_ALL)
        printf("[DEBUG] Lumax_DongleCom: result = %d\n", result);
#endif
    return result;
}

// TODO
int Lumax_SetDmxMode(void *handle, uint8_t a2, uint8_t a3) {
    // TODO: a2 und a3 haben hier keine Auswirkung,
    // da aber im Originalprogramm a2 und a3 fest auf 0 gewählt
    // wurden, ist das in Ordnung.
    int result = 0;
    uint8_t writeb[7];
    uint8_t readb;

    writeb[0] = 0xA;
    writeb[1] = 0x2;
    writeb[2] = 0;
    writeb[3] = 0;
    writeb[4] = 0;
    writeb[5] = 0x10;
    writeb[6] = writeb[5] ^ writeb[2] ^ writeb[1] ^ writeb[0];
    writeToDev(handle, writeb, 7u);
    if (readFromDev(handle, &readb, 1u) || readb != writeb[6])
        result = 1;

    writeb[0] = 0xB;
    writeb[1] = 0x2;
    writeb[2] = 0;
    writeb[3] = 0;
    writeb[4] = 0;
    writeb[5] = 0x11;
    writeb[6] = writeb[5] ^ writeb[2] ^ writeb[1] ^ writeb[0];
    writeToDev(handle, writeb, 7);
    if (readFromDev(handle, &readb, 1u) || readb != writeb[6])
        result = 1;
    
#ifdef DEBUG_POSSIBLE
    if (lumax_verbosity & DBG_SETDMXMODE || lumax_verbosity & DBG_ALL)
        printf("[DEBUG] Lumax_SetDmxMode: result = %d\n", result);
#endif
    return result;
}

// TODO
void* Lumax_OpenDevice(int numDev, int channel) {
    uint32_t ftStatus;
    struct ftdi_context *ftHandle;

    if (((0 < numDev) && (numDev < 9)) && (channel == 0)) {
        if (!openDev(numDev, (void**)&ftHandle)) {
            clearBuffer(ftHandle);
            const uint8_t idSize = 16;
            uint8_t id[idSize];
            if (!readID(ftHandle, id, idSize)) {
#ifdef DEBUG_POSSIBLE
                if (lumax_verbosity & DBG_OPENDEVICE || lumax_verbosity & DBG_ALL) {
                    printf("[DEBUG] Lumax_OpenDevice: read id successfully.\n");
                    for (int i = 0; i < idSize; ++i)
                        printf("[DEBUG] Lumax_OpenDevice: id[%d] = %d (%c).\n", i, id[i], id[i]);
                    printf("\n");
                }
#endif
                if ((id[4] & 128u) != 0) {
#ifdef DEBUG_POSSIBLE
                    if (lumax_verbosity & DBG_OPENDEVICE || lumax_verbosity & DBG_ALL)
                        printf("[DEBUG] Lumax_OpenDevice: setting flag TTLAvailable.\n");
#endif
                    TTLAvailable |= 1u;
                }

                if (id[4] >= 7u) {
#ifdef DEBUG_POSSIBLE
                    if (lumax_verbosity & DBG_OPENDEVICE || lumax_verbosity & DBG_ALL)
                        printf("[DEBUG] Lumax_OpenDevice: setting flag BufferLayout = 5.\n");
#endif
                    BufferLayout = 5;
                }

                const uint16_t devSize = 0x1cf;
                uint8_t deviceInfo[devSize];
                if (!readMemory(ftHandle, deviceInfo, 0, devSize)) { // Achtung, andere Logik als in IDA. Außerdem werden bei der Linux-Lib 15 bits statt 9 gelesen
#ifdef DEBUG_POSSIBLE
                    if (lumax_verbosity & DBG_OPENDEVICE || lumax_verbosity & DBG_ALL) {
                        printf("[DEBUG] Lumax_OpenDevice: read device Info successfully.\n");
                        for (int i = 0; i < devSize; ++i)
                            printf("[DEBUG] Lumax_OpenDevice: deviceInfo[%d] = %d (%c).\n", i, deviceInfo[i], deviceInfo[i]);
                        printf("\n");
                    }
#endif

                    int ret = 0;
                    Lumax_DongleCom(ftHandle, 1, 0x49D4B13Au, 0, &ret);
                    printf("Lumax_OpenDevice: ret = %d\n", ret);
                }
#ifdef DEBUG_POSSIBLE
                if (lumax_verbosity & DBG_GENERAL || lumax_verbosity & DBG_ALL) {
                    printf("[DEBUG] Lumax_OpenDevice: opened Lumax device with the following settings:\n");
                    printf("            BufferLayout = %d\n", BufferLayout);
                    printf("            TTLAvailable = %d\n", TTLAvailable);
                    printf("            Flavor       = %d\n", Flavor);
                }
#endif


#ifdef DEBUG_POSSIBLE
                if (lumax_verbosity & DBG_OPENDEVICE || lumax_verbosity & DBG_ALL)
                    printf("[DEBUG] Lumax_OpenDevice: calling Lumax_StopFrame.\n");
#endif
                Lumax_StopFrame(ftHandle);

#ifdef DEBUG_POSSIBLE
                if (lumax_verbosity & DBG_OPENDEVICE || lumax_verbosity & DBG_ALL)
                    printf("[DEBUG] Lumax_OpenDevice: calling Lumax_SetDmxMode.\n");
#endif
                Lumax_SetDmxMode(ftHandle, 0, 0); // schaltet DMX aus

#ifdef DEBUG_POSSIBLE
                if (lumax_verbosity & DBG_OPENDEVICE || lumax_verbosity & DBG_ALL)
                    printf("[DEBUG] Lumax_OpenDevice: calling Lumax_SetTTL.\n");
#endif
                Lumax_SetTTL(ftHandle, 0);
            }
        } 
    }
    return ftHandle;
}
