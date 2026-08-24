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
#include <signal.h>

#include "lumax.h"
#include "liblumax.h"
#include "lumax_protocol.h"


// Falls die Ausgabekarte nicht angesprochen wird, sollten zuerst folgende Punkte überprüft werden:
// - Stimmt der Flavor?
// - Ist der BufferLayout korrekt gesetzt?
// - Wird die DongleCom-Funktion aufgerufen? (bis jetzt noch nicht vollständig verstanden und implementiert)

// constants
// TODO: in Struktur oder globale Klasse anordnen
// (nicht const: sendKey() schreibt diese Werte auf Bare-Lumax-Karten neu)
int MaxPoints = 4500; // flags[36], pos = 1456
const int MinScanSpeed = 250;
int MaxScanSpeed = 70000; // flags[51]
// Danach richtet sich die Anzahl der Farbkanäle der Lumax
int Flavor = 1; // flags[49], LaserWorld Lumax = 1, Bare-Lumax = 4
const uint32_t ClockSpeed = 16000000; // flags[324144]

// Blanking delay: number of additional blank (laser-off) points to insert
// at each off->on transition. Set via Lumax_SetBlankingDelay(delay_ms).
// The delay is converted to point count based on the current scan speed.
static int BlankingDelayMs = 0; // blanking delay in milliseconds

// größte Frame-Chunk-Größe in Byte: 9 Byte/Punkt (Flavor 16, RGB + Cyan +
// DeepBlue + Yellow) * 4500 Punkte (MaxPoints) pro Chunk
#define MaxFrameBytes (9 * 4500)

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
uint32_t TimeOffset = 0; // flags[43], zu erwartende Zeit bis der nächste Frame gesendet werden kann (wird von Lumax_SendFrame gesetzt)
uint32_t NumberOfFramesSend = 0; // flags[2790]
uint32_t ScanSpeed = 1000; // flags[46]
uint32_t LastDivisor = 1000; // flags[45], zuletzt verwendete Scan-Geschwindigkeit
uint32_t NumberOfPoints = 0; // flags[47]
uint32_t DmxFlavorMask = 0; // flags[50], ungleich 0 nur bei Bare-Lumax (id[4] 9..0xb)
void *openHandle = NULL; // letztes Handle aus Lumax_OpenDevice, für Lumax_GetDeviceInfo
//uint32_t Handle = 0; // flags[1], pos = 1316
uint32_t IsBusy = 0; // flags[30], pos = 1432
uint32_t BusyTime; // flags[37], pos = 1460

// DEBUG Flags (declared in lumax.h)
const uint32_t DBG_FATAL = 1;
const uint32_t DBG_ERROR = 2;
const uint32_t DBG_WARN = 4;
const uint32_t DBG_INFO = 8;
const uint32_t DBG_GENERAL = 16;
const uint32_t DBG_WRITETODEV = 32;
const uint32_t DBG_READFROMDEV = 64;
const uint32_t DBG_WRITEFRAMEBUFFER = 128;
const uint32_t DBG_READID = 256;
const uint32_t DBG_READMEMORY = 512;
const uint32_t DBG_WAITFORBUFFER = 1024;
const uint32_t DBG_SENDFRAME = 2048;
const uint32_t DBG_SETDMXMODE = 4096;
const uint32_t DBG_OPENDEVICE = 8192;
const uint32_t DBG_CHECKIFBUSY = 16384;
const uint32_t DBG_ISOPEN = 32768;
const uint32_t DBG_ALL = 65536;
uint32_t lumax_verbosity = 0; //DBG_ALL;

// Done
#ifndef WINDOWS
uint32_t timeGetTime() {
    struct timespec _t;
    clock_gettime(CLOCK_REALTIME, &_t);
    return _t.tv_sec*1000 + lround(_t.tv_nsec/1.0e6);
}
#endif

// Byte-level protocol capture.
// When lumax_logfile names a file, every transfer to/from the device is
// appended to it as a hex dump, one line per transfer:
//     <millis> <TX|RX> <count>: <hex byte> <hex byte> ...
// The file is opened lazily on first use; clearing the name disables logging.
char lumax_logfile[256] = "";
static FILE *logHandle = NULL;

static void logBytes(const char *dir, const uint8_t *data, uint32_t len) {
    if (lumax_logfile[0] == '\0') {
        if (logHandle) { fclose(logHandle); logHandle = NULL; }
        return;
    }
    if (!logHandle)
        logHandle = fopen(lumax_logfile, "a");
    if (!logHandle)
        return;
    fprintf(logHandle, "%u %s %u:", (unsigned)timeGetTime(), dir, (unsigned)len);
    for (uint32_t i = 0; i < len; ++i)
        fprintf(logHandle, " %02x", data[i]);
    fputc('\n', logHandle);
    fflush(logHandle);
}

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
        ftdi_free(ftHandle);
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
    memset(buffer, 0, sizeof(buffer));
    for (int i = 0; i < 256; ++i)
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
// The handle is passed by reference: the busy-recovery path below closes
// the stale device and opens a fresh one, so the caller must receive the
// new context pointer (writing it into the old, freed one would be a
// use-after-free).
int isOpen(void **handle) {
    if (!handle || *handle == 0)
        return 1; // 1 means error, device is closed!

    if (IsBusy) {
        uint32_t time = timeGetTime();
        uint32_t diff = time - BusyTime;
        // if device is busy for too long:
        if (diff > 2000 && IsBusy == 1) {
            Lumax_CloseDevice(*handle);
            // open new device
            if (openDev(0, handle)) { // TODO: handle multiple devices
                // opening the device failed
                *handle = NULL;
                BusyTime = time;
                return 1;
            }
            openHandle = *handle;
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

    logBytes("TX", buffer, bytesToWrite);
    return 0;
}

int readFromDev(void *handle, uint8_t *buffer, uint32_t bytesToRead) {
    struct ftdi_context *ftHandle = (struct ftdi_context*)handle;
    int result = 0;
    uint32_t received;

    if ((result = ftdi_read_data(ftHandle, buffer, bytesToRead)) < 0) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_WARN || lumax_verbosity & DBG_ALL)
            printf("[WARN] readFromDev: ftdi_read_data failed with error code %d (%s).\n", result, ftdi_get_error_string(ftHandle));
#endif
        return 1;
    }

    received = result;

#ifdef DEBUG_POSSIBLE
    if (lumax_verbosity & DBG_READFROMDEV || lumax_verbosity & DBG_ALL)
        printf("[DEBUG] readFromDev: ftStatus = %d, bytesToRead = %d, bytesReceived = %d.\n", result, bytesToRead, received);
#endif

    if (bytesToRead != received)
        return 1;

    if (received)
        logBytes("RX", buffer, received);

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
    writeb[6] = lumax_checksum(writeb, 6);
    
    if (writeToDev(handle, writeb, 7) || writeToDev(handle, frameBuffer, numberOfBytes)) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_WRITEFRAMEBUFFER || lumax_verbosity & DBG_ALL)
            fprintf(stderr, "[ERROR] writeFrameBuffer: writeToDev failed.\n");
#endif
        return 1;
    }

    uint8_t readb[2];
    if (readFromDev(handle, readb, 2u) || readb[0] != writeb[6] || readb[1] != 0x55) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_WRITEFRAMEBUFFER || lumax_verbosity & DBG_ALL)
            fprintf(stderr, "[ERROR] writeFrameBuffer: readFromDev failed.\n");
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
    if (lastByte != lumax_checksum(arr, size)) {
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

    if (end == 0 || start + end > 463) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_READMEMORY || lumax_verbosity & DBG_ALL)
            fprintf(stderr, "[ERROR] readMemory: boundaries not ok.\n");
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
    writeb[6] = lumax_checksum(writeb, 6);
    uint8_t readb[522];

    if (writeToDev(handle, writeb, 7) || readFromDev(handle, readb, end + 2)) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_READMEMORY || lumax_verbosity & DBG_ALL)
            printf("[DEBUG] readMemory: read or write error.\n");
#endif
        return 1;
    }
    
    for (int i = 0; i < end; ++i)
        arr[i] = readb[i + 1];
    if (readb[0] != writeb[6] || readb[end + 1] != lumax_checksum(arr, end)) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_WRITEFRAMEBUFFER || lumax_verbosity & DBG_ALL)
            fprintf(stderr, "[ERROR] readMemory: checksum failed.\n");
#endif
        return 1;
    }
    
    return 0;
}

// Done
// 0xCD wire exchange: 24-byte request
//   cd <address little-endian x4> <flag> <XOR of first 6>
//   <16-byte key buffer> <XOR of key buffer>
// the device acks with the single request checksum byte, then answers
// with 14 bytes: <13 payload bytes> <XOR of payload>.
// The key buffer is filled by Lumax_DongleCom depending on the flag.
int dongleCom(void *handle, uint8_t flag, uint32_t address, uint8_t *writeBuffer, uint8_t *readBuffer) {
    uint8_t writeb[24];
    uint8_t readb[14];
    int result = 1;
    writeb[0] = 0xcd;
    writeb[1] = address;
    writeb[2] = address >> 8;
    writeb[3] = address >> 16;
    writeb[4] = address >> 24;
    writeb[5] = flag;
    writeb[6] = lumax_checksum(writeb, 6);
    for (int i = 0; i < 16; ++i)
        writeb[7 + i] = writeBuffer[i];
    writeb[23] = lumax_checksum(writeb + 7, 16);
    if (!writeToDev(handle, writeb, 24)
        && !readFromDev(handle, readb, 1)
        && readb[0] == writeb[6]
        && !readFromDev(handle, readb, 14)) {
        if (readb[13] == lumax_checksum(readb, 13)) {
            for (int i = 0; i < 13; ++i)
                readBuffer[i] = readb[i];
            result = 0;
        }
    }
    return result;
}

// Done
float Lumax_GetApiVersion() { return 0.9; }

// DEBUG: enables byte-level protocol capture (see PROTOCOL.md).
// path:  file to which a hex dump of every transfer is appended;
//        an empty or NULL path disables logging.
void Lumax_SetLogFile(const char *path) {
    if (path && path[0])
        snprintf(lumax_logfile, sizeof(lumax_logfile), "%s", path);
    else
        lumax_logfile[0] = '\0';
}

// Set the blanking delay in milliseconds. At each off->on light transition
// (i.e. when the laser is turned on), this many milliseconds worth of
// blank (laser-off) points are inserted at the same position, giving the
// galvanometers time to settle before the light comes on. The delay is
// converted to a point count using the current scan speed (PPS).
// delay_ms: blanking delay in milliseconds (0 = disabled, default).
void Lumax_SetBlankingDelay(int delay_ms) {
    if (delay_ms < 0)
        delay_ms = 0;
    BlankingDelayMs = delay_ms;
}

// Done
// infoID:    start offset (0..462) into the 463-byte device memory
// inLength:  number of bytes to read (0 = until the end of the memory)
// inBuffer:  reserved (the device memory is read-only), must be NULL
int Lumax_GetDeviceInfo(int physicalDevice, int infoID, uint8_t *inBuffer, uint16_t inLength, uint8_t *outBuffer, uint16_t outLength) {
    (void)inBuffer;
    if (physicalDevice < 1 || physicalDevice > 8)
        return 1;
    if (infoID < 0 || infoID >= 463 || !outBuffer || outLength == 0)
        return 1;

    uint16_t count = (uint16_t)(463 - infoID);
    if (inLength && inLength < count)
        count = inLength;
    if (outLength < count)
        count = outLength;

    void *devHandle = openHandle;
    int opened = 0;
    if (!devHandle) {
        devHandle = Lumax_OpenDevice(physicalDevice, 0);
        opened = 1;
        if (!devHandle)
            return 1;
    }

    int result = readMemory(devHandle, outBuffer, (uint16_t)infoID, count);
    if (opened) {
        // laser safety: blank the beam before closing a handle opened here
        Lumax_StopFrame(devHandle);
        Lumax_CloseDevice(devHandle);
    }
    return result;
}

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
            ftdi_list_free(&devlist);
            Lumax_CloseDevice(ftHandle);
            return 0;
        }

        if (serialnumber[0] == '\0') {
#ifdef DEBUG_POSSIBLE
            if (lumax_verbosity & DBG_ERROR || lumax_verbosity & DBG_ALL)
                fprintf(stderr, "[ERROR] Lumax_GetPhysicalDevices: serialnumber empty.\n");
#endif
            ftdi_list_free(&devlist);
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
        snprintf(SerialNumber, sizeof(SerialNumber), "%s", serialnumber);
        // handle next device
        curdev = curdev->next;
    }

    ftdi_list_free(&devlist);
    Lumax_CloseDevice(ftHandle);
    return numberOfDevices;
}

// Done
int Lumax_SetTTL(void *handle, uint8_t TTL) {
    if (isOpen(&handle))
        return 1; // device is not open

    TTLBuffer = TTL;
    TTLAvailable |= 2u;

    uint8_t buffer[7];
    buffer[0] = 0x1E;
    buffer[1] = TTL;
    buffer[2] = 0;
    buffer[3] = 0;
    buffer[4] = 0;
    buffer[5] = 0;
    buffer[6] = lumax_checksum(buffer, 6);
    uint8_t lastByte;
    if (writeToDev(handle, buffer, 7) || readFromDev(handle, &lastByte, 1u) || lastByte != buffer[6])
        return 1;

    return 0;
}

// Done
// timeOut == 0 with both out-params non-NULL: query mode (0x04, byte 1 = 0),
// the device answers 4 bytes: <XOR> <points low> <points high> <status>.
// Otherwise: wait mode (byte 1 = 1), the device answers <XOR> 1 once the
// buffer has been switched. The magnitude of timeOut is ignored; a wait is
// bounded by the FTDI 1000 ms read timeout.
// The original gates both modes on (flags[52] & 8) == 0; flags[52] is 0xff
// on supported cards, so the gate never triggers and is omitted.
int Lumax_WaitForBuffer(void* handle, int timeOut, int *timeToWait, int *bufferChanged) {
    if (isOpen(&handle))
        return 1; // device is closed

    int queryMode = (timeOut == 0) && timeToWait != NULL && bufferChanged != NULL;
    // byte 1 is the *wait* flag (0 = query, 1 = wait); verified on hardware:
    // sending 0 gets a 4-byte query reply, which desyncs the 2-byte wait read
    uint8_t writeb[7] = {0x04, (uint8_t)(!queryMode), 0, 0, 0, 0, 0};
    writeb[6] = lumax_checksum(writeb, 6);
    uint8_t readb[4];
    int result = 0;

    if (queryMode) {
        result = writeToDev(handle, writeb, 7u)
              || readFromDev(handle, readb, 4u)
              || readb[0] != writeb[6];
        if (!result) {
            uint8_t status = readb[3];
            int changed = (status & 0x80u) == 0;
            *bufferChanged = changed;
            uint32_t chunks = status & 0x7fu;
            uint32_t points = (chunks ? (chunks - 1u) * (uint32_t)MaxPoints : 0u)
                            + ((uint32_t)readb[2] << 8) + readb[1];
            uint32_t divisor = changed ? ScanSpeed : LastDivisor;
            if (!divisor)
                divisor = 1000;
            *timeToWait = (int)(points * 1000u / divisor);
        }
    } else {
        result = writeToDev(handle, writeb, 7u)
              || readFromDev(handle, readb, 2u)
              || readb[0] != writeb[6]
              || readb[1] != 1;
        if (bufferChanged)
            *bufferChanged = result ? 0 : 1;
        // On failure the original also stores flags[38] = 1; that store is
        // dead (never read) and omitted. *timeToWait is never touched in
        // wait mode.
    }

    if (result)
        // A timed-out read may have consumed only part of the device's
        // reply (or none at all); drop whatever is left in the RX FIFO
        // so the stray bytes cannot desync the next exchange.
        ftdi_tciflush((struct ftdi_context*)handle);

    return result;
}

// Done
int Lumax_CloseDevice(void* handle) {
    struct ftdi_context *ftHandle = (struct ftdi_context*)handle;
    if (!ftHandle)
        return 1;
#ifdef DEBUG_POSSIBLE
    if (lumax_verbosity & DBG_GENERAL || lumax_verbosity & DBG_ALL)
        printf("[DEBUG] Lumax_CloseDevice: Closing device with handle 0x%x.\n", (unsigned int)(uintptr_t)ftHandle);
#endif
    clearBuffer(ftHandle);
    int ret = ftdi_usb_close(ftHandle);
    if (ftHandle) ftdi_free(ftHandle);
    if (openHandle == ftHandle)
        openHandle = NULL;
    return ret;
}

// Done
// Internal helper: check if a point has any color channel active (laser "on")
static int pointHasColor(const TLumax_Point *p) {
    return (p->Ch3 != 0 || p->Ch4 != 0 || p->Ch5 != 0 || p->Ch6 != 0 || p->Ch7 != 0 || p->Ch8 != 0);
}

// Internal helper: create a blank point (same position, all colors off)
static void makeBlankPoint(const TLumax_Point *src, TLumax_Point *dst) {
    dst->Ch1 = src->Ch1;
    dst->Ch2 = src->Ch2;
    dst->Ch3 = 0;
    dst->Ch4 = 0;
    dst->Ch5 = 0;
    dst->Ch6 = 0;
    dst->Ch7 = 0;
    dst->Ch8 = 0;
    dst->TTL = 0;
}

// Done
int Lumax_SendFrame(void *handle, TLumax_Point *points, int numOfPoints, int scanSpeed, int updateMode, int *timeToWait) {
    // variables
    // worst case is 9 bytes per point (Flavor 16), MaxPoints (4500) per chunk
    uint8_t writeb[MaxFrameBytes];
    TLumax_Point point;
    TLumax_Point *lpoints;
    lpoints = points;

    // Apply blanking delay if configured: expand the frame by inserting
    // extra blank points at off->on transitions and extra "on" points
    // at on->off transitions.
    if (BlankingDelayMs > 0 && numOfPoints > 0 && scanSpeed > 0) {
        // Calculate how many points correspond to the delay at this scan speed
        int delayPoints = (BlankingDelayMs * scanSpeed) / 1000;
        if (delayPoints > 0) {
            // We need to expand the frame. Maximum expansion: each point could
            // be a transition, so worst case is ~2*delayPoints per transition.
            // Use a static buffer large enough for the expanded frame.
            // Max original points: 16 * MaxPoints / 2 = 36000
            // With blanking delay, could be significantly more.
            // For simplicity, process in-place by creating a new expanded array.
            static TLumax_Point expandedPoints[72000]; // 2x max original
            int expandedCount = 0;

            for (int i = 0; i < numOfPoints; ++i) {
                int hasColor = pointHasColor(&lpoints[i]);
                int prevHasColor = (i > 0) ? pointHasColor(&lpoints[i - 1]) : 0;
                int nextHasColor = (i + 1 < numOfPoints) ? pointHasColor(&lpoints[i + 1]) : 0;

                // Off->on transition: insert delayPoints blank points at this position
                if (!prevHasColor && hasColor) {
                    for (int d = 0; d < delayPoints; ++d) {
                        if (expandedCount < 72000) {
                            makeBlankPoint(&lpoints[i], &expandedPoints[expandedCount++]);
                        }
                    }
                }

                // Copy the current point
                if (expandedCount < 72000) {
                    expandedPoints[expandedCount++] = lpoints[i];
                }

                // On->off transition: add delayPoints extra "on" points before the off point
                if (hasColor && !nextHasColor) {
                    for (int d = 0; d < delayPoints; ++d) {
                        if (expandedCount < 72000) {
                            expandedPoints[expandedCount++] = lpoints[i];
                        }
                    }
                }
            }

            if (expandedCount > 0) {
                lpoints = expandedPoints;
                numOfPoints = expandedCount;
            }
        }
    }

#ifdef DEBUG_POSSIBLE
    if (lumax_verbosity & DBG_SENDFRAME || lumax_verbosity & DBG_ALL) {
        printf("[DEBUG] Lumax_SendFrame: Device handle = 0x%x.\n", (unsigned int)(uintptr_t)handle);
        printf("[DEBUG] Lumax_SendFrame: numOfPoints = %d.\n", numOfPoints);
        for (int i = 0; i < numOfPoints; ++i)
            printf("[DEBUG] Lumax_SendFrame: point%d: CH1 = %d, CH2 = %d, CH3 = %d, CH4 = %d, CH5 = %d.\n", i, lpoints[i].Ch1, lpoints[i].Ch2, lpoints[i].Ch3, lpoints[i].Ch4, lpoints[i].Ch5);
    }
#endif

    // fill the buffer and send to device
    if (isOpen(&handle)) { // device is open
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_WARN || lumax_verbosity & DBG_ALL)
            fprintf(stderr, "[WARN] Lumax_SendFrame: device is closed.\n");
#endif
        return 1;
    }

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

    if (!lpoints || numOfPoints <= 0 || numOfPoints > 16 * MaxPoints / 2) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_WARN || lumax_verbosity & DBG_ALL)
            fprintf(stderr, "[WARN] Lumax_SendFrame: numOfPoints invalid.\n");
#endif
        return 1;
    }

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

    // TTL mode for Flavor 1, see lumax_pack_point():
    //   1 = per-point TTL, 2 = global TTL (TTLBuffer, set via Lumax_SetTTL).
    // The original driver always sends 7-byte points for Flavor 1.
    uint32_t ttlMode = (Flavor == 1 && (TTLAvailable & 2u)) ? 2 : 1;

    // big loop to fill the buffer
    int readOK;
    int npoint = 0;
    for (int i = 0; i < maxLoops; ++i) {
        // The first chunk carries the residual points. If there is no residual,
        // all chunks are full (an empty first chunk would be rejected).
        int pointsPerLoop = (i == 0 && residual != 0) ? residual : MaxPoints;
        int k = 0;
        for (int j = 0; j < pointsPerLoop; ++j)
            k += (int)lumax_pack_point(Flavor, ttlMode, &lpoints[npoint++], TTLBuffer, writeb + k);
        // write to Device
        readOK = writeFrameBuffer(handle, writeb, 32768, k, NextLoopCounts + i, 0);
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
        writeb[6] = lumax_checksum(writeb, 6);
        readOK = writeToDev(handle, writeb, 7u);
        if (!readOK) {
            uint8_t readb[4];
            readOK = readFromDev(handle, readb, 4u);
            if (!readOK && readb[0] == writeb[6]) {
                // high bit set: device reports how long it will be busy
                if ((readb[3] & 128) == 0) {
                    TimeUntilFree = 0;
                } else {
                    readb[3] &= 127;
                    TimeUntilFree = readb[1] + (readb[2] << 8) + MaxPoints * (readb[3] - 1);
                    TimeUntilFree = 1000 * TimeUntilFree / ScanSpeed;
                }
                TimeOffset = 1000 * numOfPoints / scanSpeed; // die zu erwartende Zeit, bis der nächste Frame gesendet werden kann
                LastDivisor = scanSpeed; // flags[45] folgt flags[46] (ScanSpeed)
#ifdef DEBUG_POSSIBLE
                if (lumax_verbosity & DBG_SENDFRAME || lumax_verbosity & DBG_ALL)
                    printf("[DEBUG] Lumax_SendFrame: TimeOffset = %d\n", TimeOffset);
#endif
                if (timeToWait)
                    *timeToWait = TimeUntilFree;
            }
        }
        if (NextLoopCounts)
            NextLoopCounts = 0;
        else
            NextLoopCounts = 8;
    }

    return readOK;
}

// Done
// Original: send the empty (blank) frame, then wait for the device buffer
// to be free before reporting success.
int Lumax_StopFrame(void* handle) {
    // Drop stale bytes from the FTDI RX FIFO first (e.g. a late 0x04
    // reply from a wait that timed out mid-reply). If such a byte is
    // still there when the blank frame is acked, the ack read returns
    // <stale> <xor> instead of <xor> 55, writeFrameBuffer fails and the
    // frame terminator is never sent - the card then keeps playing the
    // old frame and the beam stays on.
    if (handle)
        ftdi_tciflush((struct ftdi_context*)handle);
    int result = Lumax_SendFrame(handle, NULL, 0, 0, 0, NULL);
    if (result)
        return result;
    return Lumax_WaitForBuffer(handle, 2000, NULL, NULL);
}

// Done
// Maps the 0xCD response onto the original error codes.
// readb[12] is the status byte, readb[0] the first payload byte.
static int dongleStatus(int flag, const uint8_t *readb) {
    if (readb[12] == 2) {
        switch (readb[0]) {
        case 0xff: return 0xc9;
        case 0xfe: return 0xcc;
        case 1:    return (flag == 0x5a) ? 1 : 0xca;
        case 2:    return (flag == 0x5a) ? 0xcb : 1;
        default:   return 1;
        }
    }
    if (readb[12] == 1)
        return 0xcd;
    return readb[12] ? 1 : 0;
}

// Done
// Anti-debug / license key exchange (wire format: see PROTOCOL.md, 0xCD).
// The original dispatcher is mirrored flag by flag; writeVar/readVar are
// int* in the public API, but several flags use them as (or into) a
// 16-byte key buffer or multi-dword values — the caller decides.
int Lumax_DongleCom(void* handle, int flag, int address, int *writeVar, int *readVar) {
    if (!handle)
        return 1;

    uint8_t writeb[16]; // 16-byte key buffer
    uint8_t readb[13];
    uint32_t addr = (uint32_t)address;

    // flags 1, 0x2b, 0x55 and 0x5a always use the fixed license address
    if (flag == 1 || flag == 0x2b || flag == 0x55 || flag == 0x5a)
        addr = 0x49D4B13Au;

    memset(writeb, 0, sizeof(writeb));

    switch (flag) {
    case 1: // zeroed key; returns one byte in *readVar
        if (!readVar)
            return 1;
        break;
    case 0x2b: // first key byte from *writeVar
        if (!writeVar)
            return 1;
        writeb[0] = (uint8_t)*writeVar;
        break;
    case 0x55: // *readVar must carry the 0xff555500 sentinel
        if (!readVar || *readVar != 0xff555500)
            return 1;
        break;
    case 0x5a: // caller fills the whole key buffer
        if (!writeVar)
            return 1;
        memcpy(writeb, writeVar, 16);
        break;
    case 0x81: // zeroed key; returns three dwords into readVar
        if (!readVar)
            return 1;
        break;
    case 0x82: // key byte 4 encodes *writeVar; returns one byte
        if (!writeVar || !readVar)
            return 1;
        writeb[4] = (uint8_t)(((uint32_t)*writeVar << 2) & 0xcu);
        break;
    case 0x83: // key dword 0 from *readVar; key byte 4 from *writeVar
        if (!writeVar || !readVar)
            return 1;
        {
            uint32_t v = (uint32_t)*readVar;
            memcpy(writeb, &v, 4);
            writeb[4] = (uint8_t)(((uint32_t)*writeVar << 2) & 0xcu);
        }
        break;
    case 0x84: // *writeVar must carry the 0x55ff0055 sentinel; caller fills key buffer
        if (!writeVar || !readVar || *writeVar != 0x55ff0055)
            return 1;
        memcpy(writeb, writeVar, 16);
        break;
    case 0x85: // first two dwords from writeVar; returns two dwords
    case 0x86:
        if (!writeVar || !readVar)
            return 1;
        memcpy(writeb, writeVar, 8);
        break;
    default:
        return 1;
    }

    if (dongleCom(handle, (uint8_t)flag, addr, writeb, readb))
        return 1;

    int result = dongleStatus(flag, readb);
    if (result)
        return result;

    // 0x55 additionally reports 0xc9 when the second byte is 0xff
    if (flag == 0x55 && readb[1] == 0xff)
        return 0xc9;

    // a non-zero first response byte is a 0xce error on these flags
    if ((flag == 0x2b || flag == 0x55 || flag == 0x5a ||
         flag == 0x83 || flag == 0x84) && readb[0] != 0)
        return 0xce;

    switch (flag) {
    case 1:
    case 0x82:
        *readVar = readb[0];
        break;
    case 0x81:
        for (int i = 0; i < 3; ++i) {
            uint32_t v;
            memcpy(&v, readb + i * 4, 4);
            readVar[i] = (int)v;
        }
        break;
    case 0x85:
    case 0x86:
        for (int i = 0; i < 2; ++i) {
            uint32_t v;
            memcpy(&v, readb + i * 4, 4);
            readVar[i] = (int)v;
        }
        break;
    default:
        break;
    }

#ifdef DEBUG_POSSIBLE
    if (lumax_verbosity & DBG_SETDMXMODE || lumax_verbosity & DBG_ALL)
        printf("[DEBUG] Lumax_DongleCom: result = 0\n");
#endif
    return 0;
}

// Done
// 0x09 "init key" frame; only sent on Bare-Lumax cards (DmxFlavorMask != 0).
int sendInitKey(void *handle) {
    if (isOpen(&handle))
        return 1; // device is closed
    if (DmxFlavorMask == 0)
        return 1;

    uint8_t writeb[7] = {0x09, 0, 0, 0, 0, 0, 0};
    writeb[6] = lumax_checksum(writeb, 6);
    uint8_t readb;
    if (writeToDev(handle, writeb, 7u) || readFromDev(handle, &readb, 1u) || readb != writeb[6])
        return 1;
    return 0;
}

// Done
// 0x07 "key" frame that selects the Flavor on Bare-Lumax cards.
// On a plain LaserWorld Lumax card DmxFlavorMask is 0, so this is a no-op
// that sends nothing — matching the original wire behavior.
int sendKey(void *handle, int flavor) {
    if (isOpen(&handle))
        return 1; // device is closed

    int value = flavor & (DmxFlavorMask ? DmxFlavorMask : 1);
    if (value == 0)
        value = 1;

    int divisor;
    uint32_t maxScan;
    switch (value) {
    case 1:    divisor = 7; maxScan = 0x11170; break;
    case 2:    divisor = 6; maxScan = 0x11170; break;
    case 4:    divisor = 7; maxScan = 0xfde8;  break;
    case 8:    divisor = 8; maxScan = 0xd6d8;  break;
    case 0x10: divisor = 9; maxScan = 0xafc8;  break;
    default:   return 1;
    }

    // no DMX/flavor selection on a plain Lumax: nothing goes on the wire
    if (DmxFlavorMask == 0)
        return 0;

    if (sendInitKey(handle))
        return 1;

    uint32_t pointsPerChunk = 31500u / (uint32_t)divisor;
    uint8_t writeb[7] = {0x07, (uint8_t)value, 0,
                         (uint8_t)pointsPerChunk, (uint8_t)(pointsPerChunk / 256u), 0, 0};
    writeb[6] = lumax_checksum(writeb, 6);
    uint8_t readb[2];
    // the device echoes back the raw (unmasked) flavor parameter
    if (writeToDev(handle, writeb, 7u) || readFromDev(handle, readb, 2u)
        || readb[0] != writeb[6] || readb[1] != (uint8_t)flavor)
        return 1;

    // accepted: update the module state the original would have set
    Flavor = value;
    MaxPoints = (int)pointsPerChunk;
    MaxScanSpeed = (int)maxScan;
    return 0;
}

// Done
// numOfTxChannels / numOfRxChannels: DMX512 channel counts (0..512, 0 = off).
// Each record carries channels+1; a count of 0 is encoded as 2 with the
// "off" nibble set (byte 5: 0x10 TX / 0x11 RX). The device acks with the
// single checksum byte.
int Lumax_SetDmxMode(void *handle, int numOfTxChannels, int numOfRxChannels) {
    if (isOpen(&handle))
        return 1; // device is closed

    // The original also bails out when DMX is already active; that state is
    // only set by Lumax_SendDmx / Lumax_ReceiveDmx, which are not
    // implemented here, so the guard is omitted.

    int result = 0;
    uint8_t writeb[7];
    uint8_t readb;

    for (int rx = 0; rx < 2; ++rx) {
        int count = rx ? numOfRxChannels : numOfTxChannels;
        if (count < 0)
            continue; // negative counts are ignored by the original
        if (count > 0x400) {
            result = 1;
            continue;
        }
        int off = (count == 0);
        uint16_t encoded = (uint16_t)(off ? 2 : count + 1);
        writeb[0] = rx ? 0x0b : 0x0a;
        writeb[1] = encoded;
        writeb[2] = encoded / 256;
        writeb[3] = 0x0c;
        writeb[4] = 0xfb;
        writeb[5] = (uint8_t)((off << 4) | rx);
        writeb[6] = lumax_checksum(writeb, 6);
        if (writeToDev(handle, writeb, 7u) || readFromDev(handle, &readb, 1u) || readb != writeb[6])
            result = 1;
    }

#ifdef DEBUG_POSSIBLE
    if (lumax_verbosity & DBG_SETDMXMODE || lumax_verbosity & DBG_ALL)
        printf("[DEBUG] Lumax_SetDmxMode: result = %d\n", result);
#endif
    return result;
}

// Done
// Laser-safety exit net.
// If the host process dies while the device is still open (crash, Ctrl+C
// without cleanup, kill of the process, ...), the card would otherwise
// keep playing the last frame — with the beam on. On the first
// successful open we install an atexit handler plus signal handlers that
// blank the beam, switch the TTL outputs off and close the device, so
// the card is left in the same state as after a
// Lumax_StopFrame + Lumax_CloseDevice pair.
static int exitSafetyInstalled = 0;

static void lumaxEmergencyStop(void) {
    if (!openHandle)
        return;
    IsBusy = 0; // never trigger isOpen's busy-recovery (no re-open here)
    Lumax_StopFrame(openHandle);
    Lumax_SetTTL(openHandle, 0);
    Lumax_CloseDevice(openHandle);
}

static void lumaxAtExitStop(void) {
    lumaxEmergencyStop();
}

static void lumaxSignalStop(int signum) {
    lumaxEmergencyStop();
    signal(signum, SIG_DFL);
    raise(signum); // terminate with the conventional status for the signal
}

static void installExitSafety(void) {
    if (exitSafetyInstalled)
        return;
    exitSafetyInstalled = 1;
    atexit(lumaxAtExitStop);
    signal(SIGINT, lumaxSignalStop);
    signal(SIGTERM, lumaxSignalStop);
    signal(SIGHUP, lumaxSignalStop);
    signal(SIGQUIT, lumaxSignalStop);
}

void* Lumax_OpenDevice(int numDev, int channel) {
    uint32_t ftStatus;
    struct ftdi_context *ftHandle;

    if (((numDev < 1) || (numDev > 8)) || (channel != 0)) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_ERROR || lumax_verbosity & DBG_ALL)
            fprintf(stderr, "[ERROR] Lumax_OpenDevice: numDev or channel invalid.\n");
#endif
        return NULL;
    }
        
    if (openDev(numDev, (void**)&ftHandle)) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_ERROR || lumax_verbosity & DBG_ALL)
            fprintf(stderr, "[ERROR] Lumax_OpenDevice: openDev failed.\n");
#endif
        return NULL;
    }

    clearBuffer(ftHandle);
    const uint8_t idSize = 16;
    uint8_t id[16] = {0}; // zeroed so a failed readID cannot leave garbage in id[4]
    if (readID(ftHandle, id, idSize)) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_WARN || lumax_verbosity & DBG_ALL)
            printf("[WARN] Lumax_OpenDevice: readID failed.\n");
#endif
    } else {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_INFO || lumax_verbosity & DBG_ALL) {
            for (int i = 0; i < idSize; ++i)
                printf("[INFO] Lumax_OpenDevice: id[%d] = %d (%c).\n", i, id[i], id[i]);
            printf("\n");
        }
#endif
    }

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
    uint8_t deviceInfo[0x1cf];
    if (!readMemory(ftHandle, deviceInfo, 0, devSize)) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_OPENDEVICE || lumax_verbosity & DBG_ALL) {
            printf("[DEBUG] Lumax_OpenDevice: read device Info successfully.\n");
            for (int i = 0; i < devSize; ++i)
                printf("[DEBUG] Lumax_OpenDevice: deviceInfo[%d] = %d (%c).\n", i, deviceInfo[i], deviceInfo[i]);
            printf("\n");
        }
#endif

        // The original only performs the dongle exchange on cards with
        // id[4] > 6; we keep it for all cards because the dongle exchange
        // is required to open our (id[4] <= 6) card on real hardware.
        int ret = 0;
        int dongleOK = (Lumax_DongleCom(ftHandle, 1, 0x49D4B13Au, NULL, &ret) == 0);
        // Flavor selection (0x09/0x07) — a no-op on plain Lumax cards.
        if (dongleOK)
            sendKey(ftHandle, Flavor);
    }

#ifdef DEBUG_POSSIBLE
    if (lumax_verbosity & DBG_GENERAL || lumax_verbosity & DBG_ALL) {
        printf("[DEBUG] Lumax_OpenDevice: opened Lumax device with the following settings:\n");
        printf("            BufferLayout = %d\n", BufferLayout);
        printf("            TTLAvailable = %d\n", TTLAvailable);
        printf("            Flavor       = %d\n", Flavor);
    }
#endif

    // The original runs this tail sequentially and stops at the first
    // failure; the device is only reported opened when everything
    // succeeded.
    int result = Lumax_StopFrame(ftHandle);
    if (result) {
        // A previous host that was killed or crashed (no StopFrame on
        // exit) can leave the card mid-frame; after the USB disconnect
        // the card then does not answer the 0x04 wait of the stop frame
        // until it has been cycled again (see PROTOCOL.md, "Observed
        // device behavior"). Close and re-open once, then retry.
        Lumax_CloseDevice(ftHandle);
        if (openDev(numDev, (void**)&ftHandle)) {
#ifdef DEBUG_POSSIBLE
            if (lumax_verbosity & DBG_ERROR || lumax_verbosity & DBG_ALL)
                fprintf(stderr, "[ERROR] Lumax_OpenDevice: re-open after failed stop frame failed.\n");
#endif
            return NULL;
        }
        clearBuffer(ftHandle);
        result = Lumax_StopFrame(ftHandle);
    }
    if (!result)
        result = Lumax_SetDmxMode(ftHandle, 0, 0); // schaltet DMX aus
    if (!result)
        result = Lumax_SetTTL(ftHandle, 0);
    if (result) {
#ifdef DEBUG_POSSIBLE
        if (lumax_verbosity & DBG_ERROR || lumax_verbosity & DBG_ALL)
            fprintf(stderr, "[ERROR] Lumax_OpenDevice: device initialization failed.\n");
#endif
        Lumax_CloseDevice(ftHandle);
        return NULL;
    }

    installExitSafety();
    openHandle = ftHandle;
    return ftHandle;
}
