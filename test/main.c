// gcc main.c -o main -L. -llumax

/* Laserworld USB:
  Product ID:	0xc88a
  Vendor ID:	0x0403  (Future Technology Devices International Limited)
  Version:	6.00
  Serial Number:	FTPY4YIU
  Speed:	Up to 12 Mb/s
  Manufacturer:	Minilumax
  Location ID:	0x14200000 / 3
  Current Available (mA):	500
  Current Required (mA):	500
  Extra Operating Current (mA):	0
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <stdint.h>
#include "lumax.h"

// TODO
int main(int argc, char *argv[]) {
    //lumax_verbosity = DBG_WRITETODEV | DBG_READMEMORY | DBG_OPENDEVICE;

    int result;
    int NumOfCards = Lumax_GetPhysicalDevices();
    printf("Number of MiniLumax devices: %i\n", NumOfCards);
    if (NumOfCards > 0)
    {
        void* Handle = Lumax_OpenDevice(1, 0);
        printf("Lumax_OpenDevice returned handle: 0x%x\n", (unsigned int)Handle);

        if (Handle > 0) {
            result = Lumax_SetTTL(Handle, 0xff);
            printf("Lumax_SetTTL return code: %i\n", result);

            int TimeToWait, BufferChanged;
            result = Lumax_WaitForBuffer(Handle, 0, &TimeToWait, &BufferChanged);
            printf("Lumax_WaitForBuffer return code: %i   TimeToWait=%i, BufferChanged=%i\n", result, TimeToWait, BufferChanged);

#if 1   
            const int numberOfFrames = 1;
            int f = 0;
            while(1) {
                f = (f + 1) % numberOfFrames;
                const int numOfPoints = 1;
                TLumax_Point Points[numOfPoints];
                const int mid = 32768;
                for (int i = 0; i < numOfPoints; i++) {
                    Points[i].Ch1 = mid + (f + 50) * 40 * sin((float)2 * M_PI * i / numOfPoints); // x
                    Points[i].Ch2 = mid + (f + 50) * 40 * cos((float)2 * M_PI * i / numOfPoints); // y
                    Points[i].Ch3 = 100 * 256 + 100 * 256 * sin((float)2 * M_PI * f / numberOfFrames); // r
                    Points[i].Ch4 = 100 * 256 + 100 * 256 * sin((float)2 * M_PI * f / numberOfFrames + 2 * M_PI / 3.0); // g
                    Points[i].Ch5 = 100 * 256 + 100 * 256 * sin((float)2 * M_PI * f / numberOfFrames + 4 * M_PI / 3.0); // b
                    Points[i].Ch8 = 0;
                    Points[i].Ch6 = 0;
                    Points[i].Ch7 = 0;
                    //Points[i].TTL = TTL;
                    printf("Points[%d].Chi = (0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n", i, Points[i].Ch1, Points[i].Ch2, Points[i].Ch3, Points[i].Ch4, Points[i].Ch5);
                }
                result = Lumax_SendFrame(Handle, Points, numOfPoints, 10000, 0, NULL);
                //printf("Lumax_SendFrame return code: %i\n", result);
                result = Lumax_WaitForBuffer(Handle, 17, &TimeToWait, &BufferChanged); // 17 ms/f -> 60fps
                //printf("Lumax_WaitForBuffer return code: %i   TimeToWait=%i, BufferChanged=%i\n", result, TimeToWait, BufferChanged);
                break;
            }
#endif
            usleep(1000000);
            result = Lumax_StopFrame(Handle);
            result = Lumax_CloseDevice(Handle);
            printf("Lumax_CloseDevice return code: %i\n", result);
        }
    }

    return result;
}
