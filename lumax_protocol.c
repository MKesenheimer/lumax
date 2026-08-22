// Pure Lumax wire-protocol logic (no ftdi dependency), see lumax_protocol.h.

#include "lumax_protocol.h"

uint8_t lumax_checksum(const uint8_t *data, uint32_t len) {
    if (!data)
        return 0;
    uint8_t check = 0;
    for (uint32_t i = 0; i < len; ++i)
        check ^= data[i];
    return check;
}

uint32_t lumax_pack_point(uint32_t flavor, uint32_t ttlMode,
                          const TLumax_Point *point, uint8_t ttlValue, uint8_t *out) {
    if (!point || !out)
        return 0;

    // X/Y are 16-bit values (0..65535, 32768 = center). The wire format stores
    // them shifted right by 4 bits (12 bits each), packed into 3 bytes:
    //   byte 0: high nibble of X | high nibble of Y << 4
    //   byte 1: low 8 bits of X
    //   byte 2: low 8 bits of Y
    uint16_t l0 = point->Ch1 >> 4;
    uint16_t l1 = point->Ch2 >> 4;

    uint32_t k = 0;
    out[k++] = (uint8_t)(l0 / 256 + 16 * (l1 / 256));
    out[k++] = (uint8_t)l0;
    out[k++] = (uint8_t)l1;
    out[k++] = (uint8_t)(point->Ch3 >> 8);
    out[k++] = (uint8_t)(point->Ch4 >> 8);
    out[k++] = (uint8_t)(point->Ch5 >> 8);

    switch (flavor) {
    case 1:
        if (ttlMode == 1)
            out[k++] = point->TTL;
        else if (ttlMode == 2)
            out[k++] = ttlValue;
        break;
    case 2:
        break;
    case 4:
        out[k++] = (uint8_t)(point->Ch8 >> 8);
        break;
    case 8:
        out[k++] = (uint8_t)(point->Ch8 >> 8);
        out[k++] = (uint8_t)(point->Ch6 >> 8);
        break;
    case 16:
        out[k++] = (uint8_t)(point->Ch8 >> 8);
        out[k++] = (uint8_t)(point->Ch6 >> 8);
        out[k++] = (uint8_t)(point->Ch7 >> 8);
        break;
    default:
        return 0; // unknown flavor
    }
    return k;
}
