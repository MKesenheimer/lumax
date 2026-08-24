#pragma once

#include <stdint.h>
#include "lumax.h"

// Pure Lumax wire-protocol logic, independent of ftdi.
// See PROTOCOL.md for the full documentation of the protocol.

// XOR checksum over len bytes; this is the checksum byte used in every
// protocol record (last byte of the record, echoed back by the device).
uint8_t lumax_checksum(const uint8_t *data, uint32_t len);

// Pack a single point into its wire format.
//
// flavor:  1 = RGB (LaserWorld Lumax),
//          2 = RGB, no TTL byte,
//          4 = RGB + Cyan,
//          8 = RGB + Cyan + DeepBlue,
//          16 = RGB + Cyan + DeepBlue + Yellow
// ttlMode: only meaningful for flavor 1
//          0 = no TTL byte,
//          1 = per-point TTL byte (point->TTL),
//          2 = global TTL byte (ttlValue, i.e. the TTLBuffer set via Lumax_SetTTL)
//
// Returns the number of bytes written (6..9), or 0 for an unknown flavor.
uint32_t lumax_pack_point(uint32_t flavor, uint32_t ttlMode,
                          const TLumax_Point *point, uint8_t ttlValue, uint8_t *out);
