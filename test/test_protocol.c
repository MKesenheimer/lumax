// Unit tests for the pure Lumax wire-protocol logic (lumax_protocol.c).
// Built and run by `make check`. No device, no ftdi required.
//
// These tests pin down the exact byte layout the device expects, so a
// regression in the point packing or the checksum is caught without hardware.

#include <stdio.h>
#include <string.h>
#include "lumax_protocol.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, msg) do { \
    ++checks; \
    if (!(cond)) { \
        printf("[FAIL] %s (line %d)\n", (msg), __LINE__); \
        ++failures; \
    } \
} while (0)

// Compare a packed buffer against an expected byte array.
static void check_bytes(const char *msg, const uint8_t *expected, uint32_t expectedLen,
                        uint32_t len, const uint8_t *out) {
    ++checks;
    if (len != expectedLen || memcmp(expected, out, expectedLen) != 0) {
        printf("[FAIL] %s\n", msg);
        printf("       expected (%u):", expectedLen);
        for (uint32_t i = 0; i < expectedLen; ++i) printf(" %02X", expected[i]);
        printf("\n       got      (%u):", len);
        for (uint32_t i = 0; i < len; ++i) printf(" %02X", out[i]);
        printf("\n");
        ++failures;
    } else {
        printf("[ ok ] %s\n", msg);
    }
}

// A point with 12-bit X/Y values that exercise the high-nibble packing.
//   Ch1 = 0x1234 -> l0 = 0x123  (high nibble 0x01, low byte 0x23)
//   Ch2 = 0xABCD -> l1 = 0xABC  (high nibble 0x0A, low byte 0xBC)
//   byte0 = 0x01 | (0x0A << 4) = 0xA1
static void makePoint(TLumax_Point *p) {
    p->Ch1 = 0x1234;
    p->Ch2 = 0xABCD;
    p->Ch3 = 0x11FF;
    p->Ch4 = 0x22FF;
    p->Ch5 = 0x33FF;
    p->Ch6 = 0x44FF;
    p->Ch7 = 0x55FF;
    p->Ch8 = 0x66FF;
    p->TTL = 0x77;
}

static void test_checksum(void) {
    const uint8_t a[] = {0x01, 0x02, 0x03};
    CHECK(lumax_checksum(a, 3) == 0x00, "checksum: 01^02^03 == 00");

    const uint8_t b[] = {0xc9, 0, 0, 0, 0, 0};
    CHECK(lumax_checksum(b, 6) == 0xc9, "checksum: readID record == 0xc9");

    const uint8_t c[] = {0xCA, 0x00, 0x00, 0xCF, 0x01, 0x00};
    CHECK(lumax_checksum(c, 6) == 0x04, "checksum: readMemory(0..0x1cf) record");

    CHECK(lumax_checksum((const uint8_t *)"\x00", 0) == 0x00, "checksum: len 0 == 0");
    CHECK(lumax_checksum(NULL, 5) == 0x00, "checksum: NULL data == 0");
}

static void test_pack_point(void) {
    TLumax_Point p;
    makePoint(&p);
    uint8_t out[16];

    const uint8_t f1_none[6] = {0xA1, 0x23, 0xBC, 0x11, 0x22, 0x33};
    check_bytes("pack: flavor 1, no TTL byte (6 bytes)",
                f1_none, 6, lumax_pack_point(1, 0, &p, 0, out), out);

    const uint8_t f1_perpoint[7] = {0xA1, 0x23, 0xBC, 0x11, 0x22, 0x33, 0x77};
    check_bytes("pack: flavor 1, per-point TTL (7 bytes)",
                f1_perpoint, 7, lumax_pack_point(1, 1, &p, 0, out), out);

    const uint8_t f1_global[7] = {0xA1, 0x23, 0xBC, 0x11, 0x22, 0x33, 0x99};
    check_bytes("pack: flavor 1, global TTL byte (7 bytes)",
                f1_global, 7, lumax_pack_point(1, 2, &p, 0x99, out), out);

    const uint8_t f2[6] = {0xA1, 0x23, 0xBC, 0x11, 0x22, 0x33};
    check_bytes("pack: flavor 2, no TTL (6 bytes)",
                f2, 6, lumax_pack_point(2, 0, &p, 0, out), out);

    const uint8_t f4[7] = {0xA1, 0x23, 0xBC, 0x11, 0x22, 0x33, 0x66};
    check_bytes("pack: flavor 4, +Cyan (7 bytes)",
                f4, 7, lumax_pack_point(4, 0, &p, 0, out), out);

    const uint8_t f8[8] = {0xA1, 0x23, 0xBC, 0x11, 0x22, 0x33, 0x66, 0x44};
    check_bytes("pack: flavor 8, +Cyan +DeepBlue (8 bytes)",
                f8, 8, lumax_pack_point(8, 0, &p, 0, out), out);

    const uint8_t f16[9] = {0xA1, 0x23, 0xBC, 0x11, 0x22, 0x33, 0x66, 0x44, 0x55};
    check_bytes("pack: flavor 16, +Cyan +DeepBlue +Yellow (9 bytes)",
                f16, 9, lumax_pack_point(16, 0, &p, 0, out), out);

    // Center point: X = Y = 32768 (0x8000) -> l0 = l1 = 0x800.
    TLumax_Point c;
    makePoint(&c);
    c.Ch1 = 0x8000;
    c.Ch2 = 0x8000;
    const uint8_t center[6] = {0x88, 0x00, 0x00, 0x11, 0x22, 0x33};
    check_bytes("pack: center point (X=Y=0x8000)",
                center, 6, lumax_pack_point(1, 0, &c, 0, out), out);

    // Zero point: X = Y = 0.
    TLumax_Point z;
    makePoint(&z);
    z.Ch1 = 0;
    z.Ch2 = 0;
    const uint8_t zero[6] = {0x00, 0x00, 0x00, 0x11, 0x22, 0x33};
    check_bytes("pack: zero point (X=Y=0)",
                zero, 6, lumax_pack_point(1, 0, &z, 0, out), out);

    // Unknown flavor / bad pointers are rejected.
    CHECK(lumax_pack_point(3, 0, &p, 0, out) == 0, "pack: unknown flavor rejected");
    CHECK(lumax_pack_point(1, 0, NULL, 0, out) == 0, "pack: NULL point rejected");
    CHECK(lumax_pack_point(1, 0, &p, 0, NULL) == 0, "pack: NULL out rejected");
}

int main(void) {
    test_checksum();
    test_pack_point();
    if (failures) {
        printf("\n%d of %d checks FAILED\n", failures, checks);
        return 1;
    }
    printf("\nAll %d checks passed.\n", checks);
    return 0;
}
