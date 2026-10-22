#ifndef COBS_H
#define COBS_H

#include <stddef.h>
#include <stdint.h>

/**
 * Consistent Overhead Byte Stuffing.
 *
 * Encoding removes every 0x00 from a block of bytes, for at most one extra
 * byte per 254, so that 0x00 can delimit frames on the wire: a receiver that
 * loses track simply waits for the next 0x00. Same algorithm, byte for byte,
 * as common/usb_com/python/com/framing.py.
 *
 * Arduino-free: plain C++, testable on a host.
 */

/** Largest encoded size of `len` bytes (without the delimiter). */
constexpr size_t cobs_max_encoded_size(size_t len) {
    return len + len / 254 + 1;
}

/**
 * @brief Encodes `len` bytes of `src` into `dst`, delimiter NOT added.
 * @param dst At least cobs_max_encoded_size(len) bytes.
 * @return The encoded size.
 */
size_t cobs_encode(const uint8_t* src, size_t len, uint8_t* dst);

/**
 * @brief Decodes `len` encoded bytes (delimiter excluded) into `dst`.
 * @param capacity Size of `dst`.
 * @return The decoded size, or -1 if the input is not valid COBS (a 0x00
 *     inside, a block running past the end) or does not fit in `dst`.
 */
int cobs_decode(const uint8_t* src, size_t len, uint8_t* dst, size_t capacity);

#endif  // COBS_H
