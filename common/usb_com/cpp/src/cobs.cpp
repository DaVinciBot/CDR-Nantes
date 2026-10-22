#include <cobs.h>

size_t cobs_encode(const uint8_t* src, size_t len, uint8_t* dst) {
    // Each block starts with a code byte: the distance to the next zero (or
    // to the end of a full 254-byte block), written once the block is known.
    size_t write = 1;
    size_t code_index = 0;
    uint8_t code = 1;

    for (size_t read = 0; read < len; ++read) {
        if (src[read] == 0) {
            dst[code_index] = code;
            code = 1;
            code_index = write++;
        } else {
            dst[write++] = src[read];
            ++code;
            if (code == 0xFF) {
                dst[code_index] = code;
                code = 1;
                code_index = write++;
            }
        }
    }
    dst[code_index] = code;
    return write;
}

int cobs_decode(const uint8_t* src, size_t len, uint8_t* dst, size_t capacity) {
    size_t read = 0;
    size_t write = 0;

    while (read < len) {
        const uint8_t code = src[read++];
        if (code == 0) return -1;

        for (uint8_t i = 1; i < code; ++i) {
            if (read >= len || write >= capacity) return -1;
            const uint8_t b = src[read++];
            if (b == 0) return -1;
            dst[write++] = b;
        }
        // A block shorter than 254 stands for a zero, except the last one.
        if (code != 0xFF && read < len) {
            if (write >= capacity) return -1;
            dst[write++] = 0;
        }
    }
    return static_cast<int>(write);
}
