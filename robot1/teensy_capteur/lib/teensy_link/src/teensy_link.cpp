/*
 * Inter-Teensy link codec. See include/teensy_link.h.
 *
 * No Arduino dependency: this file must build in the "native" environment.
 */

#include "teensy_link.h"

#include <string.h>

namespace teensy_link {

namespace {

/** CRC-8 polynomial, same as the USB link (CRC-8-ITU). */
constexpr uint8_t CRC8_POLYNOMIAL = 0x07;

/** Offset of the first byte covered by the CRC: the type byte. */
constexpr size_t CRC_FIRST_BYTE = 1;

/** Number of bytes NOT covered by the CRC: the leading STX and the CRC itself. */
constexpr size_t CRC_EXCLUDED_BYTES = 2;

/** Digests the covered span of a frame already laid out in a byte buffer. */
uint8_t frame_crc(const uint8_t *frame, size_t frame_size) {
    return crc8(frame + CRC_FIRST_BYTE, frame_size - CRC_EXCLUDED_BYTES);
}

}  // namespace

uint8_t crc8(const uint8_t *data, size_t size) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            // Shifting left drops the top bit, so test it before shifting.
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ CRC8_POLYNOMIAL)
                              : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

size_t frame_size_for(uint8_t type) {
    switch (type) {
        case TYPE_ENCODER:
            return ENCODER_FRAME_SIZE;
        case TYPE_POSE:
            return POSE_FRAME_SIZE;
        default:
            // Unknown type. The caller counts it and resynchronises: an
            // unrecognised type must never be fatal, so that both directions of
            // the link can gain new frame types independently.
            return 0;
    }
}

void seal(EncoderFrame &frame) {
    frame.stx = STX;
    frame.type = TYPE_ENCODER;
    frame.crc8 = frame_crc(reinterpret_cast<const uint8_t *>(&frame), ENCODER_FRAME_SIZE);
}

void seal(PoseFrame &frame) {
    frame.stx = STX;
    frame.type = TYPE_POSE;
    frame.crc8 = frame_crc(reinterpret_cast<const uint8_t *>(&frame), POSE_FRAME_SIZE);
}

uint8_t SeqTracker::observe(uint8_t seq) {
    if (!has_previous_) {
        previous_ = seq;
        has_previous_ = true;
        return 0;
    }

    // Unsigned arithmetic on uint8_t wraps, which is what we want: seq rolling
    // over from 255 to 0 is contiguous, not a gap of 255.
    const uint8_t advance = static_cast<uint8_t>(seq - previous_);
    previous_ = seq;

    // advance == 1 is the nominal case. advance == 0 means the same frame was
    // seen twice, which is not a gap either.
    return (advance > 1) ? static_cast<uint8_t>(advance - 1) : 0;
}

void Decoder::reset() {
    filled_ = 0;
    ready_type_ = 0;
}

void Decoder::discard_leading() {
    if (filled_ == 0) {
        return;
    }
    memmove(buffer_, buffer_ + 1, filled_ - 1);
    --filled_;
    ++counters_.bytes_discarded;
}

bool Decoder::push(uint8_t byte) {
    if (filled_ >= MAX_FRAME_SIZE) {
        // Defensive: parse() never leaves a full buffer behind, because a buffer
        // holding MAX_FRAME_SIZE bytes is always either a complete frame or a
        // rejected candidate. Drop the oldest byte rather than overflow.
        discard_leading();
    }
    buffer_[filled_++] = byte;
    return parse();
}

bool Decoder::parse() {
    while (filled_ > 0) {
        if (buffer_[0] != STX) {
            discard_leading();
            continue;
        }

        // Need the type byte to know how long this candidate is.
        if (filled_ < 2) {
            return false;
        }

        const size_t size = frame_size_for(buffer_[1]);
        if (size == 0) {
            ++counters_.unknown_type;
            discard_leading();
            continue;
        }

        // Wait for the rest of the candidate.
        if (filled_ < size) {
            return false;
        }

        if (frame_crc(buffer_, size) != buffer_[size - 1]) {
            ++counters_.crc_errors;
            // Resume the hunt at the byte after this STX, not after the whole
            // candidate: this STX may well have been a payload byte, and the
            // real frame can start inside what we just examined.
            discard_leading();
            continue;
        }

        memcpy(ready_, buffer_, size);
        ready_type_ = buffer_[1];

        // Consume the accepted frame, keeping any bytes already received behind
        // it so a back-to-back frame is not lost.
        memmove(buffer_, buffer_ + size, filled_ - size);
        filled_ -= size;

        ++counters_.frames_received;
        return true;
    }
    return false;
}

const EncoderFrame *Decoder::as_encoder_frame() const {
    if (ready_type_ != TYPE_ENCODER) {
        return nullptr;
    }
    return reinterpret_cast<const EncoderFrame *>(ready_);
}

const PoseFrame *Decoder::as_pose_frame() const {
    if (ready_type_ != TYPE_POSE) {
        return nullptr;
    }
    return reinterpret_cast<const PoseFrame *>(ready_);
}

}  // namespace teensy_link
