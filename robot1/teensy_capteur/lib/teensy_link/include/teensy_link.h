/*
 * Inter-Teensy link: frame definitions and codec.
 *
 * This header is the SINGLE definition of the wire format, shared by both
 * boards. The Teensy Moteur references it through lib_extra_dirs; it must
 * never be copied. The 2026 prototype defined its frames by hand on each side,
 * which is exactly how it ended up with three contradictory wiring comments and
 * no checksum at all.
 *
 * This header IS the reference for the wire format: frame layout, offsets,
 * flags, timeouts and counters. test/test_teensy_link locks it byte for byte;
 * any change here must keep those tests green.
 *
 * This file and teensy_link.cpp are deliberately free of any Arduino
 * dependency, so the codec can be built and tested in the PlatformIO "native"
 * environment. Serial access belongs to the caller: feed received bytes into
 * Decoder::push(), and write the sealed frame bytes out yourself.
 *
 * Layout assumptions, true on Teensy 4.1 (ARM Cortex-M7) and on x86-64 hosts:
 * little-endian byte order and IEEE-754 binary32 floats. The static_asserts at
 * the end of this header catch a packing mistake; they cannot catch a change of
 * endianness.
 */

#ifndef TEENSY_LINK_H
#define TEENSY_LINK_H

#include <stddef.h>
#include <stdint.h>

namespace teensy_link {

// === WIRE CONSTANTS ===

/** Start-of-frame delimiter. Not escaped: the CRC decides, not the delimiter. */
constexpr uint8_t STX = 0xA5;

/** Frame type identifiers. The type implies the length; there is no length field. */
constexpr uint8_t TYPE_ENCODER = 0x01;
constexpr uint8_t TYPE_POSE = 0x02;

constexpr size_t ENCODER_FRAME_SIZE = 21;
constexpr size_t POSE_FRAME_SIZE = 18;

/** Largest frame the decoder must be able to hold. */
constexpr size_t MAX_FRAME_SIZE = ENCODER_FRAME_SIZE;

// === EncoderFrame FLAGS (Moteur -> Capteur) ===

constexpr uint8_t ENC_FLAG_MOTORS_ENABLED = 1 << 0;
constexpr uint8_t ENC_FLAG_ENCODERS_VALID = 1 << 1;
constexpr uint8_t ENC_FLAG_SPEED_SATURATED = 1 << 2;
/** First frame after boot or a tracking reset: the deltas must be ignored. */
constexpr uint8_t ENC_FLAG_FIRST_FRAME = 1 << 3;

// === PoseFrame FLAGS (Capteur -> Moteur) ===

constexpr uint8_t POSE_FLAG_FILTER_READY = 1 << 0;
constexpr uint8_t POSE_FLAG_OPTICAL_OK = 1 << 1;
constexpr uint8_t POSE_FLAG_IMU_OK = 1 << 2;
constexpr uint8_t POSE_FLAG_LINK_DEGRADED = 1 << 3;
constexpr uint8_t POSE_FLAG_MEAS_REJECTED = 1 << 4;

// === TIMEOUTS ===
// Proposed values, to be confirmed on the bench: a threshold shorter than the
// real link jitter produces phantom cut-offs.

/** No expected frame for this long: the link is degraded. */
constexpr uint32_t LINK_TIMEOUT_MS = 50;
/** No expected frame for this long: the Moteur cuts the motors. */
constexpr uint32_t LINK_FATAL_MS = 200;

// === FRAMES ===

#pragma pack(push, 1)

/**
 * @brief Moteur -> Capteur, 100 Hz. This frame is the system tick.
 *
 * Carries RAW encoder counts, not millimetres: the filter needs three
 * independent wheel measurements with honest noise, not a pre-fused estimate.
 * This is also what lets the noise threshold be
 * expressed in counts, which is the fix for the millimetre threshold bug.
 *
 * dt_us is MEASURED, never assumed equal to the nominal period. The prototype
 * paced itself with delay(10) and ran at 83-87 Hz while announcing 100 Hz; with
 * no dt on the wire, that error went straight into the odometry.
 */
struct EncoderFrame {
    uint8_t stx;      // 0  : STX, set by seal()
    uint8_t type;     // 1  : TYPE_ENCODER, set by seal()
    uint8_t seq;      // 2  : incremented every frame, wraps at 256 by design
    uint32_t dt_us;   // 3  : microseconds since the previous EncoderFrame
    int32_t d1;       // 7  : raw count delta, wheel 1 (120 deg)
    int32_t d2;       // 11 : raw count delta, wheel 2 (240 deg)
    int32_t d3;       // 15 : raw count delta, wheel 3 (0 deg)
    uint8_t flags;    // 19 : ENC_FLAG_*
    uint8_t crc8;     // 20 : over bytes 1..19, set by seal()
};

/**
 * @brief Capteur -> Moteur, one per consumed EncoderFrame.
 *
 * Millimetres and radians, in float: the whole system is in mm/rad, and on a
 * 3000 mm table a float resolves to about 0.0002 mm.
 *
 * echo_seq is the seq of the EncoderFrame this pose was computed from. It is
 * what makes the round-trip time and, more importantly, the AGE OF THE POSE AT
 * THE PID measurable: a pose that is correct but three cycles old behaves like
 * a pure delay in the control loop.
 *
 * There is deliberately no sigma field yet: sigma only means something once the
 * EKF runs with a tuned Q. An unvalidated sigma is worse than none, because it
 * gets trusted. It will come back as a new frame type.
 */
struct PoseFrame {
    uint8_t stx;       // 0  : STX, set by seal()
    uint8_t type;      // 1  : TYPE_POSE, set by seal()
    uint8_t seq;       // 2  : Capteur's own sequence
    uint8_t echo_seq;  // 3  : seq of the EncoderFrame that produced this pose
    float x;           // 4  : mm, world frame
    float y;           // 8  : mm, world frame
    float theta;       // 12 : rad, normalized to [-pi, pi)
    uint8_t flags;     // 16 : POSE_FLAG_*
    uint8_t crc8;      // 17 : over bytes 1..16, set by seal()
};

#pragma pack(pop)

static_assert(sizeof(EncoderFrame) == ENCODER_FRAME_SIZE,
              "EncoderFrame is not packed to its wire size");
static_assert(sizeof(PoseFrame) == POSE_FRAME_SIZE,
              "PoseFrame is not packed to its wire size");

// === CRC ===

/**
 * @brief CRC-8, polynomial 0x07, init 0x00, no reflection, no final XOR.
 *
 * Bit-for-bit identical to CRC::digest() of common/usb_com/cpp/include/crc.h,
 * and interoperable with it. Reimplemented table-free here for two reasons:
 * crc.h includes <Arduino.h> (so it cannot build in the native environment),
 * and its 256-byte table is a non-static member, so every instance carries a
 * copy in RAM. The cost is negligible: 21 bytes x 8 iterations at 100 Hz.
 *
 * @param data Pointer to the bytes to digest.
 * @param size Number of bytes.
 * @return The 8-bit CRC.
 */
uint8_t crc8(const uint8_t *data, size_t size);

/** Sets stx, type and crc8. Call it after filling the data fields. */
void seal(EncoderFrame &frame);
void seal(PoseFrame &frame);

/** Returns the frame size for a known type, or 0 if the type is unknown. */
size_t frame_size_for(uint8_t type);

// === COUNTERS ===

/**
 * @brief Link health counters. Reset at boot only.
 *
 * These are what the link bench reports (RTT, lost frames, invalid CRC over
 * 10 minutes). crc_errors and bytes_discarded answer different questions: a
 * frame rejected on CRC arrived and was corrupted, while discarded bytes mean
 * the receiver was not on a frame boundary at all.
 */
struct Counters {
    uint32_t frames_sent = 0;
    uint32_t frames_received = 0;
    /** Candidate frames rejected because the CRC did not match. */
    uint32_t crc_errors = 0;
    /** Candidate frames whose type byte is not known. */
    uint32_t unknown_type = 0;
    /** Bytes thrown away while not aligned on a frame: the cost of resyncing. */
    uint32_t bytes_discarded = 0;
    /** Gaps in the received sequence, i.e. frames lost rather than corrupted. */
    uint32_t seq_gaps = 0;
    /** Crossings of LINK_TIMEOUT_MS. */
    uint32_t timeouts = 0;
};

/**
 * @brief Tracks sequence continuity of one incoming frame stream.
 *
 * Kept separate from Decoder because each board watches a different stream, and
 * because a gap is a property of the stream, not of the byte decoder.
 */
class SeqTracker {
   public:
    /**
     * @brief Account for a received sequence number.
     *
     * @param seq The seq field of the frame just received.
     * @return Number of frames missed before this one (0 when contiguous).
     */
    uint8_t observe(uint8_t seq);

    void reset() { has_previous_ = false; }

   private:
    uint8_t previous_ = 0;
    bool has_previous_ = false;
};

// === DECODER ===

/**
 * @brief Byte-at-a-time frame decoder with resynchronisation.
 *
 * No allocation, no blocking, callable one byte at a time straight from a
 * serial read loop. On a rejected candidate the hunt for STX resumes at the
 * byte AFTER the one mistaken for an STX, never after the whole candidate:
 * dropping a full frame's worth of bytes on a false STX would swallow the valid
 * frames that follow it.
 */
class Decoder {
   public:
    /**
     * @brief Feed one received byte.
     *
     * @param byte The byte read from the link.
     * @return true when a complete, CRC-valid frame became available; read it
     *         with as_encoder_frame() or as_pose_frame() before the next push().
     */
    bool push(uint8_t byte);

    /** Type of the frame made available by the last push() that returned true. */
    uint8_t ready_type() const { return ready_type_; }

    /** Returns the frame, or nullptr when the ready frame is of another type. */
    const EncoderFrame *as_encoder_frame() const;
    const PoseFrame *as_pose_frame() const;

    const Counters &counters() const { return counters_; }
    Counters &counters() { return counters_; }

    /** Drops any partially accumulated frame. Does not touch the counters. */
    void reset();

   private:
    /** Consumes what is buffered, returning true as soon as a frame is valid. */
    bool parse();
    /** Discards the leading byte and counts it. */
    void discard_leading();

    uint8_t buffer_[MAX_FRAME_SIZE] = {};
    size_t filled_ = 0;

    uint8_t ready_[MAX_FRAME_SIZE] = {};
    uint8_t ready_type_ = 0;

    Counters counters_;
};

}  // namespace teensy_link

#endif  // TEENSY_LINK_H
