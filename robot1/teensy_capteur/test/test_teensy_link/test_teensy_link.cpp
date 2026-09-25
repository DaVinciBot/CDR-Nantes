/*
 * Host tests for the inter-Teensy link codec.
 *
 * Run with: pio test -e native   (from robot1/teensy_capteur/)
 *
 * These tests are the reason lib/teensy_link must stay free of any Arduino
 * dependency. They cover what cannot be checked by staring at the code: that the
 * CRC is interoperable with the USB link, that the frame layout matches
 * teensy_link.h byte for byte, and above all that the decoder recovers from a corrupted or
 * misaligned stream instead of silently swallowing the frames that follow.
 */

#include <string.h>
#include <unity.h>

#include "teensy_link.h"

using namespace teensy_link;

/*
 * CRC-8 lookup table copied verbatim from common/usb_com/cpp/include/crc.h.
 *
 * table[i] is, by construction, the CRC of the single byte i with an init of 0.
 * Asserting crc8({i}) == table[i] for all 256 values is the interoperability
 * proof: the table-free implementation in teensy_link must agree with the USB
 * link's table-driven one on every input, not just on the ones we happened to
 * try.
 */
static const uint8_t USB_CRC_TABLE[256] = {
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
    0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
    0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
    0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
    0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
    0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
    0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
    0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
    0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
    0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
    0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
    0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
    0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
    0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
    0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
    0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
    0xfa, 0xfd, 0xf4, 0xf3};

void setUp(void) {}
void tearDown(void) {}

// === HELPERS ===

/** Builds a sealed EncoderFrame with recognisable field values. */
static EncoderFrame make_encoder_frame(uint8_t seq) {
    EncoderFrame frame;
    frame.seq = seq;
    frame.dt_us = 10000;
    frame.d1 = 1234;
    frame.d2 = -5678;
    frame.d3 = 0;
    frame.flags = ENC_FLAG_MOTORS_ENABLED | ENC_FLAG_ENCODERS_VALID;
    seal(frame);
    return frame;
}

/** Builds a sealed PoseFrame with recognisable field values. */
static PoseFrame make_pose_frame(uint8_t seq, uint8_t echo_seq) {
    PoseFrame frame;
    frame.seq = seq;
    frame.echo_seq = echo_seq;
    frame.x = 1000.0f;
    frame.y = 500.0f;
    frame.theta = 0.5f;
    frame.flags = POSE_FLAG_FILTER_READY;
    seal(frame);
    return frame;
}

/** Pushes every byte of a buffer, returning how many frames came out. */
static int push_all(Decoder &decoder, const uint8_t *bytes, size_t size,
                    uint8_t *last_type = nullptr) {
    int frames = 0;
    for (size_t i = 0; i < size; ++i) {
        if (decoder.push(bytes[i])) {
            ++frames;
            if (last_type != nullptr) {
                *last_type = decoder.ready_type();
            }
        }
    }
    return frames;
}

// === CRC ===

void test_crc8_matches_usb_table_on_every_byte(void) {
    for (int i = 0; i < 256; ++i) {
        const uint8_t byte = static_cast<uint8_t>(i);
        TEST_ASSERT_EQUAL_UINT8(USB_CRC_TABLE[i], crc8(&byte, 1));
    }
}

void test_crc8_of_empty_span_is_zero(void) {
    TEST_ASSERT_EQUAL_UINT8(0x00, crc8(nullptr, 0));
}

// === LAYOUT ===

void test_frame_sizes_match_the_wire_format(void) {
    TEST_ASSERT_EQUAL_UINT32(21u, sizeof(EncoderFrame));
    TEST_ASSERT_EQUAL_UINT32(18u, sizeof(PoseFrame));
    TEST_ASSERT_EQUAL_UINT32(21u, frame_size_for(TYPE_ENCODER));
    TEST_ASSERT_EQUAL_UINT32(18u, frame_size_for(TYPE_POSE));
}

void test_unknown_type_has_no_size(void) {
    TEST_ASSERT_EQUAL_UINT32(0u, frame_size_for(0x00));
    TEST_ASSERT_EQUAL_UINT32(0u, frame_size_for(0xFF));
}

void test_field_offsets_match_the_wire_format(void) {
    // The offsets commented in teensy_link.h are the contract; a packing change
    // must fail here and not on the bench.
    const EncoderFrame encoder = make_encoder_frame(0);
    const uint8_t *raw = reinterpret_cast<const uint8_t *>(&encoder);
    TEST_ASSERT_EQUAL_UINT32(0u, (size_t)((const uint8_t *)&encoder.stx - raw));
    TEST_ASSERT_EQUAL_UINT32(1u, (size_t)((const uint8_t *)&encoder.type - raw));
    TEST_ASSERT_EQUAL_UINT32(2u, (size_t)((const uint8_t *)&encoder.seq - raw));
    TEST_ASSERT_EQUAL_UINT32(3u, (size_t)((const uint8_t *)&encoder.dt_us - raw));
    TEST_ASSERT_EQUAL_UINT32(7u, (size_t)((const uint8_t *)&encoder.d1 - raw));
    TEST_ASSERT_EQUAL_UINT32(11u, (size_t)((const uint8_t *)&encoder.d2 - raw));
    TEST_ASSERT_EQUAL_UINT32(15u, (size_t)((const uint8_t *)&encoder.d3 - raw));
    TEST_ASSERT_EQUAL_UINT32(19u, (size_t)((const uint8_t *)&encoder.flags - raw));
    TEST_ASSERT_EQUAL_UINT32(20u, (size_t)((const uint8_t *)&encoder.crc8 - raw));

    const PoseFrame pose = make_pose_frame(0, 0);
    const uint8_t *pose_raw = reinterpret_cast<const uint8_t *>(&pose);
    TEST_ASSERT_EQUAL_UINT32(3u, (size_t)((const uint8_t *)&pose.echo_seq - pose_raw));
    TEST_ASSERT_EQUAL_UINT32(4u, (size_t)((const uint8_t *)&pose.x - pose_raw));
    TEST_ASSERT_EQUAL_UINT32(12u, (size_t)((const uint8_t *)&pose.theta - pose_raw));
    TEST_ASSERT_EQUAL_UINT32(16u, (size_t)((const uint8_t *)&pose.flags - pose_raw));
    TEST_ASSERT_EQUAL_UINT32(17u, (size_t)((const uint8_t *)&pose.crc8 - pose_raw));
}

void test_seal_sets_delimiter_and_type(void) {
    const EncoderFrame encoder = make_encoder_frame(7);
    TEST_ASSERT_EQUAL_UINT8(STX, encoder.stx);
    TEST_ASSERT_EQUAL_UINT8(TYPE_ENCODER, encoder.type);

    const PoseFrame pose = make_pose_frame(7, 9);
    TEST_ASSERT_EQUAL_UINT8(STX, pose.stx);
    TEST_ASSERT_EQUAL_UINT8(TYPE_POSE, pose.type);
}

// === ROUND TRIPS ===

void test_encoder_frame_round_trip(void) {
    Decoder decoder;
    const EncoderFrame sent = make_encoder_frame(42);

    const int frames = push_all(decoder, reinterpret_cast<const uint8_t *>(&sent),
                                sizeof(sent));

    TEST_ASSERT_EQUAL_INT(1, frames);
    TEST_ASSERT_EQUAL_UINT8(TYPE_ENCODER, decoder.ready_type());
    TEST_ASSERT_NULL(decoder.as_pose_frame());

    const EncoderFrame *received = decoder.as_encoder_frame();
    TEST_ASSERT_NOT_NULL(received);
    TEST_ASSERT_EQUAL_UINT8(42, received->seq);
    TEST_ASSERT_EQUAL_UINT32(10000u, received->dt_us);
    TEST_ASSERT_EQUAL_INT32(1234, received->d1);
    TEST_ASSERT_EQUAL_INT32(-5678, received->d2);
    TEST_ASSERT_EQUAL_INT32(0, received->d3);
    TEST_ASSERT_EQUAL_UINT32(1u, decoder.counters().frames_received);
    TEST_ASSERT_EQUAL_UINT32(0u, decoder.counters().crc_errors);
    TEST_ASSERT_EQUAL_UINT32(0u, decoder.counters().bytes_discarded);
}

void test_pose_frame_round_trip(void) {
    Decoder decoder;
    const PoseFrame sent = make_pose_frame(3, 42);

    const int frames = push_all(decoder, reinterpret_cast<const uint8_t *>(&sent),
                                sizeof(sent));

    TEST_ASSERT_EQUAL_INT(1, frames);
    TEST_ASSERT_NULL(decoder.as_encoder_frame());

    const PoseFrame *received = decoder.as_pose_frame();
    TEST_ASSERT_NOT_NULL(received);
    TEST_ASSERT_EQUAL_UINT8(3, received->seq);
    TEST_ASSERT_EQUAL_UINT8(42, received->echo_seq);
    TEST_ASSERT_EQUAL_FLOAT(1000.0f, received->x);
    TEST_ASSERT_EQUAL_FLOAT(500.0f, received->y);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, received->theta);
    TEST_ASSERT_EQUAL_UINT8(POSE_FLAG_FILTER_READY, received->flags);
}

void test_back_to_back_frames_are_both_decoded(void) {
    // A frame arriving while the previous one is still in the buffer must not be
    // lost: at 100 Hz on a 512-byte buffer this happens as soon as loop() is
    // late by one cycle.
    Decoder decoder;
    uint8_t stream[2 * ENCODER_FRAME_SIZE];
    const EncoderFrame first = make_encoder_frame(1);
    const EncoderFrame second = make_encoder_frame(2);
    memcpy(stream, &first, sizeof(first));
    memcpy(stream + sizeof(first), &second, sizeof(second));

    TEST_ASSERT_EQUAL_INT(2, push_all(decoder, stream, sizeof(stream)));
    TEST_ASSERT_EQUAL_UINT8(2, decoder.as_encoder_frame()->seq);
    TEST_ASSERT_EQUAL_UINT32(2u, decoder.counters().frames_received);
}

// === CORRUPTION AND RESYNCHRONISATION ===

void test_corrupted_payload_is_rejected(void) {
    Decoder decoder;
    EncoderFrame frame = make_encoder_frame(5);
    uint8_t bytes[ENCODER_FRAME_SIZE];
    memcpy(bytes, &frame, sizeof(frame));
    bytes[7] ^= 0xFF;  // flip a payload byte, leave the CRC alone

    TEST_ASSERT_EQUAL_INT(0, push_all(decoder, bytes, sizeof(bytes)));
    TEST_ASSERT_EQUAL_UINT32(1u, decoder.counters().crc_errors);
    TEST_ASSERT_EQUAL_UINT32(0u, decoder.counters().frames_received);
}

void test_unknown_type_is_counted_and_skipped(void) {
    Decoder decoder;
    const uint8_t noise[] = {STX, 0x7F, 0x00, 0x00};

    TEST_ASSERT_EQUAL_INT(0, push_all(decoder, noise, sizeof(noise)));
    TEST_ASSERT_EQUAL_UINT32(1u, decoder.counters().unknown_type);
}

void test_leading_garbage_is_discarded_then_frame_is_found(void) {
    Decoder decoder;
    uint8_t stream[4 + ENCODER_FRAME_SIZE];
    const uint8_t garbage[4] = {0x00, 0x11, 0x22, 0x33};
    const EncoderFrame frame = make_encoder_frame(9);
    memcpy(stream, garbage, sizeof(garbage));
    memcpy(stream + sizeof(garbage), &frame, sizeof(frame));

    TEST_ASSERT_EQUAL_INT(1, push_all(decoder, stream, sizeof(stream)));
    TEST_ASSERT_EQUAL_UINT8(9, decoder.as_encoder_frame()->seq);
    TEST_ASSERT_EQUAL_UINT32(4u, decoder.counters().bytes_discarded);
}

void test_false_stx_does_not_swallow_the_next_frame(void) {
    // THE important test. A stray STX followed by a plausible type byte makes the
    // decoder accumulate a whole candidate frame and reject it on CRC. If it then
    // resumed AFTER the candidate instead of after the false STX, it would eat
    // the real frame that starts one byte later.
    Decoder decoder;
    uint8_t stream[2 + ENCODER_FRAME_SIZE];
    stream[0] = STX;           // false delimiter
    stream[1] = TYPE_ENCODER;  // plausible type, so a full candidate is built
    const EncoderFrame frame = make_encoder_frame(77);
    memcpy(stream + 2, &frame, sizeof(frame));

    TEST_ASSERT_EQUAL_INT(1, push_all(decoder, stream, sizeof(stream)));
    TEST_ASSERT_NOT_NULL(decoder.as_encoder_frame());
    TEST_ASSERT_EQUAL_UINT8(77, decoder.as_encoder_frame()->seq);
    TEST_ASSERT_EQUAL_UINT32(1u, decoder.counters().crc_errors);
}

void test_stx_inside_payload_is_harmless(void) {
    // Frames are not byte-stuffed, so a payload byte equal to STX is expected and
    // must not break anything: the CRC decides, not the delimiter.
    Decoder decoder;
    EncoderFrame frame;
    frame.seq = STX;
    frame.dt_us = 0xA5A5A5A5;
    frame.d1 = 0;
    frame.d2 = 0;
    frame.d3 = 0;
    frame.flags = 0;
    seal(frame);

    TEST_ASSERT_EQUAL_INT(
        1, push_all(decoder, reinterpret_cast<const uint8_t *>(&frame), sizeof(frame)));
    TEST_ASSERT_EQUAL_UINT32(0xA5A5A5A5u, decoder.as_encoder_frame()->dt_us);
}

void test_truncated_frame_then_valid_frame(void) {
    // What a mid-transmission reset or an unplugged cable actually produces.
    Decoder decoder;
    const EncoderFrame truncated = make_encoder_frame(1);
    const EncoderFrame complete = make_encoder_frame(2);

    uint8_t stream[10 + ENCODER_FRAME_SIZE];
    memcpy(stream, &truncated, 10);  // first 10 bytes only
    memcpy(stream + 10, &complete, sizeof(complete));

    TEST_ASSERT_EQUAL_INT(1, push_all(decoder, stream, sizeof(stream)));
    TEST_ASSERT_EQUAL_UINT8(2, decoder.as_encoder_frame()->seq);
}

void test_reset_drops_partial_frame_but_keeps_counters(void) {
    Decoder decoder;
    const EncoderFrame frame = make_encoder_frame(1);
    push_all(decoder, reinterpret_cast<const uint8_t *>(&frame), 10);  // partial
    const uint32_t discarded_before = decoder.counters().bytes_discarded;

    decoder.reset();

    // The remaining 11 bytes are now orphans and must not be mistaken for a frame.
    TEST_ASSERT_EQUAL_INT(
        0, push_all(decoder, reinterpret_cast<const uint8_t *>(&frame) + 10, 11));
    TEST_ASSERT_EQUAL_UINT32(0u, decoder.counters().frames_received);
    TEST_ASSERT_TRUE(decoder.counters().bytes_discarded >= discarded_before);
}

// === SEQUENCE TRACKING ===

void test_seq_tracker_reports_no_gap_on_contiguous_stream(void) {
    SeqTracker tracker;
    TEST_ASSERT_EQUAL_UINT8(0, tracker.observe(10));
    TEST_ASSERT_EQUAL_UINT8(0, tracker.observe(11));
    TEST_ASSERT_EQUAL_UINT8(0, tracker.observe(12));
}

void test_seq_tracker_counts_missing_frames(void) {
    SeqTracker tracker;
    tracker.observe(10);
    TEST_ASSERT_EQUAL_UINT8(2, tracker.observe(13));  // 11 and 12 were lost
}

void test_seq_tracker_handles_wrap_around(void) {
    // seq is a uint8 and wraps by design; 255 -> 0 is contiguous, not a gap of 255.
    SeqTracker tracker;
    tracker.observe(254);
    TEST_ASSERT_EQUAL_UINT8(0, tracker.observe(255));
    TEST_ASSERT_EQUAL_UINT8(0, tracker.observe(0));
    TEST_ASSERT_EQUAL_UINT8(1, tracker.observe(2));
}

void test_seq_tracker_treats_repeat_as_no_gap(void) {
    SeqTracker tracker;
    tracker.observe(5);
    TEST_ASSERT_EQUAL_UINT8(0, tracker.observe(5));
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_crc8_matches_usb_table_on_every_byte);
    RUN_TEST(test_crc8_of_empty_span_is_zero);

    RUN_TEST(test_frame_sizes_match_the_wire_format);
    RUN_TEST(test_unknown_type_has_no_size);
    RUN_TEST(test_field_offsets_match_the_wire_format);
    RUN_TEST(test_seal_sets_delimiter_and_type);

    RUN_TEST(test_encoder_frame_round_trip);
    RUN_TEST(test_pose_frame_round_trip);
    RUN_TEST(test_back_to_back_frames_are_both_decoded);

    RUN_TEST(test_corrupted_payload_is_rejected);
    RUN_TEST(test_unknown_type_is_counted_and_skipped);
    RUN_TEST(test_leading_garbage_is_discarded_then_frame_is_found);
    RUN_TEST(test_false_stx_does_not_swallow_the_next_frame);
    RUN_TEST(test_stx_inside_payload_is_harmless);
    RUN_TEST(test_truncated_frame_then_valid_frame);
    RUN_TEST(test_reset_drops_partial_frame_but_keeps_counters);

    RUN_TEST(test_seq_tracker_reports_no_gap_on_contiguous_stream);
    RUN_TEST(test_seq_tracker_counts_missing_frames);
    RUN_TEST(test_seq_tracker_handles_wrap_around);
    RUN_TEST(test_seq_tracker_treats_repeat_as_no_gap);

    return UNITY_END();
}
