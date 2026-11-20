/*
 * Host tests for lib/log_ring: the buffer between the control tick and the USB
 * link.
 *
 * Run with: pio test -e native   (from robot1/teensy_moteur/)
 */

#include <string.h>
#include <unity.h>

#include "log_ring.h"

using namespace logring;

void setUp(void) {}
void tearDown(void) {}

static constexpr uint16_t SLOTS = 4;  // 3 frames fit
static uint8_t storage[SLOTS * SLOT_BYTES];

static void test_empty_ring_has_no_front(void) {
    LogRing ring(storage, SLOTS);
    uint16_t length = 99;
    TEST_ASSERT_NULL(ring.front(length));
    TEST_ASSERT_EQUAL_UINT16(0, length);
    TEST_ASSERT_EQUAL_UINT16(0, ring.size());
    TEST_ASSERT_EQUAL_UINT16(SLOTS - 1, ring.capacity());
    ring.pop();  // harmless on an empty ring
    TEST_ASSERT_EQUAL_UINT16(0, ring.size());
}

static void test_frames_come_out_in_order_and_intact(void) {
    LogRing ring(storage, SLOTS);
    const uint8_t a[3] = {200, 1, 2};
    const uint8_t b[5] = {131, 9, 8, 7, 6};
    TEST_ASSERT_TRUE(ring.push(a, sizeof(a)));
    TEST_ASSERT_TRUE(ring.push(b, sizeof(b)));
    TEST_ASSERT_EQUAL_UINT16(2, ring.size());

    uint16_t length = 0;
    const uint8_t *frame = ring.front(length);
    TEST_ASSERT_EQUAL_UINT16(sizeof(a), length);
    TEST_ASSERT_EQUAL_MEMORY(a, frame, sizeof(a));
    // front() does not consume: a frame the link refused is offered again.
    TEST_ASSERT_EQUAL_PTR(frame, ring.front(length));
    ring.pop();

    frame = ring.front(length);
    TEST_ASSERT_EQUAL_UINT16(sizeof(b), length);
    TEST_ASSERT_EQUAL_MEMORY(b, frame, sizeof(b));
    ring.pop();
    TEST_ASSERT_NULL(ring.front(length));
}

static void test_full_ring_drops_the_new_frame_and_counts_it(void) {
    LogRing ring(storage, SLOTS);
    uint8_t frame[1] = {0};
    for (uint8_t i = 1; i <= 3; ++i) {
        frame[0] = i;
        TEST_ASSERT_TRUE(ring.push(frame, 1));
    }
    frame[0] = 4;
    TEST_ASSERT_FALSE(ring.push(frame, 1));
    TEST_ASSERT_EQUAL_UINT32(1, ring.dropped());

    // The oldest frames are kept, not the new one.
    uint16_t length = 0;
    TEST_ASSERT_EQUAL_UINT8(1, ring.front(length)[0]);
}

static void test_largest_frame_fits_and_longer_is_dropped(void) {
    LogRing ring(storage, SLOTS);
    uint8_t big[MAX_FRAME + 1];
    for (uint16_t i = 0; i < sizeof(big); ++i) big[i] = static_cast<uint8_t>(i);

    TEST_ASSERT_TRUE(ring.push(big, MAX_FRAME));
    TEST_ASSERT_FALSE(ring.push(big, MAX_FRAME + 1));
    TEST_ASSERT_FALSE(ring.push(big, 0));
    TEST_ASSERT_EQUAL_UINT32(2, ring.dropped());

    uint16_t length = 0;
    const uint8_t *frame = ring.front(length);
    TEST_ASSERT_EQUAL_UINT16(MAX_FRAME, length);
    TEST_ASSERT_EQUAL_MEMORY(big, frame, MAX_FRAME);
}

static void test_wraps_around_many_times(void) {
    LogRing ring(storage, SLOTS);
    uint16_t length = 0;
    for (uint32_t i = 0; i < 1000; ++i) {
        const uint32_t value = i * 2654435761u;
        TEST_ASSERT_TRUE(ring.push(&value, sizeof(value)));
        if (i % 2 == 1) {
            // Drain two at a time: the indices wrap at every possible offset.
            for (int k = 0; k < 2; ++k) {
                const uint8_t *frame = ring.front(length);
                TEST_ASSERT_NOT_NULL(frame);
                uint32_t read = 0;
                memcpy(&read, frame, sizeof(read));
                TEST_ASSERT_EQUAL_UINT32((i - 1 + k) * 2654435761u, read);
                ring.pop();
            }
        }
    }
    TEST_ASSERT_EQUAL_UINT16(0, ring.size());
    TEST_ASSERT_EQUAL_UINT32(0, ring.dropped());
}

static void test_size_counts_across_the_wrap(void) {
    LogRing ring(storage, SLOTS);
    const uint8_t x = 1;
    uint16_t length = 0;
    ring.push(&x, 1);
    ring.push(&x, 1);
    ring.pop();
    ring.pop();
    // head and tail now at 2: the next frames straddle the end of the storage.
    ring.push(&x, 1);
    ring.push(&x, 1);
    ring.push(&x, 1);
    TEST_ASSERT_EQUAL_UINT16(3, ring.size());
    TEST_ASSERT_NOT_NULL(ring.front(length));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_empty_ring_has_no_front);
    RUN_TEST(test_frames_come_out_in_order_and_intact);
    RUN_TEST(test_full_ring_drops_the_new_frame_and_counts_it);
    RUN_TEST(test_largest_frame_fits_and_longer_is_dropped);
    RUN_TEST(test_wraps_around_many_times);
    RUN_TEST(test_size_counts_across_the_wrap);
    return UNITY_END();
}
