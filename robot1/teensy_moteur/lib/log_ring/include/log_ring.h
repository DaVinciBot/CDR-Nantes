/*
 * Ring buffer of log frames, between the control tick and the USB link.
 *
 * The tick (ISR) must never wait for the host: it copies its frames here and
 * returns. loop() empties the ring towards the link as fast as the link takes
 * them. A full ring drops the new frame and counts it; nothing ever blocks.
 *
 * One consumer: loop(). Producers: the ISR, and loop() itself for the rare
 * frames (events, firmware info, parameters), which it pushes with interrupts
 * off so that the two producers never interleave. The indices are atomics, so
 * a frame is visible to the consumer only once it is entirely written.
 *
 * Arduino-free: builds and is tested in the native environment. The storage is
 * handed in by the caller, so that the board can place it in DMAMEM.
 */

#ifndef LOG_RING_H
#define LOG_RING_H

#include <atomic>
#include <stdint.h>

namespace logring {

/** Bytes of one slot: a length byte, then the frame. */
constexpr uint16_t SLOT_BYTES = 192;
/** Longest frame a slot holds (ID + data). */
constexpr uint16_t MAX_FRAME = SLOT_BYTES - 1;

class LogRing {
public:
    /**
     * @param storage `slots` * SLOT_BYTES bytes, owned by the caller.
     * @param slots Number of slots; one is kept empty, slots - 1 frames fit.
     */
    LogRing(uint8_t *storage, uint16_t slots);

    /**
     * @brief Copies one frame in.
     * @return False if it was dropped (ring full, or longer than MAX_FRAME).
     */
    bool push(const void *frame, uint16_t length);

    /**
     * @brief The oldest frame, left in the ring.
     * @param length Out: its length.
     * @return Its bytes, or nullptr if the ring is empty.
     */
    const uint8_t *front(uint16_t &length) const;

    /** @brief Removes the oldest frame (after it was sent). */
    void pop();

    /** @return Frames waiting. */
    uint16_t size() const;

    /** @return Frames the ring holds at most. */
    uint16_t capacity() const { return slots_ - 1; }

    /** @return Frames dropped since construction (wraps). */
    uint32_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

private:
    uint8_t *slot(uint16_t index) const { return storage_ + uint32_t(index) * SLOT_BYTES; }
    uint16_t next(uint16_t index) const { return index + 1 == slots_ ? 0 : index + 1; }

    uint8_t *storage_;
    uint16_t slots_;
    std::atomic<uint16_t> head_{0};  // next slot to write (producer)
    std::atomic<uint16_t> tail_{0};  // oldest frame (consumer)
    std::atomic<uint32_t> dropped_{0};
};

}  // namespace logring

#endif  // LOG_RING_H
