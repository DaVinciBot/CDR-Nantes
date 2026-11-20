/*
 * Ring buffer of log frames. See include/log_ring.h.
 *
 * No Arduino dependency: this file must build in the "native" environment.
 */

#include "log_ring.h"

#include <string.h>

namespace logring {

LogRing::LogRing(uint8_t *storage, uint16_t slots) : storage_(storage), slots_(slots) {}

bool LogRing::push(const void *frame, uint16_t length) {
    const uint16_t head = head_.load(std::memory_order_relaxed);
    const uint16_t following = next(head);
    if (length == 0 || length > MAX_FRAME ||
        following == tail_.load(std::memory_order_acquire)) {
        dropped_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    uint8_t *s = slot(head);
    s[0] = static_cast<uint8_t>(length);
    memcpy(s + 1, frame, length);
    // Published only once written: the consumer never sees half a frame.
    head_.store(following, std::memory_order_release);
    return true;
}

const uint8_t *LogRing::front(uint16_t &length) const {
    const uint16_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) {
        length = 0;
        return nullptr;
    }
    const uint8_t *s = slot(tail);
    length = s[0];
    return s + 1;
}

void LogRing::pop() {
    const uint16_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) return;
    tail_.store(next(tail), std::memory_order_release);
}

uint16_t LogRing::size() const {
    const uint16_t head = head_.load(std::memory_order_acquire);
    const uint16_t tail = tail_.load(std::memory_order_acquire);
    return head >= tail ? head - tail : slots_ - tail + head;
}

}  // namespace logring
