/**
 * @file     fifo_buffer.tcc
 * @brief    A lightweight circular FIFO buffer.
 * @author   Haoze Zhang
 * @version  20200221
 *
 * Distributed under MIT License
 * Copyright (c) 2020 Haoze Zhang | Brown Engineering
 */

#ifndef INC_FIFO_BUFFER_TCC_
#define INC_FIFO_BUFFER_TCC_

namespace brown {

template <class T, class SIZE>
class FIFOBuffer {
private:
    T* values;
    const SIZE n;

    // Current head position. Next input will be placed at `values[head]`.
    SIZE head = 0;
    SIZE size_ = 0;

public:
    FIFOBuffer (T values[], SIZE capacity):
        values(values), n(capacity) {}

    void add(T newValue) {
        values[head++] = newValue;
        head %= n;
        if (size_ < n) {size_++;}
    }

    inline void reset() __attribute__((always_inline)) {
        head = 0;
        size_ = 0;
    }

    T operator[](SIZE idx) const {
        return (size_ >= n) ?
            values[(head+idx)%n] : values[idx%n];
    }

    inline T peak() const __attribute__((always_inline)) {
        return *this[0];
    }

    inline SIZE capacity() const __attribute__((always_inline)) {
        return n;
    }

    inline SIZE size() const __attribute__((always_inline)) {
        return size_;
    }

    inline SIZE isFull() const __attribute__((always_inline)) {
        return (size_ >= n);
    }
}; // class FIFOBuffer

} // namespace brown

#endif // INC_FIFO_BUFFER_TCC_
