/**
 * @file     freq_counter.hh
 * @brief    `FrequencyCounter` class.
 * @author   Haoze Zhang
 * @version  20200213
 *
 * Distributed under MIT License
 * Copyright (c) 2020 Haoze Zhang | Brown Engineering
 */

#ifndef INC_FREQUENCY_COUNTER_H_
#define INC_FREQUENCY_COUNTER_H_

#include "fir.hh"

namespace brown {

/**
 * @brief Frequency counter class.
 * @param TT Timer tick type. Typical uint16_t/uint32_t.
 * @param TF Filter/frequency type. Typical float32_t.
 *
 * It is design to estimate frequency with low accuracy encoders of which the
 * pulses are not evenly spaced (e.g. Hall sensors in a multi-pole BLDC motor).
 * Instead of evaluting the frequency with the intervals between consecutive
 * pulses whose angle of rotation is not deterministic, the period of a full
 * revolution is used.
 *
 * It should be used with a timer in input capture mode. Each capture should
 * be provided via `input` method. `timeoutCallback` should be called in timer
 * overflow ISR to detect 0/low frequency. The timer should be configured to
 * maximum period (16 bit or 32 bit) so that the interval is always the
 * difference between two consecutive ticks.
 */
template <class TT, class TF, class SIZE>
class FrequencyCounter {
private:
    // Number of ticks acquired in a timer period.
    // If no tick is acquired in a full timer period, the frequency should be
    // considered to be low/0.
    volatile TT tickCounterPerTimeout= 0;

    // The most recent input tick.
    // Next interval value will be calculated as the difference between the next
    // tick and this value.
    TT lastTick = 0;

    // The most recent computed period.
    // Instead of adding up all intervals to compute the period each time,
    // remove the oldest interval and add the new one to `lastPeriod` as the
    // new estimate.
    TF lastPeriod = 0;

    // Frequency of timer ticks in [Hz].
    const TF tickFrequency;

    // Buffer of pulse intervals in timer ticks.
    FIFOBuffer<TT, SIZE> tickBuffer;

    // Filter for the computed period.
    FIR<TF, SIZE> periodFilter;

public:
    /**
     * @brief Constructor
     * @param h FIR impulse response. Size `nP`.
     * @param periods FIR filter input buffer. Size `nP`.
     * @param nP FIR filter order.
     * @param ticks Interval buffer. Size `nT`.
     * @param nT Interval buffer size. Should be the number of beats per
     * revolution of the encoder.
     * @param tickFrequency Timer tick frequency [Hz].
     */
    FrequencyCounter(const TF h[], TF periods[], SIZE nP, TT ticks[], SIZE nT,
            TF tickFrequency):
        tickFrequency(tickFrequency), tickBuffer(ticks, nT),
        periodFilter(h, periods, nP) {}

    /**
     * @brief Input a capture register (CCR) value.
     * @param tick Captured counter tick.
     *
     * It should be an incremental tick value of the timer. And the timer has
     * to be configured to have maximum period (e.g. 0xffff for 16 bit timer).
     */
    void input(TT tick) {
        tickCounterPerTimeout++;
        // Note that this step is correct only if the timer has maximum period.
        TT interval = tick - lastTick;
        // Enough samples present
        if (tickBuffer.isFull()) {
            lastPeriod += interval;
            lastPeriod -= tickBuffer[0];
            tickBuffer.add(interval);
            periodFilter.input(lastPeriod);
        // Accumulating samples, increment period
        } else if (tickBuffer.size() > 0) {
            lastPeriod += interval;
            tickBuffer.add(interval);
        // No samples yet, add 0 as a dummy
        } else {
            lastPeriod = 0;
            tickBuffer.add(0);
        }
        lastTick = tick;
    }

    /**
     * @brief Get the frequency of the input pulse train in [Hz].
     * @return TF Frequency [Hz].
     *
     * Output 0 if frequency below lower bound, i.e. timer's timeout frequency.
     */
    TF output() const {
        return (periodFilter.isValid()) ?
            tickFrequency/periodFilter.output() : 0;
    }

    /**
     * @brief Reset the frequency counter and all buffers.
     */
    void reset() {
        tickBuffer.reset();
        periodFilter.reset();
        tickCounterPerTimeout = 0;
    }

    /**
     * @brief Callback for timer overflow ISR.
     *
     * The frequency counter should reset if no tick is acquired in a full timer
     * period as the frequency is below the lower bound.
     */
    void timeoutCallback() {
        if (tickCounterPerTimeout == 0) {reset();}
        tickCounterPerTimeout = 0;
    }
}; // class FrequencyCounter

} // namespace brown

#endif /* INC_FREQUENCY_COUNTER_H_ */
