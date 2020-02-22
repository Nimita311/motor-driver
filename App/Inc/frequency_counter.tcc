/**
 * @file     frequency_counter.tcc
 * @brief    Frequency counter.
 * @author   Haoze Zhang
 * @version  20200213
 *
 * Distributed under MIT License
 * Copyright (c) 2020 Haoze Zhang | Brown Engineering
 */

#ifndef INC_FREQUENCY_COUNTER_H_
#define INC_FREQUENCY_COUNTER_H_

#include "fir.tcc"

namespace brown {

/**
 * Frequency counter class.
 * It should be used with a timer in input capture mode. Each capture should
 * be provided via <inputTick> method. <timeout> should be called in timer
 * overflow ISR to detect 0/low frequency.
 * @param TT: timer counter type. Typical uint16_t/uint32_t.
 * @param TF: filter/frequency type. Typical float32_t.
 */
template <class TT, class TF, class SIZE>
class FrequencyCounter {
private:
	/*
	 * Number of ticks acquired in a timer period.
	 * If no tick is acquired in a full timer period, the frequency should be
	 * considered to be low/0.
	 */
	volatile TT tickCounterPerTimeout= 0;

	/*
	 * The most recent input tick.
	 * Next interval value will be calculated as the difference between the next
	 * tick and this value.
	 */
	TT lastTick = 0;
	TF lastPeriod = 0;


	/*
	 * Frequency of the tick in [Hz].
	 */
	const TF tickFrequency;

	FIFOBuffer<TT, SIZE> tickBuffer;

	/*
	 * Filter for the tick intervals.
	 */
	FIR<TF, SIZE> periodFilter;


public:
	/**
	 * Constructor
	 * @param n FIR filter order.
	 * @param h FIR impulse response. Size <n>.
	 * @param x FIR input buffer. Size <n>.
	 * @param tickFrequency Timer tick frequency.
	 */
	FrequencyCounter(const TF h[], TF periods[], SIZE nP, TT ticks[], SIZE nT,
			TF tickFrequency):
		tickFrequency(tickFrequency), tickBuffer(ticks, nT),
		periodFilter(h, periods, nP) {}

	/**
	 * Input a capture register (CCR) value.
	 * It should be an incremental tick value of the timer.
	 * @param tick Captured counter tick.
	 */
	void input(TT tick) {
		tickCounterPerTimeout++;
		TT interval = tick - lastTick;
		// Enough samples present
		if (tickBuffer.isFull()) {
			lastPeriod += interval;
			lastPeriod -= tickBuffer[0];
			tickBuffer.add(interval);
			periodFilter.input(lastPeriod);
		// Accumulating samples
		} else if (tickBuffer.size() > 0) {
			lastPeriod += interval;
			tickBuffer.add(interval);
		// No samples yet
		} else {
			lastPeriod = 0;
			tickBuffer.add(0);
		}
		lastTick = tick;
	}

	/**
	 * Get the frequency of the input pulse train in [Hz].
	 * Output 0 if frequency below lower bound, i.e. timer's full period.
	 * @return TF Frequency [Hz].
	 */
	TF output() const {
		return (periodFilter.isValid()) ?
			tickFrequency/periodFilter.output() : 0;
	}

	/**
	 * Reset the frequency counter to INIT state.
	 */
	void reset() {
		tickBuffer.reset();
		periodFilter.reset();
		tickCounterPerTimeout = 0;
	}

	/**
	 * Method to be called in timer overflow ISR.
	 * The frequency counter will be reset if no tick is acquired in a full period,
	 * as the frequency is below the lower bound.
	 */
	void timeoutCallback() {
		if (tickCounterPerTimeout == 0) {reset();}
		tickCounterPerTimeout = 0;
	}
};

} // namespace brown

#endif /* INC_FREQUENCY_COUNTER_H_ */
