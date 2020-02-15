/**
 * @file     frequency_counter.h
 * @brief    Frequency counter.
 * @author   Haoze Zhang
 * @version  20200213
 *
 * Distributed under MIT License
 * Copyright (c) 2020 Haoze Zhang | Brown Engineering
 */

#ifndef INC_FREQUENCY_COUNTER_H_
#define INC_FREQUENCY_COUNTER_H_

#include "filters.h"

namespace brown {

/**
 * Frequency counter class.
 * It should be used with a timer in input capture mode. Each capture should
 * be provided via <inputTick> method. <timeout> should be called in timer
 * overflow ISR to detect 0/low frequency.
 * @param TT: timer counter type. Typical uint16_t/uint32_t.
 * @param TF: filter/frequency type. Typical float32_t.
 */
template <class TT, class TF>
class FrequencyCounter {
private:
	/*
	 * Status of the frequency counter
	 * COUNTING: <lastTick> and all samples in <filter> are valid.
	 * LOW: <lastTick> has valid data but <filter> is not yet filled.
	 * INIT: <lastTick> and <filter> contains no valid data.
	 */
	enum class Status{
		COUNTING,
		LOW,
		INIT
	};
	Status status = Status::INIT;

	/*
	 * Number of ticks acquired in a timer period.
	 * If no tick is acquired in a full timer period, the frequency should be
	 * considered to be low/0.
	 */
	TT tickCounterPerTimerPeriod= 0;

	/*
	 * Number of valid samples in the filter.
	 *
	 */
	uint16_t tickCounterInFilter = 0;

	/*
	 * The most recent input tick.
	 * Next interval value will be calculated as the difference between the next
	 * tick and this value.
	 */
	TT lastTick = 0;

	/*
	 * Frequency of the tick in [Hz].
	 */
	const TF tickFrequency;

	/*
	 * Filter for the tick intervals.
	 */
	FIR<TF> filter;

public:
	/**
	 * Constructor
	 * @param n FIR filter order.
	 * @param h FIR impulse response. Size <n>.
	 * @param x FIR input buffer. Size <n>.
	 * @param tickFrequency Timer tick frequency.
	 */
	FrequencyCounter(uint16_t n, const TF h[], TF x[], TF tickFrequency):
		tickFrequency(tickFrequency), filter(n, h, x) {}

	/**
	 * Input a capture register (CCR) value.
	 * It should be an incremental tick value of the timer.
	 * @param tick Captured counter tick.
	 */
	void inputTick(TT tick) {
		tickCounterPerTimerPeriod++;
		TT interval = tick - lastTick;
		// COUNTING: simply use the input.
		if (status == Status::COUNTING) {
			filter.input(interval);
		// LOW: use data and check if enough data.
		} else if (status == Status::LOW) {
			filter.input(interval);
			// Transit to COUNTING from LOW if enough data is acquired
			if (++tickCounterInFilter >= filter.getOrder()) {
				status = Status::COUNTING;
			}
		// INIT: set <lastTick> and transit to LOW.
		} else { // (status == Status::INIT
			status = Status::LOW;
		}
		lastTick = tick;
	}

	/**
	 * Get the frequency of the input pulse train in [Hz].
	 * Output 0 if frequency below lower bound, i.e. timer's full period.
	 * @return TF Frequency [Hz].
	 */
	TF outputFrequency() const {
		return (status == Status::COUNTING) ?
			tickFrequency/filter.output() : 0;
	}

	/**
	 * Reset the frequency counter to INIT state.
	 */
	void reset() {
		status = Status::INIT;
		tickCounterInFilter = 0;
		tickCounterPerTimerPeriod = 0;
	}

	/**
	 * Method to be called in timer overflow ISR.
	 * The frequency counter will be reset if no tick is acquired in a full period,
	 * as the frequency is below the lower bound.
	 */
	void timeout() {
		if (tickCounterPerTimerPeriod == 0) {
			reset();
		}
		tickCounterPerTimerPeriod = 0;
	}
};

} // namespace brown

#endif /* INC_FREQUENCY_COUNTER_H_ */
