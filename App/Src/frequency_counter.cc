/**
 * @file     frequency_counter.cc
 * @brief    Frequency counter implementation.
 * @author   Haoze Zhang
 * @version  20200214
 *
 * Distributed under MIT License
 * Copyright (c) 2020 Haoze Zhang | Brown Engineering
 */


#include "frequency_counter.h"

namespace brown {

template <class TT, class TF>
void FrequencyCounter<TT, TF>::inputTick(TT tick) {
	tickCounterPerTimerPeriod++;
	// COUNTING: simply use the input.
	if (status == Status::COUNTING) {
		filter.input(tick-lastTick);
	// LOW: use data and check if enough data.
	} else if (status == Status::LOW) {
		filter.input(tick-lastTick);
		// Transit to COUNTING from LOW if enough data is acquired
		if (++tickCounterInFilter >= filter.getOrder()) {
			status == Status::COUNTING;
		}
	// INIT: set <lastTick> and transit to LOW.
	} else { // (status == Status::INIT
		lastTick = tick;
		status = Status::LOW;
	}
}

template <class TT, class TF>
TF FrequencyCounter<TT, TF>::outputFrequency() const {
	return (status == Status::COUNTING) ?
		tickFrequency/filter.output() : 0;
}

template <class TT, class TF>
void FrequencyCounter<TT, TF>::reset() {
	status = Status::INIT;
	tickCounterInFilter = 0;
	tickCounterPerTimerPeriod = 0;
}

template <class TT, class TF>
void FrequencyCounter<TT, TF>::timeout() {
	if (tickCounterPerTimerPeriod == 0) {
		reset();
	}
	tickCounterPerTimerPeriod = 0;
}

} // namespace brown
