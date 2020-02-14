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

template <class T>
class FrequencyCounter {
private:
	enum Status{
		INIT,
		COUNTING,
		LOW
	};
	T lastTick = 0;
	FIR<T> filter;

public:
	FrequencyCounter();

	void inputTick(T tick);

	void inputInterval(T interval);

	T outputFrequency();
};

} // namespace brown

#endif /* INC_FREQUENCY_COUNTER_H_ */
