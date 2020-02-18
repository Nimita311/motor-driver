/**
 * @file     fir.tcc
 * @brief    FIR digital filter.
 * @author   Haoze Zhang
 * @version  20200213
 *
 * Distributed under MIT License
 * Copyright (c) 2020 Haoze Zhang | Brown Engineering
 */

#ifndef INC_FIR_TCC_
#define INC_FIR_TCC_

namespace brown {

/**
 * @brief Finite Impulse Response (FIR) filter.
 * @param T Filter datatype.
 */
template <class T>
class FIR {
private:
    /*
     * Order of the filter, no more than 32766.
     */
    uint16_t n;

    /*
     * Impulse response of the filter, i.e. the coefficients.
     * Arranged in inverse time order, i.e. {h[n-1] h[n-2] ... h[1] h[0]}.
     */
    const T* h;

    /*
     * Delayed input from x[-N+1] to x[0]. A circular buffer.
     */
    T* x;

    /*
     * Current head position of the input circular buffer. Next input will be
     * placed at x[xHead]. It points to the oldest buffered input x[-N+1].
     */
    uint16_t xHead = 0U;

public:
    /**
     * @brief Constructor
     * @param n Order of the filter.
     * @param h Impulse response of the filter. Inverse time order. Length n. It
     * 	is recommended to use a symmetric zero phase filter.
     * @param x Buffer for the delayed input. Length n.
     * @param isZeroState If true, initialize x with 0. True by default.
     */
    FIR(uint16_t n, const T h[], T x[], bool isZeroState=true):
        n(n), h(h), x(x) {
        if (isZeroState) {reset();}
        if (n == 0U) {n = 1U;}
    }

    /**
     * @brief Reset the filter to a constant/zero state.
     * @param x0 The reset value. 0 by default.
     */
    void reset(T x0=static_cast<T>(0)) {
        for (uint16_t i = 0U; i < n; x[i++]=x0);
    }

    /**
     * @brief Provide the filter with a new input x[0].
     * @param x Latest input.
     */
    void input(T x0) {
        x[xHead++] = x0; xHead %= n;
    }

    /**
     * @brief Generate current filter output y[0].
     * @return T Current output.
     */
    T output() const {
        T y0 = static_cast<T>(0);
        for (uint16_t i = 0U; i < n; i++) {
            y0 += h[i]*x[i%n];
        }
        return y0;
    }

    /**
     * @brief Get the order of the filter.
     * @return uint16_t Filter order.
     */
    uint16_t getOrder() const {
        return n;
    }
}; // class FIR

} // namespace brown

#endif /* INC_FIR_TCC_ */
