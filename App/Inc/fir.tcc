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

#include "fifo_buffer.tcc"

namespace brown {

/**
 * @brief Finite Impulse Response (FIR) filter.
 * @param T Filter datatype.
 * @param SIZE Size datatype. It must be unsigned. Its maximum value must
 * exceed twice of the filter order.
 */
template <class T, class SIZE>
class FIR {
private:
    // Impulse response of the filter.
    const T* h;

    // Delayed input from x[-N+1] to x[0]. A circular buffer.
    FIFOBuffer<T, SIZE> x;

    // Coefficient table. For multirate-like filtering.
    const T* idxStart = NULL;
    const T* idxTable = NULL;
    SIZE nR = 0;
    SIZE nC = 0;
    SIZE activeRow = 0;


public:
    /**
     * @brief Constructor
     * @param h Impulse response of the filter. Length `n`. Arranged in inverse
     * time order, i.e. {h[N-1] h[N-2] ... h[1] h[0]}.  It is recommended to
     * use a symmetric zero phase filter.
     * @param x Buffer for the delayed input. Length `n`.
     * @param n Order of the filter.
     */
    FIR(const T h[], T x[], SIZE n):
        h(h), x(x, n) {}

    /**
     * @brief Constructor with coefficient table.
     */
    FIR(const T h[], T x[], SIZE n, const T idxStart[], const T idxTable[],
            SIZE nRow, SIZE nCol):
        idxStart(idxStart), idxTable(idxTable), nR(nRow), nC(nCol)
        {FIR(h, x, n);}

    /**
     * @brief Provide the filter with a new input x[0].
     * @param x0 Current input.
     */
    void input(T x0) {
        x.add(x0);
        if (nR != 0) {activeRow = (activeRow+1) % nR;}
    }

    /**
     * @brief Compute filter output y[0]. Return 0 if there is not enough data.
     * @return T Current output.
     */
    T output() const {
        if (!isValid()) {return 0;}

        T y0 = 0;
        // Not using coefficient table
        if (idxStart == NULL || idxTable == NULL || nR == 0 || nC == 0) {
            for (SIZE i = 0; i < x.capacity(); i++) {
                y0 += h[i]*x[i];
            }
        // Using coefficient table
        } else {
            SIZE coeffIdx = idxStart[activeRow];
            while (coeffIdx < x.capacity()) {
                y0 += h[coeffIdx]*x[coeffIdx];
                coeffIdx = idxTable[nC*activeRow+coeffIdx];
            }
        }
        return y0;
    }

    /**
     * @brief Clear filter buffer.
     */
    inline void reset() __attribute__((always_inline)) {
        x.reset();
        activeRow = 0;
    }

    /**
     * @brief Initialize/fill the filter buffer with a constant value.
     * @param x0 Initial value. Zero (0) state by default.
     */
    void initialize(T x0 = 0) {
        reset();
        for (SIZE i = 0; i < x.capacity(); x[i++] = x0);
        activeRow = 0;
    }

    /**
     * @brief Order of the filter.
     * @return SIZE Filter order.
     */
    inline SIZE order() const __attribute__((always_inline)) {
        return x.capacity();
    }

    /**
     * @brief Check if the filter has enough data to produce valid output.
     * @return bool
     */
    inline bool isValid() const __attribute__((always_inline)) {
        return x.isFull();
    }

}; // class FIR

} // namespace brown

#endif /* INC_FIR_TCC_ */
