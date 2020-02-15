/**
 * @file     pid.tcc
 * @brief    Bilinear PID controller.
 * @author   Haoze Zhang
 * @version  20200215
 *
 * Distributed under MIT License
 * Copyright (c) 2020 Haoze Zhang | Brown Engineering
 */

namespace brown {

/**
 * @brief Bilinear PID controller with anti-windup support.
 * @param T IIR filter type. Typical float or double.
 *
 * Discretized with bilinear transform from H(s) = kp + ki/s + kd*s.
 * Transfer function:
 *     H(z) = (k0*X(z) + k1*X(z)*z^-1 + k2*X(z)*x^-2) / (1 - z^-2)
 * Difference equation:
 *     y[0] = k0*x[0] + k1*x[-1] + k2*x[-2] + y[-2]
 * Marginally stable with two poles on the unit circle. May cause the close
 * loop system to be unstable with some plants.
 */
template <class T>
class PID {
private:
    // Continuous PID gains and sampling period [s]
    T kp, ki, kd, ts;

    // Digital IIR filter states
    // x[-1], x[-2], y[-1], y[-2]
    T states[4];

    // Digital IIR gains
    // x[0], x[-1], x[-2]
    T kpd[3];  // Digital gains without integration for anti-windup
    T kpid[3]; // Digital gains with integration

    // Saturation range for anti-windup
    T satMax, satMin;
    bool isAntiWindupEnabled = false;
    // Mark the anti-windup activity for the most recent output
    bool isAntiWindupActive = false;

    // Compute digital gains from continuous gains using bilinear transform
    void _deriveGains() {
        T kdOts = kd/ts;
        T kiXts = ki*ts;
        // gains with ki
        kpid[0] = kp + kiXts*0.5 + 2.0*kdOts;
        kpid[1] = kiXts - 4.0*kdOts;
        kpid[2] = -kp + 0.5*kiXts + 2.0*kdOts;
        // gains with ki=0
        kpd[0] = kp + 2.0*kdOts;
        kpd[1] = -4.0*kdOts;
        kpd[2] = -kp + 2.0*kdOts;
    }

    // IIR filtering, solve the difference equation for y[0]
    T _output(T x0, T* activeGainSet) {
        T out = 0.0;
        out += x0*activeGainSet[0];
        out += states[0]*activeGainSet[1];
        out += states[1]*activeGainSet[2];
        out += states[3]
    }

    // Update IIR states
    T _updateStates(T x0, T y0) {
        states[1] = states[0];
        states[0] = x0;
        states[3] = states[2];
        states[2] = y0;
    }

public:
    /**
     * @brief Constructor.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param ts Sampling period [s] for discretization.
     *
     * Note that the parameters are continuous gains in
     * H(s) = kp + ki/s + kd*s.
     */
    PID(T kp, T ki, T kd, T ts) {
        reset(); // set to zero state
        setGains(kp, ki, kd, ts);
    }

    /**
     * @brief Set the gains.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param ts Sampling period [s] for discretization.
     *
     * Note that the parameters are continuous gains in
     * H(s) = kp + ki/s + kd*s.
     */
    void setGains(T kp, T ki, T kd, T ts) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->ts = ts;
        _deriveGains();
    }

    /**
     * @brief Enable anti-windup.
     * @param min Saturation lower bound.
     * @param max Saturation upper bound.
     *
     * The anti-windup scheme sets zero integral gain (ki) to zero if the
     * controller output goes beyond the saturation limit to prevent error
     * integration building up.
     * The saturation limit should be set according to the physical limitation
     * of the plant. E.g. if the plant accepts PWM duty cycle as input, the
     * saturation range should be 0 to 1.
     */
    void enableAntiWindup(T min, T max) {
        satMax = max;
        satMin = min;
        isAntiWindupEnabled = true;
    }

    /**
     * @brief Disable anti-windup.
     */
    void disableAntiWindup() {
        isAntiWindupEnabled = false;
    }

    /**
     * @brief Spin the control loop.
     * @param x0 Current input.
     * @return T Compensated output.
     *
     * Since PID is an IIR system, an output is generated for every input.
     * The output is not clipped by the anti-windup saturation.
     */
    T output(T x0) {
        T y0 = _output(x0, kpid);
        if (isAntiWindupEnabled) {
            if (y0 > satMax || y0 < satMin) {
                isAntiWindupActive = true;
                y0 = _output(x0, kpd);
            } else {
                isAntiWindupActive = false;
            }
        }
        _updateStates(x0, y0)
        return y0;
    }

    /**
     * @brief Reset the PID controller to zero state.
     *
     * This method does not change gain and anti-windup settings.
     */
    void reset() {
        for (uint8_t i = 0; i < 4; i++) {
            states[i] = 0
        }
    }

    /**
     * @brief Get if the anti-windup is active for the most recent output.
     */
    bool getIsAntiWindupActive() const {
        return isAntiWindupActive;
    }

    T* getStates() const {
        return states
    }
};

} // namespace
