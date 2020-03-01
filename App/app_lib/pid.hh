/**
 * @file     pid.hh
 * @brief    Bilinear PID controller.
 * @author   Haoze Zhang
 * @version  20200215
 *
 * Distributed under MIT License
 * Copyright (c) 2020 Haoze Zhang | Brown Engineering
 */

#ifndef INC_PID_HH_
#define INC_PID_HH_
namespace brown {

/**
 * @brief Bilinear PID controller with anti-windup support.
 * @param T IIR filter type. Typical float or double.
 *
 * Discretized with bilinear transform from H(s) = kp + ki/s + kd*s.
 * With an optional first order 1/(tau*s+1) compensation for causality,
 * bandwidth limitation, and in some cases stability.
 * Transfer function:
 *     H(z) = (k0*X(z) + k1*X(z)*z^-1 + k2*X(z)*z^-2) / (1 - z^-2) or
 *     H(z) = (k0*X(z) + k1*X(z)*z^-1 + k2*X(z)*z^-2) / (a + (b-a)*Y(z)*z^-1 - b*z^-2)
 * Difference equation:
 *     y[0] = k0*x[0] + k1*x[-1] + k2*x[-2] + y[-2] or
 *     y[0] = k0/a*x[0] + k1/a*x[-1] + k2/a*x[-2] + (a-b)/a*y[-1] + b/a*y[-2]
 * Marginally stable with two poles on the unit circle without compensation.
 * May cause the close loop system to be unstable with some plants.
 */
template <class T>
class PID {
private:
    // Continuous PID gains
    T kp, ki, kd;
    // Sampling period [s]
    T ts;
    // Optional first order compensation time constant
    T tau = 0;

    // Digital IIR filter states
    // x[-1], x[-2], y[-1], y[-2]
    T states[4];

    // Digital IIR gains
    // x[0], x[-1], x[-2], y[-1], y[-2]
    T kpd[5];  // Digital gains without integration for anti-windup
    T kpid[5]; // Digital gains with integration

    // Saturation range for anti-windup
    T satMax, satMin;
    bool isAntiWindupEnabled = false;
    // Mark the anti-windup activity for the most recent output
    bool isAntiWindupActive = false;

    // Compute digital gains from continuous gains using bilinear transform
    void _deriveGains() {
        T kdOts = kd/ts;
        T kiXts = ki*ts;
        T tauOts = tau/ts;
        T a = 1 + 2*tauOts;
        T b = 1 - 2*tauOts;
        T oneOa = 1/a;
        // gains with ki
        kpid[0] = (kp + kiXts*0.5 + 2.0*kdOts) * oneOa;
        kpid[1] = (kiXts - 4.0*kdOts) * oneOa;
        kpid[2] = (-kp + 0.5*kiXts + 2.0*kdOts) * oneOa;
        kpid[3] = (a-b) * oneOa;
        kpid[4] = b * oneOa;
        // gains with ki=0
        kpd[0] = (kp + 2.0*kdOts) * oneOa;
        kpd[1] = (-4.0*kdOts) * oneOa;
        kpd[2] = (-kp + 2.0*kdOts) * oneOa;
        kpd[3] = kpid[3];
        kpd[4] = kpid[4];
    }

    // IIR filtering, solve the difference equation for y[0]
    T _output(T x0, T* activeGainSet) {
        T out = 0;
        out += x0*activeGainSet[0];
        out += states[0]*activeGainSet[1];
        out += states[1]*activeGainSet[2];
        out += states[2]*activeGainSet[3];
        out += states[3]*activeGainSet[4];
        return out;
    }

    // Update IIR states
    void _updateStates(T x0, T y0) {
        states[1] = states[0];
        states[0] = x0;
        states[3] = states[2];
        states[2] = y0;
    }

public:
    PID() {
        reset();
    }

    /**
     * @brief Constructor.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param ts Sampling period [s] for discretization.
     * @param tau Time constant of the first order compensation.
     *
     * Note that the parameters are continuous gains in
     * H(s) = kp + ki/s + kd*s. The compensated system is H(s)/(tau*s+1).
     */
    PID(T kp, T ki, T kd, T ts, T tau=0) {
        reset(); // set to zero state
        setGains(kp, ki, kd, ts, tau);
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
    void setGains(T kp, T ki, T kd, T ts, T tau=0) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->ts = ts;
        this->tau = tau;
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
        // reset the controller if windup occurred (i.e. y[-1] out of range)
        if (!isAntiWindupEnabled && (states[2] < min || state[2] > max)) {
            reset();
        }
        isAntiWindupEnabled = true;
    }

    /**
     * @brief Disable anti-windup.
     */
    void disableAntiWindup() {
        isAntiWindupEnabled = false;
        isAntiWindupActive = false;
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
        _updateStates(x0, y0);
        return y0;
    }

    /**
     * @brief Reset the PID controller to zero state.
     *
     * This method does not change gain and anti-windup settings.
     */
    void reset() {
        for (uint8_t i = 0; i < 4; i++) {
            states[i] = 0;
        }
    }

    /**
     * @brief Get if the anti-windup is active for the most recent output.
     */
    bool getIsAntiWindupActive() const {
        return isAntiWindupActive;
    }

    T* getStates() const {
        return states;
    }
}; // class PID

} // namespace brown

#endif // INC_PID_HH_
