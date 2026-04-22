#pragma once
#include <algorithm>
#include "../Filters/lpfilter.hpp"

// PID controller with derivative low-pass filter and anti-windup saturation.
//
// Usage:
//   PIDController pid;
//   pid.init(I0, tf, DT, initialError);
//   double y = pid.step(error, kp, ki, kd, minVal, maxVal, DT);

class PIDController {
public:
    PIDController() = default;

    void init(double I0, double tf, double DT, double initialError) {
        iterm  = I0;
        e_last = 0.0;
        deriv.init(tf, DT, initialError);
    }

    void init_deriv(double tf, double DT, double initialError) {
        deriv.init(tf, DT, initialError);
    }

    double step(double error, double kp, double ki, double kd, double minValue, double maxValue, double DT) {
        double EFilt = deriv.step(error);

        double PTerm = kp * error;
        iterm = iterm + DT * ki * error;
        iterm = std::min(std::max(iterm, minValue), maxValue);

        double DTerm = kd * (EFilt - e_last) / DT;
        double result = std::min(std::max(PTerm + iterm + DTerm, minValue), maxValue);

        e_last = EFilt;
        return result;
    }

private:
    double iterm  = 0;
    double e_last = 0;
    LPFilter deriv;
};
