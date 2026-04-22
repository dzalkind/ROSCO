#pragma once
#include <algorithm>

// PI controller with anti-windup saturation.
//
// Usage:
//   PIController pi;
//   pi.init(I0);                                       // set initial integrator
//   double y = pi.step(error, kp, ki, minVal, maxVal, DT);

class PIController {
public:
    PIController() = default;

    void init(double I0) {
        iterm = I0;
    }

    double step(double error, double kp, double ki, double minValue, double maxValue, double DT) {
        double PTerm = kp * error;
        iterm = iterm + DT * ki * error;
        iterm = std::min(std::max(iterm, minValue), maxValue);
        return std::min(std::max(PTerm + iterm, minValue), maxValue);
    }

private:
    double iterm = 0;
};
