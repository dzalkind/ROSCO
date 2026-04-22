#pragma once
#include <algorithm>

// PII controller (proportional + two integrators) with anti-windup saturation.
//
// Usage:
//   PIIController pii;
//   pii.init(I0);
//   double y = pii.step(error, error2, kp, ki, ki2, minVal, maxVal, DT);

class PIIController {
public:
    PIIController() = default;

    void init(double I0) {
        iterm  = I0;
        iterm2 = I0;
    }

    double step(double error, double error2, double kp, double ki, double ki2,
                double minValue, double maxValue, double DT) {
        double PTerm = kp * error;
        iterm  = iterm  + DT * ki  * error;
        iterm2 = iterm2 + DT * ki2 * error2;
        iterm  = std::min(std::max(iterm,  minValue), maxValue);
        iterm2 = std::min(std::max(iterm2, minValue), maxValue);
        double result = std::min(std::max(PTerm + iterm + iterm2, minValue), maxValue);
        return result;
    }

private:
    double iterm  = 0;
    double iterm2 = 0;
};
