#pragma once
#include <algorithm>

// Resonant controller (proportional-resonant at a specified frequency).
//
// Usage:
//   ResController rc;
//   rc.init();
//   double y = rc.step(error, kp, ki, freq, minVal, maxVal, DT);

class ResController {
public:
    ResController() = default;

    void init() {
        output_last1 = 0;
        output_last2 = 0;
        input_last1  = 0;
        input_last2  = 0;
    }

    double step(double error, double kp, double ki, double freq,
                double minValue, double maxValue, double DT);

private:
    double output_last1 = 0, output_last2 = 0;
    double input_last1  = 0, input_last2  = 0;
};
