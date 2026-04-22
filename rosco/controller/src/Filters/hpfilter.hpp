#pragma once

// First-order discrete-time high-pass filter.
//
// Usage:
//   HPFilter f;
//   f.init(CornerFreq, DT, InitialValue);
//   double y = f.step(input);

class HPFilter {
public:
    HPFilter() = default;
    HPFilter(double CornerFreq, double DT, double InitialValue);

    void init(double CornerFreq, double DT, double InitialValue);
    double step(double input);

private:
    double c1 = 0;  // K / (CornerFreq + K)
    double c2 = 0;  // (CornerFreq - K) / (CornerFreq + K)
    double input_last  = 0;
    double output_last = 0;
};
