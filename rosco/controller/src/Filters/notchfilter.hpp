#pragma once

// Second-order discrete-time notch (band-stop) filter.
//
// Usage:
//   NotchFilter f;
//   f.init(omega, betaNum, betaDen, DT, InitialValue);
//   double y = f.step(input);

class NotchFilter {
public:
    NotchFilter() = default;
    NotchFilter(double omega, double betaNum, double betaDen, double DT, double InitialValue);

    void init(double omega, double betaNum, double betaDen, double DT, double InitialValue);
    double step(double input);

private:
    double b2 = 0, b1 = 0, b0 = 0;
    double a1 = 0, a0 = 0;
    double input_last1  = 0, input_last2  = 0;
    double output_last1 = 0, output_last2 = 0;
};
