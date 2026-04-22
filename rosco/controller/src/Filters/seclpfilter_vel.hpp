#pragma once

// Second-order discrete-time low-pass velocity filter.
// Differs from SecLPFilter in numerator coefficients (velocity output).
//
// Usage:
//   SecLPFilterVel f;
//   f.init(CornerFreq, Damp, DT, InitialValue);
//   double y = f.step(input);

class SecLPFilterVel {
public:
    SecLPFilterVel() = default;
    SecLPFilterVel(double CornerFreq, double Damp, double DT, double InitialValue);

    void init(double CornerFreq, double Damp, double DT, double InitialValue);
    double step(double input);

private:
    double a2 = 0, a1 = 0, a0 = 0;
    double b2 = 0, b1 = 0, b0 = 0;
    double input_last1  = 0, input_last2  = 0;
    double output_last1 = 0, output_last2 = 0;
};
