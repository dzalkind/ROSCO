#pragma once

// Second-order notch filter with slope-based coefficients.
// Supports dynamic coefficient updates (for moving notch at rotor speed).
//
// Usage:
//   NotchFilterSlopes f;
//   f.init(CornerFreq, Damp, DT, InitialValue);
//   f.update_coeffs(CornerFreq, Damp, DT);  // call when frequency changes
//   double y = f.step(input);

class NotchFilterSlopes {
public:
    NotchFilterSlopes() = default;
    NotchFilterSlopes(double CornerFreq, double Damp, double DT, double InitialValue);

    void init(double CornerFreq, double Damp, double DT, double InitialValue);
    void update_coeffs(double CornerFreq, double Damp, double DT);
    double step(double input);

private:
    double b2 = 0, b0 = 0;
    double a2 = 0, a1 = 0, a0 = 0;
    double input_last1  = 0, input_last2  = 0;
    double output_last1 = 0, output_last2 = 0;
};
