#pragma once

// First-order discrete-time low-pass filter.
//
// Usage:
//   LPFilter f;                               // default-constructed (uninitialized)
//   f.init(CornerFreq, DT, InitialValue);     // compute coefficients, set states
//   double y = f.step(input);                 // apply filter for one timestep

class LPFilter {
public:
    LPFilter() = default;
    LPFilter(double CornerFreq, double DT, double InitialValue);

    // Initialize: compute filter coefficients from CornerFreq and DT,
    // and set both state variables to InitialValue.
    void init(double CornerFreq, double DT, double InitialValue);

    // Apply the filter for one timestep. Returns the filtered output.
    double step(double input);

private:
    // Computed filter coefficients (fixed for the lifetime of this filter instance)
    double a1 = 0, a0 = 0;     // denominator
    double b1 = 0, b0 = 0;     // numerator

    // True filter states: the only information carried between timesteps
    double input_last  = 0;
    double output_last = 0;
};
