#include "lpfilter.hpp"

LPFilter::LPFilter(double CornerFreq, double DT, double InitialValue) {
    init(CornerFreq, DT, InitialValue);
}

void LPFilter::init(double CornerFreq, double DT, double InitialValue) {
    a1 = 2.0 + CornerFreq * DT;
    a0 = CornerFreq * DT - 2.0;
    b1 = CornerFreq * DT;
    b0 = CornerFreq * DT;
    input_last  = InitialValue;
    output_last = InitialValue;
}

double LPFilter::step(double input) {
    double output = (1.0 / a1) * (-a0 * output_last + b1 * input + b0 * input_last);
    input_last  = input;
    output_last = output;
    return output;
}
