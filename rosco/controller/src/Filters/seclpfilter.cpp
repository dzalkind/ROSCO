#include "seclpfilter.hpp"

SecLPFilter::SecLPFilter(double CornerFreq, double Damp, double DT, double InitialValue) {
    init(CornerFreq, Damp, DT, InitialValue);
}

void SecLPFilter::init(double CornerFreq, double Damp, double DT, double InitialValue) {
    a2 = DT * DT * CornerFreq * CornerFreq + 4.0 + 4.0 * Damp * CornerFreq * DT;
    a1 = 2.0 * DT * DT * CornerFreq * CornerFreq - 8.0;
    a0 = DT * DT * CornerFreq * CornerFreq + 4.0 - 4.0 * Damp * CornerFreq * DT;
    b2 = DT * DT * CornerFreq * CornerFreq;
    b1 = 2.0 * DT * DT * CornerFreq * CornerFreq;
    b0 = DT * DT * CornerFreq * CornerFreq;
    input_last1  = InitialValue;
    input_last2  = InitialValue;
    output_last1 = InitialValue;
    output_last2 = InitialValue;
}

double SecLPFilter::step(double input) {
    double output = 1.0 / a2 *
        (b2 * input
         + b1 * input_last1
         + b0 * input_last2
         - a1 * output_last1
         - a0 * output_last2);
    input_last2  = input_last1;
    input_last1  = input;
    output_last2 = output_last1;
    output_last1 = output;
    return output;
}
