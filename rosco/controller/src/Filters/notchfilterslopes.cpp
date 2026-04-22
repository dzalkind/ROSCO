#include "notchfilterslopes.hpp"

NotchFilterSlopes::NotchFilterSlopes(double CornerFreq, double Damp, double DT, double InitialValue) {
    init(CornerFreq, Damp, DT, InitialValue);
}

void NotchFilterSlopes::init(double CornerFreq, double Damp, double DT, double InitialValue) {
    update_coeffs(CornerFreq, Damp, DT);
    input_last1  = InitialValue;
    input_last2  = InitialValue;
    output_last1 = InitialValue;
    output_last2 = InitialValue;
}

void NotchFilterSlopes::update_coeffs(double CornerFreq, double Damp, double DT) {
    double CornerFreq_ = (CornerFreq < 0.0) ? 0.0 : CornerFreq;
    double DT2 = DT * DT;
    double CF2 = CornerFreq_ * CornerFreq_;
    b2 =  2.0 * DT * CornerFreq_;
    b0 = -b2;
    a2 = Damp * DT2 * CF2 + 2.0 * DT * CornerFreq_ + 4.0 * Damp;
    a1 = 2.0 * Damp * DT2 * CF2 - 8.0 * Damp;
    a0 = Damp * DT2 * CF2 - 2.0 * DT * CornerFreq_ + 4.0 * Damp;
}

double NotchFilterSlopes::step(double input) {
    double output = 1.0 / a2 *
        (b2 * input
         + b0 * input_last1
         - a1 * output_last1
         - a0 * output_last2);
    input_last2  = input_last1;
    input_last1  = input;
    output_last2 = output_last1;
    output_last1 = output;
    return output;
}
