#include "hpfilter.hpp"

HPFilter::HPFilter(double CornerFreq, double DT, double InitialValue) {
    init(CornerFreq, DT, InitialValue);
}

void HPFilter::init(double CornerFreq, double DT, double InitialValue) {
    double K = 2.0 / DT;
    c1 =  K / (CornerFreq + K);
    c2 = (CornerFreq - K) / (CornerFreq + K);
    input_last  = InitialValue;
    output_last = InitialValue;
}

double HPFilter::step(double input) {
    double output = c1 * input - c1 * input_last - c2 * output_last;
    input_last  = input;
    output_last = output;
    return output;
}
