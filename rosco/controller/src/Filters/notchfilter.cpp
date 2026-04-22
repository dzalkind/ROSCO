#include "notchfilter.hpp"

NotchFilter::NotchFilter(double omega, double betaNum, double betaDen, double DT, double InitialValue) {
    init(omega, betaNum, betaDen, DT, InitialValue);
}

void NotchFilter::init(double omega, double betaNum, double betaDen, double DT, double InitialValue) {
    double K = 2.0 / DT;
    double denom = K * K + 2.0 * omega * betaDen * K + omega * omega;
    b2 = (K * K + 2.0 * omega * betaNum * K + omega * omega) / denom;
    b1 = (2.0 * omega * omega - 2.0 * K * K)                  / denom;
    b0 = (K * K - 2.0 * omega * betaNum * K + omega * omega)   / denom;
    a1 = (2.0 * omega * omega - 2.0 * K * K)                   / denom;
    a0 = (K * K - 2.0 * omega * betaDen * K + omega * omega)   / denom;
    input_last1  = InitialValue;
    input_last2  = InitialValue;
    output_last1 = InitialValue;
    output_last2 = InitialValue;
}

double NotchFilter::step(double input) {
    double output = b2 * input
                  + b1 * input_last1
                  + b0 * input_last2
                  - a1 * output_last1
                  - a0 * output_last2;
    input_last2  = input_last1;
    input_last1  = input;
    output_last2 = output_last1;
    output_last1 = output;
    return output;
}
