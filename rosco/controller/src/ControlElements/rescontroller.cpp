#include "rescontroller.hpp"
#include <cmath>

#ifndef PI
#define PI 3.14159265358979323846
#endif

double ResController::step(double error, double kp, double ki, double freq,
                           double minValue, double maxValue, double DT) {
    double omega = 2 * PI * freq;
    double omega2_DT2 = (omega * omega) * (DT * DT);
    double b0 = 4 + omega2_DT2;
    double b1 = -8 + 2 * omega2_DT2;
    double b2 = 4 + omega2_DT2;
    double a0 = b0 * kp + 2 * DT * ki;
    double a1 = b1 * kp;
    double a2 = b2 * kp - 2 * DT * ki;

    double result = 1.0 / b0 * (-b1 * output_last1 - b2 * output_last2
                                 + a0 * error + a1 * input_last1 + a2 * input_last2);
    result = std::min(std::max(result, minValue), maxValue);

    input_last2  = input_last1;
    input_last1  = error;
    output_last2 = output_last1;
    output_last1 = result;

    return result;
}
