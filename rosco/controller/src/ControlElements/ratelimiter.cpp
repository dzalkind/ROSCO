#include "ratelimiter.hpp"
#include <algorithm>

RateLimiter::RateLimiter(double initialValue) {
    init(initialValue);
}

void RateLimiter::init(double initialValue) {
    last_signal = initialValue;
}

double RateLimiter::step(double input, double minRate, double maxRate, double DT) {
    double rate = (input - last_signal) / DT;
    rate = std::min(std::max(rate, minRate), maxRate);
    double output = last_signal + rate * DT;
    last_signal = output;
    return output;
}
