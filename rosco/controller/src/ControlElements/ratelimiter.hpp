#pragma once

// Rate limiter: clamps the rate of change of a signal.
//
// Usage:
//   RateLimiter rl;
//   rl.init(initialValue);
//   double y = rl.step(input, minRate, maxRate, DT);

class RateLimiter {
public:
    RateLimiter() = default;
    RateLimiter(double initialValue);

    void init(double initialValue);
    double step(double input, double minRate, double maxRate, double DT);

private:
    double last_signal = 0;
};
