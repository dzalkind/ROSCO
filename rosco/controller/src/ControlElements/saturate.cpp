#include <algorithm>

double saturate(double val, double minval, double maxval) {
    return std::min(std::max(val, minval), maxval);
}
