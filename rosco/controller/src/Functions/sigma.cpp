#include "../include/vit_types.h"
#include <cstring>

double sigma(double x, double x0, double x1, double y0, double y1) {
    double d = x0 - x1;
    double d3 = d * d * d;
    double a3 = 2.0 / d3;
    double a2 = -3.0 * (x0 + x1) / d3;
    double a1 = 6.0 * x1 * x0 / d3;
    double a0 = (x0 - 3.0 * x1) * x0 * x0 / d3;

    double result;
    if (x < x0) {
        result = y0;
    } else if (x > x1) {
        result = y1;
    } else {
        result = (a3 * x * x * x + a2 * x * x + a1 * x + a0) * (y1 - y0) + y0;
    }

    return result;
}
