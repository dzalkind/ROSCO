#include "../include/rosco_constants.h"

double wrap_180(double x) {
    if (x <= -180.0) {
        return x + 360.0;
    } else if (x > 180.0) {
        return x - 360.0;
    } else {
        return x;
    }
}

double wrap_360(double x) {
    if (x < 0.0) {
        return x + 360.0;
    } else if (x >= 360.0) {
        return x - 360.0;
    } else {
        return x;
    }
}

#include "../include/vit_types.h"
#include <cstring>
#include <cstdio>

void unwrap(double* x, int n_x, errorvariables_t* ErrVar, double* unwrap_result) {
    // Copy input to result (Fortran: y = x)
    for (int i = 0; i < n_x; i++) {
        unwrap_result[i] = x[i];
    }

    // Unwrap: adjust elements from i onward by ±2*PI until consecutive
    // differences are in [-PI, PI]
    for (int i = 1; i < n_x; i++) {
        while (unwrap_result[i] - unwrap_result[i - 1] <= -PI) {
            for (int j = i; j < n_x; j++) {
                unwrap_result[j] += 2.0 * PI;
            }
        }
        while (unwrap_result[i] - unwrap_result[i - 1] >= PI) {
            for (int j = i; j < n_x; j++) {
                unwrap_result[j] -= 2.0 * PI;
            }
        }
    }

    // Prepend routine name to error message if aviFAIL < 0
    if (ErrVar->aviFAIL < 0) {
        char tmp[1024];
        snprintf(tmp, sizeof(tmp), "unwrap:%s", ErrVar->ErrMsg);
        int len = (int)strlen(tmp);
        memcpy(ErrVar->ErrMsg, tmp, len);
        for (int k = len; k < 1024; k++) {
            ErrVar->ErrMsg[k] = ' ';
        }
    }
}
