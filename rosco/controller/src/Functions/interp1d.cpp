#include "../include/vit_types.h"
#include "../include/rosco_array.hpp"
#include <cstring>
#include <cstdio>

// Linear interpolation of (xData, yData) at query point xq.
// Clamps to the endpoint values outside the data range.
double interp1d(ArrayView xData, ArrayView yData, double xq, errorvariables_t* ErrVar) {

    // xData and yData must be the same length
    if (xData.size != yData.size) {
        ErrVar->aviFAIL = -1;
        int len = snprintf(ErrVar->ErrMsg, 1024,
            " SIZE(xData) =%2d and SIZE(yData) =%2d are not the same",
            xData.size, yData.size);
        if (len >= 0 && len < 1024) memset(ErrVar->ErrMsg + len, ' ', 1024 - len);
    }

    // xData must be strictly increasing
    for (int i = 0; i < xData.size - 1; i++) {
        if (xData[i + 1] - xData[i] <= 0.0) {
            ErrVar->aviFAIL = -1;
            const char msg[] = " xData is not strictly increasing";
            int len = (int)sizeof(msg) - 1;
            memcpy(ErrVar->ErrMsg, msg, len);
            if (len < 1024) memset(ErrVar->ErrMsg + len, ' ', 1024 - len);
            break;
        }
    }

    // Clamp to endpoints outside the data range; interpolate within
    int n = xData.size;
    if (xq <= xData[0])      return yData[0];
    if (xq >= xData[n - 1])  return yData[n - 1];

    for (int i = 1; i < n; i++) {
        if (xq <= xData[i]) {
            return yData[i-1] + (yData[i] - yData[i-1])
                              / (xData[i] - xData[i-1])
                              * (xq        - xData[i-1]);
        }
    }

    // Prefix routine name onto any error message (matches Fortran convention)
    if (ErrVar->aviFAIL < 0) {
        int trimmed_len = 1024;
        while (trimmed_len > 0 && ErrVar->ErrMsg[trimmed_len - 1] == ' ')
            trimmed_len--;
        char buf[1024];
        const char prefix[] = "interp1d:";
        int prefix_len = (int)sizeof(prefix) - 1;
        memcpy(buf, prefix, prefix_len);
        int copy_len = (prefix_len + trimmed_len <= 1024) ? trimmed_len : 1024 - prefix_len;
        memcpy(buf + prefix_len, ErrVar->ErrMsg, copy_len);
        int total = prefix_len + copy_len;
        if (total < 1024) memset(buf + total, ' ', 1024 - total);
        memcpy(ErrVar->ErrMsg, buf, 1024);
    }

    return yData[n - 1]; // unreachable; loop above always finds a bracket
}
