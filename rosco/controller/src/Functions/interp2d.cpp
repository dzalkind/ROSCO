#include "../include/vit_types.h"
#include "../include/vit_translated.h"
#include <cmath>
#include <cstring>
#include <cstdio>

// Fortran column-major access: zData(i,j) = zData[(j-1)*n_rows + (i-1)]
// C 0-based: zData[col * n_rows + row]
#define Z(row, col) zData[(col) * n_zData_rows + (row)]

double interp2d(double* xData, int n_xData, double* yData, int n_yData,
                double* zData, int n_zData_rows, int n_zData_cols,
                double xq, double yq, errorvariables_t* ErrVar) {

    double result = 0.0;

    // Error catching: xData size must match zData columns
    if (n_xData != n_zData_cols) {
        ErrVar->aviFAIL = -1;
        snprintf(ErrVar->ErrMsg, 1024, " SIZE(xData) =%4d and SIZE(zData,1) =%4d are not the same",
                 n_xData, n_zData_cols);
        int len = (int)strlen(ErrVar->ErrMsg);
        if (len < 1024) memset(ErrVar->ErrMsg + len, ' ', 1024 - len);
    }

    // Error catching: yData size must match zData rows
    if (n_yData != n_zData_rows) {
        ErrVar->aviFAIL = -1;
        snprintf(ErrVar->ErrMsg, 1024, " SIZE(yData) =%4d and SIZE(zData,2) =%4d are not the same",
                 n_yData, n_zData_rows);
        int len = (int)strlen(ErrVar->ErrMsg);
        if (len < 1024) memset(ErrVar->ErrMsg + len, ' ', 1024 - len);
    }

    // Check xData is strictly increasing
    for (int k = 0; k < n_xData - 1; k++) {
        if (xData[k + 1] - xData[k] <= 0) {
            ErrVar->aviFAIL = -1;
            snprintf(ErrVar->ErrMsg, 1024, " xData is not strictly increasing");
            int len = (int)strlen(ErrVar->ErrMsg);
            if (len < 1024) memset(ErrVar->ErrMsg + len, ' ', 1024 - len);
            break;
        }
    }

    // Check yData is strictly increasing
    for (int k = 0; k < n_yData - 1; k++) {
        if (yData[k + 1] - yData[k] <= 0) {
            ErrVar->aviFAIL = -1;
            snprintf(ErrVar->ErrMsg, 1024, " yData is not strictly increasing");
            int len = (int)strlen(ErrVar->ErrMsg);
            if (len < 1024) memset(ErrVar->ErrMsg + len, ' ', 1024 - len);
            break;
        }
    }

    // ---- Find corner indices in x-direction (Fortran j/jj → 0-based) ----
    int j, jj;

    // Find min/max of xData
    double xMin = xData[0], xMax = xData[0];
    for (int k = 1; k < n_xData; k++) {
        if (xData[k] < xMin) xMin = xData[k];
        if (xData[k] > xMax) xMax = xData[k];
    }

    ArrayView yv = {yData, n_yData};
    ArrayView xv = {xData, n_xData};
    if (xq <= xMin || std::isnan(xq)) {
        // On lower x-bound: interp1d on column 0
        return interp1d(yv, {&zData[0], n_zData_rows}, yq, ErrVar);
    } else if (xq >= xMax) {
        // On upper x-bound: interp1d on last column
        int last_col = n_xData - 1;
        return interp1d(yv, {&zData[last_col * n_zData_rows], n_zData_rows}, yq, ErrVar);
    } else {
        jj = -1;
        for (j = 0; j < n_xData; j++) {
            if (xq == xData[j]) {
                // On axis: interp1d on this column
                return interp1d(yv, {&zData[j * n_zData_rows], n_zData_rows}, yq, ErrVar);
            } else if (xq < xData[j]) {
                jj = j;
                break;
            }
        }
        j = j - 1; // Move j back one (j is now the lower bound index)
    }

    // ---- Find corner indices in y-direction (Fortran i/ii → 0-based) ----
    int i, ii;

    // Find min/max of yData
    double yMin = yData[0], yMax = yData[0];
    for (int k = 1; k < n_yData; k++) {
        if (yData[k] < yMin) yMin = yData[k];
        if (yData[k] > yMax) yMax = yData[k];
    }

    if (yq <= yMin || std::isnan(yq)) {
        // On lower y-bound: interp1d on row 0 (strided — need temp copy)
        double* row_temp = new double[n_xData];
        for (int k = 0; k < n_xData; k++) row_temp[k] = Z(0, k);
        result = interp1d(xv, {row_temp, n_xData}, xq, ErrVar);
        delete[] row_temp;
        return result;
    } else if (yq >= yMax) {
        // On upper y-bound: interp1d on last row
        int last_row = n_yData - 1;
        double* row_temp = new double[n_xData];
        for (int k = 0; k < n_xData; k++) row_temp[k] = Z(last_row, k);
        result = interp1d(xv, {row_temp, n_xData}, xq, ErrVar);
        delete[] row_temp;
        return result;
    } else {
        ii = -1;
        for (i = 0; i < n_yData; i++) {
            if (yq == yData[i]) {
                // On axis: interp1d on this row
                double* row_temp = new double[n_xData];
                for (int k = 0; k < n_xData; k++) row_temp[k] = Z(i, k);
                result = interp1d(xv, {row_temp, n_xData}, xq, ErrVar);
                delete[] row_temp;
                return result;
            } else if (yq < yData[i]) {
                ii = i;
                break;
            }
        }
        i = i - 1; // Move i back one
    }

    // ---- Bilinear interpolation ----
    // fQ corners (Fortran 1-based i,j → C 0-based)
    double fQ_11 = Z(i, j);
    double fQ_21 = Z(ii, j);
    double fQ_12 = Z(i, jj);
    double fQ_22 = Z(ii, jj);

    // Interpolate
    double fxy1 = (xData[jj] - xq) / (xData[jj] - xData[j]) * fQ_11
                + (xq - xData[j]) / (xData[jj] - xData[j]) * fQ_12;
    double fxy2 = (xData[jj] - xq) / (xData[jj] - xData[j]) * fQ_21
                + (xq - xData[j]) / (xData[jj] - xData[j]) * fQ_22;
    result = (yData[ii] - yq) / (yData[ii] - yData[i]) * fxy1
           + (yq - yData[i]) / (yData[ii] - yData[i]) * fxy2;

    // Add RoutineName to error message
    if (ErrVar->aviFAIL < 0) {
        int trimmed_len = 1024;
        while (trimmed_len > 0 && ErrVar->ErrMsg[trimmed_len - 1] == ' ') trimmed_len--;
        char buf[1024];
        const char prefix[] = "interp2d:";
        int prefix_len = 9;
        memcpy(buf, prefix, prefix_len);
        int copy_len = trimmed_len;
        if (prefix_len + copy_len > 1024) copy_len = 1024 - prefix_len;
        memcpy(buf + prefix_len, ErrVar->ErrMsg, copy_len);
        int total = prefix_len + copy_len;
        if (total < 1024) memset(buf + total, ' ', 1024 - total);
        memcpy(ErrVar->ErrMsg, buf, 1024);
    }

    return result;
}

#undef Z
