#include "../include/vit_types.h"
#include "../include/rosco_array.hpp"
#include "../include/rosco_error.hpp"
#include <cmath>
#include <cstring>
#include <cstdio>
#include <vector>

// Linear interpolation of (xData, yData) at query point xq.
// Clamps to the endpoint values outside the data range.
double interp1d(ArrayView xData, ArrayView yData, double xq) {

    // xData and yData must be the same length
    if (xData.size != yData.size) {
        rosco_throw("interp1d", "SIZE(xData) =%2d and SIZE(yData) =%2d are not the same",
                     xData.size, yData.size);
    }

    // xData must be strictly increasing
    for (int i = 0; i < xData.size - 1; i++) {
        if (xData[i + 1] - xData[i] <= 0.0) {
            rosco_throw("interp1d", "xData is not strictly increasing");
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

    return yData[n - 1]; // unreachable; loop above always finds a bracket
}

// Fortran column-major access: zData(i,j) = zData[(j-1)*n_rows + (i-1)]
// C 0-based: zData[col * n_rows + row]
#define Z(row, col) zData[(col) * n_zData_rows + (row)]

double interp2d(const double* xData, int n_xData, const double* yData, int n_yData,
                const double* zData, int n_zData_rows, int n_zData_cols,
                double xq, double yq) {

    double result = 0.0;

    // Error catching: xData size must match zData columns
    if (n_xData != n_zData_cols) {
        rosco_throw("interp2d", "SIZE(xData) =%4d and SIZE(zData,1) =%4d are not the same",
                     n_xData, n_zData_cols);
    }

    // Error catching: yData size must match zData rows
    if (n_yData != n_zData_rows) {
        rosco_throw("interp2d", "SIZE(yData) =%4d and SIZE(zData,2) =%4d are not the same",
                     n_yData, n_zData_rows);
    }

    // Check xData is strictly increasing
    for (int k = 0; k < n_xData - 1; k++) {
        if (xData[k + 1] - xData[k] <= 0) {
            rosco_throw("interp2d", "xData is not strictly increasing");
        }
    }

    // Check yData is strictly increasing
    for (int k = 0; k < n_yData - 1; k++) {
        if (yData[k + 1] - yData[k] <= 0) {
            rosco_throw("interp2d", "yData is not strictly increasing");
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

    ArrayView yv = {const_cast<double*>(yData), n_yData};
    ArrayView xv = {const_cast<double*>(xData), n_xData};
    if (xq <= xMin || std::isnan(xq)) {
        // On lower x-bound: interp1d on column 0
        return interp1d(yv, {const_cast<double*>(&zData[0]), n_zData_rows}, yq);
    } else if (xq >= xMax) {
        // On upper x-bound: interp1d on last column
        int last_col = n_xData - 1;
        return interp1d(yv, {const_cast<double*>(&zData[last_col * n_zData_rows]), n_zData_rows}, yq);
    } else {
        jj = -1;
        for (j = 0; j < n_xData; j++) {
            if (xq == xData[j]) {
                // On axis: interp1d on this column
                return interp1d(yv, {const_cast<double*>(&zData[j * n_zData_rows]), n_zData_rows}, yq);
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
        std::vector<double> row_temp(n_xData);
        for (int k = 0; k < n_xData; k++) row_temp[k] = Z(0, k);
        return interp1d(xv, {row_temp.data(), n_xData}, xq);
    } else if (yq >= yMax) {
        // On upper y-bound: interp1d on last row
        int last_row = n_yData - 1;
        std::vector<double> row_temp(n_xData);
        for (int k = 0; k < n_xData; k++) row_temp[k] = Z(last_row, k);
        return interp1d(xv, {row_temp.data(), n_xData}, xq);
    } else {
        ii = -1;
        for (i = 0; i < n_yData; i++) {
            if (yq == yData[i]) {
                // On axis: interp1d on this row
                std::vector<double> row_temp(n_xData);
                for (int k = 0; k < n_xData; k++) row_temp[k] = Z(i, k);
                return interp1d(xv, {row_temp.data(), n_xData}, xq);
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

    return result;
}

#undef Z
