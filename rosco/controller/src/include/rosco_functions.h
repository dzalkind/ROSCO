// Declarations for standalone utility functions in Functions/.
//
// These are pure or near-pure functions that do not own persistent state.
// Controller modules, filters, and controller-blocks include this header
// instead of (or in addition to) the monolithic vit_translated.h.

#ifndef ROSCO_FUNCTIONS_H
#define ROSCO_FUNCTIONS_H

#include "vit_types.h"
#include "rosco_array.hpp"
#include "rosco_objects.hpp"

// --- Clamping / wrapping ---
double saturate(double inputValue, double minValue, double maxValue);
double wrap_180(double x);
double wrap_360(double x);
void unwrap(double* x, int n_x, double* unwrap_result);

// --- Interpolation ---
double interp1d(ArrayView xData, ArrayView yData, double xq);
double interp2d(const double* xData, int n_xData, const double* yData, int n_yData,
    const double* zData, int n_zData_rows, int n_zData_cols,
    double xq, double yq);

// --- Smooth step / sigmoid ---
double sigma(double x, double x0, double x1, double y0, double y1);

// --- Coleman (multi-blade coordinate) transforms ---
void ColemanTransform(double* rootMOOP, double aziAngle, int nHarmonic,
                      double* axTOut, double* axYOut);
void ColemanTransformInverse(double axTIn, double axYIn, double aziAngle,
                             int nHarmonic, double aziOffset, double* PitComIPC);

// --- Linear algebra ---
void identity(int n, double* identity_result);

// --- Aerodynamics ---
double AeroDynTorque(double RotSpeed, double BldPitch, double WE_Vw,
                     double WE_BladeRadius, double WE_RhoAir,
                     const PerformanceData& PerfData);

#endif // ROSCO_FUNCTIONS_H
