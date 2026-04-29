// Per-instance state structs for ROSCO filters and integrators.
//
// Replaces the flat [1024] parallel arrays in filterparameters_t, piparams_t,
// resparams_t, and rlparams_t.  Each filter call gets one element of the
// appropriate vector.  Vectors
// grow on demand the first time a new instance index is seen; after that
// the element persists across timesteps, carrying state between calls.
//
// Naming convention: matches the Fortran field names where possible so that
// diffs against the original struct definitions are easy to read.

#pragma once
#include <vector>
#include "../Filters/lpfilter.hpp"

// ---------------------------------------------------------------------------
// Second-order low-pass filter (SecLPFilter)
// ---------------------------------------------------------------------------
struct LPF2State {
    double a2 = 0, a1 = 0, a0 = 0;
    double b2 = 0, b1 = 0, b0 = 0;
    double input_last1  = 0, input_last2  = 0;
    double output_last1 = 0, output_last2 = 0;
};

// ---------------------------------------------------------------------------
// Second-order low-pass velocity filter (SecLPFilter_Vel)
// ---------------------------------------------------------------------------
struct LPFVState {
    double a2 = 0, a1 = 0, a0 = 0;
    double b2 = 0, b1 = 0, b0 = 0;
    double input_last1  = 0, input_last2  = 0;
    double output_last1 = 0, output_last2 = 0;
};

// ---------------------------------------------------------------------------
// High-pass filter (HPFilter) — coefficients computed per-call, only state stored
// ---------------------------------------------------------------------------
struct HPFState {
    double input_last  = 0;
    double output_last = 0;
};

// ---------------------------------------------------------------------------
// Notch filter with slopes (NotchFilterSlopes)
// ---------------------------------------------------------------------------
struct NotchSlopesState {
    double b2 = 0, b0 = 0;                  // numerator (nfs_b1 is always 0)
    double a2 = 0, a1 = 0, a0 = 0;          // denominator
    double input_last1  = 0, input_last2  = 0;
    double output_last1 = 0, output_last2 = 0;
};

// ---------------------------------------------------------------------------
// Notch filter (NotchFilter)
// ---------------------------------------------------------------------------
struct NotchState {
    double b2 = 0, b1 = 0, b0 = 0;
    double a1 = 0, a0 = 0;
    double input_last1  = 0, input_last2  = 0;
    double output_last1 = 0, output_last2 = 0;
};

// ---------------------------------------------------------------------------
// PI / PII controller integrator state
// ---------------------------------------------------------------------------
struct PIState {
    double iterm       = 0;   // ITerm
    double iterm_last  = 0;   // ITermLast
    double iterm2      = 0;   // ITerm2  (PII second integrator)
    double iterm2_last = 0;   // ITermLast2
    double e_last      = 0;   // ELast
    LPFilter deriv;           // derivative low-pass filter (PID only)
};

// ---------------------------------------------------------------------------
// Resonant controller state
// ---------------------------------------------------------------------------
struct ResState {
    double output_last1 = 0, output_last2 = 0;
    double input_last1  = 0, input_last2  = 0;
};

// ---------------------------------------------------------------------------
// Rate limiter state
// ---------------------------------------------------------------------------
struct RLState {
    double last_signal = 0;   // LastSignal
};

// ---------------------------------------------------------------------------
// Helper: return a reference to element idx, growing the vector if needed.
// Usage:  auto& s = inst_ref(vec, idx);
// ---------------------------------------------------------------------------
template <typename T>
inline T& inst_ref(std::vector<T>& v, int idx) {
    if (idx >= static_cast<int>(v.size()))
        v.resize(static_cast<std::size_t>(idx) + 1);
    return v[idx];
}

// ---------------------------------------------------------------------------
// Performance data (rotor Cp/Ct/Cq tables)
// Replaces performancedata_view_t (raw pointer + size pairs)
// ---------------------------------------------------------------------------
struct PerformanceData {
    std::vector<double> TSR_vec;
    std::vector<double> Beta_vec;
    std::vector<double> Cp_mat;  // column-major [n_TSR * n_Beta]
    std::vector<double> Ct_mat;
    std::vector<double> Cq_mat;
};

// ---------------------------------------------------------------------------
// External controller DLL swap array
// Replaces extcontroltype_view_t (raw float* + n_avrSWAP)
// ---------------------------------------------------------------------------
struct ExtControlType {
    std::vector<float> avrSWAP;
};
