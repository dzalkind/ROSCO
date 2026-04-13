#include "../include/vit_types.h"

double NotchFilter(double InputSignal, double DT, double omega, double betaNum, double betaDen, filterparameters_t* FP, int iStatus, int reset, int* inst, int has_InitialValue, double InitialValue) {
    int idx = *inst - 1;
    NotchState& s = inst_ref(FP->nf, idx);

    double InitialValue_ = has_InitialValue ? InitialValue : InputSignal;

    double K = 2.0 / DT;
    if (iStatus == 0 || reset) {
        s.output_last1 = InitialValue_;
        s.output_last2 = InitialValue_;
        s.input_last1  = InitialValue_;
        s.input_last2  = InitialValue_;

        s.b2 = (K * K + 2.0 * omega * betaNum * K + omega * omega) / (K * K + 2.0 * omega * betaDen * K + omega * omega);
        s.b1 = (2.0 * omega * omega - 2.0 * K * K)                  / (K * K + 2.0 * omega * betaDen * K + omega * omega);
        s.b0 = (K * K - 2.0 * omega * betaNum * K + omega * omega)   / (K * K + 2.0 * omega * betaDen * K + omega * omega);
        s.a1 = (2.0 * omega * omega - 2.0 * K * K)                   / (K * K + 2.0 * omega * betaDen * K + omega * omega);
        s.a0 = (K * K - 2.0 * omega * betaDen * K + omega * omega)   / (K * K + 2.0 * omega * betaDen * K + omega * omega);
    }

    double result = s.b2 * InputSignal
                  + s.b1 * s.input_last1
                  + s.b0 * s.input_last2
                  - s.a1 * s.output_last1
                  - s.a0 * s.output_last2;

    s.input_last2  = s.input_last1;
    s.input_last1  = InputSignal;
    s.output_last2 = s.output_last1;
    s.output_last1 = result;
    *inst = *inst + 1;

    return result;
}
