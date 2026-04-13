#include "../include/vit_types.h"

double SecLPFilter(double InputSignal, double DT, double CornerFreq, double Damp, filterparameters_t* FP, int iStatus, int reset, int* inst, int has_InitialValue, double InitialValue) {
    int idx = *inst - 1;
    LPF2State& s = inst_ref(FP->lpf2, idx);

    double InitialValue_ = has_InitialValue ? InitialValue : InputSignal;

    if (iStatus == 0 || reset) {
        s.output_last1 = InitialValue_;
        s.output_last2 = InitialValue_;
        s.input_last1  = InitialValue_;
        s.input_last2  = InitialValue_;

        s.a2 = DT * DT * CornerFreq * CornerFreq + 4.0 + 4.0 * Damp * CornerFreq * DT;
        s.a1 = 2.0 * DT * DT * CornerFreq * CornerFreq - 8.0;
        s.a0 = DT * DT * CornerFreq * CornerFreq + 4.0 - 4.0 * Damp * CornerFreq * DT;
        s.b2 = DT * DT * CornerFreq * CornerFreq;
        s.b1 = 2.0 * DT * DT * CornerFreq * CornerFreq;
        s.b0 = DT * DT * CornerFreq * CornerFreq;
    }

    double result = 1.0 / s.a2 *
        (s.b2 * InputSignal
         + s.b1 * s.input_last1
         + s.b0 * s.input_last2
         - s.a1 * s.output_last1
         - s.a0 * s.output_last2);

    s.input_last2  = s.input_last1;
    s.input_last1  = InputSignal;
    s.output_last2 = s.output_last1;
    s.output_last1 = result;
    *inst = *inst + 1;

    return result;
}
