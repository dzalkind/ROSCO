#include "../include/vit_types.h"

double LPFilter(double InputSignal, double DT, double CornerFreq, filterparameters_t* FP, int iStatus, int reset, int* inst, int has_InitialValue, double InitialValue) {
    int idx = *inst - 1;
    LPF1State& s = inst_ref(FP->lpf1, idx);

    double InitialValue_ = has_InitialValue ? InitialValue : InputSignal;

    if (iStatus == 0 || reset) {
        s.output_last = InitialValue_;
        s.input_last  = InitialValue_;
        s.a1 = 2.0 + CornerFreq * DT;
        s.a0 = CornerFreq * DT - 2.0;
        s.b1 = CornerFreq * DT;
        s.b0 = CornerFreq * DT;
    }

    double result = 1.0 / s.a1 *
        (-s.a0 * s.output_last + s.b1 * InputSignal + s.b0 * s.input_last);

    s.input_last  = InputSignal;
    s.output_last = result;
    *inst = *inst + 1;

    return result;
}
