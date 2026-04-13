#include "../include/vit_types.h"

double HPFilter(double InputSignal, double DT, double CornerFreq, filterparameters_t* FP, int iStatus, int reset, int* inst, int has_InitialValue, double InitialValue) {
    int idx = *inst - 1;
    HPFState& s = inst_ref(FP->hpf, idx);

    double InitialValue_ = has_InitialValue ? InitialValue : InputSignal;

    if (iStatus == 0 || reset) {
        s.output_last = InitialValue_;
        s.input_last  = InitialValue_;
    }

    double K = 2.0 / DT;

    double result = K / (CornerFreq + K) * InputSignal
                  - K / (CornerFreq + K) * s.input_last
                  - (CornerFreq - K) / (CornerFreq + K) * s.output_last;

    s.input_last  = InputSignal;
    s.output_last = result;
    *inst = *inst + 1;

    return result;
}
