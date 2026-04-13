#include "../include/vit_types.h"

double NotchFilterSlopes(double InputSignal, double DT, double CornerFreq, double Damp, filterparameters_t* FP, int iStatus, int reset, int* inst, int has_Moving, int Moving, int has_InitialValue, double InitialValue) {
    int idx = *inst - 1;
    NotchSlopesState& s = inst_ref(FP->nfs, idx);

    double InitialValue_ = has_InitialValue ? InitialValue : InputSignal;
    int Moving_ = has_Moving ? Moving : 0;

    double CornerFreq_ = (CornerFreq < 0.0) ? 0.0 : CornerFreq;

    if (iStatus == 0 || reset) {
        s.output_last1 = InitialValue_;
        s.output_last2 = InitialValue_;
        s.input_last1  = InitialValue_;
        s.input_last2  = InitialValue_;
    }

    if (iStatus == 0 || reset || Moving_) {
        double DT2 = DT * DT;
        double CF2 = CornerFreq_ * CornerFreq_;
        s.b2 = 2.0 * DT * CornerFreq_;
        s.b0 = -s.b2;
        s.a2 = Damp * DT2 * CF2 + 2.0 * DT * CornerFreq_ + 4.0 * Damp;
        s.a1 = 2.0 * Damp * DT2 * CF2 - 8.0 * Damp;
        s.a0 = Damp * DT2 * CF2 - 2.0 * DT * CornerFreq_ + 4.0 * Damp;
    }

    double result = 1.0 / s.a2 *
        (s.b2 * InputSignal
         + s.b0 * s.input_last1
         - s.a1 * s.output_last1
         - s.a0 * s.output_last2);

    s.input_last2  = s.input_last1;
    s.input_last1  = InputSignal;
    s.output_last2 = s.output_last1;
    s.output_last1 = result;
    *inst = *inst + 1;

    return result;
}
