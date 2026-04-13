#include "../include/vit_types.h"
#include "../include/vit_translated.h"
#include <algorithm>

double ratelimit(double inputSignal, double minRate, double maxRate, double DT, int reset, rlparams_t* rlP, int* inst, int has_ResetValue, double ResetValue) {
    double resetValue_ = has_ResetValue ? ResetValue : inputSignal;

    int idx = *inst - 1;
    RLState& s = inst_ref(rlP->rl, idx);

    double result;

    if (reset) {
        s.last_signal = resetValue_;
        result = resetValue_;
    } else {
        double rate = (inputSignal - s.last_signal) / DT;
        rate   = saturate(rate, minRate, maxRate);
        result = s.last_signal + rate * DT;
        s.last_signal = result;
    }
    *inst = *inst + 1;

    return result;
}
