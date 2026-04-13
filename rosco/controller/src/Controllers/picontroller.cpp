#include "../include/vit_types.h"
#include "../include/vit_translated.h"

double PIController(double error, double kp, double ki, double minValue, double maxValue, double DT, double I0, piparams_t* piP, int reset, int* inst) {
    int idx = *inst - 1;
    PIState& s = inst_ref(piP->pi, idx);

    double result;

    if (reset) {
        s.iterm      = I0;
        s.iterm_last = I0;
        result = I0;
    } else {
        double PTerm = kp * error;
        s.iterm = s.iterm + DT * ki * error;
        s.iterm = saturate(s.iterm, minValue, maxValue);
        result  = saturate(PTerm + s.iterm, minValue, maxValue);
        s.iterm_last = s.iterm;
    }
    *inst = *inst + 1;

    return result;
}
