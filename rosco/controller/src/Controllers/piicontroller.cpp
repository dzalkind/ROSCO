#include "../include/vit_types.h"
#include "../include/vit_translated.h"

double PIIController(double error, double error2, double kp, double ki, double ki2, double minValue, double maxValue, double DT, double I0, piparams_t* piP, int reset, int* inst) {
    int idx = *inst - 1;
    PIState& s = inst_ref(piP->pi, idx);

    double result;

    if (reset) {
        s.iterm       = I0;
        s.iterm_last  = I0;
        s.iterm2      = I0;
        s.iterm2_last = I0;
        result = I0;
    } else {
        double PTerm = kp * error;
        s.iterm  = s.iterm  + DT * ki  * error;
        s.iterm2 = s.iterm2 + DT * ki2 * error2;
        s.iterm  = saturate(s.iterm,  minValue, maxValue);
        s.iterm2 = saturate(s.iterm2, minValue, maxValue);
        result   = saturate(PTerm + s.iterm + s.iterm2, minValue, maxValue);
        s.iterm_last = s.iterm;
    }
    *inst = *inst + 1;

    return result;
}
