#include "../include/vit_types.h"
#include "../include/vit_translated.h"

double PIDController(double error, double kp, double ki, double kd, double tf, double minValue, double maxValue, double DT, double I0, piparams_t* piP, int reset, objectinstances_t* objInst, localvariables_t* LocalVar) {
    int piIdx = objInst->instPI - 1;
    PIState& s = inst_ref(piP->pi, piIdx);

    double result;

    double EFilt = LPFilter(error, DT, tf, &LocalVar->FP, LocalVar->iStatus, reset, &objInst->instLPF, 0, 0.0);

    if (reset) {
        s.iterm      = I0;
        s.iterm_last = I0;
        s.e_last     = 0.0;
        result = I0;
    } else {
        double PTerm = kp * error;

        s.iterm = s.iterm + DT * ki * error;
        s.iterm = saturate(s.iterm, minValue, maxValue);

        double DTerm = kd * (EFilt - s.e_last) / DT;

        result = saturate(PTerm + s.iterm + DTerm, minValue, maxValue);

        s.iterm_last = s.iterm;
        s.e_last     = EFilt;
    }

    objInst->instPI = objInst->instPI + 1;

    return result;
}
