#include "../include/vit_types.h"
#include "../include/vit_translated.h"
#include "../include/rosco_constants.h"

double ResController(double error, double kp, double ki, double freq, double minValue, double maxValue, double DT, resparams_t* resP, int reset, int* inst) {
    int idx = *inst - 1;
    ResState& s = inst_ref(resP->res, idx);

    double omega = 2 * PI * freq;

    double omega2_DT2 = (omega * omega) * (DT * DT);
    double b0 = 4 + omega2_DT2;
    double b1 = -8 + 2 * omega2_DT2;
    double b2 = 4 + omega2_DT2;
    double a0 = b0 * kp + 2 * DT * ki;
    double a1 = b1 * kp;
    double a2 = b2 * kp - 2 * DT * ki;

    double result = 0.0;

    if (reset) {
        s.output_last1 = 0;
        s.output_last2 = 0;
        s.input_last1  = 0;
        s.input_last2  = 0;
    } else {
        result = 1 / b0 * (-b1 * s.output_last1 - b2 * s.output_last2
                            + a0 * error + a1 * s.input_last1 + a2 * s.input_last2);
        result = saturate(result, minValue, maxValue);

        s.input_last2  = s.input_last1;
        s.input_last1  = error;
        s.output_last2 = s.output_last1;
        s.output_last1 = result;
    }
    *inst = *inst + 1;

    return result;
}
