#include <cmath>
#include "../include/rosco_constants.h"

void ColemanTransform(double* rootMOOP, double aziAngle, int nHarmonic, double* axTOut, double* axYOut) {
    const double phi2 = 2.0 / 3.0 * PI;
    const double phi3 = 4.0 / 3.0 * PI;

    *axTOut = 2.0 / 3.0 * (cos(nHarmonic * aziAngle) * rootMOOP[0]
            + cos(nHarmonic * (aziAngle + phi2)) * rootMOOP[1]
            + cos(nHarmonic * (aziAngle + phi3)) * rootMOOP[2]);
    *axYOut = 2.0 / 3.0 * (sin(nHarmonic * aziAngle) * rootMOOP[0]
            + sin(nHarmonic * (aziAngle + phi2)) * rootMOOP[1]
            + sin(nHarmonic * (aziAngle + phi3)) * rootMOOP[2]);
}

void ColemanTransformInverse(double axTIn, double axYIn, double aziAngle, int nHarmonic, double aziOffset, double* PitComIPC) {
    const double phi2 = 2.0 / 3.0 * PI;
    const double phi3 = 4.0 / 3.0 * PI;

    PitComIPC[0] = cos(nHarmonic * (aziAngle + aziOffset)) * axTIn
                 + sin(nHarmonic * (aziAngle + aziOffset)) * axYIn;
    PitComIPC[1] = cos(nHarmonic * (aziAngle + aziOffset + phi2)) * axTIn
                 + sin(nHarmonic * (aziAngle + aziOffset + phi2)) * axYIn;
    PitComIPC[2] = cos(nHarmonic * (aziAngle + aziOffset + phi3)) * axTIn
                 + sin(nHarmonic * (aziAngle + aziOffset + phi3)) * axYIn;
}
