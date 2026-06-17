#include "../include/vit_translated.h"

double PitchSaturation(LocalVariables& LocalVar, const ControlParameters& CntrPar) {

    // Define minimum blade pitch angle for peak shaving as a function of estimated wind speed
    LocalVar.PS_Min_Pitch = interp1d(CntrPar.PS_WindSpeeds,
                                       CntrPar.PS_BldPitchMin,
                                       LocalVar.WE_Vw_F);

    // Total min pitch limit is greater of peak shaving and power control pitch
    double result = (LocalVar.PS_Min_Pitch > LocalVar.PRC_Min_Pitch)
                    ? LocalVar.PS_Min_Pitch : LocalVar.PRC_Min_Pitch;

    return result;
}
