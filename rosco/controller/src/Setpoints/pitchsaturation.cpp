#include "../include/vit_types.h"
#include "../include/vit_translated.h"
#include <cstring>

double PitchSaturation(LocalVariables& LocalVar, const ControlParameters& CntrPar, debugvariables_t* DebugVar, errorvariables_t* ErrVar) {

    // Define minimum blade pitch angle for peak shaving as a function of estimated wind speed
    LocalVar.PS_Min_Pitch = interp1d(CntrPar.PS_WindSpeeds,
                                       CntrPar.PS_BldPitchMin,
                                       LocalVar.WE_Vw_F, ErrVar);

    // Total min pitch limit is greater of peak shaving and power control pitch
    double result = (LocalVar.PS_Min_Pitch > LocalVar.PRC_Min_Pitch)
                    ? LocalVar.PS_Min_Pitch : LocalVar.PRC_Min_Pitch;

    // Add RoutineName to error message
    if (ErrVar->aviFAIL < 0) {
        // Fortran: ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        int trimmed_len = 1024;
        while (trimmed_len > 0 && ErrVar->ErrMsg[trimmed_len - 1] == ' ') {
            trimmed_len--;
        }
        char buf[1024];
        const char prefix[] = "PitchSaturation:";
        int prefix_len = 16;
        memcpy(buf, prefix, prefix_len);
        int copy_len = trimmed_len;
        if (prefix_len + copy_len > 1024) copy_len = 1024 - prefix_len;
        memcpy(buf + prefix_len, ErrVar->ErrMsg, copy_len);
        int total = prefix_len + copy_len;
        if (total < 1024) memset(buf + total, ' ', 1024 - total);
        memcpy(ErrVar->ErrMsg, buf, 1024);
    }

    return result;
}
