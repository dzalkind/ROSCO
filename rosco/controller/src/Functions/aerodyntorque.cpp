#include "../include/vit_types.h"
#include "../include/vit_translated.h"
#include <cmath>
#include <cstring>
#include <limits>

#include "../include/rosco_constants.h"

double AeroDynTorque(double RotSpeed, double BldPitch, localvariables_t* LocalVar,
                     const ControlParameters& CntrPar, performancedata_view_t* PerfData,
                     errorvariables_t* ErrVar) {

    // Find Torque
    double RotorArea = PI * (CntrPar.WE_BladeRadius * CntrPar.WE_BladeRadius);
    double WindSpeed = LocalVar->WE_Vw > std::numeric_limits<double>::epsilon()
                     ? LocalVar->WE_Vw : std::numeric_limits<double>::epsilon();
    double Lambda = RotSpeed * CntrPar.WE_BladeRadius / WindSpeed;

    // Compute Cp via 2D interpolation on performance surface
    double Cp = interp2d(PerfData->Beta_vec, PerfData->n_Beta_vec,
                           PerfData->TSR_vec, PerfData->n_TSR_vec,
                           PerfData->Cp_mat, PerfData->n_Cp_mat_rows, PerfData->n_Cp_mat_cols,
                           BldPitch * R2D, Lambda, ErrVar);

    double result = 0.5 * (CntrPar.WE_RhoAir * RotorArea) * (LocalVar->WE_Vw * LocalVar->WE_Vw * LocalVar->WE_Vw / RotSpeed) * Cp;
    result = result > 0.0 ? result : 0.0;

    // Add RoutineName to error message
    if (ErrVar->aviFAIL < 0) {
        int trimmed_len = 1024;
        while (trimmed_len > 0 && ErrVar->ErrMsg[trimmed_len - 1] == ' ') trimmed_len--;
        char buf[1024];
        const char prefix[] = "AeroDynTorque:";
        int prefix_len = 14;
        std::memcpy(buf, prefix, prefix_len);
        int copy_len = trimmed_len;
        if (prefix_len + copy_len > 1024) copy_len = 1024 - prefix_len;
        std::memcpy(buf + prefix_len, ErrVar->ErrMsg, copy_len);
        int total = prefix_len + copy_len;
        if (total < 1024) std::memset(buf + total, ' ', 1024 - total);
        std::memcpy(ErrVar->ErrMsg, buf, 1024);
    }

    return result;
}
