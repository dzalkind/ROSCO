#include "../include/vit_types.h"
#include "../include/vit_translated.h"
#include <cstring>
#include <cstdio>
#include "../include/rosco_constants.h"

void CableControl(float* avrSWAP, controlparameters_view_t* CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, errorvariables_t* ErrVar) {
    // CableControl: cable length control
    //   CC_Mode = 1: user-defined step inputs
    //   CC_Mode = 2: open-loop from lookup table

    if (CntrPar->CC_Mode == 1) {
        // User-defined control — step change at t > 500
        if (LocalVar->Time > 500) {
            LocalVar->CC_DesiredL[0] = -14.51;
            LocalVar->CC_DesiredL[1] = 1.58;
            LocalVar->CC_DesiredL[2] = -10.332;
        }

    } else if (CntrPar->CC_Mode == 2) {
        // Open-loop control
        for (int I_GROUP = 0; I_GROUP < CntrPar->CC_Group_N; I_GROUP++) {
            if (CntrPar->Ind_CableControl[I_GROUP] > 0) {
                // Extract row from column-major 2D array
                int n_rows = CntrPar->n_OL_CableControl_rows;
                int n_cols = CntrPar->n_OL_CableControl_cols;
                double row_slice[n_cols];
                for (int col = 0; col < n_cols; col++) {
                    row_slice[col] = CntrPar->OL_CableControl[col * n_rows + I_GROUP];
                }
                LocalVar->CC_DesiredL[I_GROUP] = interp1d({CntrPar->OL_Breakpoints, CntrPar->n_OL_Breakpoints},
                                                          {row_slice, n_cols},
                                                          LocalVar->Time, ErrVar);
            }
        }
    }

    // Convert desired to actuated line length and delta length for all groups
    for (int I_GROUP = 0; I_GROUP < CntrPar->CC_Group_N; I_GROUP++) {
        // Get actuated deltaL via second-order low-pass filter
        LocalVar->CC_ActuatedDL[I_GROUP] = SecLPFilter_Vel(
            LocalVar->CC_DesiredL[I_GROUP],
            LocalVar->DT,
            2.0 * PI / CntrPar->CC_ActTau,   // CornerFreq
            1.0,                                  // Damp
            &LocalVar->FP,
            LocalVar->iStatus,
            (LocalVar->restart != 0),
            &objInst->instSecLPFV,
            0, 0.0  // no InitialValue
        );

        // Integrate delta-L to get actuated length
        LocalVar->CC_ActuatedL[I_GROUP] = PIController(
            LocalVar->CC_ActuatedDL[I_GROUP],
            0.0,                                  // kp
            1.0,                                  // ki (pure integrator)
            -1000.0,                              // minValue
            1000.0,                               // maxValue
            LocalVar->DT,
            LocalVar->CC_ActuatedDL[0],           // I0: initial from first group
            &LocalVar->piP,
            (LocalVar->restart != 0),
            &objInst->instPI
        );
    }

    // Assign to avrSWAP
    for (int I_GROUP = 0; I_GROUP < CntrPar->CC_Group_N; I_GROUP++) {
        // Fortran: avrSWAP(CC_GroupIndex(I_GROUP)) and +1, 1-indexed
        int idx = CntrPar->CC_GroupIndex[I_GROUP] - 1;  // 0-indexed
        avrSWAP[idx] = LocalVar->CC_ActuatedL[I_GROUP];
        avrSWAP[idx + 1] = LocalVar->CC_ActuatedDL[I_GROUP];
    }

    // Prepend routine name to error message if aviFAIL < 0
    if (ErrVar->aviFAIL < 0) {
        char tmp[1024];
        snprintf(tmp, sizeof(tmp), "CableControl:%s", ErrVar->ErrMsg);
        strncpy(ErrVar->ErrMsg, tmp, sizeof(ErrVar->ErrMsg) - 1);
        ErrVar->ErrMsg[sizeof(ErrVar->ErrMsg) - 1] = '\0';
    }
}
