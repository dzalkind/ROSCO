#include <vector>
#include "../include/vit_translated.h"
#include "../include/rosco_constants.h"
#include "../Filters/seclpfilter_vel.hpp"
#include "../ControlElements/picontroller.hpp"

void CableControl(float* avrSWAP, const ControlParameters& CntrPar, LocalVariables& LocalVar) {
    // CableControl: cable length control
    //   CC_Mode = 1: user-defined step inputs
    //   CC_Mode = 2: open-loop from lookup table

    if (CntrPar.CC_Mode == 1) {
        // User-defined control — step change at t > 500
        if (LocalVar.Time > 500) {
            LocalVar.CC_DesiredL[0] = -14.51;
            LocalVar.CC_DesiredL[1] = 1.58;
            LocalVar.CC_DesiredL[2] = -10.332;
        }

    } else if (CntrPar.CC_Mode == 2) {
        // Open-loop control
        for (int I_GROUP = 0; I_GROUP < CntrPar.CC_Group_N; I_GROUP++) {
            if (CntrPar.Ind_CableControl[I_GROUP] > 0) {
                // Extract row from column-major 2D array
                int n_rows = CntrPar.OL_CableControl_rows;
                int n_cols = CntrPar.OL_CableControl_cols;
                std::vector<double> row_slice(n_cols);
                for (int col = 0; col < n_cols; col++) {
                    row_slice[col] = CntrPar.OL_CableControl[col * n_rows + I_GROUP];
                }
                LocalVar.CC_DesiredL[I_GROUP] = interp1d(CntrPar.OL_Breakpoints,
                                                          {row_slice.data(), n_cols},
                                                          LocalVar.Time);
            }
        }
    }

    // Convert desired to actuated line length and delta length for all groups
    for (int I_GROUP = 0; I_GROUP < CntrPar.CC_Group_N; I_GROUP++) {
        // Get actuated deltaL via second-order low-pass filter
        static SecLPFilterVel ccActFilter[10];
        if (LocalVar.iStatus == 0 || LocalVar.restart != 0) ccActFilter[I_GROUP].init(2.0 * PI / CntrPar.CC_ActTau, 1.0, LocalVar.DT, LocalVar.CC_DesiredL[I_GROUP]);
        LocalVar.CC_ActuatedDL[I_GROUP] = ccActFilter[I_GROUP].step(LocalVar.CC_DesiredL[I_GROUP]);

        // Integrate delta-L to get actuated length
        static PIController ccActPI[10];
        if (LocalVar.iStatus == 0 || LocalVar.restart) {
            ccActPI[I_GROUP].init(LocalVar.CC_ActuatedDL[0]);
            LocalVar.CC_ActuatedL[I_GROUP] = LocalVar.CC_ActuatedDL[0];
        } else {
            LocalVar.CC_ActuatedL[I_GROUP] = ccActPI[I_GROUP].step(LocalVar.CC_ActuatedDL[I_GROUP],
                0.0, 1.0, -1000.0, 1000.0, LocalVar.DT);
        }
    }

    // Assign to avrSWAP
    for (int I_GROUP = 0; I_GROUP < CntrPar.CC_Group_N; I_GROUP++) {
        // Fortran: avrSWAP(CC_GroupIndex(I_GROUP)) and +1, 1-indexed
        int idx = CntrPar.CC_GroupIndex[I_GROUP] - 1;  // 0-indexed
        avrSWAP[idx] = LocalVar.CC_ActuatedL[I_GROUP];
        avrSWAP[idx + 1] = LocalVar.CC_ActuatedDL[I_GROUP];
    }
}
