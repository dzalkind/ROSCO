#include "../include/vit_types.h"
#include "../include/vit_translated.h"

void SetpointSmoother(LocalVariables& LocalVar, const ControlParameters& CntrPar) {

    // ------ Setpoint Smoothing ------
    if (CntrPar.SS_Mode == 1) {
        // Find setpoint shift amount
        double R_Total = LocalVar.PRC_R_Speed * LocalVar.PRC_R_Torque * LocalVar.PRC_R_Pitch;
        double DelOmega = ((LocalVar.BlPitchCMeas - LocalVar.PC_MinPit) / 0.524) * CntrPar.SS_VSGain
                        - ((CntrPar.VS_RtPwr * R_Total - LocalVar.VS_LastGenPwr)) / CntrPar.VS_RtPwr * CntrPar.SS_PCGain;
        DelOmega = DelOmega * CntrPar.PC_RefSpd;
        // Filter
        static LPFilter ssFilter;
        if (LocalVar.iStatus == 0 || LocalVar.restart) ssFilter.init(CntrPar.F_SSCornerFreq, LocalVar.DT, DelOmega);
        LocalVar.SS_DelOmegaF = ssFilter.step(DelOmega);
    } else {
        LocalVar.SS_DelOmegaF = 0; // No setpoint smoothing
    }
}
