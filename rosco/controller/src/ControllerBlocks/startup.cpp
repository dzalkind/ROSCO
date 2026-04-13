#include "../include/vit_types.h"
#include "../include/vit_translated.h"

void Startup(localvariables_t* LocalVar, const ControlParameters& CntrPar, objectinstances_t* objInst, errorvariables_t* ErrVar) {

    double SU_PrevLoad;

    // Filtered rotor speed
    LocalVar->SU_RotSpeedF = LPFilter(LocalVar->RotSpeed, LocalVar->DT, CntrPar.SU_RotorSpeedCornerFreq,
                                         &LocalVar->FP, LocalVar->iStatus,
                                         (LocalVar->restart != 0) ? 1 : 0,
                                         &objInst->instLPF, 0, 0.0);

    // Initialize startup stage
    if (LocalVar->iStatus == 0) {
        LocalVar->SU_Stage = -1;
    }

    if ((LocalVar->SU_Stage == -1) && (LocalVar->Time > CntrPar.SU_StartTime)) {
        LocalVar->SU_Stage = 1;
    }

    // Determine last time rotor speed was below threshold during freewheeling
    if ((LocalVar->SU_Stage == 1) &&
        (LocalVar->SU_RotSpeedF < 0.95 * CntrPar.SU_RotorSpeedThresh)) {
        LocalVar->SU_LoadStageStartTime = LocalVar->Time;
    }

    // Switch from freewheeling to load stages
    if ((LocalVar->SU_Stage == 1) &&
        (LocalVar->Time >= (CntrPar.SU_FW_MinDuration + LocalVar->SU_LoadStageStartTime))) {
        LocalVar->SU_LoadStageStartTime = LocalVar->Time;
        LocalVar->SU_Stage = 2;
    }

    // Switch to next load stage when criteria are met
    // Fortran arrays are 1-based; view arrays are 0-based
    if ((LocalVar->SU_Stage >= 2) &&
        (LocalVar->Time >= LocalVar->SU_LoadStageStartTime +
            CntrPar.SU_LoadRampDuration[LocalVar->SU_Stage - 2] +
            CntrPar.SU_LoadHoldDuration[LocalVar->SU_Stage - 2]) &&
        (LocalVar->SU_Stage <= CntrPar.SU_LoadStages_N + 1)) {
        LocalVar->SU_Stage = LocalVar->SU_Stage + 1;
        LocalVar->SU_LoadStageStartTime = LocalVar->Time;
    }

    // Set PRC_R_Speed based on SU_Stage
    if (LocalVar->SU_Stage == 1) {
        LocalVar->PRC_R_Speed = CntrPar.SU_RotorSpeedThresh * CntrPar.WE_GearboxRatio / CntrPar.PC_RefSpd;
    } else if (LocalVar->SU_Stage == 2) {
        SU_PrevLoad = 0.0;
        // Ramp up PRC_R_Speed
        LocalVar->PRC_R_Speed = sigma(LocalVar->Time, LocalVar->SU_LoadStageStartTime,
            LocalVar->SU_LoadStageStartTime + CntrPar.SU_LoadRampDuration[LocalVar->SU_Stage - 2],
            CntrPar.SU_RotorSpeedThresh * CntrPar.WE_GearboxRatio / CntrPar.PC_RefSpd, 1.0, ErrVar);
    } else if ((LocalVar->SU_Stage >= 2) && (LocalVar->SU_Stage <= CntrPar.SU_LoadStages_N + 1)) {
        // Fortran: SU_LoadStages(SU_Stage-2) → C: SU_LoadStages[SU_Stage-3]
        SU_PrevLoad = CntrPar.SU_LoadStages[LocalVar->SU_Stage - 3];
    } else if (LocalVar->SU_Stage == CntrPar.SU_LoadStages_N + 2) {
        LocalVar->SU_Stage = 0;
    }

    // Set PRC_R_Torque based on SU_Stage
    if ((LocalVar->SU_Stage == 1) || (LocalVar->SU_Stage == -1)) {
        LocalVar->PRC_R_Torque = 0.0;
    } else if ((LocalVar->SU_Stage >= 2) && (LocalVar->SU_Stage <= CntrPar.SU_LoadStages_N + 1)) {
        if (LocalVar->Time < LocalVar->SU_LoadStageStartTime + CntrPar.SU_LoadRampDuration[LocalVar->SU_Stage - 2]) {
            LocalVar->PRC_R_Torque = sigma(LocalVar->Time, LocalVar->SU_LoadStageStartTime,
                LocalVar->SU_LoadStageStartTime + CntrPar.SU_LoadRampDuration[LocalVar->SU_Stage - 2],
                SU_PrevLoad, CntrPar.SU_LoadStages[LocalVar->SU_Stage - 2], ErrVar);
        } else {
            LocalVar->PRC_R_Torque = CntrPar.SU_LoadStages[LocalVar->SU_Stage - 2];
        }
    }
}
