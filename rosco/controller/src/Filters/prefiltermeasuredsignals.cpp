#include "../include/vit_types.h"
#include "../include/vit_translated.h"
#include "../include/rosco_constants.h"
#include <cmath>

void PreFilterMeasuredSignals(const ControlParameters& CntrPar, localvariables_t* LocalVar,
                              debugvariables_t* DebugVar, objectinstances_t* objInst,
                              errorvariables_t* ErrVar) {
    // If there's an error, don't even try to run
    if (ErrVar->aviFAIL < 0) {
        return;
    }

    int reset = (LocalVar->restart != 0);

    // Filter the HSS (generator) and LSS (rotor) speed measurement:
    // Apply Low-Pass Filter (choice between first- and second-order low-pass filter)
    if (CntrPar.F_LPFType == 1) {
        static LPFilter genSpeedFilter;
        if (LocalVar->iStatus == 0 || reset) genSpeedFilter.init(CntrPar.F_LPFCornerFreq, LocalVar->DT, LocalVar->GenSpeed);
        LocalVar->GenSpeedF = genSpeedFilter.step(LocalVar->GenSpeed);

        static LPFilter rotSpeedFilter;
        if (LocalVar->iStatus == 0 || reset) rotSpeedFilter.init(CntrPar.F_LPFCornerFreq, LocalVar->DT, LocalVar->RotSpeed);
        LocalVar->RotSpeedF = rotSpeedFilter.step(LocalVar->RotSpeed);
    } else if (CntrPar.F_LPFType == 2) {
        LocalVar->GenSpeedF = SecLPFilter(LocalVar->GenSpeed, LocalVar->DT,
            CntrPar.F_LPFCornerFreq, CntrPar.F_LPFDamping, &LocalVar->FP,
            LocalVar->iStatus, reset, &objInst->instSecLPF, 0, 0.0);
        LocalVar->RotSpeedF = SecLPFilter(LocalVar->RotSpeed, LocalVar->DT,
            CntrPar.F_LPFCornerFreq, CntrPar.F_LPFDamping, &LocalVar->FP,
            LocalVar->iStatus, reset, &objInst->instSecLPF, 0, 0.0);
    }

    // Apply Notch Filter to Gen Speed
    for (int n = 1; n <= CntrPar.F_GenSpdNotch_N; n++) {
        int idx = CntrPar.F_GenSpdNotch_Ind[n - 1] - 1;
        LocalVar->GenSpeedF = NotchFilter(LocalVar->GenSpeedF, LocalVar->DT,
            CntrPar.F_NotchFreqs[idx], CntrPar.F_NotchBetaNum[idx],
            CntrPar.F_NotchBetaDen[idx], &LocalVar->FP, LocalVar->iStatus, reset,
            &objInst->instNotch, 0, 0.0);
    }

    // Filtering the tower fore-aft acceleration signal
    // Force to start at 0
    if (LocalVar->iStatus == 0 && LocalVar->Time == 0) {
        LocalVar->NacIMU_FA_RAcc = 0;
        LocalVar->FA_Acc_Nac = 0;
    }

    // Low pass
    LocalVar->NACIMU_FA_AccF = SecLPFilter(LocalVar->NacIMU_FA_RAcc, LocalVar->DT,
        CntrPar.F_FlCornerFreq[0], CntrPar.F_FlCornerFreq[1], &LocalVar->FP,
        LocalVar->iStatus, reset, &objInst->instSecLPF, 0, 0.0);
    LocalVar->FA_AccF = SecLPFilter(LocalVar->FA_Acc_Nac, LocalVar->DT,
        CntrPar.F_FlCornerFreq[0], CntrPar.F_FlCornerFreq[1], &LocalVar->FP,
        LocalVar->iStatus, reset, &objInst->instSecLPF, 0, 0.0);

    // High pass
    LocalVar->NACIMU_FA_AccF = HPFilter(LocalVar->NACIMU_FA_AccF, LocalVar->DT,
        CntrPar.F_FlHighPassFreq, &LocalVar->FP, LocalVar->iStatus, reset,
        &objInst->instHPF, 0, 0.0);
    LocalVar->FA_AccF = HPFilter(LocalVar->FA_AccF, LocalVar->DT,
        CntrPar.F_FlHighPassFreq, &LocalVar->FP, LocalVar->iStatus, reset,
        &objInst->instHPF, 0, 0.0);

    // Tower top notch filters
    for (int n = 1; n <= CntrPar.F_TwrTopNotch_N; n++) {
        int idx = CntrPar.F_TwrTopNotch_Ind[n - 1] - 1;
        LocalVar->NACIMU_FA_AccF = NotchFilter(LocalVar->NACIMU_FA_AccF, LocalVar->DT,
            CntrPar.F_NotchFreqs[idx], CntrPar.F_NotchBetaNum[idx],
            CntrPar.F_NotchBetaDen[idx], &LocalVar->FP, LocalVar->iStatus, reset,
            &objInst->instNotch, 0, 0.0);
        LocalVar->FA_AccF = NotchFilter(LocalVar->FA_AccF, LocalVar->DT,
            CntrPar.F_NotchFreqs[idx], CntrPar.F_NotchBetaNum[idx],
            CntrPar.F_NotchBetaDen[idx], &LocalVar->FP, LocalVar->iStatus, reset,
            &objInst->instNotch, 0, 0.0);
    }

    // FA acc for ForeAft damping
    if (CntrPar.TD_Mode > 0) {
        LocalVar->FA_AccHPF = HPFilter(LocalVar->FA_Acc_Nac, LocalVar->DT,
            CntrPar.FA_HPFCornerFreq, &LocalVar->FP, LocalVar->iStatus, reset,
            &objInst->instHPF, 0, 0.0);
    }

    // Filter Wind Speed Estimator Signal
    static LPFilter weVwFilter;
    if (LocalVar->iStatus == 0 || reset) weVwFilter.init(CntrPar.F_WECornerFreq, LocalVar->DT, LocalVar->WE_Vw);
    LocalVar->WE_Vw_F = weVwFilter.step(LocalVar->WE_Vw);

    // Blade root bending moment for IPC
    for (int K = 0; K < LocalVar->NumBl; K++) {
        if ((CntrPar.IPC_ControlMode > 0) || (CntrPar.Flp_Mode == 3)) {
            // Moving inverted notch at rotor speed to isolate 1P
            LocalVar->rootMOOPF[K] = NotchFilterSlopes(LocalVar->rootMOOP[K],
                LocalVar->DT, LocalVar->RotSpeedF, 0.7, &LocalVar->FP,
                LocalVar->iStatus, reset, &objInst->instNotchSlopes,
                1, 1,    // has_Moving=1, Moving=1 (.TRUE.)
                0, 0.0); // has_InitialValue=0
        } else if (CntrPar.Flp_Mode == 2) {
            // Filter Blade root bending moments
            LocalVar->rootMOOPF[K] = SecLPFilter(LocalVar->rootMOOP[K], LocalVar->DT,
                CntrPar.F_FlpCornerFreq[0], CntrPar.F_FlpCornerFreq[1], &LocalVar->FP,
                LocalVar->iStatus, reset, &objInst->instSecLPF, 0, 0.0);
            LocalVar->rootMOOPF[K] = HPFilter(LocalVar->rootMOOPF[K], LocalVar->DT,
                0.1, &LocalVar->FP, LocalVar->iStatus, reset,
                &objInst->instHPF, 0, 0.0);

            // Apply gen speed notch filters to blade root signal
            for (int n = 1; n <= CntrPar.F_GenSpdNotch_N; n++) {
                int idx = CntrPar.F_GenSpdNotch_Ind[n - 1] - 1;
                LocalVar->rootMOOPF[K] = NotchFilter(LocalVar->rootMOOPF[K], LocalVar->DT,
                    CntrPar.F_NotchFreqs[idx], CntrPar.F_NotchBetaNum[idx],
                    CntrPar.F_NotchBetaDen[idx], &LocalVar->FP, LocalVar->iStatus, reset,
                    &objInst->instNotch, 0, 0.0);
            }
        } else {
            LocalVar->rootMOOPF[K] = LocalVar->rootMOOP[K];
        }
    }

    // Control commands (used by WSE, mostly)
    LocalVar->VS_LastGenTrqF = SecLPFilter(LocalVar->VS_LastGenTrq, LocalVar->DT,
        CntrPar.F_LPFCornerFreq, 0.7, &LocalVar->FP, LocalVar->iStatus, reset,
        &objInst->instSecLPF, 0, 0.0);
    LocalVar->BlPitchCMeasF = SecLPFilter(LocalVar->BlPitchCMeas, LocalVar->DT,
        CntrPar.F_LPFCornerFreq * 0.25, 0.7, &LocalVar->FP, LocalVar->iStatus, reset,
        &objInst->instSecLPF, 0, 0.0);

    // Wind vane signal
    double NacVane_cos = cos(LocalVar->NacVane * D2R);
    double NacVane_sin = sin(LocalVar->NacVane * D2R);

    static LPFilter nacVaneCosFilter;
    if (LocalVar->iStatus == 0) nacVaneCosFilter.init(CntrPar.F_YawErr, LocalVar->DT, NacVane_cos);
    double NacVaneCosF = nacVaneCosFilter.step(NacVane_cos);

    static LPFilter nacVaneSinFilter;
    if (LocalVar->iStatus == 0) nacVaneSinFilter.init(CntrPar.F_YawErr, LocalVar->DT, NacVane_sin);
    double NacVaneSinF = nacVaneSinFilter.step(NacVane_sin);
    LocalVar->NacVaneF = wrap_180(atan2(NacVaneSinF, NacVaneCosF) * R2D);

    // Debug Variables
    DebugVar->GenSpeedF = LocalVar->GenSpeedF;
    DebugVar->RotSpeedF = LocalVar->RotSpeedF;
    DebugVar->NacIMU_FA_AccF = LocalVar->NACIMU_FA_AccF;
    DebugVar->FA_AccF = LocalVar->FA_AccF;
}
