#include "../include/vit_translated.h"
#include "../include/rosco_constants.h"
#include "hpfilter.hpp"
#include "notchfilter.hpp"
#include "notchfilterslopes.hpp"
#include "seclpfilter.hpp"
#include <cmath>
#include "../include/controller_objects.hpp"

void PreFilterMeasuredSignals(const ControlParameters& CntrPar, LocalVariables& LocalVar) {
    int reset = (LocalVar.restart != 0);

    // Filter the HSS (generator) and LSS (rotor) speed measurement:
    // Apply Low-Pass Filter (choice between first- and second-order low-pass filter)
    if (CntrPar.F_LPFType == 1) {
        auto& genSpeedFilter = ObjState.prefilter.genSpeedFilter;
        if (LocalVar.iStatus == 0 || reset) genSpeedFilter.init(CntrPar.F_LPFCornerFreq, LocalVar.DT, LocalVar.GenSpeed);
        LocalVar.GenSpeedF = genSpeedFilter.step(LocalVar.GenSpeed);

        auto& rotSpeedFilter = ObjState.prefilter.rotSpeedFilter;
        if (LocalVar.iStatus == 0 || reset) rotSpeedFilter.init(CntrPar.F_LPFCornerFreq, LocalVar.DT, LocalVar.RotSpeed);
        LocalVar.RotSpeedF = rotSpeedFilter.step(LocalVar.RotSpeed);
    } else if (CntrPar.F_LPFType == 2) {
        auto& genSpeedFilter2 = ObjState.prefilter.genSpeedFilter2;
        if (LocalVar.iStatus == 0 || reset) genSpeedFilter2.init(CntrPar.F_LPFCornerFreq, CntrPar.F_LPFDamping, LocalVar.DT, LocalVar.GenSpeed);
        LocalVar.GenSpeedF = genSpeedFilter2.step(LocalVar.GenSpeed);

        auto& rotSpeedFilter2 = ObjState.prefilter.rotSpeedFilter2;
        if (LocalVar.iStatus == 0 || reset) rotSpeedFilter2.init(CntrPar.F_LPFCornerFreq, CntrPar.F_LPFDamping, LocalVar.DT, LocalVar.RotSpeed);
        LocalVar.RotSpeedF = rotSpeedFilter2.step(LocalVar.RotSpeed);
    }

    // Apply Notch Filter to Gen Speed
    auto& genSpdNotch = ObjState.prefilter.genSpdNotch;
    for (int n = 1; n <= CntrPar.F_GenSpdNotch_N; n++) {
        int idx = CntrPar.F_GenSpdNotch_Ind[n - 1] - 1;
        if (LocalVar.iStatus == 0 || reset) genSpdNotch[n - 1].init(CntrPar.F_NotchFreqs[idx], CntrPar.F_NotchBetaNum[idx], CntrPar.F_NotchBetaDen[idx], LocalVar.DT, LocalVar.GenSpeedF);
        LocalVar.GenSpeedF = genSpdNotch[n - 1].step(LocalVar.GenSpeedF);
    }

    // Filtering the tower fore-aft acceleration signal
    // Force to start at 0
    if (LocalVar.iStatus == 0 && LocalVar.Time == 0) {
        LocalVar.NacIMU_FA_RAcc = 0;
        LocalVar.FA_Acc_Nac = 0;
    }

    // Low pass
    auto& nacImuFaAccLPF = ObjState.prefilter.nacImuFaAccLPF;
    if (LocalVar.iStatus == 0 || reset) nacImuFaAccLPF.init(CntrPar.F_FlCornerFreq[0], CntrPar.F_FlCornerFreq[1], LocalVar.DT, LocalVar.NacIMU_FA_RAcc);
    LocalVar.NACIMU_FA_AccF = nacImuFaAccLPF.step(LocalVar.NacIMU_FA_RAcc);

    auto& faAccLPF = ObjState.prefilter.faAccLPF;
    if (LocalVar.iStatus == 0 || reset) faAccLPF.init(CntrPar.F_FlCornerFreq[0], CntrPar.F_FlCornerFreq[1], LocalVar.DT, LocalVar.FA_Acc_Nac);
    LocalVar.FA_AccF = faAccLPF.step(LocalVar.FA_Acc_Nac);

    // High pass
    auto& nacImuFaAccHPF = ObjState.prefilter.nacImuFaAccHPF;
    if (LocalVar.iStatus == 0 || reset) nacImuFaAccHPF.init(CntrPar.F_FlHighPassFreq, LocalVar.DT, LocalVar.NACIMU_FA_AccF);
    LocalVar.NACIMU_FA_AccF = nacImuFaAccHPF.step(LocalVar.NACIMU_FA_AccF);

    auto& faAccHPF = ObjState.prefilter.faAccHPF;
    if (LocalVar.iStatus == 0 || reset) faAccHPF.init(CntrPar.F_FlHighPassFreq, LocalVar.DT, LocalVar.FA_AccF);
    LocalVar.FA_AccF = faAccHPF.step(LocalVar.FA_AccF);

    // Tower top notch filters
    auto& twrTopNotchNacImu = ObjState.prefilter.twrTopNotchNacImu;
    auto& twrTopNotchFaAcc = ObjState.prefilter.twrTopNotchFaAcc;
    for (int n = 1; n <= CntrPar.F_TwrTopNotch_N; n++) {
        int idx = CntrPar.F_TwrTopNotch_Ind[n - 1] - 1;
        if (LocalVar.iStatus == 0 || reset) {
            twrTopNotchNacImu[n - 1].init(CntrPar.F_NotchFreqs[idx], CntrPar.F_NotchBetaNum[idx], CntrPar.F_NotchBetaDen[idx], LocalVar.DT, LocalVar.NACIMU_FA_AccF);
            twrTopNotchFaAcc[n - 1].init(CntrPar.F_NotchFreqs[idx], CntrPar.F_NotchBetaNum[idx], CntrPar.F_NotchBetaDen[idx], LocalVar.DT, LocalVar.FA_AccF);
        }
        LocalVar.NACIMU_FA_AccF = twrTopNotchNacImu[n - 1].step(LocalVar.NACIMU_FA_AccF);
        LocalVar.FA_AccF = twrTopNotchFaAcc[n - 1].step(LocalVar.FA_AccF);
    }

    // FA acc for ForeAft damping
    if (CntrPar.TD_Mode > 0) {
        auto& faAccDampHPF = ObjState.prefilter.faAccDampHPF;
        if (LocalVar.iStatus == 0 || reset) faAccDampHPF.init(CntrPar.FA_HPFCornerFreq, LocalVar.DT, LocalVar.FA_Acc_Nac);
        LocalVar.FA_AccHPF = faAccDampHPF.step(LocalVar.FA_Acc_Nac);
    }

    // Filter Wind Speed Estimator Signal
    auto& weVwFilter = ObjState.prefilter.weVwFilter;
    if (LocalVar.iStatus == 0 || reset) weVwFilter.init(CntrPar.F_WECornerFreq, LocalVar.DT, LocalVar.WE_Vw);
    LocalVar.WE_Vw_F = weVwFilter.step(LocalVar.WE_Vw);

    // Blade root bending moment for IPC
    auto& rootMOOPNotchSlopes = ObjState.prefilter.rootMOOPNotchSlopes;
    auto& rootMOOPSecLPF = ObjState.prefilter.rootMOOPSecLPF;
    auto& rootMOOPHPF = ObjState.prefilter.rootMOOPHPF;
    auto& rootMOOPNotch = ObjState.prefilter.rootMOOPNotch;  // up to 10 notches per blade
    for (int K = 0; K < LocalVar.NumBl; K++) {
        if ((CntrPar.IPC_ControlMode > 0) || (CntrPar.Flp_Mode == 3)) {
            // Moving inverted notch at rotor speed to isolate 1P
            if (LocalVar.iStatus == 0 || reset) rootMOOPNotchSlopes[K].init(LocalVar.RotSpeedF, 0.7, LocalVar.DT, LocalVar.rootMOOP[K]);
            rootMOOPNotchSlopes[K].update_coeffs(LocalVar.RotSpeedF, 0.7, LocalVar.DT);
            LocalVar.rootMOOPF[K] = rootMOOPNotchSlopes[K].step(LocalVar.rootMOOP[K]);
        } else if (CntrPar.Flp_Mode == 2) {
            // Filter Blade root bending moments
            if (LocalVar.iStatus == 0 || reset) rootMOOPSecLPF[K].init(CntrPar.F_FlpCornerFreq[0], CntrPar.F_FlpCornerFreq[1], LocalVar.DT, LocalVar.rootMOOP[K]);
            LocalVar.rootMOOPF[K] = rootMOOPSecLPF[K].step(LocalVar.rootMOOP[K]);

            if (LocalVar.iStatus == 0 || reset) rootMOOPHPF[K].init(0.1, LocalVar.DT, LocalVar.rootMOOPF[K]);
            LocalVar.rootMOOPF[K] = rootMOOPHPF[K].step(LocalVar.rootMOOPF[K]);

            // Apply gen speed notch filters to blade root signal
            for (int n = 1; n <= CntrPar.F_GenSpdNotch_N; n++) {
                int idx = CntrPar.F_GenSpdNotch_Ind[n - 1] - 1;
                int nIdx = K * 10 + (n - 1);
                if (LocalVar.iStatus == 0 || reset) rootMOOPNotch[nIdx].init(CntrPar.F_NotchFreqs[idx], CntrPar.F_NotchBetaNum[idx], CntrPar.F_NotchBetaDen[idx], LocalVar.DT, LocalVar.rootMOOPF[K]);
                LocalVar.rootMOOPF[K] = rootMOOPNotch[nIdx].step(LocalVar.rootMOOPF[K]);
            }
        } else {
            LocalVar.rootMOOPF[K] = LocalVar.rootMOOP[K];
        }
    }

    // Control commands (used by WSE, mostly)
    auto& lastGenTrqFilter = ObjState.prefilter.lastGenTrqFilter;
    if (LocalVar.iStatus == 0 || reset) lastGenTrqFilter.init(CntrPar.F_LPFCornerFreq, 0.7, LocalVar.DT, LocalVar.VS_LastGenTrq);
    LocalVar.VS_LastGenTrqF = lastGenTrqFilter.step(LocalVar.VS_LastGenTrq);

    auto& blPitchCMeasFilter = ObjState.prefilter.blPitchCMeasFilter;
    if (LocalVar.iStatus == 0 || reset) blPitchCMeasFilter.init(CntrPar.F_LPFCornerFreq * 0.25, 0.7, LocalVar.DT, LocalVar.BlPitchCMeas);
    LocalVar.BlPitchCMeasF = blPitchCMeasFilter.step(LocalVar.BlPitchCMeas);

    // Wind vane signal
    double NacVane_cos = cos(LocalVar.NacVane * D2R);
    double NacVane_sin = sin(LocalVar.NacVane * D2R);

    auto& nacVaneCosFilter = ObjState.prefilter.nacVaneCosFilter;
    if (LocalVar.iStatus == 0) nacVaneCosFilter.init(CntrPar.F_YawErr, LocalVar.DT, NacVane_cos);
    double NacVaneCosF = nacVaneCosFilter.step(NacVane_cos);

    auto& nacVaneSinFilter = ObjState.prefilter.nacVaneSinFilter;
    if (LocalVar.iStatus == 0) nacVaneSinFilter.init(CntrPar.F_YawErr, LocalVar.DT, NacVane_sin);
    double NacVaneSinF = nacVaneSinFilter.step(NacVane_sin);
    LocalVar.NacVaneF = wrap_180(atan2(NacVaneSinF, NacVaneCosF) * R2D);
}
