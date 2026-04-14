#include "../include/vit_types.h"
#include "../include/vit_translated.h"
#include <algorithm>
#include <cstring>
#include <cstdio>

void IPC(const ControlParameters& CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, debugvariables_t* DebugVar, errorvariables_t* ErrVar) {
    // IPC: Individual Pitch Control for 1P and 2P load reduction
    // Also handles yaw-by-IPC (Y_ControlMode == 2)

    double PitComIPC[3], PitComIPCF[3], PitComIPC_1P[3], PitComIPC_2P[3];

    // Coleman transform: rootMOOP → tilt/yaw moment axes (1P and 2P)
    ColemanTransform(LocalVar->rootMOOPF, LocalVar->Azimuth, 1,
                       &LocalVar->axisTilt_1P, &LocalVar->axisYaw_1P);
    ColemanTransform(LocalVar->rootMOOPF, LocalVar->Azimuth, 2,
                       &LocalVar->axisTilt_2P, &LocalVar->axisYaw_2P);

    // High-pass filter MBC yaw component and compute yaw-by-IPC contribution
    double Y_MErrF = 0.0;
    double Y_MErrF_IPC = 0.0;
    if (CntrPar.Y_ControlMode == 2) {
        double Y_MErr = wrap_360(LocalVar->NacHeading + LocalVar->NacVane);
        static LPFilter yawErrFilter;
        if (LocalVar->iStatus == 0 || LocalVar->restart) yawErrFilter.init(CntrPar.F_YawErr, LocalVar->DT, Y_MErr);
        Y_MErrF = yawErrFilter.step(Y_MErr);
        Y_MErrF_IPC = PIController(Y_MErrF, CntrPar.Y_IPC_KP, CntrPar.Y_IPC_KI,
                                      -CntrPar.Y_IPC_IntSat, CntrPar.Y_IPC_IntSat,
                                      LocalVar->DT, 0.0, &LocalVar->piP,
                                      (LocalVar->restart != 0), &objInst->instPI);
    } else {
        LocalVar->axisYawF_1P = LocalVar->axisYaw_1P;
    }

    // Soft cutin with sigma function
    for (int i = 0; i < 2; i++) {
        LocalVar->IPC_KP[i] = sigma(LocalVar->WE_Vw, CntrPar.IPC_Vramp[0],
                                        CntrPar.IPC_Vramp[1], 0.0, CntrPar.IPC_KP[i], ErrVar);
        LocalVar->IPC_KI[i] = sigma(LocalVar->WE_Vw, CntrPar.IPC_Vramp[0],
                                        CntrPar.IPC_Vramp[1], 0.0, CntrPar.IPC_KI[i], ErrVar);
    }

    // Handle saturation limit
    if (CntrPar.IPC_SatMode == 2) {
        LocalVar->IPC_IntSat = std::min(CntrPar.IPC_IntSat,
                                         LocalVar->BlPitchCMeas - CntrPar.PC_MinPit);
    } else if (CntrPar.IPC_SatMode == 3) {
        LocalVar->IPC_IntSat = std::min(CntrPar.IPC_IntSat,
                                         LocalVar->BlPitchCMeas - LocalVar->PC_MinPit);
    } else {
        LocalVar->IPC_IntSat = CntrPar.IPC_IntSat;
    }

    // PI controllers for 1P and 2P
    if (CntrPar.IPC_ControlMode >= 1 && CntrPar.Y_ControlMode != 2) {
        LocalVar->IPC_AxisTilt_1P = PIController(
            LocalVar->axisTilt_1P, LocalVar->IPC_KP[0], LocalVar->IPC_KI[0],
            -LocalVar->IPC_IntSat, LocalVar->IPC_IntSat,
            LocalVar->DT, 0.0, &LocalVar->piP,
            (LocalVar->restart != 0), &objInst->instPI);
        LocalVar->IPC_AxisYaw_1P = PIController(
            LocalVar->axisYawF_1P, LocalVar->IPC_KP[0], LocalVar->IPC_KI[0],
            -LocalVar->IPC_IntSat, LocalVar->IPC_IntSat,
            LocalVar->DT, 0.0, &LocalVar->piP,
            (LocalVar->restart != 0), &objInst->instPI);

        if (CntrPar.IPC_ControlMode >= 2) {
            LocalVar->IPC_AxisTilt_2P = PIController(
                LocalVar->axisTilt_2P, LocalVar->IPC_KP[1], LocalVar->IPC_KI[1],
                -LocalVar->IPC_IntSat, LocalVar->IPC_IntSat,
                LocalVar->DT, 0.0, &LocalVar->piP,
                (LocalVar->restart != 0), &objInst->instPI);
            LocalVar->IPC_AxisYaw_2P = PIController(
                LocalVar->axisYawF_2P, LocalVar->IPC_KP[1], LocalVar->IPC_KI[1],
                -LocalVar->IPC_IntSat, LocalVar->IPC_IntSat,
                LocalVar->DT, 0.0, &LocalVar->piP,
                (LocalVar->restart != 0), &objInst->instPI);
        }
    } else {
        LocalVar->IPC_AxisTilt_1P = 0.0;
        LocalVar->IPC_AxisYaw_1P = 0.0;
        LocalVar->IPC_AxisTilt_2P = 0.0;
        LocalVar->IPC_AxisYaw_2P = 0.0;
    }

    // Add yaw-by-IPC contribution
    double axisYawIPC_1P = LocalVar->IPC_AxisYaw_1P + Y_MErrF_IPC;

    // Inverse Coleman transform → blade pitch commands
    ColemanTransformInverse(LocalVar->IPC_AxisTilt_1P, axisYawIPC_1P,
                               LocalVar->Azimuth, 1, CntrPar.IPC_aziOffset[0], PitComIPC_1P);
    ColemanTransformInverse(LocalVar->IPC_AxisTilt_2P, LocalVar->IPC_AxisYaw_2P,
                               LocalVar->Azimuth, 2, CntrPar.IPC_aziOffset[1], PitComIPC_2P);

    // Sum 1P and 2P contributions, optionally filter
    static LPFilter ipcActFilter[3];
    for (int K = 0; K < LocalVar->NumBl; K++) {
        PitComIPC[K] = PitComIPC_1P[K] + PitComIPC_2P[K];

        if (CntrPar.IPC_CornerFreqAct > 0.0) {
            if (LocalVar->iStatus == 0 || LocalVar->restart) ipcActFilter[K].init(CntrPar.IPC_CornerFreqAct, LocalVar->DT, PitComIPC[K]);
            PitComIPCF[K] = ipcActFilter[K].step(PitComIPC[K]);
        } else {
            PitComIPCF[K] = PitComIPC[K];
        }

        LocalVar->IPC_PitComF[K] = PitComIPCF[K];
    }

    // Prepend routine name to error message if aviFAIL < 0
    if (ErrVar->aviFAIL < 0) {
        char tmp[1024];
        snprintf(tmp, sizeof(tmp), "IPC:%s", ErrVar->ErrMsg);
        strncpy(ErrVar->ErrMsg, tmp, sizeof(ErrVar->ErrMsg) - 1);
        ErrVar->ErrMsg[sizeof(ErrVar->ErrMsg) - 1] = '\0';
    }
}
