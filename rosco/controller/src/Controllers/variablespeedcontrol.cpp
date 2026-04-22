#include "../include/vit_types.h"
#include "../include/vit_translated.h"
#include <algorithm>
#include <cstring>
#include <cstdio>
#include "../include/rosco_constants.h"
#include "../ControlElements/ratelimiter.hpp"
#include "../ControlElements/picontroller.hpp"
#include "../ControlElements/pidcontroller.hpp"

void VariableSpeedControl(float* avrSWAP, const ControlParameters& CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, errorvariables_t* ErrVar) {
    // VariableSpeedControl: generator torque controller
    // State machine with K*Omega^2 law, PI transitions, constant torque/power modes

    // Pre-compute generator torque values
    LocalVar->VS_KOmega2_GenTq = CntrPar.VS_Rgn2K * LocalVar->GenSpeedF * LocalVar->GenSpeedF;
    LocalVar->VS_ConstPwr_GenTq = (CntrPar.VS_RtPwr / (CntrPar.VS_GenEff / 100.0)) / LocalVar->GenSpeedF * LocalVar->PRC_R_Torque;

    // Determine maximum torque saturation limit
    if (CntrPar.VS_FBP == VS_FBP_Variable_Pitch) {
        if (CntrPar.VS_ConstPower == VS_Mode_ConstPwr) {
            LocalVar->VS_MaxTq = std::min(LocalVar->VS_ConstPwr_GenTq, CntrPar.VS_MaxTq);
        } else {
            LocalVar->VS_MaxTq = CntrPar.VS_RtTq * LocalVar->PRC_R_Torque;
        }
    } else {
        LocalVar->VS_MaxTq = CntrPar.VS_MaxTq;
    }

    // TSR tracking controller modes
    if ((CntrPar.VS_ControlMode == VS_Mode_WSE_TSR) ||
        (CntrPar.VS_ControlMode == VS_Mode_Power_TSR) ||
        (CntrPar.VS_ControlMode == VS_Mode_Torque_TSR)) {

        static PIController genTqPI;
        if (LocalVar->iStatus == 0 || LocalVar->restart) {
            genTqPI.init(LocalVar->VS_LastGenTrq);
            LocalVar->GenTq = LocalVar->VS_LastGenTrq;
        } else {
            LocalVar->GenTq = genTqPI.step(LocalVar->VS_SpdErr, CntrPar.VS_KP[0], CntrPar.VS_KI[0],
                CntrPar.VS_MinTq, LocalVar->VS_MaxTq, LocalVar->DT);
        }

        if (CntrPar.VS_FBP == VS_FBP_Power_Overspeed) {
            LocalVar->GenTq = std::min(LocalVar->VS_ConstPwr_GenTq, LocalVar->GenTq);
        }

    } else if (CntrPar.VS_ControlMode == VS_Mode_KOmega) {
        // K*Omega^2 with PI transitions
        static PIController genArTqPI;
        if (LocalVar->iStatus == 0 || LocalVar->restart) {
            genArTqPI.init(CntrPar.VS_MaxOMTq);
            LocalVar->GenArTq = CntrPar.VS_MaxOMTq;
        } else {
            LocalVar->GenArTq = genArTqPI.step(LocalVar->VS_SpdErrAr, CntrPar.VS_KP[0], CntrPar.VS_KI[0],
                CntrPar.VS_MaxOMTq, CntrPar.VS_ArSatTq, LocalVar->DT);
        }
        static PIController genBrTqPI;
        if (LocalVar->iStatus == 0 || LocalVar->restart) {
            genBrTqPI.init(CntrPar.VS_MinOMTq);
            LocalVar->GenBrTq = CntrPar.VS_MinOMTq;
        } else {
            LocalVar->GenBrTq = genBrTqPI.step(LocalVar->VS_SpdErrBr, CntrPar.VS_KP[0], CntrPar.VS_KI[0],
                CntrPar.VS_MinTq, CntrPar.VS_MinOMTq, LocalVar->DT);
        }

        // State machine
        if (LocalVar->VS_State == VS_State_Region_1_5) {
            LocalVar->GenTq = LocalVar->GenBrTq;
        } else if (LocalVar->VS_State == VS_State_Region_2) {
            LocalVar->GenTq = LocalVar->VS_KOmega2_GenTq;
        } else if (LocalVar->VS_State == VS_State_Region_2_5) {
            LocalVar->GenTq = LocalVar->GenArTq;
        } else if (LocalVar->VS_State == VS_State_Region_3_ConstTrq) {
            LocalVar->GenTq = CntrPar.VS_RtTq;
        } else if (LocalVar->VS_State == VS_State_Region_3_ConstPwr) {
            LocalVar->GenTq = LocalVar->VS_ConstPwr_GenTq;
        } else if (LocalVar->VS_State == VS_State_Region_3_FBP) {
            if (CntrPar.VS_FBP == VS_FBP_Power_Overspeed) {
                LocalVar->GenTq = std::min(LocalVar->VS_ConstPwr_GenTq, LocalVar->VS_KOmega2_GenTq);
            } else if ((CntrPar.VS_FBP == VS_FBP_WSE_Ref) || (CntrPar.VS_FBP == VS_FBP_Torque_Ref)) {
                LocalVar->GenTq = LocalVar->GenArTq;
            }
        }

    } else {
        // VS_ControlMode of 0
        LocalVar->GenTq = 0.0;
    }

    // Shutdown
    if (LocalVar->SD_Trigger == 0) {
        LocalVar->GenTq_SD = LocalVar->GenTq;
    } else {
        if (CntrPar.SD_Method == 1 || CntrPar.SD_Method == 2) {
            LocalVar->GenTq_SD = LocalVar->GenTq_SD - LocalVar->SD_MaxTorqueRate * LocalVar->DT;
            LocalVar->GenTq_SD = saturate(LocalVar->GenTq_SD, CntrPar.VS_MinTq, CntrPar.VS_MaxTq);
        }
        LocalVar->GenTq = LocalVar->GenTq_SD;
    }

    // Saturate based on most stringent defined maximum
    LocalVar->GenTq = saturate(LocalVar->GenTq, CntrPar.VS_MinTq,
                                  std::min(CntrPar.VS_MaxTq, LocalVar->VS_MaxTq));

    // Rate limit
    static RateLimiter genTqRL;
    if (LocalVar->iStatus == 0 || LocalVar->restart) genTqRL.init(LocalVar->GenTq);
    LocalVar->GenTq = genTqRL.step(LocalVar->GenTq, -CntrPar.VS_MaxRat, CntrPar.VS_MaxRat, LocalVar->DT);

    // Open loop torque control
    if ((CntrPar.OL_Mode > 0) && (CntrPar.Ind_GenTq > 0)) {
        if (LocalVar->Time >= CntrPar.OL_Breakpoints[0]) {
            LocalVar->GenTq = interp1d(CntrPar.OL_Breakpoints,
                                       CntrPar.OL_GenTq,
                                       LocalVar->OL_Index, ErrVar);
        }

        // Azimuth tracking control (OL_Mode == 2)
        if (CntrPar.OL_Mode == 2) {
            // Initialize azimuth buffer
            if (LocalVar->iStatus == 0) {
                LocalVar->AzBuffer[0] = LocalVar->Azimuth;
                LocalVar->AzBuffer[1] = LocalVar->Azimuth;
            }
            // Push/pop buffer
            LocalVar->AzBuffer[0] = LocalVar->AzBuffer[1];
            LocalVar->AzBuffer[1] = LocalVar->Azimuth;

            // Unwrap azimuth buffer via shared unwrap
            double az_unwrapped[2];
            unwrap(LocalVar->AzBuffer, 2, ErrVar, az_unwrapped);
            LocalVar->AzBuffer[0] = az_unwrapped[0];
            LocalVar->AzBuffer[1] = az_unwrapped[1];

            LocalVar->AzUnwrapped = LocalVar->AzBuffer[1];

            // Desired azimuth from OL file
            LocalVar->OL_Azimuth = interp1d(CntrPar.OL_Breakpoints,
                                            CntrPar.OL_Azimuth,
                                            LocalVar->Time, ErrVar);

            LocalVar->AzError = LocalVar->OL_Azimuth - LocalVar->AzUnwrapped;

            // PID controller for azimuth tracking torque
            static PIDController genTqAzPID;
            if (LocalVar->iStatus == 0 || LocalVar->restart) {
                genTqAzPID.init(0.0, CntrPar.RP_Gains[3], LocalVar->DT, LocalVar->AzError);
                LocalVar->GenTqAz = 0.0;
            } else {
                LocalVar->GenTqAz = genTqAzPID.step(LocalVar->AzError, CntrPar.RP_Gains[0], CntrPar.RP_Gains[1],
                    CntrPar.RP_Gains[2], -LocalVar->VS_MaxTq * 2.0, LocalVar->VS_MaxTq * 2.0, LocalVar->DT);
            }

            LocalVar->GenTq = LocalVar->GenTq + LocalVar->GenTqAz;
        }
    }

    // Update last values
    LocalVar->VS_LastGenTrq = LocalVar->GenTq;
    LocalVar->VS_LastGenPwr = LocalVar->VS_GenPwr;

    // Set commanded generator torque (Fortran avrSWAP(47) → C [46])
    avrSWAP[46] = std::max(0.0, LocalVar->VS_LastGenTrq);

    // Prepend routine name to error message if aviFAIL < 0
    if (ErrVar->aviFAIL < 0) {
        char tmp[1024];
        snprintf(tmp, sizeof(tmp), "VariableSpeedControl:%s", ErrVar->ErrMsg);
        strncpy(ErrVar->ErrMsg, tmp, sizeof(ErrVar->ErrMsg) - 1);
        ErrVar->ErrMsg[sizeof(ErrVar->ErrMsg) - 1] = '\0';
    }
}
