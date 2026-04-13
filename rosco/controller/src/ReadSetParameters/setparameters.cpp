#include "../include/vit_types.h"
#include "../include/rosco_types.hpp"
#include "../include/vit_translated.h"
#include "../include/rosco_constants.h"
#include <cstring>
#include <algorithm>

void SetParameters(const ControlParameters& CntrPar, localvariables_t* LocalVar,
                   float* avrSWAP, objectinstances_t* objInst,
                   errorvariables_t* ErrVar, int size_avcMSG) {

    // iStatus==0: Initialize LocalVar fields (CntrPar is populated by wrapper)
    if (LocalVar->iStatus == 0) {
        // PitCom = BlPitch
        for (int K = 0; K < 3; K++) {
            LocalVar->PitCom[K] = LocalVar->BlPitch[K];
        }

        // Wind speed estimator initialization
        LocalVar->WE_Vw = LocalVar->HorWindV;
        LocalVar->WE_VwI = LocalVar->WE_Vw - CntrPar.WE_Gamma * LocalVar->RotSpeed;
        LocalVar->WE_Op = 1;
        LocalVar->WE_Op_Last = 1;

        // Setpoint Smoother initialization
        LocalVar->SS_DelOmegaF = 0;

        // Generator torque initial condition
        if (CntrPar.VS_FBP == VS_FBP_Variable_Pitch) {
            if (LocalVar->GenSpeed > 0.98 * CntrPar.PC_RefSpd) {
                LocalVar->GenTq = CntrPar.VS_RtTq;
            } else {
                LocalVar->GenTq = std::min(CntrPar.VS_RtTq,
                    CntrPar.VS_Rgn2K * LocalVar->GenSpeed * LocalVar->GenSpeed);
            }
        } else {
            LocalVar->GenTq = interp1d(CntrPar.VS_FBP_U, CntrPar.VS_FBP_Tau,
                                       LocalVar->HorWindV, ErrVar);
        }
        LocalVar->VS_LastGenTrq = LocalVar->GenTq;
        LocalVar->VS_MaxTq = CntrPar.VS_MaxTq;
        LocalVar->VS_GenPwr = LocalVar->GenTq * LocalVar->GenSpeed * CntrPar.VS_GenEff / 100.0;

        // Initialize cable/structural control variables
        memset(LocalVar->CC_DesiredL, 0, sizeof(LocalVar->CC_DesiredL));
        memset(LocalVar->CC_ActuatedL, 0, sizeof(LocalVar->CC_ActuatedL));
        memset(LocalVar->CC_ActuatedDL, 0, sizeof(LocalVar->CC_ActuatedDL));
        memset(LocalVar->StC_Input, 0, sizeof(LocalVar->StC_Input));

        LocalVar->ZMQ_YawOffset = 0;
        memset(LocalVar->ZMQ_PitOffset, 0, sizeof(LocalVar->ZMQ_PitOffset));
        LocalVar->ZMQ_ID = CntrPar.ZMQ_ID;

        // Check validity of input parameters
        CheckInputs(LocalVar, CntrPar, avrSWAP, ErrVar, size_avcMSG);
    }

    // Per-timestep: Open Loop index
    if (CntrPar.OL_BP_Mode == 0) {
        LocalVar->OL_Index = LocalVar->Time;
    } else {
        LocalVar->OL_Index = LocalVar->WE_Vw;
        if (CntrPar.OL_BP_FiltFreq > 0) {
            LocalVar->OL_Index = LPFilter(LocalVar->WE_Vw, LocalVar->DT,
                CntrPar.OL_BP_FiltFreq, &LocalVar->FP, LocalVar->iStatus,
                (LocalVar->restart != 0), &objInst->instLPF, 0, 0.0);
        }
    }
}
