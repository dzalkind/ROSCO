#include "../include/vit_types.h"
#include "../include/vit_translated.h"
#include <algorithm>
#include <cstring>
#include <cstdio>

#include "../include/rosco_constants.h"

void PitchControl(float* avrSWAP, const ControlParameters& CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, debugvariables_t* DebugVar, errorvariables_t* ErrVar) {
    // PitchControl: master blade pitch controller
    // Orchestrates collective pitch PI, IPC, tower damping, floating feedback,
    // pitch saturation, AWC, shutdown, actuator model, and fault handling.

    // Load PC State
    if (LocalVar->PC_State == PC_State_Enabled) {
        LocalVar->PC_MaxPit = CntrPar.PC_MaxPit;
    } else {
        LocalVar->PC_MaxPit = CntrPar.PC_FinePit;
    }

    // Hold blade pitch at last value in pre-startup mode
    if ((CntrPar.SU_Mode > 0) && (LocalVar->SU_Stage == -1)) {
        LocalVar->PC_MaxPit = LocalVar->BlPitchCMeas;
        LocalVar->PC_MinPit = LocalVar->BlPitchCMeas;
    }

    // Gain scheduling via interpolation
    ArrayView gs_angles = CntrPar.PC_GS_angles;
    LocalVar->PC_KP = interp1d(gs_angles, CntrPar.PC_GS_KP, LocalVar->BlPitchCMeasF, ErrVar);
    LocalVar->PC_KI = interp1d(gs_angles, CntrPar.PC_GS_KI, LocalVar->BlPitchCMeasF, ErrVar);
    LocalVar->PC_KD = interp1d(gs_angles, CntrPar.PC_GS_KD, LocalVar->BlPitchCMeasF, ErrVar);
    LocalVar->PC_TF = interp1d(gs_angles, CntrPar.PC_GS_TF, LocalVar->BlPitchCMeasF, ErrVar);

    // Collective pitch PI controller
    LocalVar->PC_PitComT = PIController(
        LocalVar->PC_SpdErr, LocalVar->PC_KP, LocalVar->PC_KI,
        LocalVar->PC_MinPit, LocalVar->PC_MaxPit,
        LocalVar->DT, LocalVar->BlPitch[0],
        &LocalVar->piP, (LocalVar->restart != 0), &objInst->instPI);
    DebugVar->PC_PICommand = LocalVar->PC_PitComT;

    // Individual pitch control
    if ((CntrPar.IPC_ControlMode >= 1) || (CntrPar.Y_ControlMode == 2)) {
        IPC(CntrPar, LocalVar, objInst, DebugVar, ErrVar);
    } else {
        LocalVar->IPC_PitComF[0] = 0.0;
        LocalVar->IPC_PitComF[1] = 0.0;
        LocalVar->IPC_PitComF[2] = 0.0;
    }

    // Tower fore-aft damping
    if (CntrPar.TD_Mode > 0) {
        ForeAftDamping(CntrPar, LocalVar, objInst);
    } else {
        LocalVar->FA_PitCom[0] = 0.0;
        LocalVar->FA_PitCom[1] = 0.0;
        LocalVar->FA_PitCom[2] = 0.0;
    }

    // Pitch saturation
    if (CntrPar.PS_Mode > 0) {
        LocalVar->PC_MinPit = PitchSaturation(LocalVar, CntrPar, objInst, DebugVar, ErrVar);
        LocalVar->PC_MinPit = std::max(LocalVar->PC_MinPit, CntrPar.PC_FinePit);
    } else {
        LocalVar->PC_MinPit = CntrPar.PC_FinePit;
    }
    DebugVar->PC_MinPit = LocalVar->PC_MinPit;

    // Floating feedback
    if (CntrPar.Fl_Mode > 0) {
        LocalVar->Fl_PitCom = FloatingFeedback(LocalVar, CntrPar, objInst, ErrVar);
        DebugVar->Fl_PitCom = LocalVar->Fl_PitCom;
        LocalVar->PC_PitComT += LocalVar->Fl_PitCom;
    }

    // Saturate collective pitch
    LocalVar->PC_PitComT = saturate(LocalVar->PC_PitComT, LocalVar->PC_MinPit, CntrPar.PC_MaxPit);
    LocalVar->PC_PitComT = ratelimit(LocalVar->PC_PitComT, CntrPar.PC_MinRat, CntrPar.PC_MaxRat,
                                        LocalVar->DT, (LocalVar->restart != 0),
                                        &LocalVar->rlP, &objInst->instRL,
                                        1, LocalVar->BlPitchCMeas);
    LocalVar->PC_PitComT_Last = LocalVar->PC_PitComT;

    // Combine and saturate individual pitch commands
    for (int K = 0; K < LocalVar->NumBl; K++) {
        LocalVar->PitCom[K] = LocalVar->PC_PitComT + LocalVar->FA_PitCom[K];
        LocalVar->PitCom[K] = saturate(LocalVar->PitCom[K], LocalVar->PC_MinPit, CntrPar.PC_MaxPit);
        LocalVar->PitCom[K] += LocalVar->IPC_PitComF[K];

        // Hard IPC saturation by peak shaving limit
        if (CntrPar.IPC_SatMode == 1) {
            LocalVar->PitCom[K] = saturate(LocalVar->PitCom[K], LocalVar->PC_MinPit, CntrPar.PC_MaxPit);
        }

        // ZeroMQ pitch offset
        LocalVar->PitCom[K] += LocalVar->ZMQ_PitOffset[K];

        // Rate limit per blade
        LocalVar->PitCom[K] = ratelimit(LocalVar->PitCom[K], CntrPar.PC_MinRat, CntrPar.PC_MaxRat,
                                           LocalVar->DT, (LocalVar->restart != 0),
                                           &LocalVar->rlP, &objInst->instRL,
                                           1, LocalVar->BlPitch[K]);
    }

    // Open loop pitch control
    if (CntrPar.OL_Mode > 0) {
        if (LocalVar->Time >= CntrPar.OL_Breakpoints[0]) {
            ArrayView ol_bp = CntrPar.OL_Breakpoints;
            if (CntrPar.Ind_BldPitch[0] > 0) {
                LocalVar->PitCom[0] = interp1d(ol_bp, CntrPar.OL_BldPitch1, LocalVar->OL_Index, ErrVar);
            }
            if (CntrPar.Ind_BldPitch[1] > 0) {
                LocalVar->PitCom[1] = interp1d(ol_bp, CntrPar.OL_BldPitch2, LocalVar->OL_Index, ErrVar);
            }
            if (CntrPar.Ind_BldPitch[2] > 0) {
                LocalVar->PitCom[2] = interp1d(ol_bp, CntrPar.OL_BldPitch3, LocalVar->OL_Index, ErrVar);
            }
        }
    }

    // Active wake control
    if (CntrPar.AWC_Mode > 0) {
        ActiveWakeControl(CntrPar, LocalVar, DebugVar, objInst);
    }

    // Shutdown
    if (LocalVar->SD_Trigger == 0) {
        LocalVar->PitCom_SD[0] = LocalVar->PitCom[0];
        LocalVar->PitCom_SD[1] = LocalVar->PitCom[1];
        LocalVar->PitCom_SD[2] = LocalVar->PitCom[2];
    } else {
        if (CntrPar.SD_Method == 1 || CntrPar.SD_Method == 2) {
            for (int K = 0; K < LocalVar->NumBl; K++) {
                LocalVar->PitCom_SD[K] += LocalVar->SD_MaxPitchRate * LocalVar->DT;
            }
        }
        LocalVar->PitCom[0] = LocalVar->PitCom_SD[0];
        LocalVar->PitCom[1] = LocalVar->PitCom_SD[1];
        LocalVar->PitCom[2] = LocalVar->PitCom_SD[2];
    }

    // Pitch actuator model
    static LPFilter pitchActFilter[3];
    for (int K = 0; K < LocalVar->NumBl; K++) {
        if (CntrPar.PA_Mode > 0) {
            if (CntrPar.PA_Mode == 1) {
                if (LocalVar->iStatus == 0 || LocalVar->restart) pitchActFilter[K].init(CntrPar.PA_CornerFreq, LocalVar->DT, LocalVar->PitCom[K]);
                LocalVar->PitComAct[K] = pitchActFilter[K].step(LocalVar->PitCom[K]);
            } else if (CntrPar.PA_Mode == 2) {
                LocalVar->PitComAct[K] = SecLPFilter(
                    LocalVar->PitCom[K], LocalVar->DT,
                    CntrPar.PA_CornerFreq, CntrPar.PA_Damping,
                    &LocalVar->FP, LocalVar->iStatus,
                    (LocalVar->restart != 0), &objInst->instSecLPF,
                    0, 0.0);
            }
        } else {
            LocalVar->PitComAct[K] = LocalVar->PitCom[K];
        }
    }

    // Hardware saturation
    for (int K = 0; K < LocalVar->NumBl; K++) {
        LocalVar->PitComAct[K] = saturate(LocalVar->PitComAct[K], CntrPar.PC_MinPit, CntrPar.PC_MaxPit);
        LocalVar->PitComAct[K] = ratelimit(LocalVar->PitComAct[K], CntrPar.PC_MinRat, CntrPar.PC_MaxRat,
                                              LocalVar->DT, (LocalVar->restart != 0),
                                              &LocalVar->rlP, &objInst->instRL,
                                              1, LocalVar->BlPitch[K]);
    }

    // Pitch fault modes
    if (CntrPar.PF_Mode == 1) {
        for (int K = 0; K < LocalVar->NumBl; K++) {
            LocalVar->PitComAct[K] += CntrPar.PF_Offsets[K];
        }
    } else if (CntrPar.PF_Mode == 2) {
        for (int K = 0; K < LocalVar->NumBl; K++) {
            if (LocalVar->Time > CntrPar.PF_TimeStuck[K]) {
                LocalVar->PitComAct[K] = LocalVar->BlPitch[K];
            }
        }
    }

    // Command pitch to avrSWAP (Fortran 42-45 → C 41-44)
    avrSWAP[41] = LocalVar->PitComAct[0];
    avrSWAP[42] = LocalVar->PitComAct[1];
    avrSWAP[43] = LocalVar->PitComAct[2];
    avrSWAP[44] = LocalVar->PitComAct[0];  // Collective pitch = blade 1

    // Prepend routine name to error message if aviFAIL < 0
    if (ErrVar->aviFAIL < 0) {
        char tmp[1024];
        snprintf(tmp, sizeof(tmp), "PitchControl:%s", ErrVar->ErrMsg);
        strncpy(ErrVar->ErrMsg, tmp, sizeof(ErrVar->ErrMsg) - 1);
        ErrVar->ErrMsg[sizeof(ErrVar->ErrMsg) - 1] = '\0';
    }
}
