// Stage 2 — Setup
//
// First-timestep initialization (banner, config file reading, performance
// tables) and per-timestep parameter updates (OL index, external DLL,
// ZeroMQ offsets).
//
// On the first call (iStatus == 0), the caller (DISCON) invokes a
// dedicated first-call init path before this stage runs. This stage
// then handles SetParameters (which does its own first-call init
// internally) and optional external I/O on every timestep.
//
// Note on numbering: this is stage_2 despite being "setup" because
// stage_1_sensing (ReadAvrSWAP) must run first to populate iStatus,
// which gates the first-call config loading in the DISCON orchestrator.
// The numbering reflects execution order, not conceptual priority.

#include "../include/rosco_stages.h"
#include "../include/vit_translated.h"

void stage_2_setup(float* avrSWAP, ControlParameters& CntrPar, LocalVariables& LocalVar,
                   PerformanceData& PerfData, debugvariables_t* DebugVar, ExtControlType& ExtDLL)
{
    // SetParameters: initialize LocalVar on first call; update OL_Index every call
    constexpr int SWAP_MSG_LEN = 48;
    int size_avcMSG = std::max(1, (int)avrSWAP[SWAP_MSG_LEN]);
    SetParameters(CntrPar, LocalVar, avrSWAP, size_avcMSG);

    // External DLL controller (optional)
    if (CntrPar.Ext_Mode > 0) {
        ExtDLL.avrSWAP.resize(2000, 0.0f);
        ExtController(avrSWAP, CntrPar, LocalVar, ExtDLL);
    }

    // ZeroMQ: receive wind-farm-level offsets (optional).
    // Only runs during the main control loop (iStatus >= 0 or checkpoint calls).
    // The final-call ZMQ send (iStatus == -1) is handled by the DISCON orchestrator.
    bool running = (LocalVar.iStatus >= 0) || (LocalVar.iStatus <= -8);
    if (running && CntrPar.ZMQ_Mode > 0) {
        UpdateZeroMQ(LocalVar, CntrPar);
    }
}
