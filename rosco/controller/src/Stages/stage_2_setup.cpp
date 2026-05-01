// Stage 2 — Setup
//
// Per-timestep initialization and config management. Handles:
//   - Default actuator signals (safe defaults before controller runs)
//   - Warm restart (iStatus == -9): restore checkpoint, re-read config
//   - First-call init (iStatus == 0): print banner, read config files
//   - SetParameters (every call): init LocalVar on first call, update OL index
//   - External DLL controller (optional)
//   - ZeroMQ wind-farm offsets (optional)
//
// Note on numbering: this is stage_2 despite being "setup" because
// stage_1_sensing (ReadAvrSWAP) must run first to populate iStatus.

#include "../include/rosco_stages.h"
#include "../include/vit_translated.h"
#include <cstdio>

static const char* ROSCO_VERSION = "2.10.1";

void stage_2_setup(float* avrSWAP, ControlParameters& CntrPar, LocalVariables& LocalVar,
                   PerformanceData& PerfData, debugvariables_t* DebugVar, ExtControlType& ExtDLL)
{
    // Default demanded actuator signals (overwritten by stage_7_actuators)
    avrSWAP[34] = 1.0f;   // Record 35: request generator torque (1 = active)
    avrSWAP[35] = 0.0f;   // Record 36: shaft brake state (0 = off)
    avrSWAP[40] = 0.0f;   // Record 41: demanded nacelle yaw (rad)
    avrSWAP[45] = 0.0f;   // Record 46: demanded pitch — blade 1 (rad)
    avrSWAP[54] = 0.0f;   // Record 55: demanded pitch — blade 2 (rad)
    avrSWAP[55] = 0.0f;   // Record 56: demanded pitch — blade 3 (rad)
    avrSWAP[64] = 0.0f;   // Record 65: variable-slip flag
    avrSWAP[71] = 0.0f;   // Record 72: cable control output
    avrSWAP[78] = 4.0f;   // Record 79: generator torque output (Bladed expects 4)
    avrSWAP[79] = 0.0f;   // Record 80: demanded pitch rate (rad/s)
    avrSWAP[80] = 0.0f;   // Record 81: shaft brake override

    // --------------------------------------------------------
    // Warm restart: restore state from checkpoint file,
    // then re-read config (parameters may have changed).
    // Re-run ReadAvrSWAP after restore so current sensor
    // readings overwrite the checkpointed values.
    // --------------------------------------------------------
    if (LocalVar.iStatus == -9) {
        ReadRestartFile(avrSWAP, LocalVar, CntrPar, PerfData);
        read_config_files(CntrPar, LocalVar, PerfData);
        ReadAvrSWAP(avrSWAP, LocalVar, CntrPar);
        if (CntrPar.LoggingLevel > 0) {
            Debug(LocalVar, CntrPar, DebugVar, avrSWAP);
        }
    }

    // --------------------------------------------------------
    // First-call: print banner, read config files
    // --------------------------------------------------------
    if (LocalVar.iStatus == 0) {
        printf("                                                                              \n"
               "------------------------------------------------------------------------------\n"
               "Running ROSCO-%s (c++ version)\n"
               "A wind turbine controller framework for public use in the scientific field    \n"
               "Developed in collaboration: National Renewable Energy Laboratory              \n"
               "                            Delft University of Technology, The Netherlands   \n"
               "------------------------------------------------------------------------------\n",
               ROSCO_VERSION);

        read_config_files(CntrPar, LocalVar, PerfData);
    }

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
