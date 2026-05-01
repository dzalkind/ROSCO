// Stage 5 — Supervisory
//
// Power-reference setpoints, shutdown logic, and startup ramp.
//
// Order within this stage:
//   1. PowerControlSetpoints — sets baseline PRC_R_Speed/Torque/Pitch
//   2. Shutdown              — evaluates shutdown triggers, sets SD_MaxPitchRate/TorqueRate
//   3. Startup               — overrides PRC_R_Speed/Torque during startup ramp
//
// PowerControlSetpoints must run before Startup because Startup
// overrides PRC_R_Speed that PCS initializes.

#include "../include/rosco_stages.h"
#include "../include/vit_translated.h"

void stage_5_supervisory(float* avrSWAP, ControlParameters& CntrPar, LocalVariables& LocalVar,
                         PerformanceData& PerfData, debugvariables_t* DebugVar, ExtControlType& ExtDLL)
{
    PowerControlSetpoints(CntrPar, LocalVar, DebugVar);

    if (CntrPar.SD_Mode > 0) Shutdown(LocalVar, CntrPar);
    if (CntrPar.SU_Mode > 0) Startup(LocalVar, CntrPar);
}
