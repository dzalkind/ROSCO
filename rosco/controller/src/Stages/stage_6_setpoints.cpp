// Stage 6 — Setpoints
//
// Compute speed setpoints, classify operating region via the torque
// state machine, and blend setpoints between variable-speed and
// pitch-control regions.

#include "../include/rosco_stages.h"
#include "../include/vit_translated.h"

void stage_6_setpoints(float* avrSWAP, ControlParameters& CntrPar, LocalVariables& LocalVar,
                       PerformanceData& PerfData, debugvariables_t* DebugVar, ExtControlType& ExtDLL)
{
    SpeedSetpoints(CntrPar, LocalVar, DebugVar);
    TorqueStateMachine(CntrPar, LocalVar);
    SetpointSmoother(LocalVar, CntrPar);
}
