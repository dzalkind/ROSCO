// Stage 7 — Actuators
//
// Compute demanded actuator signals: generator torque, blade pitch
// (collective + individual), nacelle yaw rate, trailing-edge flap,
// cable tension, and structural control.  Results are written to
// the avrSWAP array for the simulator.

#include "../include/rosco_stages.h"
#include "../include/vit_translated.h"

void stage_7_actuators(float* avrSWAP, ControlParameters& CntrPar, LocalVariables& LocalVar,
                       PerformanceData& PerfData, debugvariables_t* DebugVar, ExtControlType& ExtDLL)
{
    TorqueControl(avrSWAP, CntrPar, LocalVar);

    if (CntrPar.PC_ControlMode > 0) PitchControl(avrSWAP, CntrPar, LocalVar, DebugVar);
    if (CntrPar.Y_ControlMode  > 0) YawRateControl(avrSWAP, CntrPar, LocalVar, DebugVar);
    if (CntrPar.Flp_Mode       > 0) FlapControl(avrSWAP, CntrPar, LocalVar);
    if (CntrPar.CC_Mode        > 0) CableControl(avrSWAP, CntrPar, LocalVar);
    if (CntrPar.StC_Mode       > 0) StructuralControl(avrSWAP, CntrPar, LocalVar);
}
