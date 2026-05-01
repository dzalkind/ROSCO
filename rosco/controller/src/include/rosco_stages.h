// Stage function declarations for the ROSCO controller pipeline.
//
// The controller executes these stages in numbered order each timestep:
//   1. Sensing      — unpack measurements from avrSWAP (+ future sensor models)
//   2. Setup        — defaults, config loading, SetParameters, external I/O
//   3. Filtering    — low-pass / notch filtering of sensor signals
//   4. Estimation   — wind speed estimation
//   5. Supervisory  — power-reference setpoints, shutdown, startup
//   6. Setpoints    — speed setpoints, torque state machine, setpoint smoother
//   7. Actuators    — torque, pitch, yaw, flap, cable, structural control
//   8. Output       — debug logging, checkpoint writing
//
// All stages share a uniform signature so they can be called generically
// or exported individually for Simulink integration.
//
// Note on stage 1/2 ordering: sensing precedes setup because ReadAvrSWAP
// sets iStatus, which gates first-call config loading in stage_2_setup.

#ifndef ROSCO_STAGES_H
#define ROSCO_STAGES_H

#include "vit_types.h"
#include "rosco_types.hpp"
#include "rosco_objects.hpp"

// Stage 1 — Sensing: unpack avrSWAP into LocalVar. Future sensor models
// (noise, bias, latency) will be added here.
void stage_1_sensing(float* avrSWAP, ControlParameters& CntrPar, LocalVariables& LocalVar,
                     PerformanceData& PerfData, debugvariables_t* DebugVar, ExtControlType& ExtDLL);

// Stage 2 — Setup: default actuator signals, config loading (first call),
// warm restart, SetParameters (init + OL index), external DLL, ZeroMQ.
void stage_2_setup(float* avrSWAP, ControlParameters& CntrPar, LocalVariables& LocalVar,
                   PerformanceData& PerfData, debugvariables_t* DebugVar, ExtControlType& ExtDLL);

// Stage 3 — Filtering: low-pass and notch filtering of measured signals.
void stage_3_filtering(float* avrSWAP, ControlParameters& CntrPar, LocalVariables& LocalVar,
                       PerformanceData& PerfData, debugvariables_t* DebugVar, ExtControlType& ExtDLL);

// Stage 4 — Estimation: wind speed estimator (EKF or LPF).
void stage_4_estimation(float* avrSWAP, ControlParameters& CntrPar, LocalVariables& LocalVar,
                        PerformanceData& PerfData, debugvariables_t* DebugVar, ExtControlType& ExtDLL);

// Stage 5 — Supervisory: power-reference setpoints, shutdown logic, startup ramp.
void stage_5_supervisory(float* avrSWAP, ControlParameters& CntrPar, LocalVariables& LocalVar,
                         PerformanceData& PerfData, debugvariables_t* DebugVar, ExtControlType& ExtDLL);

// Stage 6 — Setpoints: speed setpoints, torque state machine, setpoint smoother.
void stage_6_setpoints(float* avrSWAP, ControlParameters& CntrPar, LocalVariables& LocalVar,
                       PerformanceData& PerfData, debugvariables_t* DebugVar, ExtControlType& ExtDLL);

// Stage 7 — Actuators: torque, pitch, yaw, flap, cable, structural control.
void stage_7_actuators(float* avrSWAP, ControlParameters& CntrPar, LocalVariables& LocalVar,
                       PerformanceData& PerfData, debugvariables_t* DebugVar, ExtControlType& ExtDLL);

// Stage 8 — Output: debug logging, restart-file checkpointing.
void stage_8_output(float* avrSWAP, ControlParameters& CntrPar, LocalVariables& LocalVar,
                    PerformanceData& PerfData, debugvariables_t* DebugVar, ExtControlType& ExtDLL);

#endif // ROSCO_STAGES_H
