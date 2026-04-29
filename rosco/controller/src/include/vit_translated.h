// Function declarations for all translated ROSCO controller functions.

#ifndef VIT_TRANSLATED_H
#define VIT_TRANSLATED_H

#include "vit_types.h"
#include "rosco_array.hpp"
#include "rosco_types.hpp"
#include "rosco_objects.hpp"
#include "rosco_functions.h"
#include <stdint.h>

// Filters
void PreFilterMeasuredSignals(const ControlParameters& CntrPar, LocalVariables& LocalVar, debugvariables_t* DebugVar, errorvariables_t* ErrVar);

// Controllers
void TorqueControl(float* avrSWAP, const ControlParameters& CntrPar, LocalVariables& LocalVar, errorvariables_t* ErrVar);
void PitchControl(float* avrSWAP, const ControlParameters& CntrPar, LocalVariables& LocalVar, debugvariables_t* DebugVar, errorvariables_t* ErrVar);
void YawRateControl(float* avrSWAP, const ControlParameters& CntrPar, LocalVariables& LocalVar, debugvariables_t* DebugVar, errorvariables_t* ErrVar);
void FlapControl(float* avrSWAP, const ControlParameters& CntrPar, LocalVariables& LocalVar);
void CableControl(float* avrSWAP, const ControlParameters& CntrPar, LocalVariables& LocalVar, errorvariables_t* ErrVar);
void StructuralControl(float* avrSWAP, const ControlParameters& CntrPar, LocalVariables& LocalVar, errorvariables_t* ErrVar);

// SubControllers
void IPC(const ControlParameters& CntrPar, LocalVariables& LocalVar, debugvariables_t* DebugVar, errorvariables_t* ErrVar);
void ActiveWakeControl(const ControlParameters& CntrPar, LocalVariables& LocalVar, debugvariables_t* DebugVar);
double FloatingFeedback(LocalVariables& LocalVar, const ControlParameters& CntrPar, errorvariables_t* ErrVar);
void ForeAftDamping(const ControlParameters& CntrPar, LocalVariables& LocalVar);
void TorqueStateMachine(const ControlParameters& CntrPar, LocalVariables& LocalVar);

// Estimators
void WindSpeedEstimator(LocalVariables& LocalVar, const ControlParameters& CntrPar, const PerformanceData& PerfData, debugvariables_t* DebugVar, errorvariables_t* ErrVar);

// Setpoints
void PowerControlSetpoints(const ControlParameters& CntrPar, LocalVariables& LocalVar, debugvariables_t* DebugVar, errorvariables_t* ErrVar);
void SpeedSetpoints(const ControlParameters& CntrPar, LocalVariables& LocalVar, debugvariables_t* DebugVar, errorvariables_t* ErrVar);
double PitchSaturation(LocalVariables& LocalVar, const ControlParameters& CntrPar, debugvariables_t* DebugVar, errorvariables_t* ErrVar);
void RefSpeedExclusion(LocalVariables& LocalVar, const ControlParameters& CntrPar, debugvariables_t* DebugVar);
void SetpointSmoother(LocalVariables& LocalVar, const ControlParameters& CntrPar);

// Supervisory
void Shutdown(LocalVariables& LocalVar, const ControlParameters& CntrPar, errorvariables_t* ErrVar);
void Startup(LocalVariables& LocalVar, const ControlParameters& CntrPar, errorvariables_t* ErrVar);

// ReadSetParameters
void ReadAvrSWAP(float* avrSWAP, LocalVariables& LocalVar, const ControlParameters& CntrPar, errorvariables_t* ErrVar);
void ReadControlParameterFileSub(ControlParameters& CntrPar, LocalVariables& LocalVar, const char* filename, const char* priPath, errorvariables_t* ErrVar);
void ReadCpFile(const ControlParameters& CntrPar, PerformanceData& PerfData, errorvariables_t* ErrVar);
void SetParameters(const ControlParameters& CntrPar, LocalVariables& LocalVar, float* avrSWAP, errorvariables_t* ErrVar, int size_avcMSG);
void CheckInputs(LocalVariables& LocalVar, const ControlParameters& CntrPar, float* avrSWAP, errorvariables_t* ErrVar, int32_t size_avcMSG);

// IO
void ExtController(float* avrSWAP, const ControlParameters& CntrPar, LocalVariables& LocalVar, ExtControlType& ExtDLL, errorvariables_t* ErrVar);
void WriteRestartFile(LocalVariables& LocalVar, const ControlParameters& CntrPar, errorvariables_t* ErrVar, char* RootName, int size_avcOUTNAME);
void ReadRestartFile(float* avrSWAP, LocalVariables& LocalVar, const ControlParameters& CntrPar, const PerformanceData& PerfData, char* RootName, int size_avcOUTNAME, errorvariables_t* ErrVar);
void Debug(LocalVariables& LocalVar, const ControlParameters& CntrPar, debugvariables_t* DebugVar, errorvariables_t* ErrVar, float* avrSWAP, char* RootName, int size_avcOUTNAME);
void UpdateZeroMQ(LocalVariables& LocalVar, const ControlParameters& CntrPar, errorvariables_t* ErrVar);

#endif // VIT_TRANSLATED_H
