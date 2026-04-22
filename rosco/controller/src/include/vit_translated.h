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
void PreFilterMeasuredSignals(const ControlParameters& CntrPar, localvariables_t* LocalVar, debugvariables_t* DebugVar, objectinstances_t* objInst, errorvariables_t* ErrVar);

// Controllers
double PIController(double error, double kp, double ki, double minValue, double maxValue, double DT, double I0, piparams_t* piP, int reset, int* inst);
double PIIController(double error, double error2, double kp, double ki, double ki2, double minValue, double maxValue, double DT, double I0, piparams_t* piP, int reset, int* inst);
double PIDController(double error, double kp, double ki, double kd, double tf, double minValue, double maxValue, double DT, double I0, piparams_t* piP, int reset, objectinstances_t* objInst, localvariables_t* LocalVar);
double ResController(double error, double kp, double ki, double freq, double minValue, double maxValue, double DT, resparams_t* resP, int reset, int* inst);
void ForeAftDamping(const ControlParameters& CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst);
double FloatingFeedback(localvariables_t* LocalVar, const ControlParameters& CntrPar, objectinstances_t* objInst, errorvariables_t* ErrVar);
void StructuralControl(float* avrSWAP, const ControlParameters& CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, errorvariables_t* ErrVar);
void CableControl(float* avrSWAP, const ControlParameters& CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, errorvariables_t* ErrVar);
void FlapControl(float* avrSWAP, const ControlParameters& CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst);
void YawRateControl(float* avrSWAP, const ControlParameters& CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, debugvariables_t* DebugVar, errorvariables_t* ErrVar);
void VariableSpeedControl(float* avrSWAP, const ControlParameters& CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, errorvariables_t* ErrVar);
void IPC(const ControlParameters& CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, debugvariables_t* DebugVar, errorvariables_t* ErrVar);
void ActiveWakeControl(const ControlParameters& CntrPar, localvariables_t* LocalVar, debugvariables_t* DebugVar, objectinstances_t* objInst);
void PitchControl(float* avrSWAP, const ControlParameters& CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, debugvariables_t* DebugVar, errorvariables_t* ErrVar);

// ControllerBlocks
double PitchSaturation(localvariables_t* LocalVar, const ControlParameters& CntrPar, objectinstances_t* objInst, debugvariables_t* DebugVar, errorvariables_t* ErrVar);
void StateMachine(const ControlParameters& CntrPar, localvariables_t* LocalVar);
void SetpointSmoother(localvariables_t* LocalVar, const ControlParameters& CntrPar, objectinstances_t* objInst);
void PowerControlSetpoints(const ControlParameters& CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, debugvariables_t* DebugVar, errorvariables_t* ErrVar);
void RefSpeedExclusion(localvariables_t* LocalVar, const ControlParameters& CntrPar, objectinstances_t* objInst, debugvariables_t* DebugVar);
void ComputeVariablesSetpoints(const ControlParameters& CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, debugvariables_t* DebugVar, errorvariables_t* ErrVar);
void Shutdown(localvariables_t* LocalVar, const ControlParameters& CntrPar, objectinstances_t* objInst, errorvariables_t* ErrVar);
void Startup(localvariables_t* LocalVar, const ControlParameters& CntrPar, objectinstances_t* objInst, errorvariables_t* ErrVar);
void WindSpeedEstimator(localvariables_t* LocalVar, const ControlParameters& CntrPar, objectinstances_t* objInst, const PerformanceData& PerfData, debugvariables_t* DebugVar, errorvariables_t* ErrVar);

// ReadSetParameters
void ReadAvrSWAP(float* avrSWAP, localvariables_t* LocalVar, const ControlParameters& CntrPar, errorvariables_t* ErrVar);
void ReadControlParameterFileSub(ControlParameters& CntrPar, localvariables_t* LocalVar, const char* filename, const char* priPath, errorvariables_t* ErrVar);
void ReadCpFile(const ControlParameters& CntrPar, PerformanceData& PerfData, errorvariables_t* ErrVar);
void SetParameters(const ControlParameters& CntrPar, localvariables_t* LocalVar, float* avrSWAP, objectinstances_t* objInst, errorvariables_t* ErrVar, int size_avcMSG);
void CheckInputs(localvariables_t* LocalVar, const ControlParameters& CntrPar, float* avrSWAP, errorvariables_t* ErrVar, int32_t size_avcMSG);

// IO
void ExtController(float* avrSWAP, const ControlParameters& CntrPar, localvariables_t* LocalVar, ExtControlType& ExtDLL, errorvariables_t* ErrVar);
void WriteRestartFile(localvariables_t* LocalVar, const ControlParameters& CntrPar, errorvariables_t* ErrVar, objectinstances_t* objInst, char* RootName, int size_avcOUTNAME);
void ReadRestartFile(float* avrSWAP, localvariables_t* LocalVar, const ControlParameters& CntrPar, objectinstances_t* objInst, const PerformanceData& PerfData, char* RootName, int size_avcOUTNAME, errorvariables_t* ErrVar);
void Debug(localvariables_t* LocalVar, const ControlParameters& CntrPar, debugvariables_t* DebugVar, errorvariables_t* ErrVar, float* avrSWAP, char* RootName, int size_avcOUTNAME);
void UpdateZeroMQ(localvariables_t* LocalVar, const ControlParameters& CntrPar, errorvariables_t* ErrVar);

#endif // VIT_TRANSLATED_H
