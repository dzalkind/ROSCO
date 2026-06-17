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
void PreFilterMeasuredSignals(const ControlParameters& CntrPar, LocalVariables& LocalVar);

// Controllers
void TorqueControl(float* avrSWAP, const ControlParameters& CntrPar, LocalVariables& LocalVar);
void PitchControl(float* avrSWAP, const ControlParameters& CntrPar, LocalVariables& LocalVar);
void YawRateControl(float* avrSWAP, const ControlParameters& CntrPar, LocalVariables& LocalVar);
void FlapControl(float* avrSWAP, const ControlParameters& CntrPar, LocalVariables& LocalVar);
void CableControl(float* avrSWAP, const ControlParameters& CntrPar, LocalVariables& LocalVar);
void StructuralControl(float* avrSWAP, const ControlParameters& CntrPar, LocalVariables& LocalVar);

// SubControllers
void IPC(const ControlParameters& CntrPar, LocalVariables& LocalVar);
void ActiveWakeControl(const ControlParameters& CntrPar, LocalVariables& LocalVar);
double FloatingFeedback(LocalVariables& LocalVar, const ControlParameters& CntrPar);
void ForeAftDamping(const ControlParameters& CntrPar, LocalVariables& LocalVar);
void TorqueStateMachine(const ControlParameters& CntrPar, LocalVariables& LocalVar);

// Estimators
void WindSpeedEstimator(LocalVariables& LocalVar, const ControlParameters& CntrPar, const PerformanceData& PerfData);

// Setpoints
void PowerControlSetpoints(const ControlParameters& CntrPar, LocalVariables& LocalVar);
void SpeedSetpoints(const ControlParameters& CntrPar, LocalVariables& LocalVar);
double PitchSaturation(LocalVariables& LocalVar, const ControlParameters& CntrPar);
void RefSpeedExclusion(LocalVariables& LocalVar, const ControlParameters& CntrPar);
void SetpointSmoother(LocalVariables& LocalVar, const ControlParameters& CntrPar);

// Supervisory
void Shutdown(LocalVariables& LocalVar, const ControlParameters& CntrPar);
void Startup(LocalVariables& LocalVar, const ControlParameters& CntrPar);

// ReadSetParameters
void ReadAvrSWAP(float* avrSWAP, LocalVariables& LocalVar, const ControlParameters& CntrPar);
void ReadControlParameterFileSub(ControlParameters& CntrPar, LocalVariables& LocalVar, const char* filename, const char* priPath);
void ReadCpFile(const ControlParameters& CntrPar, PerformanceData& PerfData);
void SetParameters(const ControlParameters& CntrPar, LocalVariables& LocalVar, float* avrSWAP, int size_avcMSG);
void CheckInputs(LocalVariables& LocalVar, const ControlParameters& CntrPar, float* avrSWAP, int32_t size_avcMSG);
void read_config_files(ControlParameters& CntrPar, LocalVariables& LocalVar, PerformanceData& PerfData);
std::string GetRoot(const std::string& filename);

// IO
void ExtController(float* avrSWAP, const ControlParameters& CntrPar, LocalVariables& LocalVar, ExtControlType& ExtDLL);
void WriteRestartFile(LocalVariables& LocalVar, const ControlParameters& CntrPar);
void ReadRestartFile(float* avrSWAP, LocalVariables& LocalVar, const ControlParameters& CntrPar, const PerformanceData& PerfData);
void Debug(LocalVariables& LocalVar, const ControlParameters& CntrPar, float* avrSWAP);
void UpdateZeroMQ(LocalVariables& LocalVar, const ControlParameters& CntrPar);

#endif // VIT_TRANSLATED_H
