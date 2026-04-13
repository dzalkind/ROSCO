// Function declarations for all translated ROSCO controller functions.

#ifndef VIT_TRANSLATED_H
#define VIT_TRANSLATED_H

#include "vit_types.h"
#include "rosco_array.hpp"
#include <stdint.h>

// Functions
double saturate(double inputValue, double minValue, double maxValue);
double wrap_180(double x);
double wrap_360(double x);
double ratelimit(double inputSignal, double minRate, double maxRate, double DT, int reset, rlparams_t* rlP, int* inst, int has_ResetValue, double ResetValue);
void ColemanTransform(double* rootMOOP, double aziAngle, int nHarmonic, double* axTOut, double* axYOut);
void ColemanTransformInverse(double axTIn, double axYIn, double aziAngle, int nHarmonic, double aziOffset, double* PitComIPC);
void identity(int n, double* identity_result);
double sigma(double x, double x0, double x1, double y0, double y1, errorvariables_t* ErrVar);
double interp1d(ArrayView xData, ArrayView yData, double xq, errorvariables_t* ErrVar);
double interp2d(double* xData, int n_xData, double* yData, int n_yData, double* zData, int n_zData_rows, int n_zData_cols, double xq, double yq, errorvariables_t* ErrVar);
double AeroDynTorque(double RotSpeed, double BldPitch, localvariables_t* LocalVar, controlparameters_view_t* CntrPar, performancedata_view_t* PerfData, errorvariables_t* ErrVar);
void unwrap(double* x, int n_x, errorvariables_t* ErrVar, double* unwrap_result);

// Filters
double LPFilter(double InputSignal, double DT, double CornerFreq, filterparameters_t* FP, int iStatus, int reset, int* inst, int has_InitialValue, double InitialValue);
double HPFilter(double InputSignal, double DT, double CornerFreq, filterparameters_t* FP, int iStatus, int reset, int* inst, int has_InitialValue, double InitialValue);
double SecLPFilter(double InputSignal, double DT, double CornerFreq, double Damp, filterparameters_t* FP, int iStatus, int reset, int* inst, int has_InitialValue, double InitialValue);
double SecLPFilter_Vel(double InputSignal, double DT, double CornerFreq, double Damp, filterparameters_t* FP, int iStatus, int reset, int* inst, int has_InitialValue, double InitialValue);
double NotchFilter(double InputSignal, double DT, double omega, double betaNum, double betaDen, filterparameters_t* FP, int iStatus, int reset, int* inst, int has_InitialValue, double InitialValue);
double NotchFilterSlopes(double InputSignal, double DT, double CornerFreq, double Damp, filterparameters_t* FP, int iStatus, int reset, int* inst, int has_Moving, int Moving, int has_InitialValue, double InitialValue);
void PreFilterMeasuredSignals(controlparameters_view_t* CntrPar, localvariables_t* LocalVar, debugvariables_t* DebugVar, objectinstances_t* objInst, errorvariables_t* ErrVar);

// Controllers
double PIController(double error, double kp, double ki, double minValue, double maxValue, double DT, double I0, piparams_t* piP, int reset, int* inst);
double PIIController(double error, double error2, double kp, double ki, double ki2, double minValue, double maxValue, double DT, double I0, piparams_t* piP, int reset, int* inst);
double PIDController(double error, double kp, double ki, double kd, double tf, double minValue, double maxValue, double DT, double I0, piparams_t* piP, int reset, objectinstances_t* objInst, localvariables_t* LocalVar);
double ResController(double error, double kp, double ki, double freq, double minValue, double maxValue, double DT, resparams_t* resP, int reset, int* inst);
void ForeAftDamping(controlparameters_view_t* CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst);
double FloatingFeedback(localvariables_t* LocalVar, controlparameters_view_t* CntrPar, objectinstances_t* objInst, errorvariables_t* ErrVar);
void StructuralControl(float* avrSWAP, controlparameters_view_t* CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, errorvariables_t* ErrVar);
void CableControl(float* avrSWAP, controlparameters_view_t* CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, errorvariables_t* ErrVar);
void FlapControl(float* avrSWAP, controlparameters_view_t* CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst);
void YawRateControl(float* avrSWAP, controlparameters_view_t* CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, debugvariables_t* DebugVar, errorvariables_t* ErrVar);
void VariableSpeedControl(float* avrSWAP, controlparameters_view_t* CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, errorvariables_t* ErrVar);
void IPC(controlparameters_view_t* CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, debugvariables_t* DebugVar, errorvariables_t* ErrVar);
void ActiveWakeControl(controlparameters_view_t* CntrPar, localvariables_t* LocalVar, debugvariables_t* DebugVar, objectinstances_t* objInst);
void PitchControl(float* avrSWAP, controlparameters_view_t* CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, debugvariables_t* DebugVar, errorvariables_t* ErrVar);

// ControllerBlocks
double PitchSaturation(localvariables_t* LocalVar, controlparameters_view_t* CntrPar, objectinstances_t* objInst, debugvariables_t* DebugVar, errorvariables_t* ErrVar);
void StateMachine(controlparameters_view_t* CntrPar, localvariables_t* LocalVar);
void SetpointSmoother(localvariables_t* LocalVar, controlparameters_view_t* CntrPar, objectinstances_t* objInst);
void PowerControlSetpoints(controlparameters_view_t* CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, debugvariables_t* DebugVar, errorvariables_t* ErrVar);
void RefSpeedExclusion(localvariables_t* LocalVar, controlparameters_view_t* CntrPar, objectinstances_t* objInst, debugvariables_t* DebugVar);
void ComputeVariablesSetpoints(controlparameters_view_t* CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, debugvariables_t* DebugVar, errorvariables_t* ErrVar);
void Shutdown(localvariables_t* LocalVar, controlparameters_view_t* CntrPar, objectinstances_t* objInst, errorvariables_t* ErrVar);
void Startup(localvariables_t* LocalVar, controlparameters_view_t* CntrPar, objectinstances_t* objInst, errorvariables_t* ErrVar);
void WindSpeedEstimator(localvariables_t* LocalVar, controlparameters_view_t* CntrPar, objectinstances_t* objInst, performancedata_view_t* PerfData, debugvariables_t* DebugVar, errorvariables_t* ErrVar);

// ReadSetParameters
void ReadAvrSWAP(float* avrSWAP, localvariables_t* LocalVar, controlparameters_view_t* CntrPar, errorvariables_t* ErrVar);
void ReadControlParameterFileSub_pass1(controlparameters_view_t* CntrPar, localvariables_t* LocalVar, const char* filename, const char* priPath, errorvariables_t* ErrVar, int32_t* n_OL_rows, int32_t* OL_Count);
void ReadControlParameterFileSub_pass2(controlparameters_view_t* CntrPar, localvariables_t* LocalVar, const char* filename, const char* priPath, errorvariables_t* ErrVar);
void ReadCpFile(controlparameters_view_t* CntrPar, performancedata_view_t* PerfData, errorvariables_t* ErrVar);
void SetParameters(controlparameters_view_t* CntrPar, localvariables_t* LocalVar, float* avrSWAP, objectinstances_t* objInst, errorvariables_t* ErrVar, int size_avcMSG);
void CheckInputs(localvariables_t* LocalVar, controlparameters_view_t* CntrPar, float* avrSWAP, errorvariables_t* ErrVar, int32_t size_avcMSG);

// IO
void ExtController(float* avrSWAP, controlparameters_view_t* CntrPar, localvariables_t* LocalVar, extcontroltype_view_t* ExtDLL, errorvariables_t* ErrVar);
void WriteRestartFile(localvariables_t* LocalVar, controlparameters_view_t* CntrPar, errorvariables_t* ErrVar, objectinstances_t* objInst, char* RootName, int size_avcOUTNAME);
void ReadRestartFile(float* avrSWAP, localvariables_t* LocalVar, controlparameters_view_t* CntrPar, objectinstances_t* objInst, performancedata_view_t* PerfData, char* RootName, int size_avcOUTNAME, errorvariables_t* ErrVar);
void Debug(localvariables_t* LocalVar, controlparameters_view_t* CntrPar, debugvariables_t* DebugVar, errorvariables_t* ErrVar, float* avrSWAP, char* RootName, int size_avcOUTNAME);
void UpdateZeroMQ(localvariables_t* LocalVar, controlparameters_view_t* CntrPar, errorvariables_t* ErrVar);

#endif // VIT_TRANSLATED_H
