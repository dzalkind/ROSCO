// Stage 4 — Estimation
//
// Estimate effective hub-height wind speed using an Extended Kalman
// Filter (EKF) or low-pass filter, depending on WE_Mode.

#include "../include/rosco_stages.h"
#include "../include/vit_translated.h"

void stage_4_estimation(float* avrSWAP, ControlParameters& CntrPar, LocalVariables& LocalVar,
                        PerformanceData& PerfData, ExtControlType& ExtDLL)
{
    WindSpeedEstimator(LocalVar, CntrPar, PerfData);
}
