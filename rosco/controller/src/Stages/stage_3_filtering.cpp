// Stage 3 — Filtering
//
// Apply low-pass and notch filters to raw sensor measurements.
// Produces filtered signals (GenSpeedF, RotSpeedF, NacVaneF, etc.)
// consumed by downstream stages.

#include "../include/rosco_stages.h"
#include "../include/vit_translated.h"

void stage_3_filtering(float* avrSWAP, ControlParameters& CntrPar, LocalVariables& LocalVar,
                       PerformanceData& PerfData, ExtControlType& ExtDLL)
{
    PreFilterMeasuredSignals(CntrPar, LocalVar);
}
