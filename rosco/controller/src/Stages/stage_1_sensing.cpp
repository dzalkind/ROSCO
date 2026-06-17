// Stage 1 — Sensing
//
// Unpack raw turbine measurements from the avrSWAP array into LocalVar.
// This is the first stage because downstream stages (including stage_2_setup
// on the first timestep) depend on LocalVar.iStatus which is set here.
//
// Future work: sensor models (noise, bias, latency) can be added here
// between ReadAvrSWAP and the return, so that all downstream stages see
// "sensed" rather than "true" values.
//
// Note on numbering: stage_1 runs before stage_2 (setup/config), which
// is unconventional — setup logically comes first conceptually, but the
// data flow requires measurements to be unpacked before the first-call
// gate can check iStatus. The numbering reflects execution order, not
// conceptual priority.

#include "../include/rosco_stages.h"
#include "../include/vit_translated.h"

void stage_1_sensing(float* avrSWAP, ControlParameters& CntrPar, LocalVariables& LocalVar,
                     PerformanceData& PerfData, ExtControlType& ExtDLL)
{
    // Unpack turbine measurements from avrSWAP → LocalVar
    ReadAvrSWAP(avrSWAP, LocalVar, CntrPar);

    // Future: sensor models go here (noise injection, bias, latency, etc.)
}
