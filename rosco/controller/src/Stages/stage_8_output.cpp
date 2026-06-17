// Stage 8 — Output
//
// Write debug log file (.dbg) and checkpoint restart file.
// Debug runs every timestep when LoggingLevel > 0.
// WriteRestartFile runs only on checkpoint calls (iStatus == -8).

#include "../include/rosco_stages.h"
#include "../include/vit_translated.h"

void stage_8_output(float* avrSWAP, ControlParameters& CntrPar, LocalVariables& LocalVar,
                    PerformanceData& PerfData, ExtControlType& ExtDLL)
{
    if (LocalVar.iStatus == -8) {
        WriteRestartFile(LocalVar, CntrPar);
    }

    if (CntrPar.LoggingLevel > 0) {
        Debug(LocalVar, CntrPar, avrSWAP);
    }
}
