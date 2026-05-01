// Stage 8 — Output
//
// Write debug log file (.dbg) and checkpoint restart file.
// Debug runs every timestep when LoggingLevel > 0.
// WriteRestartFile runs only on checkpoint calls (iStatus == -8).

#include "../include/rosco_stages.h"
#include "../include/vit_translated.h"

void stage_8_output(float* avrSWAP, ControlParameters& CntrPar, LocalVariables& LocalVar,
                    PerformanceData& PerfData, debugvariables_t* DebugVar, ExtControlType& ExtDLL,
                    char* RootName, int avcOUTNAME_size)
{
    if (LocalVar.iStatus == -8) {
        WriteRestartFile(LocalVar, CntrPar, RootName, avcOUTNAME_size);
    }

    if (CntrPar.LoggingLevel > 0) {
        Debug(LocalVar, CntrPar, DebugVar, avrSWAP, RootName, avcOUTNAME_size);
    }
}
