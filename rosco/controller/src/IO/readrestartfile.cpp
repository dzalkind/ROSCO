#include "../include/restart_fields.h"
#include "../include/rosco_types.hpp"
#include "../include/rosco_objects.hpp"

void ReadRestartFile(float* avrSWAP, LocalVariables& LocalVar,
                     const ControlParameters& /*CntrPar*/,
                     const PerformanceData& /*PerfData*/, char* RootName,
                     int size_avcOUTNAME, errorvariables_t* ErrVar) {
    std::string root = trim_fortran_string(RootName, size_avcOUTNAME);
    // Fortran: NINT(avrSWAP(2)/avrSWAP(3))  — 1-indexed
    int timestep = (int)std::round((double)avrSWAP[1] / (double)avrSWAP[2]);
    std::string filename = root + std::to_string(timestep) + ".RO.chkp";

    std::ifstream f(filename, std::ios::binary);
    if (!f.is_open()) {
        ErrVar->aviFAIL = 1;
        snprintf(ErrVar->ErrMsg, sizeof(ErrVar->ErrMsg),
                 "ROSCO_IO: Cannot open checkpoint file %s for reading", filename.c_str());
        return;
    }

    checkpoint_fields(f, LocalVar, [](std::ifstream& s, auto& val) {
        read_field(s, val);
    });

    if (!f.good()) {
        ErrVar->aviFAIL = 1;
        snprintf(ErrVar->ErrMsg, sizeof(ErrVar->ErrMsg),
                 "ROSCO_IO: Error reading checkpoint file.");
    }

    // Note: ReadControlParameterFileSub and ReadCpFile calls are handled
    // by the Fortran wrapper, not here.
}
