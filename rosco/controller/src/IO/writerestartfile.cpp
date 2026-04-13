#include "../include/restart_fields.h"
#include "../include/rosco_types.hpp"

void WriteRestartFile(localvariables_t* LocalVar, const ControlParameters& /*CntrPar*/,
                      errorvariables_t* ErrVar, objectinstances_t* objInst,
                      char* RootName, int size_avcOUTNAME) {
    std::string root = trim_fortran_string(RootName, size_avcOUTNAME);
    int timestep = (int)std::round(LocalVar->Time / LocalVar->DT);
    std::string filename = root + std::to_string(timestep) + ".RO.chkp";

    std::ofstream f(filename, std::ios::binary);
    if (!f.is_open()) {
        ErrVar->aviFAIL = 1;
        snprintf(ErrVar->ErrMsg, sizeof(ErrVar->ErrMsg),
                 "ROSCO_IO: Cannot open checkpoint file %s for writing", filename.c_str());
        return;
    }

    checkpoint_fields(f, LocalVar, objInst, [](std::ofstream& s, auto& val) {
        write_field(s, val);
    });

    if (!f.good()) {
        ErrVar->aviFAIL = 1;
        snprintf(ErrVar->ErrMsg, sizeof(ErrVar->ErrMsg),
                 "ROSCO_IO: Error writing checkpoint file.");
    }
}
