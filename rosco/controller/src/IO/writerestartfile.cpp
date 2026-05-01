#include "../include/restart_fields.h"
#include "../include/rosco_types.hpp"
#include "../include/rosco_error.hpp"

void WriteRestartFile(LocalVariables& LocalVar, const ControlParameters& /*CntrPar*/) {
    const std::string& root = LocalVar.RootName;
    int timestep = (int)std::round(LocalVar.Time / LocalVar.DT);
    std::string filename = root + std::to_string(timestep) + ".RO.chkp";

    std::ofstream f(filename, std::ios::binary);
    if (!f.is_open()) {
        rosco_warn("WriteRestartFile", "Cannot open checkpoint file %s for writing", filename.c_str());
        return;
    }

    checkpoint_fields(f, LocalVar, [](std::ofstream& s, auto& val) {
        write_field(s, val);
    });

    if (!f.good()) {
        rosco_warn("WriteRestartFile", "Error writing checkpoint file.");
    }
}
