// read_config_files — load DISCON.IN or DISCON.toml + Cp/Ct/Cq tables.
//
// Called on first timestep (iStatus == 0) and on warm restart (iStatus == -9).
// Reads the config filename from LocalVar.ACC_INFILE (std::string).
// Auto-detects format by file extension (.toml → TOML, else legacy DISCON.IN).

#include "../include/vit_types.h"
#include "../include/rosco_types.hpp"
#include "../include/rosco_objects.hpp"
#include "../include/vit_translated.h"
#include <filesystem>
#include <string>

// GetRoot: strip extension from a filename
// e.g. "/path/to/Case01.outb" → "/path/to/Case01"
std::string GetRoot(const std::string& filename) {
    return std::filesystem::path(filename).replace_extension("").string();
}

void read_config_files(ControlParameters& CntrPar, LocalVariables& LocalVar,
                       PerformanceData& PerfData) {
    const std::string& filename = LocalVar.ACC_INFILE;

    // Reset parameters to defaults before re-reading
    CntrPar = ControlParameters{};

    std::filesystem::path fp(filename);
    bool is_toml = (fp.extension() == ".toml" || fp.extension() == ".TOML");

    if (is_toml) {
        CntrPar.load_from_toml(filename.c_str());
    } else {
        // Directory containing the config file — used to resolve relative paths
        std::string priPath = fp.parent_path().string();
        if (!priPath.empty()) priPath += '/';
        else priPath = "./";

        ReadControlParameterFileSub(CntrPar, LocalVar, filename.c_str(), priPath.c_str());
    }

    // Load rotor performance tables (required when WE_Mode > 0)
    PerfData = PerformanceData{};
    if (CntrPar.WE_Mode > 0) {
        ReadCpFile(CntrPar, PerfData);
    }
}
