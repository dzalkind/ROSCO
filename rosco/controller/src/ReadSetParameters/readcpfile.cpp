#include "../include/vit_types.h"
#include "../include/rosco_types.hpp"
#include "../include/rosco_objects.hpp"
#include "../include/rosco_error.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <cstring>
#include <cstdint>

// Helper: extract a trimmed std::string from a Fortran space-padded char array
static std::string trimFortranString(const char* s, int maxLen) {
    int len = maxLen;
    while (len > 0 && s[len - 1] == ' ') len--;
    return std::string(s, len);
}

// Helper: skip N lines from the input stream
static void skipLines(std::ifstream& f, int n) {
    std::string line;
    for (int i = 0; i < n; i++) {
        std::getline(f, line);
    }
}

// Helper: read a row of doubles from one line
static bool readRow(std::ifstream& f, double* dest, int n) {
    std::string line;
    if (!std::getline(f, line)) return false;
    std::istringstream iss(line);
    for (int i = 0; i < n; i++) {
        if (!(iss >> dest[i])) return false;
    }
    return true;
}

// Read a 2D matrix in column-major order (Fortran layout).
// Each file line contains one row (n_cols values).
// Fortran: mat(row, col) stored as mat[col * n_rows + row]
static void readMatrix(std::ifstream& f, double* mat, int n_rows, int n_cols,
                       const std::string& filename, const char* tableName) {
    for (int row = 0; row < n_rows; row++) {
        std::string line;
        if (!std::getline(f, line)) {
            rosco_throw("ReadCpFile", "Error reading %s %s table. Please check formatting and size of matrices in that file.",
                         filename.c_str(), tableName);
        }
        std::istringstream iss(line);
        for (int col = 0; col < n_cols; col++) {
            if (!(iss >> mat[col * n_rows + row])) {
                rosco_throw("ReadCpFile", "Error reading %s %s table. Please check formatting and size of matrices in that file.",
                             filename.c_str(), tableName);
            }
        }
    }
}

void ReadCpFile(const ControlParameters& CntrPar, PerformanceData& PerfData) {

    // PerfFileName is already a std::string
    std::string filename = CntrPar.PerfFileName;

    // Open file
    std::ifstream f(filename);
    if (!f.is_open()) {
        rosco_throw("ReadCpFile", "Error opening performance file: %s", filename.c_str());
    }

    int n_pitch = CntrPar.PerfTableSize[0];  // PerfTableSize(1) = number of pitch angles (columns)
    int n_tsr   = CntrPar.PerfTableSize[1];  // PerfTableSize(2) = number of TSR values (rows)

    // ---- Axis Definitions ----
    // Skip 4 header/comment lines
    skipLines(f, 4);

    // Read pitch angle vector (Beta_vec): n_pitch values
    PerfData.Beta_vec.resize(n_pitch, 0.0);
    if (!readRow(f, PerfData.Beta_vec.data(), n_pitch)) {
        rosco_throw("ReadCpFile", "Error reading pitch angle vector from performance file.");
    }

    // Skip 1 comment line ("# TSR vector...")
    skipLines(f, 1);

    // Read TSR vector: n_tsr values
    PerfData.TSR_vec.resize(n_tsr, 0.0);
    if (!readRow(f, PerfData.TSR_vec.data(), n_tsr)) {
        rosco_throw("ReadCpFile", "Error reading TSR vector from performance file.");
    }

    // ---- Read Cp, Ct, Cq Tables ----
    // Skip 5 lines (wind speed line, blank, "# Power coefficient", blank, blank)
    skipLines(f, 5);

    // Read Cp matrix: n_tsr rows x n_pitch cols (column-major)
    PerfData.Cp_mat.resize(n_tsr * n_pitch, 0.0);
    readMatrix(f, PerfData.Cp_mat.data(), n_tsr, n_pitch, filename, "Cp");

    // Skip 4 lines (blank, blank, "# Thrust coefficient", blank)
    skipLines(f, 4);

    // Read Ct matrix
    PerfData.Ct_mat.resize(n_tsr * n_pitch, 0.0);
    readMatrix(f, PerfData.Ct_mat.data(), n_tsr, n_pitch, filename, "Ct");

    // Skip 4 lines (blank, blank, "# Torque coefficient", blank)
    skipLines(f, 4);

    // Read Cq matrix
    PerfData.Cq_mat.resize(n_tsr * n_pitch, 0.0);
    readMatrix(f, PerfData.Cq_mat.data(), n_tsr, n_pitch, filename, "Cq");
}
