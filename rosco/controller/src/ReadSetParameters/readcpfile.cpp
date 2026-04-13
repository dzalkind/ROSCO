#include "../include/vit_types.h"
#include "../include/rosco_types.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <cstring>
#include <cstdint>

// Helper: set error with space-padded Fortran CHARACTER field
static void setError(errorvariables_t* ErrVar, const char* msg) {
    ErrVar->aviFAIL = -1;
    std::memset(ErrVar->ErrMsg, ' ', 1024);
    size_t len = std::strlen(msg);
    if (len > 1024) len = 1024;
    std::memcpy(ErrVar->ErrMsg, msg, len);
}

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
static bool readMatrix(std::ifstream& f, double* mat, int n_rows, int n_cols,
                       const std::string& filename, const char* tableName,
                       errorvariables_t* ErrVar) {
    for (int row = 0; row < n_rows; row++) {
        std::string line;
        if (!std::getline(f, line)) {
            std::string msg = "Error reading " + filename + " " + tableName +
                              " table. Please check formatting and size of matrices in that file.";
            setError(ErrVar, msg.c_str());
            return false;
        }
        std::istringstream iss(line);
        for (int col = 0; col < n_cols; col++) {
            if (!(iss >> mat[col * n_rows + row])) {
                std::string msg = "Error reading " + filename + " " + tableName +
                                  " table. Please check formatting and size of matrices in that file.";
                setError(ErrVar, msg.c_str());
                return false;
            }
        }
    }
    return true;
}

void ReadCpFile(const ControlParameters& CntrPar, performancedata_view_t* PerfData,
                errorvariables_t* ErrVar) {

    // PerfFileName is already a std::string
    std::string filename = CntrPar.PerfFileName;

    // Open file
    std::ifstream f(filename);
    if (!f.is_open()) {
        std::string msg = "Error opening performance file: " + filename;
        setError(ErrVar, msg.c_str());
        return;
    }

    int n_pitch = CntrPar.PerfTableSize[0];  // PerfTableSize(1) = number of pitch angles (columns)
    int n_tsr   = CntrPar.PerfTableSize[1];  // PerfTableSize(2) = number of TSR values (rows)

    // ---- Axis Definitions ----
    // Skip 4 header/comment lines
    skipLines(f, 4);

    // Read pitch angle vector (Beta_vec): n_pitch values
    if (!readRow(f, PerfData->Beta_vec, n_pitch)) {
        setError(ErrVar, "Error reading pitch angle vector from performance file.");
        f.close();
        return;
    }

    // Skip 1 comment line ("# TSR vector...")
    skipLines(f, 1);

    // Read TSR vector: n_tsr values
    if (!readRow(f, PerfData->TSR_vec, n_tsr)) {
        setError(ErrVar, "Error reading TSR vector from performance file.");
        f.close();
        return;
    }

    // ---- Read Cp, Ct, Cq Tables ----
    // Skip 5 lines (wind speed line, blank, "# Power coefficient", blank, blank)
    skipLines(f, 5);

    // Read Cp matrix: n_tsr rows x n_pitch cols (column-major in view)
    if (!readMatrix(f, PerfData->Cp_mat, n_tsr, n_pitch, filename, "Cp", ErrVar)) {
        f.close();
        return;
    }

    // Skip 4 lines (blank, blank, "# Thrust coefficient", blank)
    skipLines(f, 4);

    // Read Ct matrix
    if (!readMatrix(f, PerfData->Ct_mat, n_tsr, n_pitch, filename, "Ct", ErrVar)) {
        f.close();
        return;
    }

    // Skip 4 lines (blank, blank, "# Torque coefficient", blank)
    skipLines(f, 4);

    // Read Cq matrix
    if (!readMatrix(f, PerfData->Cq_mat, n_tsr, n_pitch, filename, "Cq", ErrVar)) {
        f.close();
        return;
    }

    f.close();

    // Add RoutineName to error message if there was an error
    if (ErrVar->aviFAIL < 0) {
        char tmp[1024];
        int msgLen = 1024;
        while (msgLen > 0 && ErrVar->ErrMsg[msgLen - 1] == ' ') msgLen--;
        std::snprintf(tmp, sizeof(tmp), "ReadCpFile:%.*s", msgLen, ErrVar->ErrMsg);
        setError(ErrVar, tmp);
    }
}
