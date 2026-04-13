// Phase 11A: C++ DISCON entry point
// Replaces DISCON.F90 — the Bladed DLL interface for the ROSCO controller.
// All 52 functions are already translated to C++; this file orchestrates them
// directly via _c entry points, eliminating the Fortran interop layer.

#include "include/vit_types.h"
#include "include/rosco_types.hpp"
#include "include/rosco_constants.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <vector>

static const char* ROSCO_VERSION = "2.10.1";

// ============================================================
// Callee declarations
// ============================================================
#include "include/vit_translated.h"

// ============================================================
// Static state — persists for DLL lifetime (replaces Fortran SAVE)
// ============================================================
static ControlParameters CntrParOwner;
static localvariables_t LocalVar = {};
static objectinstances_t objInst = {};
static performancedata_view_t PerfData = {};
static debugvariables_t DebugVar = {};
static errorvariables_t ErrVar = {};
static extcontroltype_view_t ExtDLL = {};

// Allocation owners for view structs not yet migrated to C++ types
static struct {
    // PerformanceData (5 fields)
    std::vector<double> TSR_vec;
    std::vector<double> Beta_vec;
    std::vector<double> Cp_mat;
    std::vector<double> Ct_mat;
    std::vector<double> Cq_mat;
    // ExtController
    std::vector<float> ExtDLL_avrSWAP;
} alloc;

// ============================================================
// GetRoot: extract root filename (strip extension)
// Replicates ROSCO_Helpers.f90:GetRoot
// ============================================================
static void GetRoot(const char* GivenFil, int len, char* RootName, int rootLen) {
    // Trim trailing spaces/nulls
    int trimLen = len;
    while (trimLen > 0 && (GivenFil[trimLen-1] == ' ' || GivenFil[trimLen-1] == '\0'))
        trimLen--;

    // Special cases: "." or ".."
    if ((trimLen == 1 && GivenFil[0] == '.') ||
        (trimLen == 2 && GivenFil[0] == '.' && GivenFil[1] == '.')) {
        int n = std::min(trimLen, rootLen);
        memcpy(RootName, GivenFil, n);
        memset(RootName + n, ' ', rootLen - n);
        return;
    }

    // Scan backward for last '.'
    for (int i = trimLen - 1; i >= 0; i--) {
        if (GivenFil[i] == '.') {
            if (i < trimLen - 1) {
                // Check next char isn't '/' or '\'
                if (GivenFil[i+1] != '/' && GivenFil[i+1] != '\\') {
                    int n = std::min(i, rootLen);
                    memcpy(RootName, GivenFil, n);
                    memset(RootName + n, ' ', rootLen - n);
                    return;
                } else {
                    // No extension
                    break;
                }
            } else {
                if (i == 0) {
                    memset(RootName, ' ', rootLen);
                    return;
                }
                int n = std::min(i, rootLen);
                memcpy(RootName, GivenFil, n);
                memset(RootName + n, ' ', rootLen - n);
                return;
            }
        }
    }

    // No '.' found — root = entire file
    int n = std::min(trimLen, rootLen);
    memcpy(RootName, GivenFil, n);
    memset(RootName + n, ' ', rootLen - n);
}

// ============================================================
// Allocate PerformanceData arrays (before ReadCpFile)
// ============================================================
static void allocate_perfdata_arrays(const ControlParameters& CntrPar, performancedata_view_t* pd) {
    int nBeta = CntrPar.PerfTableSize[0];
    int nTSR  = CntrPar.PerfTableSize[1];

    alloc.Beta_vec.assign(nBeta, 0.0);  pd->Beta_vec = alloc.Beta_vec.data(); pd->n_Beta_vec = nBeta;
    alloc.TSR_vec.assign(nTSR, 0.0);    pd->TSR_vec = alloc.TSR_vec.data(); pd->n_TSR_vec = nTSR;
    // 2D matrices: column-major (Fortran layout). Dimensions: (nTSR, nBeta)
    alloc.Cp_mat.assign(nTSR * nBeta, 0.0); pd->Cp_mat = alloc.Cp_mat.data(); pd->n_Cp_mat_rows = nTSR; pd->n_Cp_mat_cols = nBeta;
    alloc.Ct_mat.assign(nTSR * nBeta, 0.0); pd->Ct_mat = alloc.Ct_mat.data(); pd->n_Ct_mat_rows = nTSR; pd->n_Ct_mat_cols = nBeta;
    alloc.Cq_mat.assign(nTSR * nBeta, 0.0); pd->Cq_mat = alloc.Cq_mat.data(); pd->n_Cq_mat_rows = nTSR; pd->n_Cq_mat_cols = nBeta;
}

// ============================================================
// Read config files: single-pass ReadControlParameterFileSub + ReadCpFile
// Used by both iStatus==0 and iStatus==-9 (restart) paths
// ============================================================
// Returns true if filename ends with ".toml" (case-insensitive)
static bool is_toml_file(const char* filename) {
    size_t len = std::strlen(filename);
    if (len < 5) return false;
    const char* ext = filename + len - 5;
    return (ext[0]=='.' &&
            (ext[1]=='t'||ext[1]=='T') &&
            (ext[2]=='o'||ext[2]=='O') &&
            (ext[3]=='m'||ext[3]=='M') &&
            (ext[4]=='l'||ext[4]=='L'));
}

static void read_config_files(float* avrSWAP, char* accINFILE, int accINFILE_size) {
    // Extract null-terminated filename from accINFILE
    char filename[1024] = {};
    int fnLen = 0;
    for (int i = 0; i < std::min(accINFILE_size, 1023); i++) {
        if (accINFILE[i] == '\0') break;
        filename[i] = accINFILE[i];
        fnLen = i + 1;
    }
    filename[fnLen] = '\0';

    // Reset CntrParOwner to defaults
    CntrParOwner = ControlParameters{};

    if (is_toml_file(filename)) {
        // ---- TOML path: single-pass, vector-based ----
        CntrParOwner.load_from_toml(filename, &ErrVar);
        if (ErrVar.aviFAIL < 0) return;
    } else {
        // ---- Legacy DISCON.IN path: single-pass ----

        // Extract directory path (priPath) from filename
        char priPath[1024] = {};
        int lastSep = -1;
        for (int i = fnLen - 1; i >= 0; i--) {
            if (filename[i] == '/' || filename[i] == '\\') {
                lastSep = i;
                break;
            }
        }
        if (lastSep >= 0) {
            memcpy(priPath, filename, lastSep + 1);
            priPath[lastSep + 1] = '\0';
        } else {
            priPath[0] = '.'; priPath[1] = '/'; priPath[2] = '\0';
        }

        ReadControlParameterFileSub(CntrParOwner, &LocalVar, filename, priPath, &ErrVar);
        if (ErrVar.aviFAIL < 0) {
            char tmp[sizeof(ErrVar.ErrMsg)];
            snprintf(tmp, sizeof(tmp), "SetParameters:%s", ErrVar.ErrMsg);
            memcpy(ErrVar.ErrMsg, tmp, sizeof(ErrVar.ErrMsg));
            return;
        }
    }

    // ReadCpFile (performance tables) — common to both TOML and DISCON.IN paths
    if (CntrParOwner.WE_Mode > 0) {
        allocate_perfdata_arrays(CntrParOwner, &PerfData);
        ReadCpFile(CntrParOwner, &PerfData, &ErrVar);
    }
}

// ============================================================
// DISCON — Bladed DLL entry point
// ============================================================
#if defined(_WIN32)
  #define DISCON_EXPORT extern "C" __declspec(dllexport)
#else
  #define DISCON_EXPORT extern "C" __attribute__((visibility("default")))
#endif

DISCON_EXPORT void DISCON(float* avrSWAP, int* aviFAIL, char* accINFILE, char* avcOUTNAME, char* avcMSG) {

    // Extract message buffer size before try — needed in catch handlers
    int size_avcMSG     = (int)avrSWAP[48];   // avrSWAP(49) in Fortran (1-based)
    try {

    int accINFILE_size  = (int)avrSWAP[49];    // avrSWAP(50)
    int avcOUTNAME_size = (int)avrSWAP[50];    // avrSWAP(51)

    // RootName: extract from avcOUTNAME via GetRoot
    char RootName[1024] = {};
    int rootLen = std::min(avcOUTNAME_size, (int)sizeof(RootName));
    GetRoot(avcOUTNAME, avcOUTNAME_size, RootName, rootLen);

    // ============================================================
    // Per-timestep init (SetParameters wrapper logic)
    // ============================================================
    ErrVar.aviFAIL = 0;
    ErrVar.size_avcMSG = size_avcMSG;

    objInst.instLPF         = 1;
    objInst.instSecLPF      = 1;
    objInst.instSecLPFV     = 1;
    objInst.instHPF         = 1;
    objInst.instNotchSlopes = 1;
    objInst.instNotch       = 1;
    objInst.instPI          = 1;
    objInst.instRes         = 1;
    objInst.instRL          = 1;

    avrSWAP[34] = 1.0f;   // avrSWAP(35)
    avrSWAP[35] = 0.0f;   // avrSWAP(36)
    avrSWAP[40] = 0.0f;   // avrSWAP(41)
    avrSWAP[45] = 0.0f;   // avrSWAP(46)
    avrSWAP[54] = 0.0f;   // avrSWAP(55)
    avrSWAP[55] = 0.0f;   // avrSWAP(56)
    avrSWAP[64] = 0.0f;   // avrSWAP(65)
    avrSWAP[71] = 0.0f;   // avrSWAP(72)
    avrSWAP[78] = 4.0f;   // avrSWAP(79)
    avrSWAP[79] = 0.0f;   // avrSWAP(80)
    avrSWAP[80] = 0.0f;   // avrSWAP(81)

    // ============================================================
    // Check for restart (iStatus == -9)
    // ============================================================
    int iStatus = (int)avrSWAP[0];  // avrSWAP(1)

    if (iStatus == -9 && *aviFAIL >= 0) {
        ReadRestartFile(avrSWAP, &LocalVar, CntrParOwner, &objInst, &PerfData, RootName, avcOUTNAME_size, &ErrVar);
        // Callee dispatch: re-read config files (same as iStatus==0)
        read_config_files(avrSWAP, LocalVar.ACC_INFILE, LocalVar.ACC_INFILE_SIZE);
        if (CntrParOwner.LoggingLevel > 0) {
            Debug(&LocalVar, CntrParOwner, &DebugVar, &ErrVar, avrSWAP, RootName, avcOUTNAME_size);
        }
    }

    // ============================================================
    // Read avrSWAP array into derived types
    // ============================================================
    ReadAvrSWAP(avrSWAP, &LocalVar, CntrParOwner, &ErrVar);

    // ============================================================
    // Set Control Parameters
    // ============================================================
    if (ErrVar.aviFAIL >= 0) {
        if (LocalVar.iStatus == 0) {
            // First call: banner + file reading + init
            printf("                                                                              \n"
                   "------------------------------------------------------------------------------\n"
                   "Running ROSCO-%s (c++ version)\n"
                   "A wind turbine controller framework for public use in the scientific field    \n"
                   "Developed in collaboration: National Renewable Energy Laboratory              \n"
                   "                            Delft University of Technology, The Netherlands   \n"
                   "------------------------------------------------------------------------------\n",
                   ROSCO_VERSION);

            // Save accINFILE to LocalVar
            LocalVar.ACC_INFILE_SIZE = accINFILE_size;
            memset(LocalVar.ACC_INFILE, ' ', sizeof(LocalVar.ACC_INFILE));
            int copyLen = std::min(accINFILE_size, (int)sizeof(LocalVar.ACC_INFILE));
            memcpy(LocalVar.ACC_INFILE, accINFILE, copyLen);

            // Read config files (single-pass)
            read_config_files(avrSWAP, accINFILE, accINFILE_size);
            if (ErrVar.aviFAIL < 0) goto error_handling;
        }

        // SetParameters C++ logic (LocalVar init on iStatus==0, OL_Index on every call)
        SetParameters(CntrParOwner, &LocalVar, avrSWAP, &objInst, &ErrVar, size_avcMSG);

        // Error prepend for CheckInputs errors
        if (LocalVar.iStatus == 0 && ErrVar.aviFAIL < 0) {
            char tmp[sizeof(ErrVar.ErrMsg)];
            snprintf(tmp, sizeof(tmp), "SetParameters:%s", ErrVar.ErrMsg);
            memcpy(ErrVar.ErrMsg, tmp, sizeof(ErrVar.ErrMsg));
        }
    }

    // ============================================================
    // External controller
    // ============================================================
    if (CntrParOwner.Ext_Mode > 0 && ErrVar.aviFAIL >= 0) {
        // Guard-allocate ExtDLL avrSWAP
        if (alloc.ExtDLL_avrSWAP.empty()) {
            alloc.ExtDLL_avrSWAP.assign(2000, 0.0f);
            ExtDLL.avrSWAP = alloc.ExtDLL_avrSWAP.data();
            ExtDLL.n_avrSWAP = 2000;
        }
        ExtController(avrSWAP, CntrParOwner, &LocalVar, &ExtDLL, &ErrVar);
    }

    // ============================================================
    // Filter signals
    // ============================================================
    if (ErrVar.aviFAIL >= 0) {
        PreFilterMeasuredSignals(CntrParOwner, &LocalVar, &DebugVar, &objInst, &ErrVar);
    }

    // ============================================================
    // Main control calculations
    // ============================================================
    if (((LocalVar.iStatus >= 0) || (LocalVar.iStatus <= -8)) && (ErrVar.aviFAIL >= 0)) {
        if ((LocalVar.iStatus == -8) && (ErrVar.aviFAIL >= 0)) {
            WriteRestartFile(&LocalVar, CntrParOwner, &ErrVar, &objInst, RootName, avcOUTNAME_size);
        }
        if (CntrParOwner.ZMQ_Mode > 0) {
            UpdateZeroMQ(&LocalVar, CntrParOwner, &ErrVar);
        }
        if (CntrParOwner.SD_Mode > 0) {
            Shutdown(&LocalVar, CntrParOwner, &objInst, &ErrVar);
        }
        WindSpeedEstimator(&LocalVar, CntrParOwner, &objInst, &PerfData, &DebugVar, &ErrVar);
        PowerControlSetpoints(CntrParOwner, &LocalVar, &objInst, &DebugVar, &ErrVar);
        if (CntrParOwner.SU_Mode > 0) {
            Startup(&LocalVar, CntrParOwner, &objInst, &ErrVar);
        }
        ComputeVariablesSetpoints(CntrParOwner, &LocalVar, &objInst, &DebugVar, &ErrVar);
        StateMachine(CntrParOwner, &LocalVar);
        SetpointSmoother(&LocalVar, CntrParOwner, &objInst);
        VariableSpeedControl(avrSWAP, CntrParOwner, &LocalVar, &objInst, &ErrVar);
        if (CntrParOwner.PC_ControlMode > 0) {
            PitchControl(avrSWAP, CntrParOwner, &LocalVar, &objInst, &DebugVar, &ErrVar);
        }
        if (CntrParOwner.Y_ControlMode > 0) {
            YawRateControl(avrSWAP, CntrParOwner, &LocalVar, &objInst, &DebugVar, &ErrVar);
        }
        if (CntrParOwner.Flp_Mode > 0) {
            FlapControl(avrSWAP, CntrParOwner, &LocalVar, &objInst);
        }
        if (CntrParOwner.CC_Mode > 0) {
            CableControl(avrSWAP, CntrParOwner, &LocalVar, &objInst, &ErrVar);
        }
        if (CntrParOwner.StC_Mode > 0) {
            StructuralControl(avrSWAP, CntrParOwner, &LocalVar, &objInst, &ErrVar);
        }
    } else if ((LocalVar.iStatus == -1) && (CntrParOwner.ZMQ_Mode > 0)) {
        UpdateZeroMQ(&LocalVar, CntrParOwner, &ErrVar);
    }

    // ============================================================
    // Debug logging
    // ============================================================
    if ((CntrParOwner.LoggingLevel > 0) && (ErrVar.aviFAIL >= 0)) {
        Debug(&LocalVar, CntrParOwner, &DebugVar, &ErrVar, avrSWAP, RootName, avcOUTNAME_size);
    }

    // ============================================================
    // Error handling
    // ============================================================
error_handling:
    if (ErrVar.aviFAIL < 0) {
        // Prepend "ROSCO:" to error message
        char tmp[sizeof(ErrVar.ErrMsg)];
        snprintf(tmp, sizeof(tmp), "ROSCO:%s", ErrVar.ErrMsg);
        memcpy(ErrVar.ErrMsg, tmp, sizeof(ErrVar.ErrMsg));
        // Trim and print
        int trimLen = (int)sizeof(ErrVar.ErrMsg) - 1;
        while (trimLen > 0 && ErrVar.ErrMsg[trimLen-1] == ' ') trimLen--;
        ErrVar.ErrMsg[trimLen] = '\0';
        printf(" %s\n", ErrVar.ErrMsg);
    }

    // Copy ErrMsg to avcMSG (space-padded, null-terminated)
    {
        // Find trimmed length of ErrMsg
        int msgLen = (int)sizeof(ErrVar.ErrMsg);
        while (msgLen > 0 && (ErrVar.ErrMsg[msgLen-1] == ' ' || ErrVar.ErrMsg[msgLen-1] == '\0'))
            msgLen--;
        // Left-justify (ADJUSTL equivalent — ErrMsg should already be left-justified)
        int copyLen = std::min(msgLen, size_avcMSG - 1);
        memcpy(avcMSG, ErrVar.ErrMsg, copyLen);
        if (copyLen < size_avcMSG)
            avcMSG[copyLen] = '\0';
    }

    *aviFAIL = ErrVar.aviFAIL;
    memset(ErrVar.ErrMsg, ' ', sizeof(ErrVar.ErrMsg));

    } catch (const std::exception& e) {
        *aviFAIL = -1;
        int n = size_avcMSG > 1 ? size_avcMSG - 1 : 0;
        std::strncpy(avcMSG, e.what(), (size_t)n);
        avcMSG[n] = '\0';
    } catch (...) {
        *aviFAIL = -1;
        const char* msg = "Unknown C++ exception in DISCON";
        int n = size_avcMSG > 1 ? size_avcMSG - 1 : 0;
        std::strncpy(avcMSG, msg, (size_t)n);
        avcMSG[n] = '\0';
    }
}
