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

static const char* ROSCO_VERSION = "2.10.1";

// ============================================================
// Callee declarations
// ============================================================
#include "include/vit_translated.h"

// ============================================================
// Static state — persists for DLL lifetime (replaces Fortran SAVE)
// ============================================================
// New C++ struct owning memory via std::vector (replaces the 'alloc' struct for TOML path)
static ControlParameters CntrParOwner;
static controlparameters_view_t CntrPar = {};
static localvariables_t LocalVar = {};
static objectinstances_t objInst = {};
static performancedata_view_t PerfData = {};
static debugvariables_t DebugVar = {};
static errorvariables_t ErrVar = {};
static extcontroltype_view_t ExtDLL = {};

// Allocation owner — raw pointers for ALLOCATABLE fields.
// Allocated once during first call, persist for DLL lifetime.
static struct {
    // ControlParameters (44 fields)
    double* F_NotchFreqs;
    double* F_NotchBetaNum;
    double* F_NotchBetaDen;
    int32_t* F_GenSpdNotch_Ind;
    int32_t* F_TwrTopNotch_Ind;
    double* F_FlCornerFreq;
    double* F_FlpCornerFreq;
    double* PC_GS_angles;
    double* PC_GS_KP;
    double* PC_GS_KI;
    double* PC_GS_KD;
    double* PC_GS_TF;
    double* IPC_Vramp;
    double* IPC_KP;
    double* IPC_KI;
    double* IPC_aziOffset;
    double* VS_KP;
    double* VS_KI;
    double* VS_FBP_U;
    double* VS_FBP_Omega;
    double* VS_FBP_Tau;
    double* PRC_R_Table;
    double* PRC_Pitch_Table;
    double* PRC_WindSpeeds;
    double* PRC_GenSpeeds;
    int32_t* PerfTableSize;
    double* WE_FOPoles_v;
    double* WE_FOPoles;
    double* Y_ErrThresh;
    double* PS_WindSpeeds;
    double* PS_BldPitchMin;
    double* SU_LoadStages;
    double* SU_LoadRampDuration;
    double* SU_LoadHoldDuration;
    double* SD_StageTime;
    double* SD_StagePitch;
    double* SD_MaxTorqueRate;
    double* SD_MaxPitchRate;
    double* Fl_Kp;
    double* Fl_U;
    int32_t* Ind_BldPitch;
    double* RP_Gains;
    double* PF_Offsets;
    double* PF_TimeStuck;
    int32_t* AWC_n;
    int32_t* AWC_harmonic;
    double* AWC_freq;
    double* AWC_amp;
    double* AWC_clockangle;
    double* AWC_CntrGains;
    int32_t* CC_GroupIndex;
    int32_t* StC_GroupIndex;
    int32_t* Ind_CableControl;
    int32_t* Ind_StructControl;
    double* OL_Channels;
    double* OL_Breakpoints;
    double* OL_BldPitch1;
    double* OL_BldPitch2;
    double* OL_BldPitch3;
    double* OL_GenTq;
    double* OL_YawRate;
    double* OL_Azimuth;
    double* OL_R_Speed;
    double* OL_R_Torque;
    double* OL_R_Pitch;
    double* OL_CableControl;
    double* OL_StructControl;
    // PerformanceData (5 fields)
    double* TSR_vec;
    double* Beta_vec;
    double* Cp_mat;
    double* Ct_mat;
    double* Cq_mat;
    // ExtControlType (1 field)
    float* ExtDLL_avrSWAP;
} alloc = {};

// ============================================================
// Helper: allocate + set pointer and size in view struct
// ============================================================
static double* alloc_dbl(int n) {
    double* p = (double*)calloc(n, sizeof(double));
    return p;
}
static int32_t* alloc_int(int n) {
    int32_t* p = (int32_t*)calloc(n, sizeof(int32_t));
    return p;
}
static float* alloc_flt(int n) {
    float* p = (float*)calloc(n, sizeof(float));
    return p;
}
static inline int imax(int a, int b) { return a > b ? a : b; }

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
// Allocate all ALLOCATABLE arrays for ControlParameters
// Mechanical translation of readcontrolparameterfilesub_wrapper.f90:91-196
// ============================================================
static void allocate_cntrpar_arrays(controlparameters_view_t* cp, int32_t n_OL_rows, int32_t OL_Count) {
    int n;

    // Filters
    if (cp->F_NumNotchFilts > 0) {
        n = cp->F_NumNotchFilts;
    } else {
        n = 1;
    }
    alloc.F_NotchFreqs = alloc_dbl(n);    cp->F_NotchFreqs = alloc.F_NotchFreqs; cp->n_F_NotchFreqs = n;
    alloc.F_NotchBetaNum = alloc_dbl(n);  cp->F_NotchBetaNum = alloc.F_NotchBetaNum; cp->n_F_NotchBetaNum = n;
    alloc.F_NotchBetaDen = alloc_dbl(n);  cp->F_NotchBetaDen = alloc.F_NotchBetaDen; cp->n_F_NotchBetaDen = n;

    if (cp->F_GenSpdNotch_N > 0) {
        alloc.F_GenSpdNotch_Ind = alloc_int(cp->F_GenSpdNotch_N);
        cp->F_GenSpdNotch_Ind = alloc.F_GenSpdNotch_Ind; cp->n_F_GenSpdNotch_Ind = cp->F_GenSpdNotch_N;
    }
    if (cp->F_TwrTopNotch_N > 0) {
        alloc.F_TwrTopNotch_Ind = alloc_int(cp->F_TwrTopNotch_N);
        cp->F_TwrTopNotch_Ind = alloc.F_TwrTopNotch_Ind; cp->n_F_TwrTopNotch_Ind = cp->F_TwrTopNotch_N;
    }
    alloc.F_FlCornerFreq = alloc_dbl(2);   cp->F_FlCornerFreq = alloc.F_FlCornerFreq; cp->n_F_FlCornerFreq = 2;
    alloc.F_FlpCornerFreq = alloc_dbl(2);  cp->F_FlpCornerFreq = alloc.F_FlpCornerFreq; cp->n_F_FlpCornerFreq = 2;

    // Pitch Control
    n = imax(cp->PC_GS_n, 1);
    alloc.PC_GS_angles = alloc_dbl(n); cp->PC_GS_angles = alloc.PC_GS_angles; cp->n_PC_GS_angles = n;
    alloc.PC_GS_KP = alloc_dbl(n);     cp->PC_GS_KP = alloc.PC_GS_KP; cp->n_PC_GS_KP = n;
    alloc.PC_GS_KI = alloc_dbl(n);     cp->PC_GS_KI = alloc.PC_GS_KI; cp->n_PC_GS_KI = n;
    alloc.PC_GS_KD = alloc_dbl(n);     cp->PC_GS_KD = alloc.PC_GS_KD; cp->n_PC_GS_KD = n;
    alloc.PC_GS_TF = alloc_dbl(n);     cp->PC_GS_TF = alloc.PC_GS_TF; cp->n_PC_GS_TF = n;

    // IPC
    alloc.IPC_Vramp = alloc_dbl(2);     cp->IPC_Vramp = alloc.IPC_Vramp; cp->n_IPC_Vramp = 2;
    alloc.IPC_KP = alloc_dbl(2);        cp->IPC_KP = alloc.IPC_KP; cp->n_IPC_KP = 2;
    alloc.IPC_KI = alloc_dbl(2);        cp->IPC_KI = alloc.IPC_KI; cp->n_IPC_KI = 2;
    alloc.IPC_aziOffset = alloc_dbl(2); cp->IPC_aziOffset = alloc.IPC_aziOffset; cp->n_IPC_aziOffset = 2;

    // VS Torque
    n = imax(cp->VS_n, 1);
    alloc.VS_KP = alloc_dbl(n); cp->VS_KP = alloc.VS_KP; cp->n_VS_KP = n;
    alloc.VS_KI = alloc_dbl(n); cp->VS_KI = alloc.VS_KI; cp->n_VS_KI = n;
    n = imax(cp->VS_FBP_n, 1);
    alloc.VS_FBP_U = alloc_dbl(n);     cp->VS_FBP_U = alloc.VS_FBP_U; cp->n_VS_FBP_U = n;
    alloc.VS_FBP_Omega = alloc_dbl(n); cp->VS_FBP_Omega = alloc.VS_FBP_Omega; cp->n_VS_FBP_Omega = n;
    alloc.VS_FBP_Tau = alloc_dbl(n);   cp->VS_FBP_Tau = alloc.VS_FBP_Tau; cp->n_VS_FBP_Tau = n;

    // PRC
    n = imax(cp->PRC_Table_n, 1);
    alloc.PRC_R_Table = alloc_dbl(n);     cp->PRC_R_Table = alloc.PRC_R_Table; cp->n_PRC_R_Table = n;
    alloc.PRC_Pitch_Table = alloc_dbl(n); cp->PRC_Pitch_Table = alloc.PRC_Pitch_Table; cp->n_PRC_Pitch_Table = n;
    n = imax(cp->PRC_n, 1);
    alloc.PRC_WindSpeeds = alloc_dbl(n);  cp->PRC_WindSpeeds = alloc.PRC_WindSpeeds; cp->n_PRC_WindSpeeds = n;
    alloc.PRC_GenSpeeds = alloc_dbl(n);   cp->PRC_GenSpeeds = alloc.PRC_GenSpeeds; cp->n_PRC_GenSpeeds = n;

    // Wind Speed Estimator
    alloc.PerfTableSize = alloc_int(2);    cp->PerfTableSize = alloc.PerfTableSize; cp->n_PerfTableSize = 2;
    n = imax(cp->WE_FOPoles_N, 1);
    alloc.WE_FOPoles_v = alloc_dbl(n); cp->WE_FOPoles_v = alloc.WE_FOPoles_v; cp->n_WE_FOPoles_v = n;
    alloc.WE_FOPoles = alloc_dbl(n);   cp->WE_FOPoles = alloc.WE_FOPoles; cp->n_WE_FOPoles = n;

    // Yaw
    alloc.Y_ErrThresh = alloc_dbl(2); cp->Y_ErrThresh = alloc.Y_ErrThresh; cp->n_Y_ErrThresh = 2;

    // Peak Shaving
    n = imax(cp->PS_BldPitchMin_N, 1);
    alloc.PS_WindSpeeds = alloc_dbl(n);  cp->PS_WindSpeeds = alloc.PS_WindSpeeds; cp->n_PS_WindSpeeds = n;
    alloc.PS_BldPitchMin = alloc_dbl(n); cp->PS_BldPitchMin = alloc.PS_BldPitchMin; cp->n_PS_BldPitchMin = n;

    // Startup
    n = imax(cp->SU_LoadStages_N, 1);
    alloc.SU_LoadStages = alloc_dbl(n);        cp->SU_LoadStages = alloc.SU_LoadStages; cp->n_SU_LoadStages = n;
    alloc.SU_LoadRampDuration = alloc_dbl(n);  cp->SU_LoadRampDuration = alloc.SU_LoadRampDuration; cp->n_SU_LoadRampDuration = n;
    alloc.SU_LoadHoldDuration = alloc_dbl(n);  cp->SU_LoadHoldDuration = alloc.SU_LoadHoldDuration; cp->n_SU_LoadHoldDuration = n;

    // Shutdown
    n = imax(cp->SD_Stage_N, 1);
    alloc.SD_StageTime = alloc_dbl(n);      cp->SD_StageTime = alloc.SD_StageTime; cp->n_SD_StageTime = n;
    alloc.SD_StagePitch = alloc_dbl(n);     cp->SD_StagePitch = alloc.SD_StagePitch; cp->n_SD_StagePitch = n;
    alloc.SD_MaxTorqueRate = alloc_dbl(n);  cp->SD_MaxTorqueRate = alloc.SD_MaxTorqueRate; cp->n_SD_MaxTorqueRate = n;
    alloc.SD_MaxPitchRate = alloc_dbl(n);   cp->SD_MaxPitchRate = alloc.SD_MaxPitchRate; cp->n_SD_MaxPitchRate = n;

    // Floating
    alloc.Fl_Kp = alloc_dbl(cp->Fl_n); cp->Fl_Kp = alloc.Fl_Kp; cp->n_Fl_Kp = cp->Fl_n;
    alloc.Fl_U = alloc_dbl(cp->Fl_n);  cp->Fl_U = alloc.Fl_U; cp->n_Fl_U = cp->Fl_n;

    // Open Loop
    alloc.Ind_BldPitch = alloc_int(3);  cp->Ind_BldPitch = alloc.Ind_BldPitch; cp->n_Ind_BldPitch = 3;
    alloc.RP_Gains = alloc_dbl(4);      cp->RP_Gains = alloc.RP_Gains; cp->n_RP_Gains = 4;

    // Pitch Faults
    alloc.PF_Offsets = alloc_dbl(3);    cp->PF_Offsets = alloc.PF_Offsets; cp->n_PF_Offsets = 3;
    alloc.PF_TimeStuck = alloc_dbl(3);  cp->PF_TimeStuck = alloc.PF_TimeStuck; cp->n_PF_TimeStuck = 3;

    // AWC
    n = imax(cp->AWC_NumModes, 1);
    alloc.AWC_n = alloc_int(n);          cp->AWC_n = alloc.AWC_n; cp->n_AWC_n = n;
    alloc.AWC_harmonic = alloc_int(n);   cp->AWC_harmonic = alloc.AWC_harmonic; cp->n_AWC_harmonic = n;
    alloc.AWC_freq = alloc_dbl(n);       cp->AWC_freq = alloc.AWC_freq; cp->n_AWC_freq = n;
    alloc.AWC_amp = alloc_dbl(n);        cp->AWC_amp = alloc.AWC_amp; cp->n_AWC_amp = n;
    alloc.AWC_clockangle = alloc_dbl(n); cp->AWC_clockangle = alloc.AWC_clockangle; cp->n_AWC_clockangle = n;
    alloc.AWC_CntrGains = alloc_dbl(2);  cp->AWC_CntrGains = alloc.AWC_CntrGains; cp->n_AWC_CntrGains = 2;

    // Cable / Structural Control
    n = imax(cp->CC_Group_N, 1);
    alloc.CC_GroupIndex = alloc_int(n);      cp->CC_GroupIndex = alloc.CC_GroupIndex; cp->n_CC_GroupIndex = n;
    alloc.Ind_CableControl = alloc_int(n);   cp->Ind_CableControl = alloc.Ind_CableControl; cp->n_Ind_CableControl = n;
    n = imax(cp->StC_Group_N, 1);
    alloc.StC_GroupIndex = alloc_int(n);      cp->StC_GroupIndex = alloc.StC_GroupIndex; cp->n_StC_GroupIndex = n;
    alloc.Ind_StructControl = alloc_int(n);   cp->Ind_StructControl = alloc.Ind_StructControl; cp->n_Ind_StructControl = n;

    // Open Loop arrays (size depends on OL file rows)
    if (cp->OL_Mode > 0 && n_OL_rows > 0) {
        alloc.OL_Channels = alloc_dbl(n_OL_rows * OL_Count);
        cp->OL_Channels = alloc.OL_Channels; cp->n_OL_Channels_rows = n_OL_rows; cp->n_OL_Channels_cols = OL_Count;
        alloc.OL_Breakpoints = alloc_dbl(n_OL_rows); cp->OL_Breakpoints = alloc.OL_Breakpoints; cp->n_OL_Breakpoints = n_OL_rows;
        alloc.OL_BldPitch1 = alloc_dbl(n_OL_rows);   cp->OL_BldPitch1 = alloc.OL_BldPitch1; cp->n_OL_BldPitch1 = n_OL_rows;
        alloc.OL_BldPitch2 = alloc_dbl(n_OL_rows);   cp->OL_BldPitch2 = alloc.OL_BldPitch2; cp->n_OL_BldPitch2 = n_OL_rows;
        alloc.OL_BldPitch3 = alloc_dbl(n_OL_rows);   cp->OL_BldPitch3 = alloc.OL_BldPitch3; cp->n_OL_BldPitch3 = n_OL_rows;
        alloc.OL_GenTq = alloc_dbl(n_OL_rows);       cp->OL_GenTq = alloc.OL_GenTq; cp->n_OL_GenTq = n_OL_rows;
        alloc.OL_YawRate = alloc_dbl(n_OL_rows);     cp->OL_YawRate = alloc.OL_YawRate; cp->n_OL_YawRate = n_OL_rows;
        alloc.OL_Azimuth = alloc_dbl(n_OL_rows);     cp->OL_Azimuth = alloc.OL_Azimuth; cp->n_OL_Azimuth = n_OL_rows;
        alloc.OL_R_Speed = alloc_dbl(n_OL_rows);     cp->OL_R_Speed = alloc.OL_R_Speed; cp->n_OL_R_Speed = n_OL_rows;
        alloc.OL_R_Torque = alloc_dbl(n_OL_rows);    cp->OL_R_Torque = alloc.OL_R_Torque; cp->n_OL_R_Torque = n_OL_rows;
        alloc.OL_R_Pitch = alloc_dbl(n_OL_rows);     cp->OL_R_Pitch = alloc.OL_R_Pitch; cp->n_OL_R_Pitch = n_OL_rows;
        // OL_CableControl and OL_StructControl allocated after pass2
        // (sizes depend on Ind_CableControl/Ind_StructControl values filled by pass2)
    }
}

// ============================================================
// Allocate PerformanceData arrays (before ReadCpFile)
// ============================================================
static void allocate_perfdata_arrays(controlparameters_view_t* cp, performancedata_view_t* pd) {
    int nBeta = cp->PerfTableSize[0];
    int nTSR  = cp->PerfTableSize[1];

    alloc.Beta_vec = alloc_dbl(nBeta);  pd->Beta_vec = alloc.Beta_vec; pd->n_Beta_vec = nBeta;
    alloc.TSR_vec = alloc_dbl(nTSR);    pd->TSR_vec = alloc.TSR_vec; pd->n_TSR_vec = nTSR;
    // 2D matrices: column-major (Fortran layout). Dimensions: (nTSR, nBeta)
    alloc.Cp_mat = alloc_dbl(nTSR * nBeta); pd->Cp_mat = alloc.Cp_mat; pd->n_Cp_mat_rows = nTSR; pd->n_Cp_mat_cols = nBeta;
    alloc.Ct_mat = alloc_dbl(nTSR * nBeta); pd->Ct_mat = alloc.Ct_mat; pd->n_Ct_mat_rows = nTSR; pd->n_Ct_mat_cols = nBeta;
    alloc.Cq_mat = alloc_dbl(nTSR * nBeta); pd->Cq_mat = alloc.Cq_mat; pd->n_Cq_mat_rows = nTSR; pd->n_Cq_mat_cols = nBeta;
}

// ============================================================
// Read config files: two-pass ALLOCATE protocol + ReadCpFile
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

    if (is_toml_file(filename)) {
        // ---- TOML path: single-pass, vector-based, no manual allocation ----
        CntrParOwner = ControlParameters{};   // reset to defaults
        CntrParOwner.load_from_toml(filename, &ErrVar);
        if (ErrVar.aviFAIL < 0) return;
        CntrParOwner.populate_view(&CntrPar);
    } else {
        // ---- Legacy DISCON.IN path (two-pass) ----

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

        // Pass 1: parse scalars + count OL rows
        int32_t n_OL_rows = 0, OL_Count = 0;
        ReadControlParameterFileSub_pass1(&CntrPar, &LocalVar, filename, priPath, &ErrVar, &n_OL_rows, &OL_Count);
        if (ErrVar.aviFAIL < 0) {
            char tmp[sizeof(ErrVar.ErrMsg)];
            snprintf(tmp, sizeof(tmp), "SetParameters:%s", ErrVar.ErrMsg);
            memcpy(ErrVar.ErrMsg, tmp, sizeof(ErrVar.ErrMsg));
            return;
        }

        // Allocate all ALLOCATABLE arrays
        allocate_cntrpar_arrays(&CntrPar, n_OL_rows, OL_Count);

        // Pass 2: fill arrays + computed constants
        ReadControlParameterFileSub_pass2(&CntrPar, &LocalVar, filename, priPath, &ErrVar);

        // Allocate and populate OL_CableControl/OL_StructControl from OL_Channels
        // (must happen after pass2 fills Ind_CableControl/Ind_StructControl values
        // and OL_Channels data)
        if (CntrPar.OL_Mode > 0 && CntrPar.n_OL_Channels_rows > 0) {
        int nRows = CntrPar.n_OL_Channels_rows;
        // OL_CableControl
        int nOlCables = 0;
        for (int i = 0; i < CntrPar.n_Ind_CableControl; i++) {
            if (CntrPar.Ind_CableControl[i] > 0) nOlCables++;
        }
        if (nOlCables > 0) {
            alloc.OL_CableControl = alloc_dbl(nOlCables * nRows);
            CntrPar.OL_CableControl = alloc.OL_CableControl;
            CntrPar.n_OL_CableControl_rows = nOlCables;
            CntrPar.n_OL_CableControl_cols = nRows;
            int iOL = 0;
            for (int i = 0; i < CntrPar.n_Ind_CableControl; i++) {
                if (CntrPar.Ind_CableControl[i] > 0) {
                    int col = CntrPar.Ind_CableControl[i] - 1;
                    for (int r = 0; r < nRows; r++) {
                        CntrPar.OL_CableControl[r * nOlCables + iOL] = CntrPar.OL_Channels[col * nRows + r];
                    }
                    iOL++;
                }
            }
        }
        // OL_StructControl
        int nOlStCs = 0;
        for (int i = 0; i < CntrPar.n_Ind_StructControl; i++) {
            if (CntrPar.Ind_StructControl[i] > 0) nOlStCs++;
        }
        if (nOlStCs > 0) {
            alloc.OL_StructControl = alloc_dbl(nOlStCs * nRows);
            CntrPar.OL_StructControl = alloc.OL_StructControl;
            CntrPar.n_OL_StructControl_rows = nOlStCs;
            CntrPar.n_OL_StructControl_cols = nRows;
            int iOL = 0;
            for (int i = 0; i < CntrPar.n_Ind_StructControl; i++) {
                if (CntrPar.Ind_StructControl[i] > 0) {
                    int col = CntrPar.Ind_StructControl[i] - 1;
                    for (int r = 0; r < nRows; r++) {
                        CntrPar.OL_StructControl[r * nOlStCs + iOL] = CntrPar.OL_Channels[col * nRows + r];
                    }
                    iOL++;
                }
            }
        }
        } // end OL_Mode block
    } // end DISCON.IN else branch

    // ReadCpFile (performance tables) — common to both TOML and DISCON.IN paths
    if (CntrPar.WE_Mode > 0) {
        allocate_perfdata_arrays(&CntrPar, &PerfData);
        ReadCpFile(&CntrPar, &PerfData, &ErrVar);
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
        ReadRestartFile(avrSWAP, &LocalVar, &CntrPar, &objInst, &PerfData, RootName, avcOUTNAME_size, &ErrVar);
        // Callee dispatch: re-read config files (same as iStatus==0)
        read_config_files(avrSWAP, LocalVar.ACC_INFILE, LocalVar.ACC_INFILE_SIZE);
        if (CntrPar.LoggingLevel > 0) {
            Debug(&LocalVar, &CntrPar, &DebugVar, &ErrVar, avrSWAP, RootName, avcOUTNAME_size);
        }
    }

    // ============================================================
    // Read avrSWAP array into derived types
    // ============================================================
    ReadAvrSWAP(avrSWAP, &LocalVar, &CntrPar, &ErrVar);

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

            // Read config files (two-pass ALLOCATE protocol)
            read_config_files(avrSWAP, accINFILE, accINFILE_size);
            if (ErrVar.aviFAIL < 0) goto error_handling;
        }

        // SetParameters C++ logic (LocalVar init on iStatus==0, OL_Index on every call)
        SetParameters(&CntrPar, &LocalVar, avrSWAP, &objInst, &ErrVar, size_avcMSG);

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
    if (CntrPar.Ext_Mode > 0 && ErrVar.aviFAIL >= 0) {
        // Guard-allocate ExtDLL avrSWAP
        if (alloc.ExtDLL_avrSWAP == nullptr) {
            alloc.ExtDLL_avrSWAP = alloc_flt(2000);
            ExtDLL.avrSWAP = alloc.ExtDLL_avrSWAP;
            ExtDLL.n_avrSWAP = 2000;
        }
        ExtController(avrSWAP, &CntrPar, &LocalVar, &ExtDLL, &ErrVar);
    }

    // ============================================================
    // Filter signals
    // ============================================================
    if (ErrVar.aviFAIL >= 0) {
        PreFilterMeasuredSignals(&CntrPar, &LocalVar, &DebugVar, &objInst, &ErrVar);
    }

    // ============================================================
    // Main control calculations
    // ============================================================
    if (((LocalVar.iStatus >= 0) || (LocalVar.iStatus <= -8)) && (ErrVar.aviFAIL >= 0)) {
        if ((LocalVar.iStatus == -8) && (ErrVar.aviFAIL >= 0)) {
            WriteRestartFile(&LocalVar, &CntrPar, &ErrVar, &objInst, RootName, avcOUTNAME_size);
        }
        if (CntrPar.ZMQ_Mode > 0) {
            UpdateZeroMQ(&LocalVar, &CntrPar, &ErrVar);
        }
        if (CntrPar.SD_Mode > 0) {
            Shutdown(&LocalVar, &CntrPar, &objInst, &ErrVar);
        }
        WindSpeedEstimator(&LocalVar, &CntrPar, &objInst, &PerfData, &DebugVar, &ErrVar);
        PowerControlSetpoints(&CntrPar, &LocalVar, &objInst, &DebugVar, &ErrVar);
        if (CntrPar.SU_Mode > 0) {
            Startup(&LocalVar, &CntrPar, &objInst, &ErrVar);
        }
        ComputeVariablesSetpoints(&CntrPar, &LocalVar, &objInst, &DebugVar, &ErrVar);
        StateMachine(&CntrPar, &LocalVar);
        SetpointSmoother(&LocalVar, &CntrPar, &objInst);
        VariableSpeedControl(avrSWAP, &CntrPar, &LocalVar, &objInst, &ErrVar);
        if (CntrPar.PC_ControlMode > 0) {
            PitchControl(avrSWAP, &CntrPar, &LocalVar, &objInst, &DebugVar, &ErrVar);
        }
        if (CntrPar.Y_ControlMode > 0) {
            YawRateControl(avrSWAP, &CntrPar, &LocalVar, &objInst, &DebugVar, &ErrVar);
        }
        if (CntrPar.Flp_Mode > 0) {
            FlapControl(avrSWAP, &CntrPar, &LocalVar, &objInst);
        }
        if (CntrPar.CC_Mode > 0) {
            CableControl(avrSWAP, &CntrPar, &LocalVar, &objInst, &ErrVar);
        }
        if (CntrPar.StC_Mode > 0) {
            StructuralControl(avrSWAP, &CntrPar, &LocalVar, &objInst, &ErrVar);
        }
    } else if ((LocalVar.iStatus == -1) && (CntrPar.ZMQ_Mode > 0)) {
        UpdateZeroMQ(&LocalVar, &CntrPar, &ErrVar);
    }

    // ============================================================
    // Debug logging
    // ============================================================
    if ((CntrPar.LoggingLevel > 0) && (ErrVar.aviFAIL >= 0)) {
        Debug(&LocalVar, &CntrPar, &DebugVar, &ErrVar, avrSWAP, RootName, avcOUTNAME_size);
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
