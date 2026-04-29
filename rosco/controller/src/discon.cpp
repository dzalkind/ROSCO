// DISCON — Bladed DLL entry point for the ROSCO wind turbine controller.
//
// This file is the top-level orchestrator. Each timestep, the simulation
// calls DISCON() with the avrSWAP array (turbine measurements in, control
// demands out). DISCON reads the measurements, runs the controller modules
// in sequence, and writes the demanded actuator signals back.
//
// Controller call sequence (per timestep):
//   ReadAvrSWAP          — unpack turbine measurements from avrSWAP
//   SetParameters        — initialize state on first call; update OL index
//   PreFilterMeasuredSignals — low-pass / notch filtering of sensor signals
//   WindSpeedEstimator   — estimate effective hub-height wind speed
//   PowerControlSetpoints — compute power-reference setpoints
//   SpeedSetpoints    — compute rated-speed / torque setpoints
//   TorqueStateMachine — determine operating region
//   SetpointSmoother     — blend setpoints between VS and PC regions
//   TorqueControl        — generator torque demand
//   PitchControl         — collective + individual pitch demand
//   YawRateControl       — yaw rate demand (if enabled)
//   FlapControl          — trailing-edge flap demand (if enabled)
//   CableControl / StructuralControl — mooring / StC demand (if enabled)
//   Debug                — write log file
//
// Config file: DISCON.IN (legacy key-value) or DISCON.toml — auto-detected
// by file extension.

#include "include/vit_types.h"
#include "include/rosco_types.hpp"
#include "include/rosco_objects.hpp"
#include "include/rosco_constants.h"
#include "include/rosco_error.hpp"
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <filesystem>
#include <string>

static const char* ROSCO_VERSION = "2.10.1";

// Controller function declarations
#include "include/vit_translated.h"

// ============================================================
// Controller state — persists across timesteps for the full
// simulation (equivalent to Fortran SAVE variables)
// ============================================================
static ControlParameters  CntrPar  = {};  // tuning parameters read from config file
static LocalVariables     LocalVar = {};  // turbine measurements + derived signals
static PerformanceData    PerfData = {};  // rotor Cp/Ct/Cq lookup tables
static debugvariables_t   DebugVar = {};  // quantities written to the log file
static ExtControlType     ExtDLL   = {};  // external controller DLL swap buffer

// ============================================================
// Bladed avrSWAP record indices
// (0-based in C; Bladed documentation uses 1-based Fortran numbering)
// ============================================================
constexpr int SWAP_STATUS       =  0;   // Record  1: iStatus (0=init, -1=final, >0=running)
constexpr int SWAP_MSG_LEN      = 48;   // Record 49: message buffer length (bytes)
constexpr int SWAP_INFILE_LEN   = 49;   // Record 50: input filename length
constexpr int SWAP_OUTNAME_LEN  = 50;   // Record 51: output name length

// ============================================================
// GetRoot: strip extension from a Bladed space-padded filename
// e.g. "/path/to/Case01.outb" → "/path/to/Case01"
// ============================================================
static std::string GetRoot(const char* s, int len) {
    while (len > 0 && (s[len-1] == ' ' || s[len-1] == '\0')) len--;
    return std::filesystem::path(std::string(s, len)).replace_extension("").string();
}

// ============================================================
// read_config_files: load DISCON.IN or DISCON.toml + Cp/Ct/Cq tables
// Called on first timestep and on warm-restart (iStatus == -9)
// ============================================================
static void read_config_files(char* accINFILE, int accINFILE_size) {
    // Extract null-terminated filename from the Bladed space-padded buffer
    int len = std::min(accINFILE_size, 1023);
    std::string filename(accINFILE, len);
    filename = filename.substr(0, filename.find('\0'));
    while (!filename.empty() && filename.back() == ' ') filename.pop_back();

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

// ============================================================
// DISCON — Bladed DLL entry point (called every timestep)
// ============================================================
#if defined(_WIN32)
  #define DISCON_EXPORT extern "C" __declspec(dllexport)
#else
  #define DISCON_EXPORT extern "C" __attribute__((visibility("default")))
#endif

DISCON_EXPORT void DISCON(float* avrSWAP, int* aviFAIL, char* accINFILE, char* avcOUTNAME, char* avcMSG) {

    // avcMSG buffer size is available before try — needed in the catch handlers
    const int size_avcMSG = std::max(1, (int)avrSWAP[SWAP_MSG_LEN]);

    // Wrap the entire body in try/catch so that C++ exceptions (e.g. bad_alloc)
    // cannot propagate into LabVIEW or the Bladed process — they are caught and
    // converted to the aviFAIL = -1 / avcMSG error reporting channel instead.
    try {

        const int accINFILE_size  = std::max(0, (int)avrSWAP[SWAP_INFILE_LEN]);
        const int avcOUTNAME_size = std::max(0, (int)avrSWAP[SWAP_OUTNAME_LEN]);
        const int iStatus         = (int)avrSWAP[SWAP_STATUS];

        // Root name used for checkpoint and log files (output name without extension)
        std::string rootStr = GetRoot(avcOUTNAME, avcOUTNAME_size);
        // Downstream functions still expect a space-padded char buffer
        char RootName[1024] = {};
        int rn = std::min((int)rootStr.size(), (int)sizeof(RootName) - 1);
        memcpy(RootName, rootStr.c_str(), rn);
        memset(RootName + rn, ' ', sizeof(RootName) - rn);

        // Default demanded actuator signals (overwritten below by controller modules)
        avrSWAP[34] = 1.0f;   // Record 35: request generator torque (1 = active)
        avrSWAP[35] = 0.0f;   // Record 36: shaft brake state (0 = off)
        avrSWAP[40] = 0.0f;   // Record 41: demanded nacelle yaw (rad)
        avrSWAP[45] = 0.0f;   // Record 46: demanded pitch — blade 1 (rad)
        avrSWAP[54] = 0.0f;   // Record 55: demanded pitch — blade 2 (rad)
        avrSWAP[55] = 0.0f;   // Record 56: demanded pitch — blade 3 (rad)
        avrSWAP[64] = 0.0f;   // Record 65: variable-slip flag
        avrSWAP[71] = 0.0f;   // Record 72: cable control output
        avrSWAP[78] = 4.0f;   // Record 79: generator torque output (Bladed expects 4)
        avrSWAP[79] = 0.0f;   // Record 80: demanded pitch rate (rad/s)
        avrSWAP[80] = 0.0f;   // Record 81: shaft brake override

        // --------------------------------------------------------
        // Warm restart: restore state from checkpoint file
        // then re-read config (parameters may have changed)
        // --------------------------------------------------------
        if (iStatus == -9) {
            ReadRestartFile(avrSWAP, LocalVar, CntrPar, PerfData, RootName, avcOUTNAME_size);
            read_config_files(LocalVar.ACC_INFILE, LocalVar.ACC_INFILE_SIZE);
            if (CntrPar.LoggingLevel > 0) {
                Debug(LocalVar, CntrPar, &DebugVar, avrSWAP, RootName, avcOUTNAME_size);
            }
        }

        // --------------------------------------------------------
        // Unpack turbine measurements from avrSWAP → LocalVar
        // --------------------------------------------------------
        ReadAvrSWAP(avrSWAP, LocalVar, CntrPar);

        // --------------------------------------------------------
        // First timestep: print banner, read config, initialize state
        // --------------------------------------------------------
        if (LocalVar.iStatus == 0) {
            printf("                                                                              \n"
                   "------------------------------------------------------------------------------\n"
                   "Running ROSCO-%s (c++ version)\n"
                   "A wind turbine controller framework for public use in the scientific field    \n"
                   "Developed in collaboration: National Renewable Energy Laboratory              \n"
                   "                            Delft University of Technology, The Netherlands   \n"
                   "------------------------------------------------------------------------------\n",
                   ROSCO_VERSION);

            // Save input filename so it can be re-read on warm restart
            LocalVar.ACC_INFILE_SIZE = accINFILE_size;
            memset(LocalVar.ACC_INFILE, ' ', sizeof(LocalVar.ACC_INFILE));
            memcpy(LocalVar.ACC_INFILE, accINFILE,
                   std::min(accINFILE_size, (int)sizeof(LocalVar.ACC_INFILE)));

            read_config_files(accINFILE, accINFILE_size);
        }

        // SetParameters: initialize LocalVar on first call; update OL_Index every call
        SetParameters(CntrPar, LocalVar, avrSWAP, size_avcMSG);

        // --------------------------------------------------------
        // External DLL controller (optional, Ext_Mode > 0)
        // --------------------------------------------------------
        if (CntrPar.Ext_Mode > 0) {
            ExtDLL.avrSWAP.resize(2000, 0.0f);
            ExtController(avrSWAP, CntrPar, LocalVar, ExtDLL);
        }

        // --------------------------------------------------------
        // Main control loop — runs on normal timesteps (iStatus >= 0)
        // and on the final call to write a checkpoint (iStatus == -8)
        // --------------------------------------------------------
        bool running = (LocalVar.iStatus >= 0) || (LocalVar.iStatus <= -8);
        if (running) {

            if (LocalVar.iStatus == -8) {
                WriteRestartFile(LocalVar, CntrPar, RootName, avcOUTNAME_size);
            }

            if (CntrPar.ZMQ_Mode > 0)  UpdateZeroMQ(LocalVar, CntrPar);
            if (CntrPar.SD_Mode  > 0)  Shutdown(LocalVar, CntrPar);

            PreFilterMeasuredSignals(CntrPar, LocalVar, &DebugVar);
            WindSpeedEstimator(LocalVar, CntrPar, PerfData, &DebugVar);
            PowerControlSetpoints(CntrPar, LocalVar, &DebugVar);

            if (CntrPar.SU_Mode > 0)   Startup(LocalVar, CntrPar);

            SpeedSetpoints(CntrPar, LocalVar, &DebugVar);
            TorqueStateMachine(CntrPar, LocalVar);
            SetpointSmoother(LocalVar, CntrPar);
            TorqueControl(avrSWAP, CntrPar, LocalVar);

            if (CntrPar.PC_ControlMode > 0) PitchControl(avrSWAP, CntrPar, LocalVar, &DebugVar);
            if (CntrPar.Y_ControlMode  > 0) YawRateControl(avrSWAP, CntrPar, LocalVar, &DebugVar);
            if (CntrPar.Flp_Mode       > 0) FlapControl(avrSWAP, CntrPar, LocalVar);
            if (CntrPar.CC_Mode        > 0) CableControl(avrSWAP, CntrPar, LocalVar);
            if (CntrPar.StC_Mode       > 0) StructuralControl(avrSWAP, CntrPar, LocalVar);

        } else if (LocalVar.iStatus == -1 && CntrPar.ZMQ_Mode > 0) {
            // Final call: send last measurement to ZMQ coordinator
            UpdateZeroMQ(LocalVar, CntrPar);
        }

        // --------------------------------------------------------
        // Debug logging
        // --------------------------------------------------------
        if (CntrPar.LoggingLevel > 0) {
            Debug(LocalVar, CntrPar, &DebugVar, avrSWAP, RootName, avcOUTNAME_size);
        }

        // No error — report success
        *aviFAIL = 0;
        if (size_avcMSG > 0) avcMSG[0] = '\0';

    } catch (const RoscoError& e) {
        *aviFAIL = -1;
        int n = size_avcMSG > 1 ? size_avcMSG - 1 : 0;
        std::strncpy(avcMSG, e.what(), (size_t)n);
        avcMSG[n] = '\0';
        printf(" ROSCO ERROR: %s\n", e.what());
    } catch (const std::exception& e) {
        *aviFAIL = -1;
        int n = size_avcMSG > 1 ? size_avcMSG - 1 : 0;
        std::strncpy(avcMSG, e.what(), (size_t)n);
        avcMSG[n] = '\0';
        printf(" ROSCO ERROR (std::exception): %s\n", e.what());
    } catch (...) {
        *aviFAIL = -1;
        const char* msg = "Unknown C++ exception in DISCON";
        int n = size_avcMSG > 1 ? size_avcMSG - 1 : 0;
        std::strncpy(avcMSG, msg, (size_t)n);
        avcMSG[n] = '\0';
        printf(" ROSCO ERROR: %s\n", msg);
    }
}
