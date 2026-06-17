// DISCON — Bladed DLL entry point for the ROSCO wind turbine controller.
//
// This file is the top-level orchestrator. Each timestep, the simulation
// calls DISCON() with the avrSWAP array (turbine measurements in, control
// demands out). DISCON stores caller-provided file paths in LocalVar,
// calls the controller stages in numbered order, and converts any C++
// exceptions to the aviFAIL/avcMSG error channel.
//
// Controller stage sequence (per timestep):
//   stage_1_sensing      — unpack measurements from avrSWAP
//   stage_2_setup        — defaults, config, SetParameters, external I/O
//   stage_3_filtering    — low-pass / notch filtering of sensor signals
//   stage_4_estimation   — wind speed estimation
//   stage_5_supervisory  — power-reference setpoints, shutdown, startup
//   stage_6_setpoints    — speed setpoints, torque state machine, smoother
//   stage_7_actuators    — torque, pitch, yaw, flap, cable, structural
//   stage_8_output       — debug logging, checkpoint writing

#include "include/vit_types.h"
#include "include/rosco_types.hpp"
#include "include/rosco_objects.hpp"
#include "include/rosco_constants.h"
#include "include/rosco_error.hpp"
#include "include/rosco_stages.h"
#include "include/vit_translated.h"
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <string>

// ============================================================
// Controller state — persists across timesteps for the full
// simulation (equivalent to Fortran SAVE variables)
// ============================================================
static ControlParameters  CntrPar  = {};  // tuning parameters read from config file
static LocalVariables     LocalVar = {};  // turbine measurements + derived signals
static PerformanceData    PerfData = {};  // rotor Cp/Ct/Cq lookup tables
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
// TrimBladedString: extract a clean std::string from a
// Bladed space-padded char buffer + length
// ============================================================
static std::string TrimBladedString(const char* s, int len) {
    while (len > 0 && (s[len-1] == ' ' || s[len-1] == '\0')) len--;
    return std::string(s, len);
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

        // Store caller-provided file paths in LocalVar so stages can access them.
        // ACC_INFILE is used by stage_2_setup for config loading.
        // RootName is used by stage_8_output for debug logs and checkpoint files.
        LocalVar.ACC_INFILE = TrimBladedString(accINFILE, accINFILE_size);
        LocalVar.RootName   = GetRoot(TrimBladedString(avcOUTNAME, avcOUTNAME_size));

        // --------------------------------------------------------
        // Stage pipeline
        // --------------------------------------------------------
        stage_1_sensing    (avrSWAP, CntrPar, LocalVar, PerfData, ExtDLL);
        stage_2_setup      (avrSWAP, CntrPar, LocalVar, PerfData, ExtDLL);

        bool running = (LocalVar.iStatus >= 0) || (LocalVar.iStatus <= -8);
        if (running) {
            stage_3_filtering  (avrSWAP, CntrPar, LocalVar, PerfData, ExtDLL);
            stage_4_estimation (avrSWAP, CntrPar, LocalVar, PerfData, ExtDLL);
            stage_5_supervisory(avrSWAP, CntrPar, LocalVar, PerfData, ExtDLL);
            stage_6_setpoints  (avrSWAP, CntrPar, LocalVar, PerfData, ExtDLL);
            stage_7_actuators  (avrSWAP, CntrPar, LocalVar, PerfData, ExtDLL);
        } else if (LocalVar.iStatus == -1 && CntrPar.ZMQ_Mode > 0) {
            // Final call: send last measurement to ZMQ coordinator
            UpdateZeroMQ(LocalVar, CntrPar);
        }

        stage_8_output     (avrSWAP, CntrPar, LocalVar, PerfData, ExtDLL);

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
