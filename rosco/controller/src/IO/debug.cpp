#include "../include/vit_types.h"
#include "../include/rosco_types.hpp"
#include "../include/rosco_constants.h"
#include <fstream>
#include <cmath>
#include <string>
#include <cstring>
#include <cstdio>
#include <ctime>
#include <vector>
#include <algorithm>

static const char* ROSCO_VERSION = "2.10.1";

namespace {

std::string trim_fortran_string(const char* s, int len) {
    int end = len - 1;
    while (end >= 0 && (s[end] == ' ' || s[end] == '\0')) end--;
    return std::string(s, end + 1);
}

std::string current_date() {
    time_t now = time(nullptr);
    struct tm* t = localtime(&now);
    char buf[32];
    strftime(buf, sizeof(buf), "%d-%b-%Y", t);
    return std::string(buf);
}

std::string current_time() {
    time_t now = time(nullptr);
    struct tm* t = localtime(&now);
    char buf[32];
    strftime(buf, sizeof(buf), "%H:%M:%S", t);
    return std::string(buf);
}

// Clamp value to [1E-99, 1E+99] for display, zero out tiny values
double clamp_debug(double val) {
    if (std::abs(val) < 1E-99) return 0.0;
    if (std::abs(val) > 1E+99) return 1E+99;
    return val;
}

// Format a row of debug data matching Fortran format:
// (F20.5,TR5,N(ES20.5E2,TR5:))
void write_debug_row(std::ofstream& f, double time_val, const double* data, int n) {
    char buf[32];
    // Time field: F20.5 (20 chars wide, 5 decimal places)
    snprintf(buf, sizeof(buf), "%20.5f", time_val);
    f << buf;
    for (int i = 0; i < n; i++) {
        // TR5 = 5 spaces separator
        f << "     ";
        // ES20.5E2 = scientific notation, 20 chars, 5 decimal places, 2-digit exponent
        snprintf(buf, sizeof(buf), "%20.5E", data[i]);
        f << buf;
    }
    f << "\n";
}

} // anonymous namespace

// Static file handles (replacing Fortran SAVE variables)
static std::ofstream dbg_file;
static std::ofstream dbg2_file;
static std::ofstream dbg3_file;
static std::vector<int32_t> avr_indices;

void Debug(LocalVariables& LocalVar, const ControlParameters& CntrPar,
           debugvariables_t* DebugVar,
           float* avrSWAP) {

    const std::string& root = LocalVar.RootName;

    // --- Debug output data (26 fields) ---
    const int nDebugOuts = 26;
    double DebugOutData[nDebugOuts] = {
        DebugVar->WE_Cp, DebugVar->WE_b, DebugVar->WE_w, DebugVar->WE_t,
        DebugVar->WE_Vm, DebugVar->WE_Vt, DebugVar->WE_Vw, DebugVar->WE_lambda,
        DebugVar->PC_PICommand, DebugVar->GenSpeedF, DebugVar->RotSpeedF,
        DebugVar->NacIMU_FA_AccF, DebugVar->FA_AccF, DebugVar->Fl_PitCom,
        DebugVar->PC_MinPit, DebugVar->axisTilt_1P, DebugVar->axisYaw_1P,
        DebugVar->axisTilt_2P, DebugVar->axisYaw_2P, DebugVar->YawRateCom,
        DebugVar->NacHeadingTarget, DebugVar->NacVaneOffset, DebugVar->Yaw_Err,
        DebugVar->YawState, DebugVar->VS_RefSpd, DebugVar->PC_RefSpd
    };
    const char* DebugOutStrings[nDebugOuts] = {
        "WE_Cp", "WE_b", "WE_w", "WE_t", "WE_Vm",
        "WE_Vt", "WE_Vw", "WE_lambda", "PC_PICommand", "GenSpeedF",
        "RotSpeedF", "NacIMU_FA_AccF", "FA_AccF", "Fl_PitCom", "PC_MinPit",
        "axisTilt_1P", "axisYaw_1P", "axisTilt_2P", "axisYaw_2P", "YawRateCom",
        "NacHeadingTarget", "NacVaneOffset", "Yaw_Err", "YawState", "VS_RefSpd",
        "PC_RefSpd"
    };
    const char* DebugOutUnits[nDebugOuts] = {
        "[-]", "[-]", "[-]", "[-]", "[m/s]",
        "[m/s]", "[m/s]", "[rad]", "[rad]", "[rad/s]",
        "[rad/s]", "[rad/s]", "[m/s]", "[rad]", "[rad]",
        "[N/A]", "[N/A]", "[N/A]", "[N/A]", "[rad/s]",
        "[deg]", "[deg]", "[deg]", "[N/A]", "[rad/s]",
        "[rad/s]"
    };

    // --- LocalVar output data (159 fields) ---
    const int nLocalVars = 159;
    double LocalVarOutData[nLocalVars] = {
        (double)LocalVar.iStatus, (double)LocalVar.AlreadyInitialized,
        (double)LocalVar.RestartWSE, LocalVar.Time, LocalVar.DT,
        (double)LocalVar.n_DT, LocalVar.Time_Last, LocalVar.VS_GenPwr,
        LocalVar.GenSpeed, LocalVar.RotSpeed, LocalVar.NacHeading,
        LocalVar.NacVane, LocalVar.NacVaneF, LocalVar.WindDir,
        LocalVar.HorWindV, LocalVar.HorWindV_F, LocalVar.rootMOOP[0],
        LocalVar.rootMOOPF[0], LocalVar.BlPitch[0], LocalVar.BlPitchCMeas,
        LocalVar.Azimuth, LocalVar.OL_Azimuth, LocalVar.AzUnwrapped,
        LocalVar.AzError, LocalVar.GenTqAz, LocalVar.AzBuffer[0],
        (double)LocalVar.NumBl, LocalVar.FA_Acc_TT, LocalVar.SS_Acc_TT,
        LocalVar.FA_Acc_Nac, LocalVar.NacIMU_FA_RAcc, LocalVar.FA_AccHPF,
        LocalVar.FA_AccHPFI, LocalVar.FA_PitCom[0], LocalVar.VS_RefSpd,
        LocalVar.VS_RefSpd_TSR, LocalVar.VS_RefSpd_TRA, LocalVar.VS_RefSpd_RL,
        LocalVar.PC_RefSpd, LocalVar.PC_RefSpd_SS, LocalVar.PC_RefSpd_PRC,
        LocalVar.RotSpeedF, LocalVar.GenSpeedF, LocalVar.GenTq,
        LocalVar.GenTqMeas, LocalVar.GenArTq, LocalVar.GenBrTq,
        LocalVar.VS_KOmega2_GenTq, LocalVar.VS_ConstPwr_GenTq,
        LocalVar.IPC_PitComF[0], LocalVar.PC_KP, LocalVar.PC_KI,
        LocalVar.PC_KD, LocalVar.PC_TF, LocalVar.PC_MaxPit,
        LocalVar.PC_MinPit, LocalVar.PC_PitComT, LocalVar.PC_PitComT_Last,
        LocalVar.BlPitchCMeasF, LocalVar.PC_PitComT_IPC[0],
        LocalVar.PC_PwrErr, LocalVar.PC_SpdErr, LocalVar.IPC_AxisTilt_1P,
        LocalVar.IPC_AxisYaw_1P, LocalVar.IPC_AxisTilt_2P, LocalVar.IPC_AxisYaw_2P,
        LocalVar.axisTilt_1P, LocalVar.axisYaw_1P, LocalVar.axisYawF_1P,
        LocalVar.axisTilt_2P, LocalVar.axisYaw_2P, LocalVar.axisYawF_2P,
        LocalVar.IPC_KI[0], LocalVar.IPC_KP[0], LocalVar.IPC_IntSat,
        (double)LocalVar.PC_State, LocalVar.PitCom[0], LocalVar.PitCom_SD[0],
        LocalVar.PitComAct[0], LocalVar.SS_DelOmegaF, LocalVar.TestType,
        LocalVar.Kp_Float, LocalVar.VS_MaxTq, LocalVar.VS_LastGenTrq,
        LocalVar.VS_LastGenPwr, LocalVar.VS_MechGenPwr, LocalVar.VS_SpdErrAr,
        LocalVar.VS_SpdErrBr, LocalVar.VS_SpdErr, (double)LocalVar.VS_State,
        LocalVar.VS_Rgn3Pitch, LocalVar.WE_Vw, LocalVar.WE_Vw_F,
        LocalVar.WE_VwI, LocalVar.WE_VwIdot, (double)LocalVar.WE_Op,
        (double)LocalVar.WE_Op_Last, LocalVar.VS_LastGenTrqF, LocalVar.PRC_WSE_F,
        LocalVar.PRC_R_Speed, LocalVar.PRC_R_Torque, LocalVar.PRC_R_Pitch,
        LocalVar.PRC_R_Total, LocalVar.PRC_Min_Pitch, LocalVar.PS_Min_Pitch,
        LocalVar.OL_Index, (double)LocalVar.SU_Stage,
        LocalVar.SU_LoadStageStartTime, LocalVar.SU_RotSpeedF,
        (double)LocalVar.SD_Trigger, LocalVar.SD_BlPitchF, LocalVar.SD_NacVaneF,
        LocalVar.SD_GenSpeedF, (double)LocalVar.SD_Stage,
        LocalVar.SD_StageStartTime, LocalVar.SD_MaxPitchRate,
        LocalVar.SD_MaxTorqueRate, LocalVar.GenTq_SD, LocalVar.Fl_PitCom,
        LocalVar.NACIMU_FA_AccF, LocalVar.FA_AccF, (double)LocalVar.FA_Hist,
        LocalVar.TRA_LastRefSpd, LocalVar.VS_RefSpeed,
        LocalVar.PtfmTDX, LocalVar.PtfmTDY, LocalVar.PtfmTDZ,
        LocalVar.PtfmRDX, LocalVar.PtfmRDY, LocalVar.PtfmRDZ,
        LocalVar.PtfmTVX, LocalVar.PtfmTVY, LocalVar.PtfmTVZ,
        LocalVar.PtfmRVX, LocalVar.PtfmRVY, LocalVar.PtfmRVZ,
        LocalVar.PtfmTAX, LocalVar.PtfmTAY, LocalVar.PtfmTAZ,
        LocalVar.PtfmRAX, LocalVar.PtfmRAY, LocalVar.PtfmRAZ,
        LocalVar.CC_DesiredL[0], LocalVar.CC_ActuatedL[0],
        LocalVar.CC_ActuatedDL[0], LocalVar.StC_Input[0],
        LocalVar.Flp_Angle[0], LocalVar.RootMyb_Last[0],
        (double)LocalVar.ACC_INFILE.size(), LocalVar.AWC_complexangle_re[0],
        LocalVar.TiltMean, LocalVar.YawMean, (double)LocalVar.ZMQ_ID,
        LocalVar.ZMQ_YawOffset, LocalVar.ZMQ_TorqueOffset,
        LocalVar.ZMQ_PitOffset[0], LocalVar.ZMQ_R_Speed,
        LocalVar.ZMQ_R_Torque, LocalVar.ZMQ_R_Pitch
    };
    const char* LocalVarOutStrings[nLocalVars] = {
        "iStatus", "AlreadyInitialized", "RestartWSE", "Time", "DT",
        "n_DT", "Time_Last", "VS_GenPwr", "GenSpeed", "RotSpeed",
        "NacHeading", "NacVane", "NacVaneF", "WindDir", "HorWindV",
        "HorWindV_F", "rootMOOP", "rootMOOPF", "BlPitch", "BlPitchCMeas",
        "Azimuth", "OL_Azimuth", "AzUnwrapped", "AzError", "GenTqAz",
        "AzBuffer", "NumBl", "FA_Acc_TT", "SS_Acc_TT", "FA_Acc_Nac",
        "NacIMU_FA_RAcc", "FA_AccHPF", "FA_AccHPFI", "FA_PitCom", "VS_RefSpd",
        "VS_RefSpd_TSR", "VS_RefSpd_TRA", "VS_RefSpd_RL", "PC_RefSpd",
        "PC_RefSpd_SS", "PC_RefSpd_PRC", "RotSpeedF", "GenSpeedF", "GenTq",
        "GenTqMeas", "GenArTq", "GenBrTq", "VS_KOmega2_GenTq",
        "VS_ConstPwr_GenTq", "IPC_PitComF", "PC_KP", "PC_KI", "PC_KD",
        "PC_TF", "PC_MaxPit", "PC_MinPit", "PC_PitComT", "PC_PitComT_Last",
        "BlPitchCMeasF", "PC_PitComT_IPC", "PC_PwrErr", "PC_SpdErr",
        "IPC_AxisTilt_1P", "IPC_AxisYaw_1P", "IPC_AxisTilt_2P",
        "IPC_AxisYaw_2P", "axisTilt_1P", "axisYaw_1P", "axisYawF_1P",
        "axisTilt_2P", "axisYaw_2P", "axisYawF_2P", "IPC_KI", "IPC_KP",
        "IPC_IntSat", "PC_State", "PitCom", "PitCom_SD", "PitComAct",
        "SS_DelOmegaF", "TestType", "Kp_Float", "VS_MaxTq", "VS_LastGenTrq",
        "VS_LastGenPwr", "VS_MechGenPwr", "VS_SpdErrAr", "VS_SpdErrBr",
        "VS_SpdErr", "VS_State", "VS_Rgn3Pitch", "WE_Vw", "WE_Vw_F",
        "WE_VwI", "WE_VwIdot", "WE_Op", "WE_Op_Last", "VS_LastGenTrqF",
        "PRC_WSE_F", "PRC_R_Speed", "PRC_R_Torque", "PRC_R_Pitch",
        "PRC_R_Total", "PRC_Min_Pitch", "PS_Min_Pitch", "OL_Index",
        "SU_Stage", "SU_LoadStageStartTime", "SU_RotSpeedF", "SD_Trigger",
        "SD_BlPitchF", "SD_NacVaneF", "SD_GenSpeedF", "SD_Stage",
        "SD_StageStartTime", "SD_MaxPitchRate", "SD_MaxTorqueRate",
        "GenTq_SD", "Fl_PitCom", "NACIMU_FA_AccF", "FA_AccF", "FA_Hist",
        "TRA_LastRefSpd", "VS_RefSpeed", "PtfmTDX", "PtfmTDY", "PtfmTDZ",
        "PtfmRDX", "PtfmRDY", "PtfmRDZ", "PtfmTVX", "PtfmTVY", "PtfmTVZ",
        "PtfmRVX", "PtfmRVY", "PtfmRVZ", "PtfmTAX", "PtfmTAY", "PtfmTAZ",
        "PtfmRAX", "PtfmRAY", "PtfmRAZ", "CC_DesiredL", "CC_ActuatedL",
        "CC_ActuatedDL", "StC_Input", "Flp_Angle", "RootMyb_Last",
        "ACC_INFILE_SIZE", "AWC_complexangle", "TiltMean", "YawMean",
        "ZMQ_ID", "ZMQ_YawOffset", "ZMQ_TorqueOffset", "ZMQ_PitOffset",
        "ZMQ_R_Speed", "ZMQ_R_Torque", "ZMQ_R_Pitch"
    };

    // --- Initialize debug files on first call ---
    if (LocalVar.iStatus == 0 || LocalVar.iStatus == -9) {
        if (CntrPar.LoggingLevel > 0) {
            std::string dbg_path = root + ".RO.dbg";
            dbg_file.open(dbg_path);
            dbg_file << " Generated on " << current_date() << " at "
                     << current_time() << " using ROSCO-" << ROSCO_VERSION << "\n";
            // Header: column names
            char hdr[32];
            snprintf(hdr, sizeof(hdr), "%20s", "Time");
            dbg_file << hdr;
            for (int i = 0; i < nDebugOuts; i++) {
                snprintf(hdr, sizeof(hdr), "     %20s", DebugOutStrings[i]);
                dbg_file << hdr;
            }
            dbg_file << "\n";
            // Header: units
            snprintf(hdr, sizeof(hdr), "%20s", "(sec)");
            dbg_file << hdr;
            for (int i = 0; i < nDebugOuts; i++) {
                snprintf(hdr, sizeof(hdr), "     %20s", DebugOutUnits[i]);
                dbg_file << hdr;
            }
            dbg_file << "\n";
        }

        if (CntrPar.LoggingLevel > 1) {
            std::string dbg2_path = root + ".RO.dbg2";
            dbg2_file.open(dbg2_path);
            dbg2_file << " Generated on " << current_date() << " at "
                      << current_time() << " using ROSCO-" << ROSCO_VERSION << "\n";
            char hdr[32];
            snprintf(hdr, sizeof(hdr), "%20s", "Time");
            dbg2_file << hdr;
            for (int i = 0; i < nLocalVars; i++) {
                snprintf(hdr, sizeof(hdr), "     %20s", LocalVarOutStrings[i]);
                dbg2_file << hdr;
            }
            dbg2_file << "\n";
            // Empty units line (Fortran writes blank units for dbg2)
            snprintf(hdr, sizeof(hdr), "%20s", "");
            dbg2_file << hdr;
            for (int i = 0; i < nLocalVars; i++) {
                snprintf(hdr, sizeof(hdr), "     %20s", "");
                dbg2_file << hdr;
            }
            dbg2_file << "\n";
        }

        if (CntrPar.LoggingLevel > 2) {
            // Build avrIndices: base 85 indices + CC_GroupIndex + StC_GroupIndex
            avr_indices.clear();
            int avrBaseLength = 85;
            for (int i = 1; i <= avrBaseLength; i++) {
                avr_indices.push_back(i);
            }
            if (CntrPar.CC_Mode > 0) {
                for (int i = 0; i < (int)CntrPar.CC_GroupIndex.size(); i++) {
                    avr_indices.push_back(CntrPar.CC_GroupIndex[i]);
                    avr_indices.push_back(CntrPar.CC_GroupIndex[i] + 1);
                }
            }
            if (CntrPar.StC_Mode > 0) {
                for (int i = 0; i < (int)CntrPar.StC_GroupIndex.size(); i++) {
                    avr_indices.push_back(CntrPar.StC_GroupIndex[i]);
                }
            }

            std::string dbg3_path = root + ".RO.dbg3";
            dbg3_file.open(dbg3_path);
            // 5 blank lines (Fortran WRITE(UnDb3,'(/////)'))
            dbg3_file << "\n\n\n\n\n\n";
            // Header with avrSWAP index labels
            char buf[32];
            snprintf(buf, sizeof(buf), "%21s", "LocalVar%Time ");
            dbg3_file << buf;
            for (size_t i = 0; i < avr_indices.size(); i++) {
                snprintf(buf, sizeof(buf), "            AvrSWAP(%4d)", avr_indices[i]);
                dbg3_file << buf;
            }
            dbg3_file << "\n";
            // Units line
            snprintf(buf, sizeof(buf), "%21s", "(s)");
            dbg3_file << buf;
            for (size_t i = 0; i < avr_indices.size(); i++) {
                snprintf(buf, sizeof(buf), "%22s", "(-)");
                dbg3_file << buf;
            }
            dbg3_file << "\n";
        }
    }

    // --- Console output every 10 seconds ---
    if (std::fmod(LocalVar.Time, 10.0) == 0.0) {
        printf("Generator speed: %6.1f RPM, Pitch angle: %5.1f deg, Power: %7.1f kW, Est. wind Speed: %5.1f m/s\n",
               LocalVar.GenSpeedF * RPS2RPM,
               LocalVar.BlPitch[0] * R2D,
               (double)avrSWAP[14] / 1000.0,  // Fortran avrSWAP(15) → C index 14
               LocalVar.WE_Vw);
    }

    // --- Clamp debug data ---
    for (int i = 0; i < nDebugOuts; i++) {
        DebugOutData[i] = clamp_debug(DebugOutData[i]);
    }
    for (int i = 0; i < nLocalVars; i++) {
        LocalVarOutData[i] = clamp_debug(LocalVarOutData[i]);
    }

    // --- Write debug data ---
    if (LocalVar.n_DT % CntrPar.n_DT_Out == 0) {
        if (CntrPar.LoggingLevel > 0 && LocalVar.iStatus >= 0) {
            write_debug_row(dbg_file, LocalVar.Time, DebugOutData, nDebugOuts);
        }
        if (CntrPar.LoggingLevel > 1 && LocalVar.iStatus >= 0) {
            write_debug_row(dbg2_file, LocalVar.Time, LocalVarOutData, nLocalVars);
        }
        if (CntrPar.LoggingLevel > 2 && LocalVar.iStatus >= 0) {
            // Write avrSWAP values at selected indices
            char buf[32];
            snprintf(buf, sizeof(buf), "%20.5f", LocalVar.Time);
            dbg3_file << buf;
            for (size_t i = 0; i < avr_indices.size(); i++) {
                double val = (double)avrSWAP[avr_indices[i] - 1]; // Fortran 1-indexed
                val = clamp_debug(val);
                dbg3_file << "     ";
                snprintf(buf, sizeof(buf), "%20.5E", val);
                dbg3_file << buf;
            }
            dbg3_file << "\n";
        }
    }

    // --- Close files on shutdown ---
    if (LocalVar.iStatus < 0) {
        if (dbg_file.is_open()) dbg_file.close();
        if (dbg2_file.is_open()) dbg2_file.close();
        if (dbg3_file.is_open()) dbg3_file.close();
    }
}
