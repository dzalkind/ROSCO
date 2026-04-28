#include "../include/vit_types.h"
#include "../include/rosco_types.hpp"
#include "../include/rosco_constants.h"
#include "../include/vit_translated.h"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstring>
#include <cstdint>
#include <cstdio>
#include <cctype>
#include <algorithm>
#include <cmath>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static void setError(errorvariables_t* ErrVar, const char* msg) {
    ErrVar->aviFAIL = -1;
    std::memset(ErrVar->ErrMsg, ' ', 1024);
    size_t len = std::strlen(msg);
    if (len > 1024) len = 1024;
    std::memcpy(ErrVar->ErrMsg, msg, len);
}

static std::string toUpper(const std::string& s) {
    std::string r = s;
    for (auto& c : r) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return r;
}

static std::string stripQuotes(const std::string& s) {
    if (s.size() >= 2 && s.front() == '"' && s.back() == '"') {
        return s.substr(1, s.size() - 2);
    }
    return s;
}

static bool pathIsRelative(const std::string& path) {
    if (path.empty()) return true;
    if (path.size() >= 2) {
        if (path[1] == ':' && (path[2] == '/' || path[2] == '\\')) return false;
    }
    if (path[0] == '/' || path[0] == '\\') return false;
    return true;
}

// ---------------------------------------------------------------------------
// DisconParser — reads a DISCON.IN parameter file
// ---------------------------------------------------------------------------

class DisconParser {
    std::vector<std::string> lines_;
    std::string filename_;
    errorvariables_t* errVar_;

    int findParam(const std::string& name) {
        std::string nameUp = toUpper(name);
        for (int i = 0; i < (int)lines_.size(); i++) {
            const std::string& line = lines_[i];
            size_t firstNonSpace = line.find_first_not_of(" \t");
            if (firstNonSpace == std::string::npos) continue;
            if (line[firstNonSpace] == '!') continue;

            size_t bangPos = line.find('!');
            if (bangPos == std::string::npos) continue;

            size_t wordStart = bangPos + 1;
            while (wordStart < line.size() && (line[wordStart] == ' ' || line[wordStart] == '\t'))
                wordStart++;
            size_t wordEnd = wordStart;
            while (wordEnd < line.size() && line[wordEnd] != ' ' && line[wordEnd] != '\t' &&
                   line[wordEnd] != '-' && line[wordEnd] != '!')
                wordEnd++;
            std::string paramName = line.substr(wordStart, wordEnd - wordStart);
            if (toUpper(paramName) == nameUp) {
                return i;
            }
        }
        return -1;
    }

    std::string getValuePart(int idx) {
        const std::string& line = lines_[idx];
        size_t bangPos = line.find('!');
        if (bangPos == std::string::npos) return line;
        return line.substr(0, bangPos);
    }

public:
    bool load(const std::string& filename, errorvariables_t* err) {
        filename_ = filename;
        errVar_ = err;
        lines_.clear();
        std::ifstream f(filename);
        if (!f.is_open()) {
            std::string msg = "Cannot open file: " + filename;
            setError(err, msg.c_str());
            return false;
        }
        std::string line;
        while (std::getline(f, line)) {
            lines_.push_back(line);
        }
        return true;
    }

    bool parseDbl(const char* name, double& val, bool allowDefault = true) {
        int idx = findParam(name);
        if (idx < 0) {
            if (allowDefault) { val = 0.0; return true; }
            std::string msg = std::string("Could not find parameter ") + name + " in " + filename_;
            setError(errVar_, msg.c_str());
            return false;
        }
        std::istringstream iss(getValuePart(idx));
        if (!(iss >> val)) {
            if (allowDefault) { val = 0.0; return true; }
            std::string msg = std::string("Error reading value for ") + name + " in " + filename_;
            setError(errVar_, msg.c_str());
            return false;
        }
        return true;
    }

    bool parseInt(const char* name, int& val, bool allowDefault = true) {
        int idx = findParam(name);
        if (idx < 0) {
            if (allowDefault) { val = 0; return true; }
            std::string msg = std::string("Could not find parameter ") + name + " in " + filename_;
            setError(errVar_, msg.c_str());
            return false;
        }
        std::istringstream iss(getValuePart(idx));
        if (!(iss >> val)) {
            if (allowDefault) { val = 0; return true; }
            std::string msg = std::string("Error reading value for ") + name + " in " + filename_;
            setError(errVar_, msg.c_str());
            return false;
        }
        return true;
    }

    bool parseStdStr(const char* name, std::string& val, bool allowDefault = true) {
        int idx = findParam(name);
        if (idx < 0) {
            if (allowDefault) { val = ""; return true; }
            std::string msg = std::string("Could not find parameter ") + name + " in " + filename_;
            setError(errVar_, msg.c_str());
            return false;
        }
        std::istringstream iss(getValuePart(idx));
        std::string sv;
        if (!(iss >> sv)) {
            if (allowDefault) { val = ""; return true; }
            std::string msg = std::string("Error reading value for ") + name + " in " + filename_;
            setError(errVar_, msg.c_str());
            return false;
        }
        val = stripQuotes(sv);
        if (val == "unused" || val == "UNUSED") val = "";
        return true;
    }

    bool parseDblParamArray(const char* name, ParamArray& arr, int n, bool allowDefault = true) {
        arr.clear();
        if (n <= 0) return true;
        arr.resize(n);
        int idx = findParam(name);
        if (idx < 0) {
            if (allowDefault) {
                for (int i = 0; i < n; i++) arr[i] = 0.0;
                return true;
            }
            std::string msg = std::string("Could not find parameter ") + name + " in " + filename_;
            setError(errVar_, msg.c_str());
            return false;
        }
        std::istringstream iss(getValuePart(idx));
        for (int i = 0; i < n; i++) {
            if (!(iss >> arr[i])) {
                if (allowDefault) {
                    for (int j = i; j < n; j++) arr[j] = 0.0;
                    return true;
                }
                std::string msg = std::string("Error reading array element ") +
                                  std::to_string(i + 1) + " for " + name + " in " + filename_;
                setError(errVar_, msg.c_str());
                return false;
            }
        }
        return true;
    }

    bool parseIntVec(const char* name, std::vector<int>& arr, int n, bool allowDefault = true) {
        arr.clear();
        if (n <= 0) return true;
        arr.resize(n, 0);
        int idx = findParam(name);
        if (idx < 0) {
            if (allowDefault) return true;
            std::string msg = std::string("Could not find parameter ") + name + " in " + filename_;
            setError(errVar_, msg.c_str());
            return false;
        }
        std::istringstream iss(getValuePart(idx));
        for (int i = 0; i < n; i++) {
            if (!(iss >> arr[i])) {
                if (allowDefault) return true;
                std::string msg = std::string("Error reading array element ") +
                                  std::to_string(i + 1) + " for " + name + " in " + filename_;
                setError(errVar_, msg.c_str());
                return false;
            }
        }
        return true;
    }
};

// ---------------------------------------------------------------------------
// unwrap declared in vit_translated.h
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Single-pass DISCON.IN reader — fills ControlParameters directly
// ---------------------------------------------------------------------------

void ReadControlParameterFileSub(
    ControlParameters& CntrPar,
    LocalVariables& LocalVar,
    const char* filename,
    const char* priPath,
    errorvariables_t* ErrVar)
{
    DisconParser parser;
    std::string filenameStr(filename);
    std::string priPathStr(priPath);

    if (!parser.load(filenameStr, ErrVar)) return;

    // ----------------------- Simulation Control --------------------------
    parser.parseInt("Echo",          CntrPar.Echo,          true);
    if (ErrVar->aviFAIL < 0) return;
    parser.parseInt("LoggingLevel",  CntrPar.LoggingLevel,  true);
    parser.parseDbl("DT_Out",        CntrPar.DT_Out,        true);
    parser.parseInt("Ext_Interface", CntrPar.Ext_Interface,  true);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Controller Flags --------------------------
    parser.parseInt("F_LPFType",       CntrPar.F_LPFType);
    parser.parseInt("IPC_ControlMode", CntrPar.IPC_ControlMode);
    parser.parseInt("VS_ControlMode",  CntrPar.VS_ControlMode);
    parser.parseInt("VS_ConstPower",   CntrPar.VS_ConstPower,   true);
    parser.parseInt("VS_FBP",          CntrPar.VS_FBP,          true);
    parser.parseInt("PC_ControlMode",  CntrPar.PC_ControlMode);
    parser.parseInt("Y_ControlMode",   CntrPar.Y_ControlMode);
    parser.parseInt("SS_Mode",         CntrPar.SS_Mode);
    parser.parseInt("PRC_Mode",        CntrPar.PRC_Mode);
    parser.parseInt("WE_Mode",         CntrPar.WE_Mode);
    parser.parseInt("PS_Mode",         CntrPar.PS_Mode);
    parser.parseInt("SU_Mode",         CntrPar.SU_Mode);
    parser.parseInt("SD_Mode",         CntrPar.SD_Mode);
    parser.parseInt("FL_Mode",         CntrPar.Fl_Mode);
    parser.parseInt("TD_Mode",         CntrPar.TD_Mode);
    parser.parseInt("TRA_Mode",        CntrPar.TRA_Mode);
    parser.parseInt("Flp_Mode",        CntrPar.Flp_Mode);
    parser.parseInt("OL_Mode",         CntrPar.OL_Mode);
    parser.parseInt("PA_Mode",         CntrPar.PA_Mode);
    parser.parseInt("PF_Mode",         CntrPar.PF_Mode);
    parser.parseInt("AWC_Mode",        CntrPar.AWC_Mode);
    parser.parseInt("Ext_Mode",        CntrPar.Ext_Mode);
    parser.parseInt("ZMQ_Mode",        CntrPar.ZMQ_Mode);
    parser.parseInt("CC_Mode",         CntrPar.CC_Mode);
    parser.parseInt("StC_Mode",        CntrPar.StC_Mode);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Filter Constants --------------------------
    parser.parseDbl("F_LPFCornerFreq",      CntrPar.F_LPFCornerFreq,      false);
    parser.parseDbl("F_LPFDamping",         CntrPar.F_LPFDamping,         CntrPar.F_LPFType == 1);
    parser.parseInt("F_NumNotchFilts",      CntrPar.F_NumNotchFilts,      true);
    parser.parseInt("F_GenSpdNotch_N",      CntrPar.F_GenSpdNotch_N,      CntrPar.F_NumNotchFilts == 0);
    parser.parseInt("F_TwrTopNotch_N",      CntrPar.F_TwrTopNotch_N,      CntrPar.F_NumNotchFilts == 0);
    parser.parseDbl("F_SSCornerFreq",       CntrPar.F_SSCornerFreq,       CntrPar.SS_Mode == 0);
    parser.parseDbl("F_WECornerFreq",       CntrPar.F_WECornerFreq,       false);
    parser.parseDbl("F_YawErr",             CntrPar.F_YawErr,             CntrPar.Y_ControlMode == 0);
    parser.parseDbl("F_FlHighPassFreq",     CntrPar.F_FlHighPassFreq,     CntrPar.Fl_Mode == 0);
    parser.parseDbl("F_VSRefSpdCornerFreq", CntrPar.F_VSRefSpdCornerFreq, CntrPar.VS_ControlMode < 2);
    if (ErrVar->aviFAIL < 0) return;

    // Filter arrays
    parser.parseDblParamArray("F_NotchFreqs",   CntrPar.F_NotchFreqs,   CntrPar.F_NumNotchFilts, CntrPar.F_NumNotchFilts == 0);
    parser.parseDblParamArray("F_NotchBetaNum", CntrPar.F_NotchBetaNum, CntrPar.F_NumNotchFilts, CntrPar.F_NumNotchFilts == 0);
    parser.parseDblParamArray("F_NotchBetaDen", CntrPar.F_NotchBetaDen, CntrPar.F_NumNotchFilts, CntrPar.F_NumNotchFilts == 0);
    parser.parseIntVec("F_GenSpdNotch_Ind", CntrPar.F_GenSpdNotch_Ind, CntrPar.F_GenSpdNotch_N, CntrPar.F_NumNotchFilts == 0);
    parser.parseIntVec("F_TwrTopNotch_Ind", CntrPar.F_TwrTopNotch_Ind, CntrPar.F_TwrTopNotch_N, CntrPar.F_NumNotchFilts == 0);
    parser.parseDblParamArray("F_FlCornerFreq",  CntrPar.F_FlCornerFreq,  2, CntrPar.Fl_Mode == 0);
    parser.parseDblParamArray("F_FlpCornerFreq", CntrPar.F_FlpCornerFreq, 2, CntrPar.Flp_Mode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Blade Pitch Control --------------------------
    parser.parseInt("PC_GS_n",      CntrPar.PC_GS_n,      CntrPar.PC_ControlMode == 0);
    parser.parseDbl("PC_MaxPit",    CntrPar.PC_MaxPit,    CntrPar.PC_ControlMode == 0);
    parser.parseDbl("PC_MinPit",    CntrPar.PC_MinPit,    CntrPar.PC_ControlMode == 0);
    parser.parseDbl("PC_MaxRat",    CntrPar.PC_MaxRat,    CntrPar.PC_ControlMode == 0);
    parser.parseDbl("PC_MinRat",    CntrPar.PC_MinRat,    CntrPar.PC_ControlMode == 0);
    parser.parseDbl("PC_RefSpd",    CntrPar.PC_RefSpd,    CntrPar.PC_ControlMode == 0);
    parser.parseDbl("PC_FinePit",   CntrPar.PC_FinePit,   CntrPar.PC_ControlMode == 0);
    parser.parseDbl("PC_Switch",    CntrPar.PC_Switch,    CntrPar.PC_ControlMode == 0);
    // Pitch gain-scheduled arrays
    parser.parseDblParamArray("PC_GS_angles", CntrPar.PC_GS_angles, CntrPar.PC_GS_n, CntrPar.PC_ControlMode == 0);
    parser.parseDblParamArray("PC_GS_KP",     CntrPar.PC_GS_KP,     CntrPar.PC_GS_n, CntrPar.PC_ControlMode == 0);
    parser.parseDblParamArray("PC_GS_KI",     CntrPar.PC_GS_KI,     CntrPar.PC_GS_n, CntrPar.PC_ControlMode == 0);
    parser.parseDblParamArray("PC_GS_KD",     CntrPar.PC_GS_KD,     CntrPar.PC_GS_n, CntrPar.PC_ControlMode == 0);
    parser.parseDblParamArray("PC_GS_TF",     CntrPar.PC_GS_TF,     CntrPar.PC_GS_n, CntrPar.PC_ControlMode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- IPC --------------------------
    parser.parseInt("IPC_SatMode",       CntrPar.IPC_SatMode,       CntrPar.IPC_ControlMode == 0);
    parser.parseDbl("IPC_IntSat",        CntrPar.IPC_IntSat,        CntrPar.IPC_ControlMode == 0);
    parser.parseDbl("IPC_CornerFreqAct", CntrPar.IPC_CornerFreqAct, CntrPar.IPC_ControlMode == 0);
    parser.parseDblParamArray("IPC_Vramp",     CntrPar.IPC_Vramp,     2, CntrPar.IPC_ControlMode == 0);
    parser.parseDblParamArray("IPC_KP",        CntrPar.IPC_KP,        2, CntrPar.IPC_ControlMode == 0);
    parser.parseDblParamArray("IPC_KI",        CntrPar.IPC_KI,        2, CntrPar.IPC_ControlMode == 0);
    parser.parseDblParamArray("IPC_aziOffset", CntrPar.IPC_aziOffset, 2, CntrPar.IPC_ControlMode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- VS Torque Control --------------------------
    parser.parseDbl("VS_GenEff",   CntrPar.VS_GenEff,   false);
    parser.parseDbl("VS_ArSatTq",  CntrPar.VS_ArSatTq,  CntrPar.VS_ControlMode != 1);
    parser.parseDbl("VS_MaxRat",   CntrPar.VS_MaxRat,   CntrPar.VS_ControlMode != 1);
    parser.parseDbl("VS_MaxTq",    CntrPar.VS_MaxTq,    false);
    parser.parseDbl("VS_MinTq",    CntrPar.VS_MinTq,    false);
    parser.parseDbl("VS_MinOMSpd", CntrPar.VS_MinOMSpd, true);
    parser.parseDbl("VS_Rgn2K",    CntrPar.VS_Rgn2K,    CntrPar.VS_ControlMode == 2);
    parser.parseDbl("VS_RtPwr",    CntrPar.VS_RtPwr,    false);
    parser.parseDbl("VS_RtTq",     CntrPar.VS_RtTq,     false);
    parser.parseDbl("VS_RefSpd",   CntrPar.VS_RefSpd,   false);
    parser.parseInt("VS_n",        CntrPar.VS_n,        false);
    parser.parseDbl("VS_TSRopt",   CntrPar.VS_TSRopt,   CntrPar.VS_ControlMode < 2);
    parser.parseDblParamArray("VS_KP", CntrPar.VS_KP, CntrPar.VS_n, false);
    parser.parseDblParamArray("VS_KI", CntrPar.VS_KI, CntrPar.VS_n, false);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Fixed-Pitch Region 3 --------------------------
    parser.parseInt("VS_FBP_n", CntrPar.VS_FBP_n, CntrPar.VS_FBP == 0);
    parser.parseDblParamArray("VS_FBP_U",     CntrPar.VS_FBP_U,     CntrPar.VS_FBP_n, CntrPar.VS_FBP == 0);
    parser.parseDblParamArray("VS_FBP_Omega", CntrPar.VS_FBP_Omega, CntrPar.VS_FBP_n, CntrPar.VS_FBP == 0);
    parser.parseDblParamArray("VS_FBP_Tau",   CntrPar.VS_FBP_Tau,   CntrPar.VS_FBP_n, CntrPar.VS_FBP == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Setpoint Smoother --------------------------
    parser.parseDbl("SS_VSGain", CntrPar.SS_VSGain, CntrPar.SS_Mode == 0);
    parser.parseDbl("SS_PCGain", CntrPar.SS_PCGain, CntrPar.SS_Mode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Power Reference --------------------------
    parser.parseInt("PRC_Comm",      CntrPar.PRC_Comm,      CntrPar.PRC_Mode != 1);
    parser.parseDbl("PRC_R_Torque",  CntrPar.PRC_R_Torque,  (CntrPar.PRC_Mode != 2) || (CntrPar.PRC_Comm != 0));
    parser.parseDbl("PRC_R_Speed",   CntrPar.PRC_R_Speed,   (CntrPar.PRC_Mode != 2) || (CntrPar.PRC_Comm != 0));
    parser.parseDbl("PRC_R_Pitch",   CntrPar.PRC_R_Pitch,   (CntrPar.PRC_Mode != 2) || (CntrPar.PRC_Comm != 0));
    parser.parseInt("PRC_Table_n",   CntrPar.PRC_Table_n,   (CntrPar.PRC_Mode != 2) || (CntrPar.PRC_R_Pitch == 1.0));
    parser.parseInt("PRC_n",         CntrPar.PRC_n,         CntrPar.PRC_Mode == 0);
    parser.parseDbl("PRC_LPF_Freq",  CntrPar.PRC_LPF_Freq,  CntrPar.PRC_Mode == 0);
    parser.parseDblParamArray("PRC_R_Table",     CntrPar.PRC_R_Table,     CntrPar.PRC_Table_n, (CntrPar.PRC_Mode != 2) || (CntrPar.PRC_R_Pitch == 1.0));
    parser.parseDblParamArray("PRC_Pitch_Table", CntrPar.PRC_Pitch_Table, CntrPar.PRC_Table_n, (CntrPar.PRC_Mode != 2) || (CntrPar.PRC_R_Pitch == 1.0));
    parser.parseDblParamArray("PRC_WindSpeeds",  CntrPar.PRC_WindSpeeds,  CntrPar.PRC_n, CntrPar.PRC_Mode == 0);
    parser.parseDblParamArray("PRC_GenSpeeds",   CntrPar.PRC_GenSpeeds,   CntrPar.PRC_n, CntrPar.PRC_Mode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Wind Speed Estimator --------------------------
    parser.parseDbl("WE_BladeRadius",  CntrPar.WE_BladeRadius,  false);
    parser.parseDbl("WE_Gamma",        CntrPar.WE_Gamma,        CntrPar.WE_Mode != 1);
    parser.parseDbl("WE_GearboxRatio", CntrPar.WE_GearboxRatio, false);
    parser.parseDbl("WE_Jtot",         CntrPar.WE_Jtot,         CntrPar.WE_Mode == 0);
    parser.parseDbl("WE_RhoAir",       CntrPar.WE_RhoAir,       CntrPar.WE_Mode != 2);
    parser.parseStdStr("PerfFileName", CntrPar.PerfFileName,    CntrPar.WE_Mode == 0);
    parser.parseInt("WE_FOPoles_N",    CntrPar.WE_FOPoles_N,    CntrPar.WE_Mode != 2);
    parser.parseIntVec("PerfTableSize",  CntrPar.PerfTableSize,   2, CntrPar.WE_Mode == 0);
    parser.parseDblParamArray("WE_FOPoles_v", CntrPar.WE_FOPoles_v, CntrPar.WE_FOPoles_N, CntrPar.WE_Mode != 2);
    parser.parseDblParamArray("WE_FOPoles",   CntrPar.WE_FOPoles,   CntrPar.WE_FOPoles_N, CntrPar.WE_Mode != 2);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Yaw Control --------------------------
    parser.parseDbl("Y_uSwitch",    CntrPar.Y_uSwitch,    CntrPar.Y_ControlMode == 0);
    parser.parseDbl("Y_Rate",       CntrPar.Y_Rate,       CntrPar.Y_ControlMode == 0);
    parser.parseDbl("Y_MErrSet",    CntrPar.Y_MErrSet,    CntrPar.Y_ControlMode == 0);
    parser.parseDbl("Y_IPC_IntSat", CntrPar.Y_IPC_IntSat, CntrPar.Y_ControlMode == 0);
    parser.parseDbl("Y_IPC_KP",     CntrPar.Y_IPC_KP,     CntrPar.Y_ControlMode == 0);
    parser.parseDbl("Y_IPC_KI",     CntrPar.Y_IPC_KI,     CntrPar.Y_ControlMode == 0);
    parser.parseDblParamArray("Y_ErrThresh", CntrPar.Y_ErrThresh, 2, CntrPar.Y_ControlMode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Tower Damper / TRA --------------------------
    parser.parseDbl("TRA_ExclSpeed",    CntrPar.TRA_ExclSpeed,    CntrPar.TRA_Mode == 0);
    parser.parseDbl("TRA_ExclBand",     CntrPar.TRA_ExclBand,     CntrPar.TRA_Mode == 0);
    parser.parseDbl("TRA_RateLimit",    CntrPar.TRA_RateLimit,    CntrPar.TRA_Mode == 0);
    parser.parseDbl("FA_KI",            CntrPar.FA_KI,            CntrPar.TD_Mode == 0);
    parser.parseDbl("FA_HPFCornerFreq", CntrPar.FA_HPFCornerFreq, CntrPar.TD_Mode == 0);
    parser.parseDbl("FA_IntSat",        CntrPar.FA_IntSat,        CntrPar.TD_Mode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Peak Shaving --------------------------
    parser.parseInt("PS_BldPitchMin_N", CntrPar.PS_BldPitchMin_N, CntrPar.PS_Mode == 0);
    parser.parseDblParamArray("PS_WindSpeeds",  CntrPar.PS_WindSpeeds,  CntrPar.PS_BldPitchMin_N, CntrPar.PS_Mode == 0);
    parser.parseDblParamArray("PS_BldPitchMin", CntrPar.PS_BldPitchMin, CntrPar.PS_BldPitchMin_N, CntrPar.PS_Mode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Startup --------------------------
    parser.parseDbl("SU_StartTime",            CntrPar.SU_StartTime,            CntrPar.SU_Mode == 0);
    parser.parseDbl("SU_FW_MinDuration",       CntrPar.SU_FW_MinDuration,       CntrPar.SU_Mode == 0);
    parser.parseDbl("SU_RotorSpeedThresh",     CntrPar.SU_RotorSpeedThresh,     CntrPar.SU_Mode == 0);
    parser.parseDbl("SU_RotorSpeedCornerFreq", CntrPar.SU_RotorSpeedCornerFreq, CntrPar.SU_Mode == 0);
    parser.parseInt("SU_LoadStages_N",         CntrPar.SU_LoadStages_N,         CntrPar.SU_Mode == 0);
    parser.parseDblParamArray("SU_LoadStages",       CntrPar.SU_LoadStages,       CntrPar.SU_LoadStages_N, CntrPar.SU_LoadStages_N == 0);
    parser.parseDblParamArray("SU_LoadRampDuration", CntrPar.SU_LoadRampDuration, CntrPar.SU_LoadStages_N, CntrPar.SU_LoadStages_N == 0);
    parser.parseDblParamArray("SU_LoadHoldDuration", CntrPar.SU_LoadHoldDuration, CntrPar.SU_LoadStages_N, CntrPar.SU_LoadStages_N == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Shutdown --------------------------
    parser.parseDbl("SD_TimeActivate",       CntrPar.SD_TimeActivate,       CntrPar.SD_Mode == 0);
    parser.parseInt("SD_EnablePitch",        CntrPar.SD_EnablePitch,        CntrPar.SD_Mode == 0);
    parser.parseInt("SD_EnableYawError",     CntrPar.SD_EnableYawError,     CntrPar.SD_Mode == 0);
    parser.parseInt("SD_EnableGenSpeed",     CntrPar.SD_EnableGenSpeed,     CntrPar.SD_Mode == 0);
    parser.parseInt("SD_EnableTime",         CntrPar.SD_EnableTime,         CntrPar.SD_Mode == 0);
    parser.parseDbl("SD_MaxPit",             CntrPar.SD_MaxPit,             CntrPar.SD_Mode == 0);
    parser.parseDbl("SD_PitchCornerFreq",    CntrPar.SD_PitchCornerFreq,    CntrPar.SD_Mode == 0);
    parser.parseDbl("SD_MaxYawError",        CntrPar.SD_MaxYawError,        CntrPar.SD_Mode == 0);
    parser.parseDbl("SD_YawErrorCornerFreq", CntrPar.SD_YawErrorCornerFreq, CntrPar.SD_Mode == 0);
    parser.parseDbl("SD_MaxGenSpd",          CntrPar.SD_MaxGenSpd,          CntrPar.SD_Mode == 0);
    parser.parseDbl("SD_GenSpdCornerFreq",   CntrPar.SD_GenSpdCornerFreq,   CntrPar.SD_Mode == 0);
    parser.parseDbl("SD_Time",               CntrPar.SD_Time,               CntrPar.SD_Mode == 0);
    parser.parseInt("SD_Method",             CntrPar.SD_Method,             CntrPar.SD_Mode == 0);
    parser.parseInt("SD_Stage_N",            CntrPar.SD_Stage_N,            CntrPar.SD_Mode == 0);
    parser.parseDblParamArray("SD_StageTime",     CntrPar.SD_StageTime,     CntrPar.SD_Stage_N, CntrPar.SD_Method != 1);
    parser.parseDblParamArray("SD_StagePitch",    CntrPar.SD_StagePitch,    CntrPar.SD_Stage_N, CntrPar.SD_Method != 2);
    parser.parseDblParamArray("SD_MaxTorqueRate", CntrPar.SD_MaxTorqueRate, CntrPar.SD_Stage_N, CntrPar.SD_Mode == 0);
    parser.parseDblParamArray("SD_MaxPitchRate",  CntrPar.SD_MaxPitchRate,  CntrPar.SD_Stage_N, CntrPar.SD_Mode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Floating --------------------------
    parser.parseInt("Fl_n", CntrPar.Fl_n, true);
    if (CntrPar.Fl_n == 0) CntrPar.Fl_n = 1;
    parser.parseDblParamArray("Fl_Kp", CntrPar.Fl_Kp, CntrPar.Fl_n, CntrPar.Fl_Mode == 0);
    parser.parseDblParamArray("Fl_U",  CntrPar.Fl_U,  CntrPar.Fl_n, CntrPar.Fl_n == 1);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Flaps --------------------------
    parser.parseDbl("Flp_Angle",  CntrPar.Flp_Angle,  CntrPar.Flp_Mode == 0);
    parser.parseDbl("Flp_Kp",     CntrPar.Flp_Kp,     CntrPar.Flp_Mode == 0);
    parser.parseDbl("Flp_Ki",     CntrPar.Flp_Ki,     CntrPar.Flp_Mode == 0);
    parser.parseDbl("Flp_MaxPit", CntrPar.Flp_MaxPit, CntrPar.Flp_Mode == 0);
    parser.parseDblParamArray("F_FlpCornerFreq", CntrPar.F_FlpCornerFreq, 2, CntrPar.Flp_Mode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Open Loop --------------------------
    parser.parseStdStr("OL_Filename",    CntrPar.OL_Filename,    CntrPar.OL_Mode == 0);
    parser.parseInt("OL_BP_Mode",        CntrPar.OL_BP_Mode,     CntrPar.OL_Mode == 0);
    parser.parseDbl("OL_BP_FiltFreq",    CntrPar.OL_BP_FiltFreq, CntrPar.OL_Mode == 0);
    parser.parseInt("Ind_Breakpoint",    CntrPar.Ind_Breakpoint,  true);
    parser.parseInt("Ind_GenTq",         CntrPar.Ind_GenTq,       true);
    parser.parseInt("Ind_YawRate",       CntrPar.Ind_YawRate,     true);
    parser.parseInt("Ind_Azimuth",       CntrPar.Ind_Azimuth,     CntrPar.OL_Mode != 2);
    parser.parseInt("Ind_R_Speed",       CntrPar.Ind_R_Speed,     CntrPar.OL_Mode != 2);
    parser.parseInt("Ind_R_Torque",      CntrPar.Ind_R_Torque,    CntrPar.OL_Mode != 2);
    parser.parseInt("Ind_R_Pitch",       CntrPar.Ind_R_Pitch,     CntrPar.OL_Mode != 2);
    parser.parseIntVec("Ind_BldPitch",   CntrPar.Ind_BldPitch,    3, true);
    parser.parseDblParamArray("RP_Gains", CntrPar.RP_Gains,       4, CntrPar.OL_Mode != 2);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Pitch Actuator --------------------------
    parser.parseDbl("PA_CornerFreq", CntrPar.PA_CornerFreq, CntrPar.PA_Mode == 0);
    parser.parseDbl("PA_Damping",    CntrPar.PA_Damping,    CntrPar.PA_Mode == 0);
    parser.parseDblParamArray("PF_Offsets",   CntrPar.PF_Offsets,   3, CntrPar.PF_Mode != 1);
    parser.parseDblParamArray("PF_TimeStuck", CntrPar.PF_TimeStuck, 3, CntrPar.PF_Mode != 2);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- AWC --------------------------
    parser.parseInt("AWC_NumModes",    CntrPar.AWC_NumModes,    CntrPar.AWC_Mode == 0);
    parser.parseDbl("AWC_phaseoffset", CntrPar.AWC_phaseoffset, CntrPar.AWC_Mode == 0);
    parser.parseIntVec("AWC_n",          CntrPar.AWC_n,          CntrPar.AWC_NumModes, CntrPar.AWC_Mode != 1);
    parser.parseIntVec("AWC_harmonic",   CntrPar.AWC_harmonic,   CntrPar.AWC_NumModes, CntrPar.AWC_Mode < 2);
    parser.parseDblParamArray("AWC_freq",       CntrPar.AWC_freq,       CntrPar.AWC_NumModes, CntrPar.AWC_Mode == 0);
    parser.parseDblParamArray("AWC_amp",        CntrPar.AWC_amp,        CntrPar.AWC_NumModes, CntrPar.AWC_Mode == 0);
    parser.parseDblParamArray("AWC_clockangle", CntrPar.AWC_clockangle, CntrPar.AWC_NumModes, CntrPar.AWC_Mode == 0);
    parser.parseDblParamArray("AWC_CntrGains",  CntrPar.AWC_CntrGains,  2,                    CntrPar.AWC_Mode < 3);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- External Control --------------------------
    parser.parseStdStr("DLL_FileName", CntrPar.DLL_FileName, CntrPar.Ext_Mode == 0);
    parser.parseStdStr("DLL_InFile",   CntrPar.DLL_InFile,   CntrPar.Ext_Mode == 0);
    parser.parseStdStr("DLL_ProcName", CntrPar.DLL_ProcName, CntrPar.Ext_Mode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- ZeroMQ --------------------------
    parser.parseInt("ZMQ_ID",           CntrPar.ZMQ_ID,           true);
    parser.parseStdStr("ZMQ_CommAddress", CntrPar.ZMQ_CommAddress, CntrPar.ZMQ_Mode == 0);
    parser.parseDbl("ZMQ_UpdatePeriod", CntrPar.ZMQ_UpdatePeriod, CntrPar.ZMQ_Mode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Cable Control --------------------------
    parser.parseInt("CC_Group_N", CntrPar.CC_Group_N, CntrPar.CC_Mode == 0);
    parser.parseDbl("CC_ActTau",  CntrPar.CC_ActTau,  CntrPar.CC_Mode == 0);
    parser.parseIntVec("CC_GroupIndex",    CntrPar.CC_GroupIndex,    CntrPar.CC_Group_N, CntrPar.CC_Mode == 0);
    parser.parseIntVec("Ind_CableControl", CntrPar.Ind_CableControl, CntrPar.CC_Group_N, CntrPar.CC_Mode != 2);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Structural Control --------------------------
    parser.parseInt("StC_Group_N", CntrPar.StC_Group_N, CntrPar.StC_Mode == 0);
    parser.parseIntVec("StC_GroupIndex",    CntrPar.StC_GroupIndex,    CntrPar.StC_Group_N, CntrPar.StC_Mode == 0);
    parser.parseIntVec("Ind_StructControl", CntrPar.Ind_StructControl, CntrPar.StC_Group_N, CntrPar.StC_Mode != 2);
    if (ErrVar->aviFAIL < 0) return;

    // ---------------------------------------------------------------
    // Computed Constants
    // ---------------------------------------------------------------

    if (CntrPar.DT_Out == 0.0) CntrPar.DT_Out = LocalVar.DT;
    CntrPar.n_DT_Out = static_cast<int>(std::round(CntrPar.DT_Out / LocalVar.DT));
    CntrPar.n_DT_ZMQ = static_cast<int>(std::round(CntrPar.ZMQ_UpdatePeriod / LocalVar.DT));

    // Path resolution for PerfFileName
    if (!CntrPar.PerfFileName.empty() && pathIsRelative(CntrPar.PerfFileName)) {
        CntrPar.PerfFileName = priPathStr + CntrPar.PerfFileName;
    }

    // Path resolution for OL_Filename
    if (!CntrPar.OL_Filename.empty() && pathIsRelative(CntrPar.OL_Filename)) {
        CntrPar.OL_Filename = priPathStr + CntrPar.OL_Filename;
    }

    // Y_Rate conversion from rad/s to deg/s
    CntrPar.Y_Rate = CntrPar.Y_Rate * R2D;

    // VS computed constants
    CntrPar.PC_RtTq99    = CntrPar.VS_RtTq * 0.99;
    CntrPar.VS_MinOMTq   = CntrPar.VS_Rgn2K * CntrPar.VS_MinOMSpd * CntrPar.VS_MinOMSpd;
    CntrPar.VS_MaxOMTq   = CntrPar.VS_Rgn2K * CntrPar.VS_RefSpd   * CntrPar.VS_RefSpd;

    // ---------------------------------------------------------------
    // Open Loop CSV Loading (if OL_Mode > 0)
    // ---------------------------------------------------------------
    if (CntrPar.OL_Mode > 0 && !CntrPar.OL_Filename.empty()) {

        // Print channel list
        std::string olString;
        if (!CntrPar.Ind_BldPitch.empty() && CntrPar.Ind_BldPitch[0] > 0) olString += " BldPitch1 ";
        if (!CntrPar.Ind_BldPitch.empty() && CntrPar.Ind_BldPitch[1] > 0) olString += " BldPitch2 ";
        if (!CntrPar.Ind_BldPitch.empty() && CntrPar.Ind_BldPitch[2] > 0) olString += " BldPitch3 ";
        if (CntrPar.Ind_GenTq > 0)                                          olString += " GenTq ";
        if (CntrPar.Ind_YawRate > 0)                                        olString += " YawRate ";
        if (CntrPar.Ind_Azimuth > 0 && CntrPar.OL_Mode == 2)               olString += " Azimuth ";
        if (CntrPar.Ind_R_Speed > 0  && CntrPar.OL_Mode == 1)              olString += " R_Speed ";
        if (CntrPar.Ind_R_Torque > 0 && CntrPar.OL_Mode == 1)              olString += " R_Torque ";
        if (CntrPar.Ind_R_Pitch > 0  && CntrPar.OL_Mode == 1)              olString += " R_Pitch ";
        int icc = 0;
        for (int idx : CntrPar.Ind_CableControl) {
            if (idx > 0) olString += " Cable" + std::to_string(++icc) + " ";
        }
        int istc = 0;
        for (int idx : CntrPar.Ind_StructControl) {
            if (idx > 0) olString += " StC" + std::to_string(++istc) + " ";
        }
        std::printf(" ROSCO: Implementing open loop control for%s\n", olString.c_str());
        if (CntrPar.OL_Mode == 2) {
            std::printf(" ROSCO: OL_Mode = 2 will change generator torque control for Azimuth tracking\n");
        }

        // Count nCols (number of columns in the OL file)
        int nCols = 1; // breakpoint column
        if (!CntrPar.Ind_BldPitch.empty()) {
            if (CntrPar.Ind_BldPitch[0] > 0) nCols++;
            if (CntrPar.Ind_BldPitch[1] > 0 &&
                !(CntrPar.Ind_BldPitch[1] == CntrPar.Ind_BldPitch[0] ||
                  CntrPar.Ind_BldPitch[1] == CntrPar.Ind_BldPitch[2])) nCols++;
            if (CntrPar.Ind_BldPitch[2] > 0 &&
                !(CntrPar.Ind_BldPitch[2] == CntrPar.Ind_BldPitch[0] ||
                  CntrPar.Ind_BldPitch[2] == CntrPar.Ind_BldPitch[1])) nCols++;
        }
        if (CntrPar.Ind_GenTq > 0) nCols++;
        if (CntrPar.Ind_YawRate > 0) nCols++;
        if (CntrPar.Ind_Azimuth > 0 && CntrPar.OL_Mode == 2) nCols++;
        if (CntrPar.Ind_R_Speed > 0  && CntrPar.OL_Mode == 1) nCols++;
        if (CntrPar.Ind_R_Torque > 0 && CntrPar.OL_Mode == 1) nCols++;
        if (CntrPar.Ind_R_Pitch > 0  && CntrPar.OL_Mode == 1) nCols++;
        for (int idx : CntrPar.Ind_CableControl)  { if (idx > 0) nCols++; }
        for (int idx : CntrPar.Ind_StructControl) { if (idx > 0) nCols++; }

        // Open and read OL file
        std::ifstream olFile(CntrPar.OL_Filename);
        if (!olFile.is_open()) {
            std::string msg = CntrPar.OL_Filename + " does not exist";
            setError(ErrVar, msg.c_str());
            return;
        }

        std::vector<std::string> dataLines;
        {
            std::string line;
            while (std::getline(olFile, line)) {
                if (line.empty()) continue;
                size_t first = line.find_first_not_of(" \t");
                if (first == std::string::npos) continue;
                char c = line[first];
                if (c == '!' || c == '#' || c == '%') continue;
                dataLines.push_back(line);
            }
        }

        int nRows = static_cast<int>(dataLines.size());
        if (nRows < 1) {
            setError(ErrVar, "Error: No data lines found in OL input file.");
            return;
        }

        // Allocate and fill OL_Channels (column-major: col * nRows + row)
        CntrPar.OL_Channels.resize(nRows * nCols);
        CntrPar.OL_Channels_rows = nRows;
        CntrPar.OL_Channels_cols = nCols;
        for (int i = 0; i < nRows; i++) {
            std::istringstream iss(dataLines[i]);
            for (int j = 0; j < nCols; j++) {
                double val = 0.0;
                iss >> val;
                CntrPar.OL_Channels[j * nRows + i] = val;
            }
        }

        // Extract 1D arrays from OL_Channels columns
        auto extractCol = [&](int ind, ParamArray& arr) {
            if (ind <= 0) return;
            int col = ind - 1;
            arr.resize(nRows);
            for (int i = 0; i < nRows; i++) {
                arr[i] = CntrPar.OL_Channels[col * nRows + i];
            }
        };

        if (CntrPar.Ind_Breakpoint > 0) extractCol(CntrPar.Ind_Breakpoint, CntrPar.OL_Breakpoints);
        if (!CntrPar.Ind_BldPitch.empty()) {
            extractCol(CntrPar.Ind_BldPitch[0], CntrPar.OL_BldPitch1);
            extractCol(CntrPar.Ind_BldPitch[1], CntrPar.OL_BldPitch2);
            extractCol(CntrPar.Ind_BldPitch[2], CntrPar.OL_BldPitch3);
        }
        extractCol(CntrPar.Ind_GenTq,   CntrPar.OL_GenTq);
        extractCol(CntrPar.Ind_YawRate,  CntrPar.OL_YawRate);
        extractCol(CntrPar.Ind_R_Speed,  CntrPar.OL_R_Speed);
        extractCol(CntrPar.Ind_R_Torque, CntrPar.OL_R_Torque);
        extractCol(CntrPar.Ind_R_Pitch,  CntrPar.OL_R_Pitch);

        if (CntrPar.Ind_Azimuth > 0) {
            extractCol(CntrPar.Ind_Azimuth, CntrPar.OL_Azimuth);
            unwrap(CntrPar.OL_Azimuth.data(), nRows, ErrVar, CntrPar.OL_Azimuth.data());
        }

        // OL_CableControl (2D, column-major)
        int nOlCables = 0;
        for (int idx : CntrPar.Ind_CableControl) { if (idx > 0) nOlCables++; }
        if (nOlCables > 0) {
            CntrPar.OL_CableControl.resize(nOlCables * nRows);
            CntrPar.OL_CableControl_rows = nOlCables;
            CntrPar.OL_CableControl_cols = nRows;
            int iOL = 0;
            for (int idx : CntrPar.Ind_CableControl) {
                if (idx > 0) {
                    int col = idx - 1;
                    for (int r = 0; r < nRows; r++) {
                        CntrPar.OL_CableControl[r * nOlCables + iOL] = CntrPar.OL_Channels[col * nRows + r];
                    }
                    iOL++;
                }
            }
        }

        // OL_StructControl (2D, column-major)
        int nOlStCs = 0;
        for (int idx : CntrPar.Ind_StructControl) { if (idx > 0) nOlStCs++; }
        if (nOlStCs > 0) {
            CntrPar.OL_StructControl.resize(nOlStCs * nRows);
            CntrPar.OL_StructControl_rows = nOlStCs;
            CntrPar.OL_StructControl_cols = nRows;
            int iOL = 0;
            for (int idx : CntrPar.Ind_StructControl) {
                if (idx > 0) {
                    int col = idx - 1;
                    for (int r = 0; r < nRows; r++) {
                        CntrPar.OL_StructControl[r * nOlStCs + iOL] = CntrPar.OL_Channels[col * nRows + r];
                    }
                    iOL++;
                }
            }
        }
    }

    // ---------------------------------------------------------------
    // Housekeeping
    // ---------------------------------------------------------------
    if (ErrVar->aviFAIL < 0) {
        std::string current(ErrVar->ErrMsg, 1024);
        size_t end = current.find_last_not_of(' ');
        if (end != std::string::npos) current = current.substr(0, end + 1);
        std::string prefixed = "ReadControlParameterFileSub:" + current;
        std::memset(ErrVar->ErrMsg, ' ', 1024);
        size_t len = std::min(prefixed.size(), (size_t)1024);
        std::memcpy(ErrVar->ErrMsg, prefixed.c_str(), len);
    }
}
