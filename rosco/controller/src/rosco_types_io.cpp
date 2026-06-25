// AUTO-GENERATED from rosco_types.yaml
// Do not edit manually — run write_registry.py to regenerate.
#include "include/rosco_types.hpp"
#include <toml++/toml.hpp>
#include <cstring>
#include <cstdio>

// Space-pad a C char array in Fortran style
static void set_fstr(char* dest, int maxLen, const std::string& src) {
    std::memset(dest, ' ', maxLen);
    size_t n = src.size() < (size_t)maxLen ? src.size() : (size_t)maxLen;
    std::memcpy(dest, src.c_str(), n);
}

bool ControlParameters::load_from_toml(const std::string& path, errorvariables_t* err) {
    toml::table tbl;
    try {
        tbl = toml::parse_file(path);
    } catch (const toml::parse_error& e) {
        err->aviFAIL = -1;
        std::snprintf(err->ErrMsg, 1024, "TOML parse error in %s: %s",
                      path.c_str(), e.description().data());
        return false;
    }

    ZMQ_ID = (int)tbl["ZMQ_ID"].value_or((int64_t)0);
    LoggingLevel = (int)tbl["LoggingLevel"].value_or((int64_t)0);
    Echo = (int)tbl["Echo"].value_or((int64_t)0);
    Ext_Interface = (int)tbl["Ext_Interface"].value_or((int64_t)0);
    DT_Out = tbl["DT_Out"].value_or(0.0);
    n_DT_Out = (int)tbl["n_DT_Out"].value_or((int64_t)0);
    n_DT_ZMQ = (int)tbl["n_DT_ZMQ"].value_or((int64_t)0);
    F_LPFType = (int)tbl["F_LPFType"].value_or((int64_t)0);
    F_LPFCornerFreq = tbl["F_LPFCornerFreq"].value_or(0.0);
    F_LPFDamping = tbl["F_LPFDamping"].value_or(0.0);
    F_NumNotchFilts = (int)tbl["F_NumNotchFilts"].value_or((int64_t)0);
    F_GenSpdNotch_N = (int)tbl["F_GenSpdNotch_N"].value_or((int64_t)0);
    if (auto* arr = tbl["F_GenSpdNotch_Ind"].as_array()) {
        F_GenSpdNotch_Ind.clear();
        for (auto& el : *arr) F_GenSpdNotch_Ind.push_back((int)el.value_or((int64_t)0));
    }
    F_TwrTopNotch_N = (int)tbl["F_TwrTopNotch_N"].value_or((int64_t)0);
    if (auto* arr = tbl["F_TwrTopNotch_Ind"].as_array()) {
        F_TwrTopNotch_Ind.clear();
        for (auto& el : *arr) F_TwrTopNotch_Ind.push_back((int)el.value_or((int64_t)0));
    }
    if (auto* arr = tbl["F_NotchFreqs"].as_array()) {
        F_NotchFreqs.clear();
        for (auto& el : *arr) F_NotchFreqs.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["F_NotchBetaNum"].as_array()) {
        F_NotchBetaNum.clear();
        for (auto& el : *arr) F_NotchBetaNum.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["F_NotchBetaDen"].as_array()) {
        F_NotchBetaDen.clear();
        for (auto& el : *arr) F_NotchBetaDen.push_back(el.value_or(0.0));
    }
    F_SSCornerFreq = tbl["F_SSCornerFreq"].value_or(0.0);
    F_WECornerFreq = tbl["F_WECornerFreq"].value_or(0.0);
    if (auto* arr = tbl["F_FlCornerFreq"].as_array()) {
        F_FlCornerFreq.clear();
        for (auto& el : *arr) F_FlCornerFreq.push_back(el.value_or(0.0));
    }
    F_FlHighPassFreq = tbl["F_FlHighPassFreq"].value_or(0.0);
    F_YawErr = tbl["F_YawErr"].value_or(0.0);
    if (auto* arr = tbl["F_FlpCornerFreq"].as_array()) {
        F_FlpCornerFreq.clear();
        for (auto& el : *arr) F_FlpCornerFreq.push_back(el.value_or(0.0));
    }
    F_VSRefSpdCornerFreq = tbl["F_VSRefSpdCornerFreq"].value_or(0.0);
    TRA_Mode = (int)tbl["TRA_Mode"].value_or((int64_t)0);
    TRA_ExclSpeed = tbl["TRA_ExclSpeed"].value_or(0.0);
    TRA_ExclBand = tbl["TRA_ExclBand"].value_or(0.0);
    TRA_RateLimit = tbl["TRA_RateLimit"].value_or(0.0);
    TD_Mode = (int)tbl["TD_Mode"].value_or((int64_t)0);
    FA_HPFCornerFreq = tbl["FA_HPFCornerFreq"].value_or(0.0);
    FA_IntSat = tbl["FA_IntSat"].value_or(0.0);
    FA_KI = tbl["FA_KI"].value_or(0.0);
    IPC_ControlMode = (int)tbl["IPC_ControlMode"].value_or((int64_t)0);
    if (auto* arr = tbl["IPC_Vramp"].as_array()) {
        IPC_Vramp.clear();
        for (auto& el : *arr) IPC_Vramp.push_back(el.value_or(0.0));
    }
    IPC_IntSat = tbl["IPC_IntSat"].value_or(0.0);
    IPC_SatMode = (int)tbl["IPC_SatMode"].value_or((int64_t)0);
    if (auto* arr = tbl["IPC_KP"].as_array()) {
        IPC_KP.clear();
        for (auto& el : *arr) IPC_KP.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["IPC_KI"].as_array()) {
        IPC_KI.clear();
        for (auto& el : *arr) IPC_KI.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["IPC_aziOffset"].as_array()) {
        IPC_aziOffset.clear();
        for (auto& el : *arr) IPC_aziOffset.push_back(el.value_or(0.0));
    }
    IPC_CornerFreqAct = tbl["IPC_CornerFreqAct"].value_or(0.0);
    PC_ControlMode = (int)tbl["PC_ControlMode"].value_or((int64_t)0);
    PC_GS_n = (int)tbl["PC_GS_n"].value_or((int64_t)0);
    if (auto* arr = tbl["PC_GS_angles"].as_array()) {
        PC_GS_angles.clear();
        for (auto& el : *arr) PC_GS_angles.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["PC_GS_KP"].as_array()) {
        PC_GS_KP.clear();
        for (auto& el : *arr) PC_GS_KP.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["PC_GS_KI"].as_array()) {
        PC_GS_KI.clear();
        for (auto& el : *arr) PC_GS_KI.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["PC_GS_KD"].as_array()) {
        PC_GS_KD.clear();
        for (auto& el : *arr) PC_GS_KD.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["PC_GS_TF"].as_array()) {
        PC_GS_TF.clear();
        for (auto& el : *arr) PC_GS_TF.push_back(el.value_or(0.0));
    }
    PC_MaxPit = tbl["PC_MaxPit"].value_or(0.0);
    PC_MinPit = tbl["PC_MinPit"].value_or(0.0);
    PC_MaxRat = tbl["PC_MaxRat"].value_or(0.0);
    PC_MinRat = tbl["PC_MinRat"].value_or(0.0);
    PC_RefSpd = tbl["PC_RefSpd"].value_or(0.0);
    PC_FinePit = tbl["PC_FinePit"].value_or(0.0);
    PC_Switch = tbl["PC_Switch"].value_or(0.0);
    VS_ControlMode = (int)tbl["VS_ControlMode"].value_or((int64_t)0);
    VS_ConstPower = (int)tbl["VS_ConstPower"].value_or((int64_t)0);
    VS_FBP = (int)tbl["VS_FBP"].value_or((int64_t)0);
    VS_GenEff = tbl["VS_GenEff"].value_or(0.0);
    VS_ArSatTq = tbl["VS_ArSatTq"].value_or(0.0);
    VS_MaxRat = tbl["VS_MaxRat"].value_or(0.0);
    VS_MaxTq = tbl["VS_MaxTq"].value_or(0.0);
    VS_MinTq = tbl["VS_MinTq"].value_or(0.0);
    VS_MinOMSpd = tbl["VS_MinOMSpd"].value_or(0.0);
    VS_Rgn2K = tbl["VS_Rgn2K"].value_or(0.0);
    VS_RtPwr = tbl["VS_RtPwr"].value_or(0.0);
    VS_RtTq = tbl["VS_RtTq"].value_or(0.0);
    VS_RefSpd = tbl["VS_RefSpd"].value_or(0.0);
    VS_n = (int)tbl["VS_n"].value_or((int64_t)0);
    if (auto* arr = tbl["VS_KP"].as_array()) {
        VS_KP.clear();
        for (auto& el : *arr) VS_KP.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["VS_KI"].as_array()) {
        VS_KI.clear();
        for (auto& el : *arr) VS_KI.push_back(el.value_or(0.0));
    }
    VS_TSRopt = tbl["VS_TSRopt"].value_or(0.0);
    VS_FBP_n = (int)tbl["VS_FBP_n"].value_or((int64_t)0);
    if (auto* arr = tbl["VS_FBP_U"].as_array()) {
        VS_FBP_U.clear();
        for (auto& el : *arr) VS_FBP_U.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["VS_FBP_Omega"].as_array()) {
        VS_FBP_Omega.clear();
        for (auto& el : *arr) VS_FBP_Omega.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["VS_FBP_Tau"].as_array()) {
        VS_FBP_Tau.clear();
        for (auto& el : *arr) VS_FBP_Tau.push_back(el.value_or(0.0));
    }
    SS_Mode = (int)tbl["SS_Mode"].value_or((int64_t)0);
    SS_VSGain = tbl["SS_VSGain"].value_or(0.0);
    SS_PCGain = tbl["SS_PCGain"].value_or(0.0);
    PRC_Mode = (int)tbl["PRC_Mode"].value_or((int64_t)0);
    PRC_Comm = (int)tbl["PRC_Comm"].value_or((int64_t)0);
    if (auto* arr = tbl["PRC_WindSpeeds"].as_array()) {
        PRC_WindSpeeds.clear();
        for (auto& el : *arr) PRC_WindSpeeds.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["PRC_GenSpeeds"].as_array()) {
        PRC_GenSpeeds.clear();
        for (auto& el : *arr) PRC_GenSpeeds.push_back(el.value_or(0.0));
    }
    PRC_n = (int)tbl["PRC_n"].value_or((int64_t)0);
    PRC_LPF_Freq = tbl["PRC_LPF_Freq"].value_or(0.0);
    PRC_R_Torque = tbl["PRC_R_Torque"].value_or(0.0);
    PRC_R_Speed = tbl["PRC_R_Speed"].value_or(0.0);
    PRC_R_Pitch = tbl["PRC_R_Pitch"].value_or(0.0);
    PRC_Table_n = (int)tbl["PRC_Table_n"].value_or((int64_t)0);
    if (auto* arr = tbl["PRC_Pitch_Table"].as_array()) {
        PRC_Pitch_Table.clear();
        for (auto& el : *arr) PRC_Pitch_Table.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["PRC_R_Table"].as_array()) {
        PRC_R_Table.clear();
        for (auto& el : *arr) PRC_R_Table.push_back(el.value_or(0.0));
    }
    WE_Mode = (int)tbl["WE_Mode"].value_or((int64_t)0);
    WE_BladeRadius = tbl["WE_BladeRadius"].value_or(0.0);
    WE_CP_n = (int)tbl["WE_CP_n"].value_or((int64_t)0);
    if (auto* arr = tbl["WE_CP"].as_array()) {
        WE_CP.clear();
        for (auto& el : *arr) WE_CP.push_back(el.value_or(0.0));
    }
    WE_Gamma = tbl["WE_Gamma"].value_or(0.0);
    WE_GearboxRatio = tbl["WE_GearboxRatio"].value_or(0.0);
    WE_Jtot = tbl["WE_Jtot"].value_or(0.0);
    WE_RhoAir = tbl["WE_RhoAir"].value_or(0.0);
    PerfFileName = tbl["PerfFileName"].value_or(std::string{});
    if (auto* arr = tbl["PerfTableSize"].as_array()) {
        PerfTableSize.clear();
        for (auto& el : *arr) PerfTableSize.push_back((int)el.value_or((int64_t)0));
    }
    WE_FOPoles_N = (int)tbl["WE_FOPoles_N"].value_or((int64_t)0);
    if (auto* arr = tbl["WE_FOPoles_v"].as_array()) {
        WE_FOPoles_v.clear();
        for (auto& el : *arr) WE_FOPoles_v.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["WE_FOPoles"].as_array()) {
        WE_FOPoles.clear();
        for (auto& el : *arr) WE_FOPoles.push_back(el.value_or(0.0));
    }
    Y_ControlMode = (int)tbl["Y_ControlMode"].value_or((int64_t)0);
    Y_uSwitch = tbl["Y_uSwitch"].value_or(0.0);
    if (auto* arr = tbl["Y_ErrThresh"].as_array()) {
        Y_ErrThresh.clear();
        for (auto& el : *arr) Y_ErrThresh.push_back(el.value_or(0.0));
    }
    Y_Rate = tbl["Y_Rate"].value_or(0.0);
    Y_MErrSet = tbl["Y_MErrSet"].value_or(0.0);
    Y_IPC_IntSat = tbl["Y_IPC_IntSat"].value_or(0.0);
    Y_IPC_KP = tbl["Y_IPC_KP"].value_or(0.0);
    Y_IPC_KI = tbl["Y_IPC_KI"].value_or(0.0);
    PS_Mode = (int)tbl["PS_Mode"].value_or((int64_t)0);
    PS_BldPitchMin_N = (int)tbl["PS_BldPitchMin_N"].value_or((int64_t)0);
    if (auto* arr = tbl["PS_WindSpeeds"].as_array()) {
        PS_WindSpeeds.clear();
        for (auto& el : *arr) PS_WindSpeeds.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["PS_BldPitchMin"].as_array()) {
        PS_BldPitchMin.clear();
        for (auto& el : *arr) PS_BldPitchMin.push_back(el.value_or(0.0));
    }
    SU_Mode = (int)tbl["SU_Mode"].value_or((int64_t)0);
    SU_StartTime = tbl["SU_StartTime"].value_or(0.0);
    SU_FW_MinDuration = tbl["SU_FW_MinDuration"].value_or(0.0);
    SU_RotorSpeedThresh = tbl["SU_RotorSpeedThresh"].value_or(0.0);
    SU_RotorSpeedCornerFreq = tbl["SU_RotorSpeedCornerFreq"].value_or(0.0);
    SU_LoadStages_N = (int)tbl["SU_LoadStages_N"].value_or((int64_t)0);
    if (auto* arr = tbl["SU_LoadStages"].as_array()) {
        SU_LoadStages.clear();
        for (auto& el : *arr) SU_LoadStages.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["SU_LoadRampDuration"].as_array()) {
        SU_LoadRampDuration.clear();
        for (auto& el : *arr) SU_LoadRampDuration.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["SU_LoadHoldDuration"].as_array()) {
        SU_LoadHoldDuration.clear();
        for (auto& el : *arr) SU_LoadHoldDuration.push_back(el.value_or(0.0));
    }
    SD_Mode = (int)tbl["SD_Mode"].value_or((int64_t)0);
    SD_TimeActivate = tbl["SD_TimeActivate"].value_or(0.0);
    SD_EnablePitch = (int)tbl["SD_EnablePitch"].value_or((int64_t)0);
    SD_EnableYawError = (int)tbl["SD_EnableYawError"].value_or((int64_t)0);
    SD_EnableGenSpeed = (int)tbl["SD_EnableGenSpeed"].value_or((int64_t)0);
    SD_EnableTime = (int)tbl["SD_EnableTime"].value_or((int64_t)0);
    SD_MaxPit = tbl["SD_MaxPit"].value_or(0.0);
    SD_PitchCornerFreq = tbl["SD_PitchCornerFreq"].value_or(0.0);
    SD_MaxYawError = tbl["SD_MaxYawError"].value_or(0.0);
    SD_YawErrorCornerFreq = tbl["SD_YawErrorCornerFreq"].value_or(0.0);
    SD_MaxGenSpd = tbl["SD_MaxGenSpd"].value_or(0.0);
    SD_GenSpdCornerFreq = tbl["SD_GenSpdCornerFreq"].value_or(0.0);
    SD_Time = tbl["SD_Time"].value_or(0.0);
    SD_Method = (int)tbl["SD_Method"].value_or((int64_t)0);
    if (auto* arr = tbl["SD_MaxTorqueRate"].as_array()) {
        SD_MaxTorqueRate.clear();
        for (auto& el : *arr) SD_MaxTorqueRate.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["SD_MaxPitchRate"].as_array()) {
        SD_MaxPitchRate.clear();
        for (auto& el : *arr) SD_MaxPitchRate.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["SD_StagePitch"].as_array()) {
        SD_StagePitch.clear();
        for (auto& el : *arr) SD_StagePitch.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["SD_StageTime"].as_array()) {
        SD_StageTime.clear();
        for (auto& el : *arr) SD_StageTime.push_back(el.value_or(0.0));
    }
    SD_Stage_N = (int)tbl["SD_Stage_N"].value_or((int64_t)0);
    Fl_Mode = (int)tbl["Fl_Mode"].value_or((int64_t)0);
    Fl_n = (int)tbl["Fl_n"].value_or((int64_t)0);
    if (auto* arr = tbl["Fl_Kp"].as_array()) {
        Fl_Kp.clear();
        for (auto& el : *arr) Fl_Kp.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["Fl_U"].as_array()) {
        Fl_U.clear();
        for (auto& el : *arr) Fl_U.push_back(el.value_or(0.0));
    }
    Flp_Mode = (int)tbl["Flp_Mode"].value_or((int64_t)0);
    Flp_Angle = tbl["Flp_Angle"].value_or(0.0);
    Flp_Kp = tbl["Flp_Kp"].value_or(0.0);
    Flp_Ki = tbl["Flp_Ki"].value_or(0.0);
    Flp_MaxPit = tbl["Flp_MaxPit"].value_or(0.0);
    OL_Filename = tbl["OL_Filename"].value_or(std::string{});
    OL_Mode = (int)tbl["OL_Mode"].value_or((int64_t)0);
    OL_BP_Mode = (int)tbl["OL_BP_Mode"].value_or((int64_t)0);
    OL_BP_FiltFreq = tbl["OL_BP_FiltFreq"].value_or(0.0);
    Ind_Breakpoint = (int)tbl["Ind_Breakpoint"].value_or((int64_t)0);
    if (auto* arr = tbl["Ind_BldPitch"].as_array()) {
        Ind_BldPitch.clear();
        for (auto& el : *arr) Ind_BldPitch.push_back((int)el.value_or((int64_t)0));
    }
    Ind_GenTq = (int)tbl["Ind_GenTq"].value_or((int64_t)0);
    Ind_YawRate = (int)tbl["Ind_YawRate"].value_or((int64_t)0);
    Ind_R_Speed = (int)tbl["Ind_R_Speed"].value_or((int64_t)0);
    Ind_R_Torque = (int)tbl["Ind_R_Torque"].value_or((int64_t)0);
    Ind_R_Pitch = (int)tbl["Ind_R_Pitch"].value_or((int64_t)0);
    Ind_Azimuth = (int)tbl["Ind_Azimuth"].value_or((int64_t)0);
    if (auto* arr = tbl["RP_Gains"].as_array()) {
        RP_Gains.clear();
        for (auto& el : *arr) RP_Gains.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["Ind_CableControl"].as_array()) {
        Ind_CableControl.clear();
        for (auto& el : *arr) Ind_CableControl.push_back((int)el.value_or((int64_t)0));
    }
    if (auto* arr = tbl["Ind_StructControl"].as_array()) {
        Ind_StructControl.clear();
        for (auto& el : *arr) Ind_StructControl.push_back((int)el.value_or((int64_t)0));
    }
    if (auto* arr = tbl["OL_Breakpoints"].as_array()) {
        OL_Breakpoints.clear();
        for (auto& el : *arr) OL_Breakpoints.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["OL_BldPitch1"].as_array()) {
        OL_BldPitch1.clear();
        for (auto& el : *arr) OL_BldPitch1.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["OL_BldPitch2"].as_array()) {
        OL_BldPitch2.clear();
        for (auto& el : *arr) OL_BldPitch2.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["OL_BldPitch3"].as_array()) {
        OL_BldPitch3.clear();
        for (auto& el : *arr) OL_BldPitch3.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["OL_CableControl"].as_array()) {
        OL_CableControl.clear();
        std::vector<std::vector<double>> _tmp_OL_CableControl;
        for (auto& _row : *arr) {
            if (auto* _ra = _row.as_array()) {
                std::vector<double> _rv;
                for (auto& el : *_ra) _rv.push_back(el.value_or(0.0));
                _tmp_OL_CableControl.push_back(std::move(_rv));
            }
        }
        OL_CableControl_rows = (int)_tmp_OL_CableControl.size();
        OL_CableControl_cols = OL_CableControl_rows > 0 ? (int)_tmp_OL_CableControl[0].size() : 0;
        OL_CableControl.resize((size_t)OL_CableControl_rows * OL_CableControl_cols);
        for (int _c = 0; _c < OL_CableControl_cols; ++_c)
            for (int _r = 0; _r < OL_CableControl_rows; ++_r)
                OL_CableControl[_c * OL_CableControl_rows + _r] = _tmp_OL_CableControl[_r][_c];
    }
    if (auto* arr = tbl["OL_StructControl"].as_array()) {
        OL_StructControl.clear();
        std::vector<std::vector<double>> _tmp_OL_StructControl;
        for (auto& _row : *arr) {
            if (auto* _ra = _row.as_array()) {
                std::vector<double> _rv;
                for (auto& el : *_ra) _rv.push_back(el.value_or(0.0));
                _tmp_OL_StructControl.push_back(std::move(_rv));
            }
        }
        OL_StructControl_rows = (int)_tmp_OL_StructControl.size();
        OL_StructControl_cols = OL_StructControl_rows > 0 ? (int)_tmp_OL_StructControl[0].size() : 0;
        OL_StructControl.resize((size_t)OL_StructControl_rows * OL_StructControl_cols);
        for (int _c = 0; _c < OL_StructControl_cols; ++_c)
            for (int _r = 0; _r < OL_StructControl_rows; ++_r)
                OL_StructControl[_c * OL_StructControl_rows + _r] = _tmp_OL_StructControl[_r][_c];
    }
    if (auto* arr = tbl["OL_GenTq"].as_array()) {
        OL_GenTq.clear();
        for (auto& el : *arr) OL_GenTq.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["OL_YawRate"].as_array()) {
        OL_YawRate.clear();
        for (auto& el : *arr) OL_YawRate.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["OL_Azimuth"].as_array()) {
        OL_Azimuth.clear();
        for (auto& el : *arr) OL_Azimuth.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["OL_R_Speed"].as_array()) {
        OL_R_Speed.clear();
        for (auto& el : *arr) OL_R_Speed.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["OL_R_Torque"].as_array()) {
        OL_R_Torque.clear();
        for (auto& el : *arr) OL_R_Torque.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["OL_R_Pitch"].as_array()) {
        OL_R_Pitch.clear();
        for (auto& el : *arr) OL_R_Pitch.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["OL_Channels"].as_array()) {
        OL_Channels.clear();
        std::vector<std::vector<double>> _tmp_OL_Channels;
        for (auto& _row : *arr) {
            if (auto* _ra = _row.as_array()) {
                std::vector<double> _rv;
                for (auto& el : *_ra) _rv.push_back(el.value_or(0.0));
                _tmp_OL_Channels.push_back(std::move(_rv));
            }
        }
        OL_Channels_rows = (int)_tmp_OL_Channels.size();
        OL_Channels_cols = OL_Channels_rows > 0 ? (int)_tmp_OL_Channels[0].size() : 0;
        OL_Channels.resize((size_t)OL_Channels_rows * OL_Channels_cols);
        for (int _c = 0; _c < OL_Channels_cols; ++_c)
            for (int _r = 0; _r < OL_Channels_rows; ++_r)
                OL_Channels[_c * OL_Channels_rows + _r] = _tmp_OL_Channels[_r][_c];
    }
    PA_Mode = (int)tbl["PA_Mode"].value_or((int64_t)0);
    PA_CornerFreq = tbl["PA_CornerFreq"].value_or(0.0);
    PA_Damping = tbl["PA_Damping"].value_or(0.0);
    AWC_Mode = (int)tbl["AWC_Mode"].value_or((int64_t)0);
    AWC_NumModes = (int)tbl["AWC_NumModes"].value_or((int64_t)0);
    if (auto* arr = tbl["AWC_n"].as_array()) {
        AWC_n.clear();
        for (auto& el : *arr) AWC_n.push_back((int)el.value_or((int64_t)0));
    }
    if (auto* arr = tbl["AWC_harmonic"].as_array()) {
        AWC_harmonic.clear();
        for (auto& el : *arr) AWC_harmonic.push_back((int)el.value_or((int64_t)0));
    }
    if (auto* arr = tbl["AWC_freq"].as_array()) {
        AWC_freq.clear();
        for (auto& el : *arr) AWC_freq.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["AWC_amp"].as_array()) {
        AWC_amp.clear();
        for (auto& el : *arr) AWC_amp.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["AWC_clockangle"].as_array()) {
        AWC_clockangle.clear();
        for (auto& el : *arr) AWC_clockangle.push_back(el.value_or(0.0));
    }
    AWC_phaseoffset = tbl["AWC_phaseoffset"].value_or(0.0);
    if (auto* arr = tbl["AWC_CntrGains"].as_array()) {
        AWC_CntrGains.clear();
        for (auto& el : *arr) AWC_CntrGains.push_back(el.value_or(0.0));
    }
    PF_Mode = (int)tbl["PF_Mode"].value_or((int64_t)0);
    if (auto* arr = tbl["PF_Offsets"].as_array()) {
        PF_Offsets.clear();
        for (auto& el : *arr) PF_Offsets.push_back(el.value_or(0.0));
    }
    if (auto* arr = tbl["PF_TimeStuck"].as_array()) {
        PF_TimeStuck.clear();
        for (auto& el : *arr) PF_TimeStuck.push_back(el.value_or(0.0));
    }
    Ext_Mode = (int)tbl["Ext_Mode"].value_or((int64_t)0);
    DLL_FileName = tbl["DLL_FileName"].value_or(std::string{});
    DLL_InFile = tbl["DLL_InFile"].value_or(std::string{});
    DLL_ProcName = tbl["DLL_ProcName"].value_or(std::string{});
    ZMQ_Mode = (int)tbl["ZMQ_Mode"].value_or((int64_t)0);
    ZMQ_CommAddress = tbl["ZMQ_CommAddress"].value_or(std::string{});
    ZMQ_UpdatePeriod = tbl["ZMQ_UpdatePeriod"].value_or(0.0);
    CC_Mode = (int)tbl["CC_Mode"].value_or((int64_t)0);
    CC_Group_N = (int)tbl["CC_Group_N"].value_or((int64_t)0);
    CC_ActTau = tbl["CC_ActTau"].value_or(0.0);
    if (auto* arr = tbl["CC_GroupIndex"].as_array()) {
        CC_GroupIndex.clear();
        for (auto& el : *arr) CC_GroupIndex.push_back((int)el.value_or((int64_t)0));
    }
    StC_Mode = (int)tbl["StC_Mode"].value_or((int64_t)0);
    StC_Group_N = (int)tbl["StC_Group_N"].value_or((int64_t)0);
    if (auto* arr = tbl["StC_GroupIndex"].as_array()) {
        StC_GroupIndex.clear();
        for (auto& el : *arr) StC_GroupIndex.push_back((int)el.value_or((int64_t)0));
    }
    PC_RtTq99 = tbl["PC_RtTq99"].value_or(0.0);
    VS_MaxOMTq = tbl["VS_MaxOMTq"].value_or(0.0);
    VS_MinOMTq = tbl["VS_MinOMTq"].value_or(0.0);

    return err->aviFAIL >= 0;
}

void ControlParameters::populate_view(controlparameters_view_t* v) const {
    v->ZMQ_ID = ZMQ_ID;
    v->LoggingLevel = LoggingLevel;
    v->Echo = Echo;
    v->Ext_Interface = Ext_Interface;
    v->DT_Out = DT_Out;
    v->n_DT_Out = n_DT_Out;
    v->n_DT_ZMQ = n_DT_ZMQ;
    v->F_LPFType = F_LPFType;
    v->F_LPFCornerFreq = F_LPFCornerFreq;
    v->F_LPFDamping = F_LPFDamping;
    v->F_NumNotchFilts = F_NumNotchFilts;
    v->F_GenSpdNotch_N = F_GenSpdNotch_N;
    v->F_GenSpdNotch_Ind = F_GenSpdNotch_Ind.empty() ? nullptr : const_cast<int*>(F_GenSpdNotch_Ind.data());
    v->n_F_GenSpdNotch_Ind = (int32_t)F_GenSpdNotch_Ind.size();
    v->F_TwrTopNotch_N = F_TwrTopNotch_N;
    v->F_TwrTopNotch_Ind = F_TwrTopNotch_Ind.empty() ? nullptr : const_cast<int*>(F_TwrTopNotch_Ind.data());
    v->n_F_TwrTopNotch_Ind = (int32_t)F_TwrTopNotch_Ind.size();
    v->F_NotchFreqs = F_NotchFreqs.empty() ? nullptr : const_cast<double*>(F_NotchFreqs.data());
    v->n_F_NotchFreqs = (int32_t)F_NotchFreqs.size();
    v->F_NotchBetaNum = F_NotchBetaNum.empty() ? nullptr : const_cast<double*>(F_NotchBetaNum.data());
    v->n_F_NotchBetaNum = (int32_t)F_NotchBetaNum.size();
    v->F_NotchBetaDen = F_NotchBetaDen.empty() ? nullptr : const_cast<double*>(F_NotchBetaDen.data());
    v->n_F_NotchBetaDen = (int32_t)F_NotchBetaDen.size();
    v->F_SSCornerFreq = F_SSCornerFreq;
    v->F_WECornerFreq = F_WECornerFreq;
    v->F_FlCornerFreq = F_FlCornerFreq.empty() ? nullptr : const_cast<double*>(F_FlCornerFreq.data());
    v->n_F_FlCornerFreq = (int32_t)F_FlCornerFreq.size();
    v->F_FlHighPassFreq = F_FlHighPassFreq;
    v->F_YawErr = F_YawErr;
    v->F_FlpCornerFreq = F_FlpCornerFreq.empty() ? nullptr : const_cast<double*>(F_FlpCornerFreq.data());
    v->n_F_FlpCornerFreq = (int32_t)F_FlpCornerFreq.size();
    v->F_VSRefSpdCornerFreq = F_VSRefSpdCornerFreq;
    v->TRA_Mode = TRA_Mode;
    v->TRA_ExclSpeed = TRA_ExclSpeed;
    v->TRA_ExclBand = TRA_ExclBand;
    v->TRA_RateLimit = TRA_RateLimit;
    v->TD_Mode = TD_Mode;
    v->FA_HPFCornerFreq = FA_HPFCornerFreq;
    v->FA_IntSat = FA_IntSat;
    v->FA_KI = FA_KI;
    v->IPC_ControlMode = IPC_ControlMode;
    v->IPC_Vramp = IPC_Vramp.empty() ? nullptr : const_cast<double*>(IPC_Vramp.data());
    v->n_IPC_Vramp = (int32_t)IPC_Vramp.size();
    v->IPC_IntSat = IPC_IntSat;
    v->IPC_SatMode = IPC_SatMode;
    v->IPC_KP = IPC_KP.empty() ? nullptr : const_cast<double*>(IPC_KP.data());
    v->n_IPC_KP = (int32_t)IPC_KP.size();
    v->IPC_KI = IPC_KI.empty() ? nullptr : const_cast<double*>(IPC_KI.data());
    v->n_IPC_KI = (int32_t)IPC_KI.size();
    v->IPC_aziOffset = IPC_aziOffset.empty() ? nullptr : const_cast<double*>(IPC_aziOffset.data());
    v->n_IPC_aziOffset = (int32_t)IPC_aziOffset.size();
    v->IPC_CornerFreqAct = IPC_CornerFreqAct;
    v->PC_ControlMode = PC_ControlMode;
    v->PC_GS_n = PC_GS_n;
    v->PC_GS_angles = PC_GS_angles.empty() ? nullptr : const_cast<double*>(PC_GS_angles.data());
    v->n_PC_GS_angles = (int32_t)PC_GS_angles.size();
    v->PC_GS_KP = PC_GS_KP.empty() ? nullptr : const_cast<double*>(PC_GS_KP.data());
    v->n_PC_GS_KP = (int32_t)PC_GS_KP.size();
    v->PC_GS_KI = PC_GS_KI.empty() ? nullptr : const_cast<double*>(PC_GS_KI.data());
    v->n_PC_GS_KI = (int32_t)PC_GS_KI.size();
    v->PC_GS_KD = PC_GS_KD.empty() ? nullptr : const_cast<double*>(PC_GS_KD.data());
    v->n_PC_GS_KD = (int32_t)PC_GS_KD.size();
    v->PC_GS_TF = PC_GS_TF.empty() ? nullptr : const_cast<double*>(PC_GS_TF.data());
    v->n_PC_GS_TF = (int32_t)PC_GS_TF.size();
    v->PC_MaxPit = PC_MaxPit;
    v->PC_MinPit = PC_MinPit;
    v->PC_MaxRat = PC_MaxRat;
    v->PC_MinRat = PC_MinRat;
    v->PC_RefSpd = PC_RefSpd;
    v->PC_FinePit = PC_FinePit;
    v->PC_Switch = PC_Switch;
    v->VS_ControlMode = VS_ControlMode;
    v->VS_ConstPower = VS_ConstPower;
    v->VS_FBP = VS_FBP;
    v->VS_GenEff = VS_GenEff;
    v->VS_ArSatTq = VS_ArSatTq;
    v->VS_MaxRat = VS_MaxRat;
    v->VS_MaxTq = VS_MaxTq;
    v->VS_MinTq = VS_MinTq;
    v->VS_MinOMSpd = VS_MinOMSpd;
    v->VS_Rgn2K = VS_Rgn2K;
    v->VS_RtPwr = VS_RtPwr;
    v->VS_RtTq = VS_RtTq;
    v->VS_RefSpd = VS_RefSpd;
    v->VS_n = VS_n;
    v->VS_KP = VS_KP.empty() ? nullptr : const_cast<double*>(VS_KP.data());
    v->n_VS_KP = (int32_t)VS_KP.size();
    v->VS_KI = VS_KI.empty() ? nullptr : const_cast<double*>(VS_KI.data());
    v->n_VS_KI = (int32_t)VS_KI.size();
    v->VS_TSRopt = VS_TSRopt;
    v->VS_FBP_n = VS_FBP_n;
    v->VS_FBP_U = VS_FBP_U.empty() ? nullptr : const_cast<double*>(VS_FBP_U.data());
    v->n_VS_FBP_U = (int32_t)VS_FBP_U.size();
    v->VS_FBP_Omega = VS_FBP_Omega.empty() ? nullptr : const_cast<double*>(VS_FBP_Omega.data());
    v->n_VS_FBP_Omega = (int32_t)VS_FBP_Omega.size();
    v->VS_FBP_Tau = VS_FBP_Tau.empty() ? nullptr : const_cast<double*>(VS_FBP_Tau.data());
    v->n_VS_FBP_Tau = (int32_t)VS_FBP_Tau.size();
    v->SS_Mode = SS_Mode;
    v->SS_VSGain = SS_VSGain;
    v->SS_PCGain = SS_PCGain;
    v->PRC_Mode = PRC_Mode;
    v->PRC_Comm = PRC_Comm;
    v->PRC_WindSpeeds = PRC_WindSpeeds.empty() ? nullptr : const_cast<double*>(PRC_WindSpeeds.data());
    v->n_PRC_WindSpeeds = (int32_t)PRC_WindSpeeds.size();
    v->PRC_GenSpeeds = PRC_GenSpeeds.empty() ? nullptr : const_cast<double*>(PRC_GenSpeeds.data());
    v->n_PRC_GenSpeeds = (int32_t)PRC_GenSpeeds.size();
    v->PRC_n = PRC_n;
    v->PRC_LPF_Freq = PRC_LPF_Freq;
    v->PRC_R_Torque = PRC_R_Torque;
    v->PRC_R_Speed = PRC_R_Speed;
    v->PRC_R_Pitch = PRC_R_Pitch;
    v->PRC_Table_n = PRC_Table_n;
    v->PRC_Pitch_Table = PRC_Pitch_Table.empty() ? nullptr : const_cast<double*>(PRC_Pitch_Table.data());
    v->n_PRC_Pitch_Table = (int32_t)PRC_Pitch_Table.size();
    v->PRC_R_Table = PRC_R_Table.empty() ? nullptr : const_cast<double*>(PRC_R_Table.data());
    v->n_PRC_R_Table = (int32_t)PRC_R_Table.size();
    v->WE_Mode = WE_Mode;
    v->WE_BladeRadius = WE_BladeRadius;
    v->WE_CP_n = WE_CP_n;
    v->WE_CP = WE_CP.empty() ? nullptr : const_cast<double*>(WE_CP.data());
    v->n_WE_CP = (int32_t)WE_CP.size();
    v->WE_Gamma = WE_Gamma;
    v->WE_GearboxRatio = WE_GearboxRatio;
    v->WE_Jtot = WE_Jtot;
    v->WE_RhoAir = WE_RhoAir;
    set_fstr(v->PerfFileName, 1024, PerfFileName);
    v->PerfTableSize = PerfTableSize.empty() ? nullptr : const_cast<int*>(PerfTableSize.data());
    v->n_PerfTableSize = (int32_t)PerfTableSize.size();
    v->WE_FOPoles_N = WE_FOPoles_N;
    v->WE_FOPoles_v = WE_FOPoles_v.empty() ? nullptr : const_cast<double*>(WE_FOPoles_v.data());
    v->n_WE_FOPoles_v = (int32_t)WE_FOPoles_v.size();
    v->WE_FOPoles = WE_FOPoles.empty() ? nullptr : const_cast<double*>(WE_FOPoles.data());
    v->n_WE_FOPoles = (int32_t)WE_FOPoles.size();
    v->Y_ControlMode = Y_ControlMode;
    v->Y_uSwitch = Y_uSwitch;
    v->Y_ErrThresh = Y_ErrThresh.empty() ? nullptr : const_cast<double*>(Y_ErrThresh.data());
    v->n_Y_ErrThresh = (int32_t)Y_ErrThresh.size();
    v->Y_Rate = Y_Rate;
    v->Y_MErrSet = Y_MErrSet;
    v->Y_IPC_IntSat = Y_IPC_IntSat;
    v->Y_IPC_KP = Y_IPC_KP;
    v->Y_IPC_KI = Y_IPC_KI;
    v->PS_Mode = PS_Mode;
    v->PS_BldPitchMin_N = PS_BldPitchMin_N;
    v->PS_WindSpeeds = PS_WindSpeeds.empty() ? nullptr : const_cast<double*>(PS_WindSpeeds.data());
    v->n_PS_WindSpeeds = (int32_t)PS_WindSpeeds.size();
    v->PS_BldPitchMin = PS_BldPitchMin.empty() ? nullptr : const_cast<double*>(PS_BldPitchMin.data());
    v->n_PS_BldPitchMin = (int32_t)PS_BldPitchMin.size();
    v->SU_Mode = SU_Mode;
    v->SU_StartTime = SU_StartTime;
    v->SU_FW_MinDuration = SU_FW_MinDuration;
    v->SU_RotorSpeedThresh = SU_RotorSpeedThresh;
    v->SU_RotorSpeedCornerFreq = SU_RotorSpeedCornerFreq;
    v->SU_LoadStages_N = SU_LoadStages_N;
    v->SU_LoadStages = SU_LoadStages.empty() ? nullptr : const_cast<double*>(SU_LoadStages.data());
    v->n_SU_LoadStages = (int32_t)SU_LoadStages.size();
    v->SU_LoadRampDuration = SU_LoadRampDuration.empty() ? nullptr : const_cast<double*>(SU_LoadRampDuration.data());
    v->n_SU_LoadRampDuration = (int32_t)SU_LoadRampDuration.size();
    v->SU_LoadHoldDuration = SU_LoadHoldDuration.empty() ? nullptr : const_cast<double*>(SU_LoadHoldDuration.data());
    v->n_SU_LoadHoldDuration = (int32_t)SU_LoadHoldDuration.size();
    v->SD_Mode = SD_Mode;
    v->SD_TimeActivate = SD_TimeActivate;
    v->SD_EnablePitch = SD_EnablePitch;
    v->SD_EnableYawError = SD_EnableYawError;
    v->SD_EnableGenSpeed = SD_EnableGenSpeed;
    v->SD_EnableTime = SD_EnableTime;
    v->SD_MaxPit = SD_MaxPit;
    v->SD_PitchCornerFreq = SD_PitchCornerFreq;
    v->SD_MaxYawError = SD_MaxYawError;
    v->SD_YawErrorCornerFreq = SD_YawErrorCornerFreq;
    v->SD_MaxGenSpd = SD_MaxGenSpd;
    v->SD_GenSpdCornerFreq = SD_GenSpdCornerFreq;
    v->SD_Time = SD_Time;
    v->SD_Method = SD_Method;
    v->SD_MaxTorqueRate = SD_MaxTorqueRate.empty() ? nullptr : const_cast<double*>(SD_MaxTorqueRate.data());
    v->n_SD_MaxTorqueRate = (int32_t)SD_MaxTorqueRate.size();
    v->SD_MaxPitchRate = SD_MaxPitchRate.empty() ? nullptr : const_cast<double*>(SD_MaxPitchRate.data());
    v->n_SD_MaxPitchRate = (int32_t)SD_MaxPitchRate.size();
    v->SD_StagePitch = SD_StagePitch.empty() ? nullptr : const_cast<double*>(SD_StagePitch.data());
    v->n_SD_StagePitch = (int32_t)SD_StagePitch.size();
    v->SD_StageTime = SD_StageTime.empty() ? nullptr : const_cast<double*>(SD_StageTime.data());
    v->n_SD_StageTime = (int32_t)SD_StageTime.size();
    v->SD_Stage_N = SD_Stage_N;
    v->Fl_Mode = Fl_Mode;
    v->Fl_n = Fl_n;
    v->Fl_Kp = Fl_Kp.empty() ? nullptr : const_cast<double*>(Fl_Kp.data());
    v->n_Fl_Kp = (int32_t)Fl_Kp.size();
    v->Fl_U = Fl_U.empty() ? nullptr : const_cast<double*>(Fl_U.data());
    v->n_Fl_U = (int32_t)Fl_U.size();
    v->Flp_Mode = Flp_Mode;
    v->Flp_Angle = Flp_Angle;
    v->Flp_Kp = Flp_Kp;
    v->Flp_Ki = Flp_Ki;
    v->Flp_MaxPit = Flp_MaxPit;
    set_fstr(v->OL_Filename, 1024, OL_Filename);
    v->OL_Mode = OL_Mode;
    v->OL_BP_Mode = OL_BP_Mode;
    v->OL_BP_FiltFreq = OL_BP_FiltFreq;
    v->Ind_Breakpoint = Ind_Breakpoint;
    v->Ind_BldPitch = Ind_BldPitch.empty() ? nullptr : const_cast<int*>(Ind_BldPitch.data());
    v->n_Ind_BldPitch = (int32_t)Ind_BldPitch.size();
    v->Ind_GenTq = Ind_GenTq;
    v->Ind_YawRate = Ind_YawRate;
    v->Ind_R_Speed = Ind_R_Speed;
    v->Ind_R_Torque = Ind_R_Torque;
    v->Ind_R_Pitch = Ind_R_Pitch;
    v->Ind_Azimuth = Ind_Azimuth;
    v->RP_Gains = RP_Gains.empty() ? nullptr : const_cast<double*>(RP_Gains.data());
    v->n_RP_Gains = (int32_t)RP_Gains.size();
    v->Ind_CableControl = Ind_CableControl.empty() ? nullptr : const_cast<int*>(Ind_CableControl.data());
    v->n_Ind_CableControl = (int32_t)Ind_CableControl.size();
    v->Ind_StructControl = Ind_StructControl.empty() ? nullptr : const_cast<int*>(Ind_StructControl.data());
    v->n_Ind_StructControl = (int32_t)Ind_StructControl.size();
    v->OL_Breakpoints = OL_Breakpoints.empty() ? nullptr : const_cast<double*>(OL_Breakpoints.data());
    v->n_OL_Breakpoints = (int32_t)OL_Breakpoints.size();
    v->OL_BldPitch1 = OL_BldPitch1.empty() ? nullptr : const_cast<double*>(OL_BldPitch1.data());
    v->n_OL_BldPitch1 = (int32_t)OL_BldPitch1.size();
    v->OL_BldPitch2 = OL_BldPitch2.empty() ? nullptr : const_cast<double*>(OL_BldPitch2.data());
    v->n_OL_BldPitch2 = (int32_t)OL_BldPitch2.size();
    v->OL_BldPitch3 = OL_BldPitch3.empty() ? nullptr : const_cast<double*>(OL_BldPitch3.data());
    v->n_OL_BldPitch3 = (int32_t)OL_BldPitch3.size();
    v->OL_CableControl = OL_CableControl.empty() ? nullptr : const_cast<double*>(OL_CableControl.data());
    v->n_OL_CableControl_rows = (int32_t)OL_CableControl_rows;
    v->n_OL_CableControl_cols = (int32_t)OL_CableControl_cols;
    v->OL_StructControl = OL_StructControl.empty() ? nullptr : const_cast<double*>(OL_StructControl.data());
    v->n_OL_StructControl_rows = (int32_t)OL_StructControl_rows;
    v->n_OL_StructControl_cols = (int32_t)OL_StructControl_cols;
    v->OL_GenTq = OL_GenTq.empty() ? nullptr : const_cast<double*>(OL_GenTq.data());
    v->n_OL_GenTq = (int32_t)OL_GenTq.size();
    v->OL_YawRate = OL_YawRate.empty() ? nullptr : const_cast<double*>(OL_YawRate.data());
    v->n_OL_YawRate = (int32_t)OL_YawRate.size();
    v->OL_Azimuth = OL_Azimuth.empty() ? nullptr : const_cast<double*>(OL_Azimuth.data());
    v->n_OL_Azimuth = (int32_t)OL_Azimuth.size();
    v->OL_R_Speed = OL_R_Speed.empty() ? nullptr : const_cast<double*>(OL_R_Speed.data());
    v->n_OL_R_Speed = (int32_t)OL_R_Speed.size();
    v->OL_R_Torque = OL_R_Torque.empty() ? nullptr : const_cast<double*>(OL_R_Torque.data());
    v->n_OL_R_Torque = (int32_t)OL_R_Torque.size();
    v->OL_R_Pitch = OL_R_Pitch.empty() ? nullptr : const_cast<double*>(OL_R_Pitch.data());
    v->n_OL_R_Pitch = (int32_t)OL_R_Pitch.size();
    v->OL_Channels = OL_Channels.empty() ? nullptr : const_cast<double*>(OL_Channels.data());
    v->n_OL_Channels_rows = (int32_t)OL_Channels_rows;
    v->n_OL_Channels_cols = (int32_t)OL_Channels_cols;
    v->PA_Mode = PA_Mode;
    v->PA_CornerFreq = PA_CornerFreq;
    v->PA_Damping = PA_Damping;
    v->AWC_Mode = AWC_Mode;
    v->AWC_NumModes = AWC_NumModes;
    v->AWC_n = AWC_n.empty() ? nullptr : const_cast<int*>(AWC_n.data());
    v->n_AWC_n = (int32_t)AWC_n.size();
    v->AWC_harmonic = AWC_harmonic.empty() ? nullptr : const_cast<int*>(AWC_harmonic.data());
    v->n_AWC_harmonic = (int32_t)AWC_harmonic.size();
    v->AWC_freq = AWC_freq.empty() ? nullptr : const_cast<double*>(AWC_freq.data());
    v->n_AWC_freq = (int32_t)AWC_freq.size();
    v->AWC_amp = AWC_amp.empty() ? nullptr : const_cast<double*>(AWC_amp.data());
    v->n_AWC_amp = (int32_t)AWC_amp.size();
    v->AWC_clockangle = AWC_clockangle.empty() ? nullptr : const_cast<double*>(AWC_clockangle.data());
    v->n_AWC_clockangle = (int32_t)AWC_clockangle.size();
    v->AWC_phaseoffset = AWC_phaseoffset;
    v->AWC_CntrGains = AWC_CntrGains.empty() ? nullptr : const_cast<double*>(AWC_CntrGains.data());
    v->n_AWC_CntrGains = (int32_t)AWC_CntrGains.size();
    v->PF_Mode = PF_Mode;
    v->PF_Offsets = PF_Offsets.empty() ? nullptr : const_cast<double*>(PF_Offsets.data());
    v->n_PF_Offsets = (int32_t)PF_Offsets.size();
    v->PF_TimeStuck = PF_TimeStuck.empty() ? nullptr : const_cast<double*>(PF_TimeStuck.data());
    v->n_PF_TimeStuck = (int32_t)PF_TimeStuck.size();
    v->Ext_Mode = Ext_Mode;
    set_fstr(v->DLL_FileName, 1024, DLL_FileName);
    set_fstr(v->DLL_InFile, 1024, DLL_InFile);
    set_fstr(v->DLL_ProcName, 1024, DLL_ProcName);
    v->ZMQ_Mode = ZMQ_Mode;
    set_fstr(v->ZMQ_CommAddress, 256, ZMQ_CommAddress);
    v->ZMQ_UpdatePeriod = ZMQ_UpdatePeriod;
    v->CC_Mode = CC_Mode;
    v->CC_Group_N = CC_Group_N;
    v->CC_ActTau = CC_ActTau;
    v->CC_GroupIndex = CC_GroupIndex.empty() ? nullptr : const_cast<int*>(CC_GroupIndex.data());
    v->n_CC_GroupIndex = (int32_t)CC_GroupIndex.size();
    v->StC_Mode = StC_Mode;
    v->StC_Group_N = StC_Group_N;
    v->StC_GroupIndex = StC_GroupIndex.empty() ? nullptr : const_cast<int*>(StC_GroupIndex.data());
    v->n_StC_GroupIndex = (int32_t)StC_GroupIndex.size();
    v->PC_RtTq99 = PC_RtTq99;
    v->VS_MaxOMTq = VS_MaxOMTq;
    v->VS_MinOMTq = VS_MinOMTq;
}

void ControlParameters::sync_from_view(const controlparameters_view_t& v) {
    ZMQ_ID = v.ZMQ_ID;
    LoggingLevel = v.LoggingLevel;
    Echo = v.Echo;
    Ext_Interface = v.Ext_Interface;
    DT_Out = v.DT_Out;
    n_DT_Out = v.n_DT_Out;
    n_DT_ZMQ = v.n_DT_ZMQ;
    F_LPFType = v.F_LPFType;
    F_LPFCornerFreq = v.F_LPFCornerFreq;
    F_LPFDamping = v.F_LPFDamping;
    F_NumNotchFilts = v.F_NumNotchFilts;
    F_GenSpdNotch_N = v.F_GenSpdNotch_N;
    if (v.F_GenSpdNotch_Ind && v.n_F_GenSpdNotch_Ind > 0)
        F_GenSpdNotch_Ind.assign(v.F_GenSpdNotch_Ind, v.F_GenSpdNotch_Ind + v.n_F_GenSpdNotch_Ind);
    F_TwrTopNotch_N = v.F_TwrTopNotch_N;
    if (v.F_TwrTopNotch_Ind && v.n_F_TwrTopNotch_Ind > 0)
        F_TwrTopNotch_Ind.assign(v.F_TwrTopNotch_Ind, v.F_TwrTopNotch_Ind + v.n_F_TwrTopNotch_Ind);
    if (v.F_NotchFreqs && v.n_F_NotchFreqs > 0)
        F_NotchFreqs.storage.assign(v.F_NotchFreqs, v.F_NotchFreqs + v.n_F_NotchFreqs);
    if (v.F_NotchBetaNum && v.n_F_NotchBetaNum > 0)
        F_NotchBetaNum.storage.assign(v.F_NotchBetaNum, v.F_NotchBetaNum + v.n_F_NotchBetaNum);
    if (v.F_NotchBetaDen && v.n_F_NotchBetaDen > 0)
        F_NotchBetaDen.storage.assign(v.F_NotchBetaDen, v.F_NotchBetaDen + v.n_F_NotchBetaDen);
    F_SSCornerFreq = v.F_SSCornerFreq;
    F_WECornerFreq = v.F_WECornerFreq;
    if (v.F_FlCornerFreq && v.n_F_FlCornerFreq > 0)
        F_FlCornerFreq.storage.assign(v.F_FlCornerFreq, v.F_FlCornerFreq + v.n_F_FlCornerFreq);
    F_FlHighPassFreq = v.F_FlHighPassFreq;
    F_YawErr = v.F_YawErr;
    if (v.F_FlpCornerFreq && v.n_F_FlpCornerFreq > 0)
        F_FlpCornerFreq.storage.assign(v.F_FlpCornerFreq, v.F_FlpCornerFreq + v.n_F_FlpCornerFreq);
    F_VSRefSpdCornerFreq = v.F_VSRefSpdCornerFreq;
    TRA_Mode = v.TRA_Mode;
    TRA_ExclSpeed = v.TRA_ExclSpeed;
    TRA_ExclBand = v.TRA_ExclBand;
    TRA_RateLimit = v.TRA_RateLimit;
    TD_Mode = v.TD_Mode;
    FA_HPFCornerFreq = v.FA_HPFCornerFreq;
    FA_IntSat = v.FA_IntSat;
    FA_KI = v.FA_KI;
    IPC_ControlMode = v.IPC_ControlMode;
    if (v.IPC_Vramp && v.n_IPC_Vramp > 0)
        IPC_Vramp.storage.assign(v.IPC_Vramp, v.IPC_Vramp + v.n_IPC_Vramp);
    IPC_IntSat = v.IPC_IntSat;
    IPC_SatMode = v.IPC_SatMode;
    if (v.IPC_KP && v.n_IPC_KP > 0)
        IPC_KP.storage.assign(v.IPC_KP, v.IPC_KP + v.n_IPC_KP);
    if (v.IPC_KI && v.n_IPC_KI > 0)
        IPC_KI.storage.assign(v.IPC_KI, v.IPC_KI + v.n_IPC_KI);
    if (v.IPC_aziOffset && v.n_IPC_aziOffset > 0)
        IPC_aziOffset.storage.assign(v.IPC_aziOffset, v.IPC_aziOffset + v.n_IPC_aziOffset);
    IPC_CornerFreqAct = v.IPC_CornerFreqAct;
    PC_ControlMode = v.PC_ControlMode;
    PC_GS_n = v.PC_GS_n;
    if (v.PC_GS_angles && v.n_PC_GS_angles > 0)
        PC_GS_angles.storage.assign(v.PC_GS_angles, v.PC_GS_angles + v.n_PC_GS_angles);
    if (v.PC_GS_KP && v.n_PC_GS_KP > 0)
        PC_GS_KP.storage.assign(v.PC_GS_KP, v.PC_GS_KP + v.n_PC_GS_KP);
    if (v.PC_GS_KI && v.n_PC_GS_KI > 0)
        PC_GS_KI.storage.assign(v.PC_GS_KI, v.PC_GS_KI + v.n_PC_GS_KI);
    if (v.PC_GS_KD && v.n_PC_GS_KD > 0)
        PC_GS_KD.storage.assign(v.PC_GS_KD, v.PC_GS_KD + v.n_PC_GS_KD);
    if (v.PC_GS_TF && v.n_PC_GS_TF > 0)
        PC_GS_TF.storage.assign(v.PC_GS_TF, v.PC_GS_TF + v.n_PC_GS_TF);
    PC_MaxPit = v.PC_MaxPit;
    PC_MinPit = v.PC_MinPit;
    PC_MaxRat = v.PC_MaxRat;
    PC_MinRat = v.PC_MinRat;
    PC_RefSpd = v.PC_RefSpd;
    PC_FinePit = v.PC_FinePit;
    PC_Switch = v.PC_Switch;
    VS_ControlMode = v.VS_ControlMode;
    VS_ConstPower = v.VS_ConstPower;
    VS_FBP = v.VS_FBP;
    VS_GenEff = v.VS_GenEff;
    VS_ArSatTq = v.VS_ArSatTq;
    VS_MaxRat = v.VS_MaxRat;
    VS_MaxTq = v.VS_MaxTq;
    VS_MinTq = v.VS_MinTq;
    VS_MinOMSpd = v.VS_MinOMSpd;
    VS_Rgn2K = v.VS_Rgn2K;
    VS_RtPwr = v.VS_RtPwr;
    VS_RtTq = v.VS_RtTq;
    VS_RefSpd = v.VS_RefSpd;
    VS_n = v.VS_n;
    if (v.VS_KP && v.n_VS_KP > 0)
        VS_KP.storage.assign(v.VS_KP, v.VS_KP + v.n_VS_KP);
    if (v.VS_KI && v.n_VS_KI > 0)
        VS_KI.storage.assign(v.VS_KI, v.VS_KI + v.n_VS_KI);
    VS_TSRopt = v.VS_TSRopt;
    VS_FBP_n = v.VS_FBP_n;
    if (v.VS_FBP_U && v.n_VS_FBP_U > 0)
        VS_FBP_U.storage.assign(v.VS_FBP_U, v.VS_FBP_U + v.n_VS_FBP_U);
    if (v.VS_FBP_Omega && v.n_VS_FBP_Omega > 0)
        VS_FBP_Omega.storage.assign(v.VS_FBP_Omega, v.VS_FBP_Omega + v.n_VS_FBP_Omega);
    if (v.VS_FBP_Tau && v.n_VS_FBP_Tau > 0)
        VS_FBP_Tau.storage.assign(v.VS_FBP_Tau, v.VS_FBP_Tau + v.n_VS_FBP_Tau);
    SS_Mode = v.SS_Mode;
    SS_VSGain = v.SS_VSGain;
    SS_PCGain = v.SS_PCGain;
    PRC_Mode = v.PRC_Mode;
    PRC_Comm = v.PRC_Comm;
    if (v.PRC_WindSpeeds && v.n_PRC_WindSpeeds > 0)
        PRC_WindSpeeds.storage.assign(v.PRC_WindSpeeds, v.PRC_WindSpeeds + v.n_PRC_WindSpeeds);
    if (v.PRC_GenSpeeds && v.n_PRC_GenSpeeds > 0)
        PRC_GenSpeeds.storage.assign(v.PRC_GenSpeeds, v.PRC_GenSpeeds + v.n_PRC_GenSpeeds);
    PRC_n = v.PRC_n;
    PRC_LPF_Freq = v.PRC_LPF_Freq;
    PRC_R_Torque = v.PRC_R_Torque;
    PRC_R_Speed = v.PRC_R_Speed;
    PRC_R_Pitch = v.PRC_R_Pitch;
    PRC_Table_n = v.PRC_Table_n;
    if (v.PRC_Pitch_Table && v.n_PRC_Pitch_Table > 0)
        PRC_Pitch_Table.storage.assign(v.PRC_Pitch_Table, v.PRC_Pitch_Table + v.n_PRC_Pitch_Table);
    if (v.PRC_R_Table && v.n_PRC_R_Table > 0)
        PRC_R_Table.storage.assign(v.PRC_R_Table, v.PRC_R_Table + v.n_PRC_R_Table);
    WE_Mode = v.WE_Mode;
    WE_BladeRadius = v.WE_BladeRadius;
    WE_CP_n = v.WE_CP_n;
    if (v.WE_CP && v.n_WE_CP > 0)
        WE_CP.storage.assign(v.WE_CP, v.WE_CP + v.n_WE_CP);
    WE_Gamma = v.WE_Gamma;
    WE_GearboxRatio = v.WE_GearboxRatio;
    WE_Jtot = v.WE_Jtot;
    WE_RhoAir = v.WE_RhoAir;
    {
        int _len = 1024;
        while (_len > 0 && v.PerfFileName[_len-1] == ' ') _len--;
        PerfFileName = std::string(v.PerfFileName, _len);
    }
    if (v.PerfTableSize && v.n_PerfTableSize > 0)
        PerfTableSize.assign(v.PerfTableSize, v.PerfTableSize + v.n_PerfTableSize);
    WE_FOPoles_N = v.WE_FOPoles_N;
    if (v.WE_FOPoles_v && v.n_WE_FOPoles_v > 0)
        WE_FOPoles_v.storage.assign(v.WE_FOPoles_v, v.WE_FOPoles_v + v.n_WE_FOPoles_v);
    if (v.WE_FOPoles && v.n_WE_FOPoles > 0)
        WE_FOPoles.storage.assign(v.WE_FOPoles, v.WE_FOPoles + v.n_WE_FOPoles);
    Y_ControlMode = v.Y_ControlMode;
    Y_uSwitch = v.Y_uSwitch;
    if (v.Y_ErrThresh && v.n_Y_ErrThresh > 0)
        Y_ErrThresh.storage.assign(v.Y_ErrThresh, v.Y_ErrThresh + v.n_Y_ErrThresh);
    Y_Rate = v.Y_Rate;
    Y_MErrSet = v.Y_MErrSet;
    Y_IPC_IntSat = v.Y_IPC_IntSat;
    Y_IPC_KP = v.Y_IPC_KP;
    Y_IPC_KI = v.Y_IPC_KI;
    PS_Mode = v.PS_Mode;
    PS_BldPitchMin_N = v.PS_BldPitchMin_N;
    if (v.PS_WindSpeeds && v.n_PS_WindSpeeds > 0)
        PS_WindSpeeds.storage.assign(v.PS_WindSpeeds, v.PS_WindSpeeds + v.n_PS_WindSpeeds);
    if (v.PS_BldPitchMin && v.n_PS_BldPitchMin > 0)
        PS_BldPitchMin.storage.assign(v.PS_BldPitchMin, v.PS_BldPitchMin + v.n_PS_BldPitchMin);
    SU_Mode = v.SU_Mode;
    SU_StartTime = v.SU_StartTime;
    SU_FW_MinDuration = v.SU_FW_MinDuration;
    SU_RotorSpeedThresh = v.SU_RotorSpeedThresh;
    SU_RotorSpeedCornerFreq = v.SU_RotorSpeedCornerFreq;
    SU_LoadStages_N = v.SU_LoadStages_N;
    if (v.SU_LoadStages && v.n_SU_LoadStages > 0)
        SU_LoadStages.storage.assign(v.SU_LoadStages, v.SU_LoadStages + v.n_SU_LoadStages);
    if (v.SU_LoadRampDuration && v.n_SU_LoadRampDuration > 0)
        SU_LoadRampDuration.storage.assign(v.SU_LoadRampDuration, v.SU_LoadRampDuration + v.n_SU_LoadRampDuration);
    if (v.SU_LoadHoldDuration && v.n_SU_LoadHoldDuration > 0)
        SU_LoadHoldDuration.storage.assign(v.SU_LoadHoldDuration, v.SU_LoadHoldDuration + v.n_SU_LoadHoldDuration);
    SD_Mode = v.SD_Mode;
    SD_TimeActivate = v.SD_TimeActivate;
    SD_EnablePitch = v.SD_EnablePitch;
    SD_EnableYawError = v.SD_EnableYawError;
    SD_EnableGenSpeed = v.SD_EnableGenSpeed;
    SD_EnableTime = v.SD_EnableTime;
    SD_MaxPit = v.SD_MaxPit;
    SD_PitchCornerFreq = v.SD_PitchCornerFreq;
    SD_MaxYawError = v.SD_MaxYawError;
    SD_YawErrorCornerFreq = v.SD_YawErrorCornerFreq;
    SD_MaxGenSpd = v.SD_MaxGenSpd;
    SD_GenSpdCornerFreq = v.SD_GenSpdCornerFreq;
    SD_Time = v.SD_Time;
    SD_Method = v.SD_Method;
    if (v.SD_MaxTorqueRate && v.n_SD_MaxTorqueRate > 0)
        SD_MaxTorqueRate.storage.assign(v.SD_MaxTorqueRate, v.SD_MaxTorqueRate + v.n_SD_MaxTorqueRate);
    if (v.SD_MaxPitchRate && v.n_SD_MaxPitchRate > 0)
        SD_MaxPitchRate.storage.assign(v.SD_MaxPitchRate, v.SD_MaxPitchRate + v.n_SD_MaxPitchRate);
    if (v.SD_StagePitch && v.n_SD_StagePitch > 0)
        SD_StagePitch.storage.assign(v.SD_StagePitch, v.SD_StagePitch + v.n_SD_StagePitch);
    if (v.SD_StageTime && v.n_SD_StageTime > 0)
        SD_StageTime.storage.assign(v.SD_StageTime, v.SD_StageTime + v.n_SD_StageTime);
    SD_Stage_N = v.SD_Stage_N;
    Fl_Mode = v.Fl_Mode;
    Fl_n = v.Fl_n;
    if (v.Fl_Kp && v.n_Fl_Kp > 0)
        Fl_Kp.storage.assign(v.Fl_Kp, v.Fl_Kp + v.n_Fl_Kp);
    if (v.Fl_U && v.n_Fl_U > 0)
        Fl_U.storage.assign(v.Fl_U, v.Fl_U + v.n_Fl_U);
    Flp_Mode = v.Flp_Mode;
    Flp_Angle = v.Flp_Angle;
    Flp_Kp = v.Flp_Kp;
    Flp_Ki = v.Flp_Ki;
    Flp_MaxPit = v.Flp_MaxPit;
    {
        int _len = 1024;
        while (_len > 0 && v.OL_Filename[_len-1] == ' ') _len--;
        OL_Filename = std::string(v.OL_Filename, _len);
    }
    OL_Mode = v.OL_Mode;
    OL_BP_Mode = v.OL_BP_Mode;
    OL_BP_FiltFreq = v.OL_BP_FiltFreq;
    Ind_Breakpoint = v.Ind_Breakpoint;
    if (v.Ind_BldPitch && v.n_Ind_BldPitch > 0)
        Ind_BldPitch.assign(v.Ind_BldPitch, v.Ind_BldPitch + v.n_Ind_BldPitch);
    Ind_GenTq = v.Ind_GenTq;
    Ind_YawRate = v.Ind_YawRate;
    Ind_R_Speed = v.Ind_R_Speed;
    Ind_R_Torque = v.Ind_R_Torque;
    Ind_R_Pitch = v.Ind_R_Pitch;
    Ind_Azimuth = v.Ind_Azimuth;
    if (v.RP_Gains && v.n_RP_Gains > 0)
        RP_Gains.storage.assign(v.RP_Gains, v.RP_Gains + v.n_RP_Gains);
    if (v.Ind_CableControl && v.n_Ind_CableControl > 0)
        Ind_CableControl.assign(v.Ind_CableControl, v.Ind_CableControl + v.n_Ind_CableControl);
    if (v.Ind_StructControl && v.n_Ind_StructControl > 0)
        Ind_StructControl.assign(v.Ind_StructControl, v.Ind_StructControl + v.n_Ind_StructControl);
    if (v.OL_Breakpoints && v.n_OL_Breakpoints > 0)
        OL_Breakpoints.storage.assign(v.OL_Breakpoints, v.OL_Breakpoints + v.n_OL_Breakpoints);
    if (v.OL_BldPitch1 && v.n_OL_BldPitch1 > 0)
        OL_BldPitch1.storage.assign(v.OL_BldPitch1, v.OL_BldPitch1 + v.n_OL_BldPitch1);
    if (v.OL_BldPitch2 && v.n_OL_BldPitch2 > 0)
        OL_BldPitch2.storage.assign(v.OL_BldPitch2, v.OL_BldPitch2 + v.n_OL_BldPitch2);
    if (v.OL_BldPitch3 && v.n_OL_BldPitch3 > 0)
        OL_BldPitch3.storage.assign(v.OL_BldPitch3, v.OL_BldPitch3 + v.n_OL_BldPitch3);
    if (v.OL_CableControl && v.n_OL_CableControl_rows > 0 && v.n_OL_CableControl_cols > 0) {
        OL_CableControl.storage.assign(v.OL_CableControl, v.OL_CableControl + (size_t)v.n_OL_CableControl_rows * v.n_OL_CableControl_cols);
        OL_CableControl_rows = v.n_OL_CableControl_rows;
        OL_CableControl_cols = v.n_OL_CableControl_cols;
    }
    if (v.OL_StructControl && v.n_OL_StructControl_rows > 0 && v.n_OL_StructControl_cols > 0) {
        OL_StructControl.storage.assign(v.OL_StructControl, v.OL_StructControl + (size_t)v.n_OL_StructControl_rows * v.n_OL_StructControl_cols);
        OL_StructControl_rows = v.n_OL_StructControl_rows;
        OL_StructControl_cols = v.n_OL_StructControl_cols;
    }
    if (v.OL_GenTq && v.n_OL_GenTq > 0)
        OL_GenTq.storage.assign(v.OL_GenTq, v.OL_GenTq + v.n_OL_GenTq);
    if (v.OL_YawRate && v.n_OL_YawRate > 0)
        OL_YawRate.storage.assign(v.OL_YawRate, v.OL_YawRate + v.n_OL_YawRate);
    if (v.OL_Azimuth && v.n_OL_Azimuth > 0)
        OL_Azimuth.storage.assign(v.OL_Azimuth, v.OL_Azimuth + v.n_OL_Azimuth);
    if (v.OL_R_Speed && v.n_OL_R_Speed > 0)
        OL_R_Speed.storage.assign(v.OL_R_Speed, v.OL_R_Speed + v.n_OL_R_Speed);
    if (v.OL_R_Torque && v.n_OL_R_Torque > 0)
        OL_R_Torque.storage.assign(v.OL_R_Torque, v.OL_R_Torque + v.n_OL_R_Torque);
    if (v.OL_R_Pitch && v.n_OL_R_Pitch > 0)
        OL_R_Pitch.storage.assign(v.OL_R_Pitch, v.OL_R_Pitch + v.n_OL_R_Pitch);
    if (v.OL_Channels && v.n_OL_Channels_rows > 0 && v.n_OL_Channels_cols > 0) {
        OL_Channels.storage.assign(v.OL_Channels, v.OL_Channels + (size_t)v.n_OL_Channels_rows * v.n_OL_Channels_cols);
        OL_Channels_rows = v.n_OL_Channels_rows;
        OL_Channels_cols = v.n_OL_Channels_cols;
    }
    PA_Mode = v.PA_Mode;
    PA_CornerFreq = v.PA_CornerFreq;
    PA_Damping = v.PA_Damping;
    AWC_Mode = v.AWC_Mode;
    AWC_NumModes = v.AWC_NumModes;
    if (v.AWC_n && v.n_AWC_n > 0)
        AWC_n.assign(v.AWC_n, v.AWC_n + v.n_AWC_n);
    if (v.AWC_harmonic && v.n_AWC_harmonic > 0)
        AWC_harmonic.assign(v.AWC_harmonic, v.AWC_harmonic + v.n_AWC_harmonic);
    if (v.AWC_freq && v.n_AWC_freq > 0)
        AWC_freq.storage.assign(v.AWC_freq, v.AWC_freq + v.n_AWC_freq);
    if (v.AWC_amp && v.n_AWC_amp > 0)
        AWC_amp.storage.assign(v.AWC_amp, v.AWC_amp + v.n_AWC_amp);
    if (v.AWC_clockangle && v.n_AWC_clockangle > 0)
        AWC_clockangle.storage.assign(v.AWC_clockangle, v.AWC_clockangle + v.n_AWC_clockangle);
    AWC_phaseoffset = v.AWC_phaseoffset;
    if (v.AWC_CntrGains && v.n_AWC_CntrGains > 0)
        AWC_CntrGains.storage.assign(v.AWC_CntrGains, v.AWC_CntrGains + v.n_AWC_CntrGains);
    PF_Mode = v.PF_Mode;
    if (v.PF_Offsets && v.n_PF_Offsets > 0)
        PF_Offsets.storage.assign(v.PF_Offsets, v.PF_Offsets + v.n_PF_Offsets);
    if (v.PF_TimeStuck && v.n_PF_TimeStuck > 0)
        PF_TimeStuck.storage.assign(v.PF_TimeStuck, v.PF_TimeStuck + v.n_PF_TimeStuck);
    Ext_Mode = v.Ext_Mode;
    {
        int _len = 1024;
        while (_len > 0 && v.DLL_FileName[_len-1] == ' ') _len--;
        DLL_FileName = std::string(v.DLL_FileName, _len);
    }
    {
        int _len = 1024;
        while (_len > 0 && v.DLL_InFile[_len-1] == ' ') _len--;
        DLL_InFile = std::string(v.DLL_InFile, _len);
    }
    {
        int _len = 1024;
        while (_len > 0 && v.DLL_ProcName[_len-1] == ' ') _len--;
        DLL_ProcName = std::string(v.DLL_ProcName, _len);
    }
    ZMQ_Mode = v.ZMQ_Mode;
    {
        int _len = 256;
        while (_len > 0 && v.ZMQ_CommAddress[_len-1] == ' ') _len--;
        ZMQ_CommAddress = std::string(v.ZMQ_CommAddress, _len);
    }
    ZMQ_UpdatePeriod = v.ZMQ_UpdatePeriod;
    CC_Mode = v.CC_Mode;
    CC_Group_N = v.CC_Group_N;
    CC_ActTau = v.CC_ActTau;
    if (v.CC_GroupIndex && v.n_CC_GroupIndex > 0)
        CC_GroupIndex.assign(v.CC_GroupIndex, v.CC_GroupIndex + v.n_CC_GroupIndex);
    StC_Mode = v.StC_Mode;
    StC_Group_N = v.StC_Group_N;
    if (v.StC_GroupIndex && v.n_StC_GroupIndex > 0)
        StC_GroupIndex.assign(v.StC_GroupIndex, v.StC_GroupIndex + v.n_StC_GroupIndex);
    PC_RtTq99 = v.PC_RtTq99;
    VS_MaxOMTq = v.VS_MaxOMTq;
    VS_MinOMTq = v.VS_MinOMTq;
}
