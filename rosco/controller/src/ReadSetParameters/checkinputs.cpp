#include "../include/vit_types.h"
#include "../include/rosco_types.hpp"
#include "../include/rosco_constants.h"
#include <cmath>
#include <cstring>
#include <vector>
#include <algorithm>
#include <cstdint>
#include <cstdio>

// Helper: set error with space-padded Fortran CHARACTER field
static void setError(errorvariables_t* ErrVar, const char* msg) {
    ErrVar->aviFAIL = -1;
    std::memset(ErrVar->ErrMsg, ' ', 1024);
    size_t len = std::strlen(msg);
    if (len > 1024) len = 1024;
    std::memcpy(ErrVar->ErrMsg, msg, len);
}

// Helper: check if a double array is strictly non-decreasing
// Returns false if any adjacent pair has arr[i+1] - arr[i] <= 0.0
static bool NonDecreasing(const double* arr, int n) {
    for (int i = 0; i < n - 1; i++) {
        if (arr[i + 1] - arr[i] <= 0.0) {
            return false;
        }
    }
    return true;
}

void CheckInputs(LocalVariables& LocalVar, const ControlParameters& CntrPar,
                 float* avrSWAP, errorvariables_t* ErrVar, int32_t size_avcMSG) {

    int Imode;
    int I;

    //..............................................................................................................................
    // Check validity of input parameters:
    //..............................................................................................................................

    //------- DEBUG ------------------------------------------------------------

    // LoggingLevel
    if ((CntrPar.LoggingLevel < 0) || (CntrPar.LoggingLevel > 3)) {
        setError(ErrVar, "LoggingLevel must be 0 - 3.");
    }

    if (CntrPar.DT_Out <= 0) {
        setError(ErrVar, "DT_Out must be greater than 0");
    }

    if (CntrPar.DT_Out < LocalVar.DT) {
        setError(ErrVar, "DT_Out must be greater than or equal to DT in OpenFAST");
    }

    if (std::abs(CntrPar.DT_Out - LocalVar.DT * CntrPar.n_DT_Out) > 0.001) {
        setError(ErrVar, "DT_Out must be a factor of DT in OpenFAST");
    }

    if (CntrPar.ZMQ_Mode > 0) {
        if (std::abs(CntrPar.ZMQ_UpdatePeriod - LocalVar.DT * CntrPar.n_DT_ZMQ) > 0.001) {
            setError(ErrVar, "ZMQ_UpdatePeriod must be a factor of DT in OpenFAST");
        }
    }

    //------- CONTROLLER FLAGS -------------------------------------------------

    // F_LPFType
    if ((CntrPar.F_LPFType < 1) || (CntrPar.F_LPFType > 2)) {
        setError(ErrVar, "F_LPFType must be 1 or 2.");
    }

    // IPC_ControlMode
    if ((CntrPar.IPC_ControlMode < 0) || (CntrPar.IPC_ControlMode > 2)) {
        setError(ErrVar, "IPC_ControlMode must be 0, 1, or 2.");
    }

    // VS_ControlMode
    if ((CntrPar.VS_ControlMode < 0) || (CntrPar.VS_ControlMode > 4)) {
        setError(ErrVar, "VS_ControlMode must be between 0 and 4.");
    }

    // VS_ConstPower
    if ((CntrPar.VS_ConstPower < 0) || (CntrPar.VS_ConstPower > 1)) {
        setError(ErrVar, "VS_ConstPower must be 0 or 1.");
    }

    // VS_FBP
    if ((CntrPar.VS_FBP < 0) || (CntrPar.VS_FBP > 3)) {
        setError(ErrVar, "VS_FBP must be between 0 and 3.");
    }
    if ((CntrPar.VS_FBP > 0) && (CntrPar.PC_ControlMode > 0)) {
        setError(ErrVar, "VS_FBP and PC_ControlMode cannot both be greater than 0.");
    }

    if ((CntrPar.VS_FBP > 0) && (CntrPar.PRC_Mode > 0)) {
        setError(ErrVar, "Fixed blade pitch control (VS_FBP) and power reference control (PRC_Mode) cannot both be enabled.");
    }

    if ((CntrPar.VS_FBP > 0) && (CntrPar.VS_ConstPower > 0)) {
        setError(ErrVar, "Fixed blade pitch control (VS_FBP) and constant power torque control (VS_ConstPower) cannot both be enabled.");
    }

    // PC_ControlMode
    if ((CntrPar.PC_ControlMode < 0) || (CntrPar.PC_ControlMode > 1)) {
        setError(ErrVar, "PC_ControlMode must be 0 or 1.");
    }

    // Y_ControlMode
    if ((CntrPar.Y_ControlMode < 0) || (CntrPar.Y_ControlMode > 2)) {
        setError(ErrVar, "Y_ControlMode must be 0, 1 or 2.");
    }

    if ((CntrPar.IPC_ControlMode > 0) && (CntrPar.Y_ControlMode > 1)) {
        setError(ErrVar, "IPC control for load reductions and yaw-by-IPC cannot be activated simultaneously");
    }

    // SS_Mode
    if ((CntrPar.SS_Mode < 0) || (CntrPar.SS_Mode > 1)) {
        setError(ErrVar, "SS_Mode must be 0 or 1.");
    }

    // WE_Mode
    if ((CntrPar.WE_Mode < 0) || (CntrPar.WE_Mode > 2)) {
        setError(ErrVar, "WE_Mode must be 0, 1, or 2.");
    }

    // PS_Mode
    if ((CntrPar.PS_Mode < 0) || (CntrPar.PS_Mode > 3)) {
        setError(ErrVar, "PS_Mode must be 0 or 1.");
    }

    // SU_Mode
    if ((CntrPar.SU_Mode < 0) || (CntrPar.SU_Mode > 1)) {
        setError(ErrVar, "SU_Mode must be 0 or 1.");
    }

    // SD_Mode
    if ((CntrPar.SD_Mode < 0) || (CntrPar.SD_Mode > 1)) {
        setError(ErrVar, "SD_Mode must be 0 or 1.");
    }

    // Fl_Mode
    if ((CntrPar.Fl_Mode < 0) || (CntrPar.Fl_Mode > 2)) {
        setError(ErrVar, "Fl_Mode must be 0, 1, or 2.");
    }

    // Flp_Mode
    if ((CntrPar.Flp_Mode < 0) || (CntrPar.Flp_Mode > 3)) {
        setError(ErrVar, "Flp_Mode must be 0, 1, 2, or 3.");
    }

    if ((CntrPar.IPC_ControlMode > 0) && (CntrPar.Flp_Mode > 0)) {
        setError(ErrVar, "ROSCO does not currently support IPC_ControlMode and Flp_Mode > 0");
    }

    //------- FILTERS ----------------------------------------------------------

    // F_LPFCornerFreq
    if (CntrPar.F_LPFCornerFreq <= 0.0) {
        setError(ErrVar, "F_LPFCornerFreq must be greater than zero.");
    }

    // F_LPFDamping
    if (CntrPar.F_LPFType == 2) {
        if (CntrPar.F_LPFDamping <= 0.0) {
            setError(ErrVar, "F_LPFDamping must be greater than zero.");
        }
    }

    // Notch Filter Params
    if (CntrPar.F_NumNotchFilts > 0) {

        // F_NotchCornerFreq
        {
            bool any_le_zero = false;
            for (int i = 0; i < (int)CntrPar.F_NotchFreqs.size(); i++) {
                if (CntrPar.F_NotchFreqs[i] <= 0.0) { any_le_zero = true; break; }
            }
            if (any_le_zero) {
                setError(ErrVar, "F_NotchFreqs must be greater than zero.");
            }
        }

        // F_NotchBetaDen
        {
            bool any_le_zero = false;
            for (int i = 0; i < (int)CntrPar.F_NotchBetaDen.size(); i++) {
                if (CntrPar.F_NotchBetaDen[i] <= 0.0) { any_le_zero = true; break; }
            }
            if (any_le_zero) {
                setError(ErrVar, "F_NotchBetaDen must be greater than zero.");
            }
        }
    }

    // F_SSCornerFreq
    if (CntrPar.F_SSCornerFreq <= 0.0) {
        setError(ErrVar, "F_SSCornerFreq must be greater than zero.");
    }

    // F_WECornerFreq
    if (CntrPar.F_WECornerFreq <= 0.0) {
        setError(ErrVar, "F_WECornerFreq must be greater than zero.");
    }

    if (CntrPar.Fl_Mode > 0) {
        // F_FlCornerFreq(1) (frequency)
        if (CntrPar.F_FlCornerFreq[0] <= 0.0) {
            setError(ErrVar, "F_FlCornerFreq(1) must be greater than zero.");
        }

        // F_FlCornerFreq(2) (damping)
        if (CntrPar.F_FlCornerFreq[1] <= 0.0) {
            setError(ErrVar, "F_FlCornerFreq(2) must be greater than zero.");
        }

        // F_FlHighPassFreq
        if (CntrPar.F_FlHighPassFreq <= 0.0) {
            setError(ErrVar, "F_FlHighPassFreq must be greater than zero.");
        }
    }

    if (CntrPar.Flp_Mode > 0) {
        // F_FlpCornerFreq(1) (frequency)
        if (CntrPar.F_FlpCornerFreq[0] <= 0.0) {
            setError(ErrVar, "F_FlpCornerFreq(1) must be greater than zero.");
        }

        // F_FlpCornerFreq(2) (damping)
        if (CntrPar.F_FlpCornerFreq[1] < 0.0) {
            setError(ErrVar, "F_FlpCornerFreq(2) must be greater than or equal to zero.");
        }
    }

    //------- BLADE PITCH CONTROL ----------------------------------------------

    // PC_GS_n
    if (CntrPar.PC_GS_n <= 0) {
        setError(ErrVar, "PC_GS_n must be greater than 0");
    }

    // PC_GS_angles
    if (CntrPar.PC_ControlMode != 0 && !NonDecreasing(CntrPar.PC_GS_angles.data(), (int)CntrPar.PC_GS_angles.size())) {
        setError(ErrVar, "PC_GS_angles must be non-decreasing");
    }

    // PC_MinPit and PC_MaxPit
    if (CntrPar.PC_MinPit >= CntrPar.PC_MaxPit) {
        setError(ErrVar, "PC_MinPit must be less than PC_MaxPit.");
    }

    // PC_RefSpd
    if (CntrPar.PC_RefSpd <= 0.0) {
        setError(ErrVar, "PC_RefSpd must be greater than zero.");
    }

    // PC_MaxRat
    if (CntrPar.PC_MaxRat <= 0.0) {
        setError(ErrVar, "PC_MaxRat must be greater than zero.");
    }

    // PC_MinRat
    if (CntrPar.PC_MinRat >= 0.0) {
        setError(ErrVar, "PC_MinRat must be less than zero.");
    }

    //------- INDIVIDUAL PITCH CONTROL -----------------------------------------

    if (CntrPar.IPC_CornerFreqAct < 0.0) {
        setError(ErrVar, "Corner frequency of IPC actuator model must be positive, or set to 0 to disable.");
    }

    if (CntrPar.IPC_SatMode < 0 || CntrPar.IPC_SatMode > 3) {
        setError(ErrVar, "IPC_SatMode must be 0, 1, 2, or 3.");
    }

    if (CntrPar.IPC_KI[0] < 0.0) {
        setError(ErrVar, "IPC_KI(1) must be zero or greater than zero.");
    }

    if (CntrPar.IPC_KI[1] < 0.0) {
        setError(ErrVar, "IPC_KI(2) must be zero or greater than zero.");
    }

    // NOTE: Fortran bug reproduced faithfully — checks IPC_KI(1) and IPC_KI(2)
    // instead of IPC_KP(1) and IPC_KP(2) for the IPC_KP error messages
    if (CntrPar.IPC_KI[0] < 0.0) {
        setError(ErrVar, "IPC_KP(1) must be zero or greater than zero.");
    }

    if (CntrPar.IPC_KI[1] < 0.0) {
        setError(ErrVar, "IPC_KP(2) must be zero or greater than zero.");
    }

    //------- VS TORQUE CONTROL ------------------------------------------------

    if (CntrPar.VS_MaxRat <= 0.0) {
        setError(ErrVar, "VS_MaxRat must be greater than zero.");
    }

    // VS_Rgn2K
    if (CntrPar.VS_Rgn2K < 0.0) {
        setError(ErrVar, "VS_Rgn2K must not be negative.");
    }

    // VS_RtTq
    if (CntrPar.VS_MaxTq < CntrPar.VS_RtTq) {
        setError(ErrVar, "VS_RtTq must not be greater than VS_MaxTq.");
    }

    // VS_RtPwr
    if (CntrPar.VS_RtPwr < 0.0) {
        setError(ErrVar, "VS_RtPwr must not be negative.");
    }

    // VS_RtTq
    if (CntrPar.VS_RtTq < 0.0) {
        setError(ErrVar, "VS_RtTq must not be negative.");
    }

    // VS_KP
    if (CntrPar.VS_KP[0] > 0.0) {
        setError(ErrVar, "VS_KP must be less than zero.");
    }

    // VS_KI
    if (CntrPar.VS_KI[0] > 0.0) {
        setError(ErrVar, "VS_KI must be less than zero.");
    }

    // VS_TSRopt
    if (CntrPar.VS_TSRopt < 0.0) {
        setError(ErrVar, "VS_TSRopt must be greater than zero.");
    }

    //------- SETPOINT SMOOTHER ---------------------------------------------

    // SS_VSGain
    if (CntrPar.SS_VSGain < 0.0) {
        setError(ErrVar, "SS_VSGain must be greater than zero.");
    }

    // SS_PCGain
    if (CntrPar.SS_PCGain < 0.0) {
        setError(ErrVar, "SS_PCGain must be greater than zero.");
    }

    if ((CntrPar.PRC_Mode < 0) || (CntrPar.PRC_Mode > 2)) {
        setError(ErrVar, "PRC_Mode must be 0, 1, or 2.");
    }

    if (CntrPar.PRC_Mode == 2) {
        printf(" Note: PRC Mode = %12d, which will affect VS_RefSpeed, VS_TSRopt, and PC_RefSpeed\n",
               CntrPar.PRC_Mode);

        if (CntrPar.PRC_Comm == 0) {
            if (CntrPar.PRC_R_Pitch < 0) {
                setError(ErrVar, "PRC_R_Pitch must be greater than or equal to zero.");
            }

            if (CntrPar.PRC_R_Speed < 0) {
                setError(ErrVar, "PRC_R_Speed must be greater than or equal to zero.");
            }

            if (CntrPar.PRC_R_Torque < 0) {
                setError(ErrVar, "PRC_R_Torque must be greater than or equal to zero.");
            }
        }

        if ((CntrPar.PRC_Comm == 1) && (CntrPar.OL_Mode != 1)) {
            setError(ErrVar, "OL_Mode must be 1 to use open loop inputs for power control (PRC_Comm = 1).");
        }

        if ((CntrPar.PRC_Comm == 2) && (CntrPar.ZMQ_Mode != 1)) {
            setError(ErrVar, "ZMQ_Mode must be 1 to use ZeroMQ inputs for power control (PRC_Comm = 2).");
        }
    }

    //------- WIND SPEED ESTIMATOR ---------------------------------------------

    // WE_BladeRadius
    if (CntrPar.WE_BladeRadius < 0.0) {
        setError(ErrVar, "WE_BladeRadius must be greater than zero.");
    }

    // WE_GearboxRatio
    if (CntrPar.WE_GearboxRatio < 0.0) {
        setError(ErrVar, "WE_GearboxRatio must be greater than zero.");
    }

    // WE_Jtot
    if (CntrPar.WE_Jtot < 0.0) {
        setError(ErrVar, "WE_Jtot must be greater than zero.");
    }

    // WE_RhoAir
    if (CntrPar.WE_RhoAir < 0.0) {
        setError(ErrVar, "WE_RhoAir must be greater than zero.");
    }

    // PerfTableSize(1)
    if (CntrPar.PerfTableSize[0] < 0.0) {
        setError(ErrVar, "PerfTableSize(1) must be greater than zero.");
    }

    // PerfTableSize(2)
    if (CntrPar.PerfTableSize[1] < 0.0) {
        setError(ErrVar, "PerfTableSize(2) must be greater than zero.");
    }

    // WE_FOPoles_N
    if (CntrPar.WE_FOPoles_N < 0) {
        setError(ErrVar, "WE_FOPoles_N must be greater than zero.");
    }

    // WE_FOPoles_v
    if (CntrPar.WE_Mode == 2 && !NonDecreasing(CntrPar.WE_FOPoles_v.data(), (int)CntrPar.WE_FOPoles_v.size())) {
        setError(ErrVar, "WE_FOPoles_v must be non-decreasing.");
    }

    // ---- Yaw Control ----
    if (CntrPar.Y_ControlMode > 0) {
        if (CntrPar.Y_ControlMode == 1) {
            if (CntrPar.Y_ErrThresh[0] <= 0.0) {
                setError(ErrVar, "Y_ErrThresh must be greater than zero.");
            }

            if (CntrPar.Y_Rate <= 0.0) {
                setError(ErrVar, "CntrPar%Y_Rate must be greater than zero.");
            }
        }
    }

    // ---- Tower Control ----
    if (CntrPar.TD_Mode < 0 || CntrPar.TD_Mode > 1) {
        setError(ErrVar, "TD_Mode must be 0 or 1.");
    }

    if (CntrPar.TRA_Mode < 0 || CntrPar.TRA_Mode > 1) {
        setError(ErrVar, "TRA_Mode must be 0 or 1.");
    }

    if (CntrPar.TRA_Mode > 1) {  // Frequency avoidance is active
        if (CntrPar.TRA_ExclSpeed < 0) {
            setError(ErrVar, "TRA_ExclSpeed must be greater than 0.");
        }

        if (CntrPar.TRA_ExclBand < 0) {
            setError(ErrVar, "TRA_ExclBand must be greater than 0.");
        }

        if (CntrPar.TRA_RateLimit < 0) {
            setError(ErrVar, "TRA_RateLimit must be greater than 0.");
        }

        if (!((CntrPar.VS_ControlMode == VS_Mode_WSE_TSR) || (CntrPar.VS_ControlMode == VS_Mode_Power_TSR))) {
            setError(ErrVar, "VS_ControlMode must be 2 or 3 to use frequency avoidance control.");
        }

        if (CntrPar.PRC_Mode == 1) {
            printf(" ROSCO Warning: Note that frequency avoidance control (TRA_Mode > 1) will affect PRC set points\n");
        }
    }

    //------- MINIMUM PITCH SATURATION -------------------------------------------
    if (CntrPar.PS_Mode > 0) {

        // PS_BldPitchMin_N
        if (CntrPar.PS_BldPitchMin_N < 0) {
            setError(ErrVar, "PS_BldPitchMin_N must be greater than zero.");
        }

        // PS_WindSpeeds
        if (!NonDecreasing(CntrPar.PS_WindSpeeds.data(), (int)CntrPar.PS_WindSpeeds.size())) {
            setError(ErrVar, "PS_WindSpeeds must be non-decreasing.");
        }
    }

    // --- Startup ---
    if (CntrPar.SU_Mode > 0) {

        // SU_FW_MinDuration
        if (CntrPar.SU_FW_MinDuration < 0.0) {
            setError(ErrVar, "SU_FW_MinDuration must be greater than zero.");
        }

        // SU_RotorSpeedThresh
        if (CntrPar.SU_RotorSpeedThresh < 0.0) {
            setError(ErrVar, "SU_RotorSpeedThresh must be greater than zero.");
        }

        // SU_RotorSpeedCornerFreq
        if (CntrPar.SU_RotorSpeedCornerFreq < 0) {
            setError(ErrVar, "SU_RotorSpeedCornerFreq must be greater than or equal to 0.");
        }

        // SU_LoadStages_N
        if (CntrPar.SU_LoadStages_N < 0) {
            setError(ErrVar, "SU_LoadStages_N must be greater than or equal to 0.");
        }

        // SU_LoadStages
        {
            bool any_neg = false;
            for (int i = 0; i < (int)CntrPar.SU_LoadStages.size(); i++) {
                if (CntrPar.SU_LoadStages[i] < 0) { any_neg = true; break; }
            }
            if (any_neg) {
                setError(ErrVar, "SU_LoadStages must be positive.");
            }
        }

        // SU_LoadRampDuration
        {
            bool any_neg = false;
            for (int i = 0; i < (int)CntrPar.SU_LoadRampDuration.size(); i++) {
                if (CntrPar.SU_LoadRampDuration[i] < 0) { any_neg = true; break; }
            }
            if (any_neg) {
                setError(ErrVar, "SU_LoadRampDuration must be positive.");
            }
        }

        // SU_LoadHoldDuration
        {
            bool any_neg = false;
            for (int i = 0; i < (int)CntrPar.SU_LoadHoldDuration.size(); i++) {
                if (CntrPar.SU_LoadHoldDuration[i] < 0) { any_neg = true; break; }
            }
            if (any_neg) {
                setError(ErrVar, "SU_LoadHoldDuration must be positive.");
            }
        }
    }

    // --- Shutdown ---
    if (CntrPar.SD_Mode > 0) {

        // SD_Method
        if (CntrPar.SD_Method < 1 || CntrPar.SD_Method > 2) {
            setError(ErrVar, "SD_Method must be 1 or 2.");
        }

        // SD_MaxPitchRate
        {
            double maxval = *std::max_element(CntrPar.SD_MaxPitchRate.data(),
                                              CntrPar.SD_MaxPitchRate.data() + CntrPar.SD_MaxPitchRate.size());
            if (maxval > CntrPar.PC_MaxRat) {
                setError(ErrVar, "SD_MaxPitchRate(s) should be less than or equal to PC_MaxRat.");
            }
        }

        // SD_MaxTorqueRate
        {
            double maxval = *std::max_element(CntrPar.SD_MaxTorqueRate.data(),
                                              CntrPar.SD_MaxTorqueRate.data() + CntrPar.SD_MaxTorqueRate.size());
            if (maxval > CntrPar.VS_MaxRat) {
                setError(ErrVar, "SD_MaxTorqueRate(s) should be less than or equal to VS_MaxRat.");
            }
        }

        if (CntrPar.SD_Stage_N < 1) {
            setError(ErrVar, "SD_Stage_N must be greater than or equal to 1.");
        }

        if (CntrPar.SD_Method == 1) {
            // SD_StageTime must be greater than zero
            {
                double minval = *std::min_element(CntrPar.SD_StageTime.data(),
                                                  CntrPar.SD_StageTime.data() + CntrPar.SD_StageTime.size());
                if (minval < 0.0) {
                    setError(ErrVar, "SD_StageTime(s) must be greater than or equal to zero.");
                }
            }
        } else if (CntrPar.SD_Method == 2) {
            // SD_StagePitch must be increasing
            if (!NonDecreasing(CntrPar.SD_StagePitch.data(), (int)CntrPar.SD_StagePitch.size())) {
                setError(ErrVar, "SD_StagePitch must be non-decreasing.");
            }
        }
    }

    // --- Open loop control ---
    if (CntrPar.OL_Mode > 0) {
        // Get all open loop indices
        std::vector<int> All_OL_Indices;

        // Ind_BldPitch is a vector<int>
        for (int i = 0; i < (int)CntrPar.Ind_BldPitch.size(); i++) {
            All_OL_Indices.push_back(CntrPar.Ind_BldPitch[i]);
        }
        All_OL_Indices.push_back(CntrPar.Ind_GenTq);
        All_OL_Indices.push_back(CntrPar.Ind_YawRate);
        All_OL_Indices.push_back(CntrPar.Ind_R_Speed);
        All_OL_Indices.push_back(CntrPar.Ind_R_Torque);
        All_OL_Indices.push_back(CntrPar.Ind_R_Pitch);

        for (int i = 0; i < (int)CntrPar.Ind_CableControl.size(); i++) {
            All_OL_Indices.push_back(CntrPar.Ind_CableControl[i]);
        }

        for (int i = 0; i < (int)CntrPar.Ind_StructControl.size(); i++) {
            All_OL_Indices.push_back(CntrPar.Ind_StructControl[i]);
        }

        // ANY(All_OL_Indices < 0)
        {
            bool any_neg = false;
            for (size_t i = 0; i < All_OL_Indices.size(); i++) {
                if (All_OL_Indices[i] < 0) { any_neg = true; break; }
            }
            if (any_neg) {
                setError(ErrVar, "All open loop control indices must be greater than zero");
            }
        }

        if (CntrPar.Ind_Breakpoint < 1) {
            setError(ErrVar, "Ind_Breakpoint must be non-zero if OL_Mode is non-zero");
        }

        // ALL(All_OL_Indices < 1)
        {
            bool all_lt_1 = true;
            for (size_t i = 0; i < All_OL_Indices.size(); i++) {
                if (All_OL_Indices[i] >= 1) { all_lt_1 = false; break; }
            }
            if (all_lt_1) {
                setError(ErrVar, "At least one open loop input channel must be non-zero");
            }
        }

        if (CntrPar.OL_Mode == 2) {
            if ((CntrPar.Ind_BldPitch[0] == 0) ||
                (CntrPar.Ind_BldPitch[1] == 0) ||
                (CntrPar.Ind_BldPitch[2] == 0) ||
                (CntrPar.Ind_GenTq == 0) ||
                (CntrPar.Ind_Azimuth == 0)) {
                setError(ErrVar, "If OL_Mode = 2, Ind_BldPitch, Ind_GenTq, and Ind_Azimuth must be greater than zero");
            }
        }

        // ANY(Ind_CableControl > 0)
        {
            bool any_pos = false;
            for (int i = 0; i < (int)CntrPar.Ind_CableControl.size(); i++) {
                if (CntrPar.Ind_CableControl[i] > 0) { any_pos = true; break; }
            }
            if (any_pos && CntrPar.CC_Mode != 2) {
                setError(ErrVar, "CC_Mode must be 2 if using open loop cable control via Ind_CableControl");
            }
        }

        // ANY(Ind_StructControl > 0)
        {
            bool any_pos = false;
            for (int i = 0; i < (int)CntrPar.Ind_StructControl.size(); i++) {
                if (CntrPar.Ind_StructControl[i] > 0) { any_pos = true; break; }
            }
            if (any_pos && CntrPar.StC_Mode != 2) {
                // NOTE: Fortran bug reproduced — error message says "CC_Mode" but means "StC_Mode"
                setError(ErrVar, "CC_Mode must be 2 if using open loop struct control via Ind_StructControl");
            }
        }

        if ((CntrPar.OL_BP_Mode < 0) || (CntrPar.OL_BP_Mode > 1)) {
            setError(ErrVar, "OL_BP_Mode must be 0 or 1.");
        }

        if (CntrPar.OL_BP_FiltFreq < 0) {
            setError(ErrVar, "OL_BP_FiltFreq must be greater than or equal to 0.");
        }

        if ((CntrPar.OL_BP_Mode == 1) && (CntrPar.OL_Mode == 2)) {
            setError(ErrVar, "Rotor position control (OL_Mode = 2) is not compatible with wind speed breakpoints (OL_BP_Mode = 1)");
        }
    }

    // ---- AWC vs. IPC
    if (CntrPar.AWC_Mode > 0 && CntrPar.IPC_ControlMode > 0) {
        printf(" ROSCO WARNING: Individual pitch control and active wake control are both enabled. Performance may be compromised.\n");
    }

    // --- Pitch Actuator ---
    if (CntrPar.PA_Mode > 0) {
        if ((CntrPar.PA_Mode < 0) || (CntrPar.PA_Mode > 2)) {
            setError(ErrVar, "PA_Mode must be 0, 1, or 2");
        }
        if (CntrPar.PA_CornerFreq < 0) {
            setError(ErrVar, "PA_CornerFreq must be greater than 0");
        }
        if (CntrPar.PA_Damping < 0) {
            setError(ErrVar, "PA_Damping must be greater than 0");
        }
    }

    // --- Active Wake Control ---
    if (CntrPar.AWC_Mode > 0) {
        if (CntrPar.AWC_NumModes < 0) {
            setError(ErrVar, "AWC_NumModes must be a positive integer if AWC_Mode = 1");
        }
        for (Imode = 0; Imode < CntrPar.AWC_NumModes; Imode++) {
            if (CntrPar.AWC_freq[Imode] < 0.0) {
                setError(ErrVar, "AWC_freq cannot be less than 0");
            }
        }
        if (CntrPar.AWC_Mode == 1) {
            for (Imode = 0; Imode < CntrPar.AWC_NumModes; Imode++) {
                if ((CntrPar.AWC_clockangle[Imode] > 360.0) || (CntrPar.AWC_clockangle[Imode] < 0.0)) {
                    setError(ErrVar, "AWC_clockangle must be between 0 and 360 in AWC_Mode = 1");
                }
            }
        }

        if (CntrPar.AWC_Mode == 2) {
            if ((CntrPar.AWC_NumModes > 2) || (CntrPar.AWC_NumModes < 1)) {
                setError(ErrVar, "AWC_NumModes must be either 1 or 2 if AWC_Mode = 2");
            }
            for (Imode = 0; Imode < CntrPar.AWC_NumModes; Imode++) {
                if ((CntrPar.AWC_clockangle[Imode] > 360.0) || (CntrPar.AWC_clockangle[Imode] < -360.0)) {
                    setError(ErrVar, "AWC_clockangle must be between -360 and 360 in AWC_Mode = 2");
                }
                if (CntrPar.AWC_harmonic[Imode] < 0) {
                    setError(ErrVar, "AWC_harmonic must be a positive integer");
                }
            }
        }
    }

    if ((CntrPar.CC_Mode < 0) || (CntrPar.CC_Mode > 2)) {
        setError(ErrVar, "CC_Mode must be 0 or 1");
    }

    if (CntrPar.CC_Mode > 0) {

        // Extended avrSWAP must be used
        if (CntrPar.Ext_Interface == 0) {
            setError(ErrVar, "The OpenFAST extended bladed interface must be used with Ext_Interface > 0 in the DISCON");
        }

        if (CntrPar.CC_ActTau <= 0) {
            setError(ErrVar, "CC_ActTau must be greater than 0.");
        }

        for (I = 0; I < CntrPar.CC_Group_N; I++) {
            if (CntrPar.CC_GroupIndex[I] < 2601) {
                setError(ErrVar, "CC_GroupIndices must be greater than 2601.");
            }
        }
    }

    if (CntrPar.StC_Mode > 0) {

        // Extended avrSWAP must be used
        if (CntrPar.Ext_Interface == 0) {
            setError(ErrVar, "The OpenFAST extended bladed interface must be used with Ext_Interface > 0 in the DISCON");
        }

        // Check indices
        for (I = 0; I < CntrPar.StC_Group_N; I++) {
            if (CntrPar.StC_GroupIndex[I] < 2801) {
                setError(ErrVar, "StC_GroupIndices must be greater than 2801.");
            }
        }
    }

    // Check that open loop control active if using open loop cable/struct control
    if (CntrPar.CC_Mode == 2 && CntrPar.OL_Mode != 1) {
        setError(ErrVar, "OL_Mode must be 1 if using CC_Mode = 2 (open loop)");
    }

    if (CntrPar.StC_Mode == 2 && CntrPar.OL_Mode != 1) {
        setError(ErrVar, "OL_Mode must be 1 if using StC_Mode = 2 (open loop)");
    }

    // Abort if the user has not requested a pitch angle actuator (See Appendix A
    // of Bladed User's Guide):
    if (static_cast<int>(std::round(avrSWAP[9])) != 0) {  // avrSWAP(10) -> [9]
        setError(ErrVar, "Pitch angle actuator not requested.");
    }

    if ((static_cast<int>(std::round(avrSWAP[27])) == 0) &&  // avrSWAP(28) -> [27]
        ((CntrPar.IPC_ControlMode > 0) ||
         (CntrPar.Y_ControlMode > 1) ||
         (CntrPar.Ind_BldPitch[1] > 0) ||    // Ind_BldPitch(2) -> [1]
         (CntrPar.Ind_BldPitch[2] > 0)        // Ind_BldPitch(3) -> [2]
        )) {
        setError(ErrVar, "IPC enabled, but Ptch_Cntrl in ServoDyn has a value of 0. Set it to 1 for individual pitch control.");
    }

    // PF_Mode = 1
    if (static_cast<int>(std::round(avrSWAP[27])) == 0 && (CntrPar.PF_Mode == 1)) {
        setError(ErrVar, "Pitch offset fault enabled (PF_Mode = 1), but Ptch_Cntrl in ServoDyn has a value of 0. Set it to 1 for individual pitch control.");
    }

    if (static_cast<int>(std::round(avrSWAP[27])) == 0 && (CntrPar.AWC_Mode > 1)) {
        setError(ErrVar, "AWC enabled, but Ptch_Cntrl in ServoDyn has a value of 0. Set it to 1 for individual pitch control.");
    }

    // DT
    if (LocalVar.DT <= 0.0) {
        setError(ErrVar, "DT must be greater than zero.");
    }

    if (ErrVar->aviFAIL < 0) {
        // Prepend "CheckInputs:" to ErrMsg
        // Fortran: RoutineName//':'//TRIM(ErrVar%ErrMsg)
        // ErrMsg is space-padded; find the last non-space character
        int msgLen = 1024;
        while (msgLen > 0 && ErrVar->ErrMsg[msgLen - 1] == ' ') {
            msgLen--;
        }
        // Build the prefixed message
        const char* prefix = "CheckInputs:";
        size_t prefixLen = std::strlen(prefix);
        // Temporary buffer for the combined message
        char tmp[1024];
        std::memset(tmp, ' ', 1024);
        size_t totalLen = prefixLen + msgLen;
        if (totalLen > 1024) totalLen = 1024;
        std::memcpy(tmp, prefix, prefixLen);
        size_t copyLen = (totalLen > prefixLen) ? (totalLen - prefixLen) : 0;
        if (copyLen > 0) {
            std::memcpy(tmp + prefixLen, ErrVar->ErrMsg, copyLen);
        }
        std::memcpy(ErrVar->ErrMsg, tmp, 1024);
    }
}
