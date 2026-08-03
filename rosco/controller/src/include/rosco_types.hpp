// AUTO-GENERATED from rosco_types.yaml
// Do not edit manually — run write_registry.py to regenerate.
#pragma once
#include <vector>
#include <string>
#include "rosco_array.hpp"
#include "vit_types.h"

struct ControlParameters {
    // 0000 - 9999, Identifier of the rosco, used for zeromq interface only
    int      ZMQ_ID = 0;
    // 0 - write no debug files, 1 - write standard output .dbg-file, 2 - write standard output .dbg-file and complete avrSWAP-...
    int      LoggingLevel = 0;
    // 0 - text output (.dbg files), 1 - HDF5 binary output (.RO.h5)
    int      OutputFormat = 0;
    // 0 - no Echo, 1 - Echo input data to <RootName>.echo
    int      Echo = 0;
    // 0 - use standard bladed interface, 1 - Use the extened DLL interface introduced in OpenFAST 3.5.0.
    int      Ext_Interface = 0;
    // Output time step
    double   DT_Out = 0.0;
    // output every this many steps
    int      n_DT_Out = 0;
    // Send measurements to ZMQ after this many time steps
    int      n_DT_ZMQ = 0;

    // --- Filters ---
    // Low pass filter on the rotor and generator speed {1 - first-order low-pass filter, 2 - second-order low-pass filter}, [r...
    int      F_LPFType = 0;
    // Corner frequency (-3dB point) in the first-order low-pass filter, [rad/s]
    double   F_LPFCornerFreq = 0.0;
    // Damping coefficient [used only when F_FilterType = 2]
    double   F_LPFDamping = 0.0;
    // Number of notch filters
    int      F_NumNotchFilts = 0;
    // Number of gen speed notch filters
    int      F_GenSpdNotch_N = 0;
    // Indices of gen speed notch filters
    std::vector<int> F_GenSpdNotch_Ind;
    // Number of tower top notch filters
    int      F_TwrTopNotch_N = 0;
    // Indices of tower top notch filters
    std::vector<int> F_TwrTopNotch_Ind;
    // Natural frequencies of notch filters, [rad/s]
    ParamArray F_NotchFreqs;
    // Notch Filter Numerator damping (determines width)
    ParamArray F_NotchBetaNum;
    // Notch Filter Numerator damping (determines depth?)
    ParamArray F_NotchBetaDen;
    // Corner frequency (-3dB point) in the first order low pass filter for the setpoint smoother [rad/s]
    double   F_SSCornerFreq = 0.0;
    // Corner frequency (-3dB point) in the first order low pass filter for the wind speed estimate [rad/s]
    double   F_WECornerFreq = 0.0;
    // Corner frequency (-3dB point) in the second order low pass filter of the tower-top fore-aft motion for floating feedback...
    ParamArray F_FlCornerFreq;
    // Natural frequency of first-roder high-pass filter for nacelle fore-aft motion [rad/s].
    double   F_FlHighPassFreq = 0.0;
    // Corner low pass filter corner frequency for yaw controller [rad/s].
    double   F_YawErr = 0.0;
    // Corner frequency (-3dB point) in the second order low pass filter of the blade root bending moment for flap control [rad...
    ParamArray F_FlpCornerFreq;
    // Corner frequency (-3dB point) in the first order low pass filter of the generator speed reference used for TSR tracking ...
    double   F_VSRefSpdCornerFreq = 0.0;

    // --- Tower fore-aft damping ---
    // Tower Fore-Aft control mode {0 - no fore-aft control, 1 - Tower fore-aft damping, 2 -Frequency exclusion zone, 3- Option...
    int      TRA_Mode = 0;
    // Rotor speed for exclusion [LSS] [rad/s]
    double   TRA_ExclSpeed = 0.0;
    // One-half of the total frequency exclusion band. Torque controller reference will be TRA_ExclFreq +/- TRA_ExlBand [rad/s]
    double   TRA_ExclBand = 0.0;
    // Time constant for gain change when in exclusion zone [s]
    double   TRA_RateLimit = 0.0;
    // Tower Fore-Aft control mode {0 - no fore-aft control, 1 - Tower fore-aft damping, 2 -Frequency exclusion zone, 3- Option...
    int      TD_Mode = 0;
    // Corner frequency (-3dB point) in the high-pass filter on the fore-aft acceleration signal [rad/s]
    double   FA_HPFCornerFreq = 0.0;
    // Integrator saturation (maximum signal amplitude contrbution to pitch from FA damper), [rad]
    double   FA_IntSat = 0.0;
    // Integral gain for the fore-aft tower damper controller, -1 = off / >0 = on [rad s/m]
    double   FA_KI = 0.0;

    // --- Individual Pitch Control ---
    // Turn Individual Pitch Control (IPC) for fatigue load reductions (pitch contribution) {0 - off, 1 - 1P reductions, 2 - 1P...
    int      IPC_ControlMode = 0;
    // Wind speeds for IPC cut-in sigma function [m/s]
    ParamArray IPC_Vramp;
    // Integrator saturation (maximum signal amplitude contrbution to pitch from IPC)
    double   IPC_IntSat = 0.0;
    // IPC Saturation method IPC Saturation method (0 - no saturation (except by PC_MinPit), 1 - saturate by PS_BldPitchMin, 2 ...
    int      IPC_SatMode = 0;
    // Integral gain for the individual pitch controller, [-].
    ParamArray IPC_KP;
    // Integral gain for the individual pitch controller, [-].
    ParamArray IPC_KI;
    // Phase offset added to the azimuth angle for the individual pitch controller, [rad].
    ParamArray IPC_aziOffset;
    // Corner frequency of the first-order actuators model, to induce a phase lag in the IPC signal {0 - Disable}, [rad/s]
    double   IPC_CornerFreqAct = 0.0;

    // --- Collective Pitch Controller ---
    // Blade pitch control mode {0 - No pitch, fix to fine pitch, 1 - active PI blade pitch control}
    int      PC_ControlMode = 0;
    // Amount of gain-scheduling table entries
    int      PC_GS_n = 0;
    // Gain-schedule table - pitch angles
    ParamArray PC_GS_angles;
    // Gain-schedule table - pitch controller kp gains
    ParamArray PC_GS_KP;
    // Gain-schedule table - pitch controller ki gains
    ParamArray PC_GS_KI;
    // Gain-schedule table - pitch controller kd gains
    ParamArray PC_GS_KD;
    // Gain-schedule table - pitch controller tf gains (derivative filter)
    ParamArray PC_GS_TF;
    // Maximum physical pitch limit, [rad].
    double   PC_MaxPit = 0.0;
    // Minimum physical pitch limit, [rad].
    double   PC_MinPit = 0.0;
    // Maximum pitch rate (in absolute value) in pitch controller, [rad/s].
    double   PC_MaxRat = 0.0;
    // Minimum pitch rate (in absolute value) in pitch controller, [rad/s].
    double   PC_MinRat = 0.0;
    // Desired (reference) HSS speed for pitch controller, [rad/s].
    double   PC_RefSpd = 0.0;
    // Record 5 - Below-rated pitch angle set-point (deg) [used only with Bladed Interface]
    double   PC_FinePit = 0.0;
    // Angle above lowest minimum pitch angle for switch [rad]
    double   PC_Switch = 0.0;

    // --- Generator Torque Controller ---
    // Generator torque control mode in below rated conditions {0 - no torque control, 1 - komega^2 with PI trans, 2 - WSE TSR ...
    int      VS_ControlMode = 0;
    // Constant power torque control
    int      VS_ConstPower = 0;
    // Fixed blade pitch control mode in above rated conditions {0 - variable pitch (defer to PC_ControlMode and VS_ConstPower)...
    int      VS_FBP = 0;
    // Generator efficiency mechanical power -> electrical power [-]
    double   VS_GenEff = 0.0;
    // Above rated generator torque PI control saturation, [Nm]
    double   VS_ArSatTq = 0.0;
    // Maximum torque rate (in absolute value) in torque controller, [Nm/s].
    double   VS_MaxRat = 0.0;
    // Maximum generator torque in Region 3 (HSS side), [Nm].
    double   VS_MaxTq = 0.0;
    // Minimum generator torque (HSS side), [Nm].
    double   VS_MinTq = 0.0;
    // Optimal mode minimum speed, [rad/s]
    double   VS_MinOMSpd = 0.0;
    // Generator torque constant in Region 2 (HSS side), N-m/(rad/s)^2
    double   VS_Rgn2K = 0.0;
    // Wind turbine rated power [W]
    double   VS_RtPwr = 0.0;
    // Rated torque, [Nm].
    double   VS_RtTq = 0.0;
    // Rated generator speed [rad/s]
    double   VS_RefSpd = 0.0;
    // Number of controller gains
    int      VS_n = 0;
    // Proportional gain for generator PI torque controller, used in the transitional 2.5 region
    ParamArray VS_KP;
    // Integral gain for generator PI torque controller, used in the transitional 2.5 region
    ParamArray VS_KI;
    // Power-maximizing region 2 tip-speed ratio [rad]
    double   VS_TSRopt = 0.0;
    // Number of operating schedule entries for fixed blade pitch control
    int      VS_FBP_n = 0;
    // Operating schedule for fixed blade pitch control - Wind speed
    ParamArray VS_FBP_U;
    // Operating schedule for fixed blade pitch control - Generator speed
    ParamArray VS_FBP_Omega;
    // Operating schedule for fixed blade pitch control - Generator torque
    ParamArray VS_FBP_Tau;

    // --- Setpoint Smoother ---
    // Setpoint Smoother mode {0 - no setpoint smoothing, 1 - introduce setpoint smoothing}
    int      SS_Mode = 0;
    // Variable speed torque controller setpoint smoother gain, [-].
    double   SS_VSGain = 0.0;
    // Collective pitch controller setpoint smoother gain, [-].
    double   SS_PCGain = 0.0;

    // --- Power reference tracking ---
    // Power reference tracking mode, 0- use standard rotor speed set points, 1- use PRC rotor speed setpoints
    int      PRC_Mode = 0;
    // Power reference communication mode, 0- use constant DISCON inputs, 1- use open loop inputs, 2- use ZMQ inputs
    int      PRC_Comm = 0;
    // Array of wind speeds used in rotor speed vs. wind speed lookup table
    ParamArray PRC_WindSpeeds;
    // Array of rotor speeds corresponding to PRC_WindSpeeds
    ParamArray PRC_GenSpeeds;
    // Number of elements in PRC_WindSpeeds and PRC_GenSpeeds array
    int      PRC_n = 0;
    // Frequency of the low pass filter on the wind speed estimate used to set PRC_GenSpeeds [rad/s]
    double   PRC_LPF_Freq = 0.0;
    // Power rating through changing the rated torque, default is 1, effective above rated [-]
    double   PRC_R_Torque = 0.0;
    // Power rating through changing the rated generator speed, default is 1, effective above rated [-]
    double   PRC_R_Speed = 0.0;
    // Power rating through changing the fine pitch angle, default is 1, effective below rated [-]
    double   PRC_R_Pitch = 0.0;
    // Number of elements in PRC_R to _Pitch table
    int      PRC_Table_n = 0;
    // Table of fine pitch versus PRC_R_Table, length should be PRC_Table_n [rad]
    ParamArray PRC_Pitch_Table;
    // Table of turbine rating versus fine pitch (PRC_Pitch_Table), length should be PRC_Table_n, default is 1 [-]
    ParamArray PRC_R_Table;

    // --- Wind Speed Estimator ---
    // Wind speed estimator mode {0 - One-second low pass filtered hub height wind speed, 1 - Imersion and Invariance Estimator...
    int      WE_Mode = 0;
    // Blade length [m]
    double   WE_BladeRadius = 0.0;
    // Amount of parameters in the Cp array
    int      WE_CP_n = 0;
    // Parameters that define the parameterized CP(\lambda) function
    ParamArray WE_CP;
    // Adaption gain of the wind speed estimator algorithm [m/rad]
    double   WE_Gamma = 0.0;
    // Gearbox ratio, >=1  [-]
    double   WE_GearboxRatio = 0.0;
    // Total drivetrain inertia, including blades, hub and casted generator inertia to LSS [kg m^2]
    double   WE_Jtot = 0.0;
    // Air density [kg m^-3]
    double   WE_RhoAir = 0.0;
    // File containing rotor performance tables (Cp,Ct,Cq)
    std::string PerfFileName;
    // Size of rotor performance tables, first number refers to number of blade pitch angles, second number referse to number o...
    std::vector<int> PerfTableSize;
    // Number of first-order system poles used in EKF
    int      WE_FOPoles_N = 0;
    // Wind speeds corresponding to first-order system poles [m/s]
    ParamArray WE_FOPoles_v;
    // First order system poles
    ParamArray WE_FOPoles;

    // --- Yaw Controller ---
    // Yaw control mode {0 - no yaw control, 1 - yaw rate control}
    int      Y_ControlMode = 0;
    // Wind speed to switch between Y_ErrThresh. If zero, only the first value of Y_ErrThresh is used [m/s]
    double   Y_uSwitch = 0.0;
    // Error threshold [rad]. Turbine begins to yaw when it passes this
    ParamArray Y_ErrThresh;
    // Yaw rate [rad/s]
    double   Y_Rate = 0.0;
    // Yaw alignment error, setpoint (for wake steering) [rad]
    double   Y_MErrSet = 0.0;
    // Integrator saturation (maximum signal amplitude contrbution to pitch from yaw-by-IPC)
    double   Y_IPC_IntSat = 0.0;
    // Yaw-by-IPC proportional controller gain Kp
    double   Y_IPC_KP = 0.0;
    // Yaw-by-IPC integral controller gain Ki
    double   Y_IPC_KI = 0.0;

    // --- Pitch Saturation ---
    // Pitch saturation mode {0 - no peak shaving, 1 -  implement pitch saturation}
    int      PS_Mode = 0;
    // Number of values in minimum blade pitch lookup table (should equal number of values in PS_WindSpeeds and PS_BldPitchMin)
    int      PS_BldPitchMin_N = 0;
    // Wind speeds corresponding to minimum blade pitch angles [m/s]
    ParamArray PS_WindSpeeds;
    // Minimum blade pitch angles [rad]
    ParamArray PS_BldPitchMin;

    // --- Startup ---
    // Startup mode {0 - no startup procedure, 1 - enable startup}
    int      SU_Mode = 0;
    // Time to start start up routine
    double   SU_StartTime = 0.0;
    // Free-wheel minimum duration [s]
    double   SU_FW_MinDuration = 0.0;
    // Rotor speed threshhold to switch from freewheel to loads [rad/s]
    double   SU_RotorSpeedThresh = 0.0;
    // Cutoff Frequency for first order low-pass filter for rotor speed for startup, [rad/s]
    double   SU_RotorSpeedCornerFreq = 0.0;
    // Number of load staged for startup (should equal number of values in SU_LoadStages, SU_LoadRampDuration and SU_LoadHoldDu...
    int      SU_LoadStages_N = 0;
    // Array containing loads as a fraction of full generator torque during startup [-]
    ParamArray SU_LoadStages;
    // Array containing ramp duration to reach the corresponding partial loads during startup [s]
    ParamArray SU_LoadRampDuration;
    // Array containing duration to hold the partial loads during startup [s]
    ParamArray SU_LoadHoldDuration;

    // --- Shutdown ---
    // Shutdown mode {0 - no shutdown procedure, 1 - enable shutdown}
    int      SD_Mode = 0;
    // Time to acitvate shutdown modes, [s]
    double   SD_TimeActivate = 0.0;
    // Shutdown when collective blade pitch exceeds a threshold, [-]
    int      SD_EnablePitch = 0;
    // Shutdown when yaw error exceeds a threshold, [-]
    int      SD_EnableYawError = 0;
    // Shutdown when generator speed exceeds a threshold, [-]
    int      SD_EnableGenSpeed = 0;
    // Shutdown at a predefined time, [-]
    int      SD_EnableTime = 0;
    // Maximum blade pitch angle to initiate shutdown, [rad]
    double   SD_MaxPit = 0.0;
    // Cutoff Frequency for first order low-pass filter for blade pitch angle for shutdown, [rad/s]
    double   SD_PitchCornerFreq = 0.0;
    // Maximum yaw error to initiate shutdown, [deg]
    double   SD_MaxYawError = 0.0;
    // Cutoff Frequency for first order low-pass filter for yaw error for shutdown, [rad/s]
    double   SD_YawErrorCornerFreq = 0.0;
    // Maximum generator speed to initiate shutdown, [rad/s]
    double   SD_MaxGenSpd = 0.0;
    // Cutoff Frequency for first order low-pass filter for generator speed for shutdown, [rad/s]
    double   SD_GenSpdCornerFreq = 0.0;
    // Shutdown time, [s]
    double   SD_Time = 0.0;
    // Shutdown method {1 - Reduce generator torque and increase blade pitch}, [-]
    int      SD_Method = 0;
    // Maximum torque rate for shutdown, [Nm/s]
    ParamArray SD_MaxTorqueRate;
    // Maximum pitch rate used for shutdown, [rad/s]
    ParamArray SD_MaxPitchRate;
    // Array containing the pitch angle to reach in each shutdown stage [rad]
    ParamArray SD_StagePitch;
    // Array containing the time to spend in each shutdown stage [s]
    ParamArray SD_StageTime;
    // Number of shutdown stages (should equal number of values in SD_MaxPitchRate and SD_MaxTorqueRate) [-]
    int      SD_Stage_N = 0;

    // --- Floating ---
    // Floating specific feedback mode {0 - no nacelle velocity feedback, 1 - nacelle velocity feedback}
    int      Fl_Mode = 0;
    // Number of Fl_Kp for gain scheduling
    int      Fl_n = 0;
    // Nacelle velocity proportional feedback gain [s]
    ParamArray Fl_Kp;
    // Wind speeds for scheduling Fl_Kp [m/s]
    ParamArray Fl_U;

    // --- Trailing edge flaps ---
    // Flap actuator mode {0 - off, 1 - fixed flap position, 2 - PI flap control}
    int      Flp_Mode = 0;
    // Fixed flap angle (degrees)
    double   Flp_Angle = 0.0;
    // PI flap control proportional gain
    double   Flp_Kp = 0.0;
    // PI flap control integral gain
    double   Flp_Ki = 0.0;
    // Maximum (and minimum) flap pitch angle [rad]
    double   Flp_MaxPit = 0.0;

    // --- Open-loop Control ---
    // Input file with open loop timeseries
    std::string OL_Filename;
    // Open loop control mode {0 - no open loop control, 1 - open loop control vs. time, 2 - open loop control vs. wind speed}
    int      OL_Mode = 0;
    // Open loop control mode {0 - no open loop control, 1 - open loop control vs. time, 2 - open loop control vs. wind speed}
    int      OL_BP_Mode = 0;
    // First order low pass filter cutoff frequency for open loop breakpoint
    double   OL_BP_FiltFreq = 0.0;
    // The column in OL_Filename that contains the breakpoint (time if OL_Mode = 1)
    int      Ind_Breakpoint = 0;
    // The columns in OL_Filename that contains the blade pitch inputs (1,2,3) in rad
    std::vector<int> Ind_BldPitch;
    // The column in OL_Filename that contains the generator torque in Nm
    int      Ind_GenTq = 0;
    // The column in OL_Filename that contains the generator torque in Nm
    int      Ind_YawRate = 0;
    // The column in OL_Filename that contains the R_Speed input
    int      Ind_R_Speed = 0;
    // The column in OL_Filename that contains the R_Torque input
    int      Ind_R_Torque = 0;
    // The column in OL_Filename that contains the R_Pitch input
    int      Ind_R_Pitch = 0;
    // The column in OL_Filename that contains the desired azimuth position in rad (used if OL_Mode = 2)
    int      Ind_Azimuth = 0;
    // PID gains and Tf on derivative term for rotor position control (used if OL_Mode = 2)
    ParamArray RP_Gains;
    // The column in OL_Filename that contains the cable control inputs in m
    std::vector<int> Ind_CableControl;
    // The column in OL_Filename that contains the structural control inputs in various units
    std::vector<int> Ind_StructControl;
    // Open loop breakpoints in timeseries
    ParamArray OL_Breakpoints;
    // Open loop blade pitch 1 timeseries
    ParamArray OL_BldPitch1;
    // Open loop blade pitch 2 timeseries
    ParamArray OL_BldPitch2;
    // Open loop blade pitch 3 timeseries
    ParamArray OL_BldPitch3;
    ParamArray OL_CableControl;
    int OL_CableControl_rows = 0;
    int OL_CableControl_cols = 0;
    ParamArray OL_StructControl;
    int OL_StructControl_rows = 0;
    int OL_StructControl_cols = 0;
    // Open loop generator torque timeseries
    ParamArray OL_GenTq;
    // Open loop yaw rate timeseries
    ParamArray OL_YawRate;
    // Open loop azimuth timeseries
    ParamArray OL_Azimuth;
    // Open loop R_Speed timeseries
    ParamArray OL_R_Speed;
    // Open loop R_Torque timeseries
    ParamArray OL_R_Torque;
    // Open loop R_Pitch timeseries
    ParamArray OL_R_Pitch;
    // Open loop channels in timeseries
    ParamArray OL_Channels;
    int OL_Channels_rows = 0;
    int OL_Channels_cols = 0;

    // --- Pitch actuator ---
    // Pitch actuator mode {0 - not used, 1 - first order filter, 2 - second order filter}
    int      PA_Mode = 0;
    // Pitch actuator bandwidth/cut-off frequency [rad/s]
    double   PA_CornerFreq = 0.0;
    // Pitch actuator damping ratio [-, unused if PA_Mode = 1]
    double   PA_Damping = 0.0;

    // --- Active wake control ---
    // Active wake control mode [0 - unused, 1 - complex number method, 2 - Coleman transform method]
    int      AWC_Mode = 0;
    // AWC- Number of modes to include [-]
    int      AWC_NumModes = 0;
    // AWC azimuthal mode [-]
    std::vector<int> AWC_n;
    // AWC Coleman transform harmonic [-]
    std::vector<int> AWC_harmonic;
    // AWC frequency [Hz]
    ParamArray AWC_freq;
    // AWC amplitude [deg]
    ParamArray AWC_amp;
    // AWC clocking angle [deg]
    ParamArray AWC_clockangle;
    // AWC azimuth offset for Coleman transform [deg]
    double   AWC_phaseoffset = 0.0;
    // AWC KP and KI/KR gain of the controller [-]
    ParamArray AWC_CntrGains;

    // --- Pitch actuator error ---
    // Pitch actuator fault mode {0 - not used, 1 - offsets on one or more blades}
    int      PF_Mode = 0;
    // Pitch actuator fault offsets for blade 1-3 [rad/s]
    ParamArray PF_Offsets;
    // Time for pitch actuator fault to be stuck for blade 1-3 [s]
    ParamArray PF_TimeStuck;

    // --- External Control ---
    // External control mode (0 - not used, 1 - call external control library)
    int      Ext_Mode = 0;
    // File name of external dynamic library
    std::string DLL_FileName;
    // Name of input file called by dynamic library (DISCON.IN, e.g.)
    std::string DLL_InFile;
    // Process name of subprocess called in DLL_Filename (Usually DISCON)
    std::string DLL_ProcName;

    // --- ZeroMQ ---
    // Flag for ZeroMQ (0-off, 1-yaw}
    int      ZMQ_Mode = 0;
    // Comm Address to zeroMQ client
    std::string ZMQ_CommAddress;
    // Integer for zeromq update frequency
    double   ZMQ_UpdatePeriod = 0.0;

    // --- Cable control ---
    // Flag for ZeroMQ (0-off, 1-yaw}
    int      CC_Mode = 0;
    // Number of cable control groups
    int      CC_Group_N = 0;
    // Time constant for line actuator [s]
    double   CC_ActTau = 0.0;
    // Cable control group indices
    std::vector<int> CC_GroupIndex;

    // --- StC Control ---
    // Flag for StC Control
    int      StC_Mode = 0;
    // Number of cable control groups
    int      StC_Group_N = 0;
    // Cable control group indices
    std::vector<int> StC_GroupIndex;

    // --- Calculated ---
    // 99% of the rated torque value, using for switching between pitch and torque control, [Nm].
    double   PC_RtTq99 = 0.0;
    // Maximum torque at the end of the below-rated region 2, [Nm]
    double   VS_MaxOMTq = 0.0;
    // Minimum torque at the beginning of the below-rated region 2, [Nm]
    double   VS_MinOMTq = 0.0;

    // Load all parameters from a TOML file (replaces DISCON.IN two-pass parser)
    bool load_from_toml(const std::string& path, errorvariables_t* err);

    // Populate legacy controlparameters_view_t for translated functions
    void populate_view(controlparameters_view_t* v) const;

    // Copy all fields from a populated controlparameters_view_t (DISCON.IN path bridge)
    void sync_from_view(const controlparameters_view_t& v);
};
