// Shared checkpoint field helpers for WriteRestartFile and ReadRestartFile.
// Both functions use checkpoint_fields() with the same field order to guarantee
// binary checkpoint consistency.

#ifndef RESTART_FIELDS_H
#define RESTART_FIELDS_H

#include "vit_types.h"
#include <fstream>
#include <cmath>
#include <string>
#include <cstring>
#include <cstdio>

namespace {

std::string trim_fortran_string(const char* s, int len) {
    int end = len - 1;
    while (end >= 0 && (s[end] == ' ' || s[end] == '\0')) end--;
    return std::string(s, end + 1);
}

template<typename T>
void write_field(std::ofstream& f, const T& val) {
    f.write(reinterpret_cast<const char*>(&val), sizeof(val));
}

template<typename T>
void read_field(std::ifstream& f, T& val) {
    f.read(reinterpret_cast<char*>(&val), sizeof(val));
}

// Vector overloads: write/read count + elements as raw bytes.
// Format: uint32_t n, then n * sizeof(T) bytes of element data.
template<typename T>
void write_field(std::ofstream& f, const std::vector<T>& v) {
    uint32_t n = static_cast<uint32_t>(v.size());
    f.write(reinterpret_cast<const char*>(&n), sizeof(n));
    if (n > 0)
        f.write(reinterpret_cast<const char*>(v.data()), n * sizeof(T));
}

template<typename T>
void read_field(std::ifstream& f, std::vector<T>& v) {
    uint32_t n = 0;
    f.read(reinterpret_cast<char*>(&n), sizeof(n));
    v.resize(n);
    if (n > 0)
        f.read(reinterpret_cast<char*>(v.data()), n * sizeof(T));
}

// Shared field order for Write and Read — follows ROSCO_IO.f90 lines 39-349 exactly.
template<typename Stream, typename FieldOp>
void checkpoint_fields(Stream& f, LocalVariables& LocalVar, FieldOp field_op) {
    // --- LocalVar scalars ---
    field_op(f, LocalVar.iStatus);
    field_op(f, LocalVar.AlreadyInitialized);
    field_op(f, LocalVar.RestartWSE);
    field_op(f, LocalVar.Time);
    field_op(f, LocalVar.DT);
    field_op(f, LocalVar.WriteThisStep);
    field_op(f, LocalVar.n_DT);
    field_op(f, LocalVar.Time_Last);
    field_op(f, LocalVar.VS_GenPwr);
    field_op(f, LocalVar.GenSpeed);
    field_op(f, LocalVar.RotSpeed);
    field_op(f, LocalVar.NacHeading);
    field_op(f, LocalVar.NacVane);
    field_op(f, LocalVar.NacVaneF);
    field_op(f, LocalVar.WindDir);
    field_op(f, LocalVar.HorWindV);
    field_op(f, LocalVar.HorWindV_F);

    for (int i = 0; i < 3; i++) field_op(f, LocalVar.rootMOOP[i]);
    for (int i = 0; i < 3; i++) field_op(f, LocalVar.rootMOOPF[i]);
    for (int i = 0; i < 3; i++) field_op(f, LocalVar.BlPitch[i]);

    field_op(f, LocalVar.BlPitchCMeas);
    field_op(f, LocalVar.Azimuth);
    field_op(f, LocalVar.OL_Azimuth);
    field_op(f, LocalVar.AzUnwrapped);
    field_op(f, LocalVar.AzError);
    field_op(f, LocalVar.GenTqAz);
    for (int i = 0; i < 2; i++) field_op(f, LocalVar.AzBuffer[i]);
    field_op(f, LocalVar.NumBl);
    field_op(f, LocalVar.FA_Acc_TT);
    field_op(f, LocalVar.SS_Acc_TT);
    field_op(f, LocalVar.FA_Acc_Nac);
    field_op(f, LocalVar.NacIMU_FA_RAcc);
    field_op(f, LocalVar.FA_AccHPF);
    field_op(f, LocalVar.FA_AccHPFI);
    for (int i = 0; i < 3; i++) field_op(f, LocalVar.FA_PitCom[i]);

    field_op(f, LocalVar.VS_RefSpd);
    field_op(f, LocalVar.VS_RefSpd_TSR);
    field_op(f, LocalVar.VS_RefSpd_TRA);
    field_op(f, LocalVar.VS_RefSpd_RL);
    field_op(f, LocalVar.PC_RefSpd);
    field_op(f, LocalVar.PC_RefSpd_SS);
    field_op(f, LocalVar.PC_RefSpd_PRC);
    field_op(f, LocalVar.RotSpeedF);
    field_op(f, LocalVar.GenSpeedF);
    field_op(f, LocalVar.GenTq);
    field_op(f, LocalVar.GenTqMeas);
    field_op(f, LocalVar.GenArTq);
    field_op(f, LocalVar.GenBrTq);
    field_op(f, LocalVar.VS_KOmega2_GenTq);
    field_op(f, LocalVar.VS_ConstPwr_GenTq);
    for (int i = 0; i < 3; i++) field_op(f, LocalVar.IPC_PitComF[i]);

    field_op(f, LocalVar.PC_KP);
    field_op(f, LocalVar.PC_KI);
    field_op(f, LocalVar.PC_KD);
    field_op(f, LocalVar.PC_TF);
    field_op(f, LocalVar.PC_MaxPit);
    field_op(f, LocalVar.PC_MinPit);
    field_op(f, LocalVar.PC_PitComT);
    field_op(f, LocalVar.PC_PitComT_Last);
    field_op(f, LocalVar.BlPitchCMeasF);
    for (int i = 0; i < 3; i++) field_op(f, LocalVar.PC_PitComT_IPC[i]);
    field_op(f, LocalVar.PC_PwrErr);
    field_op(f, LocalVar.PC_SpdErr);
    field_op(f, LocalVar.IPC_AxisTilt_1P);
    field_op(f, LocalVar.IPC_AxisYaw_1P);
    field_op(f, LocalVar.IPC_AxisTilt_2P);
    field_op(f, LocalVar.IPC_AxisYaw_2P);
    field_op(f, LocalVar.axisTilt_1P);
    field_op(f, LocalVar.axisYaw_1P);
    field_op(f, LocalVar.axisYawF_1P);
    field_op(f, LocalVar.axisTilt_2P);
    field_op(f, LocalVar.axisYaw_2P);
    field_op(f, LocalVar.axisYawF_2P);
    for (int i = 0; i < 2; i++) field_op(f, LocalVar.IPC_KI[i]);
    for (int i = 0; i < 2; i++) field_op(f, LocalVar.IPC_KP[i]);
    field_op(f, LocalVar.IPC_IntSat);

    field_op(f, LocalVar.PC_State);
    for (int i = 0; i < 3; i++) field_op(f, LocalVar.PitCom[i]);
    for (int i = 0; i < 3; i++) field_op(f, LocalVar.PitCom_SD[i]);
    for (int i = 0; i < 3; i++) field_op(f, LocalVar.PitComAct[i]);

    field_op(f, LocalVar.SS_DelOmegaF);
    field_op(f, LocalVar.TestType);
    field_op(f, LocalVar.Kp_Float);
    field_op(f, LocalVar.VS_MaxTq);
    field_op(f, LocalVar.VS_LastGenTrq);
    field_op(f, LocalVar.VS_LastGenPwr);
    field_op(f, LocalVar.VS_MechGenPwr);
    field_op(f, LocalVar.VS_SpdErrAr);
    field_op(f, LocalVar.VS_SpdErrBr);
    field_op(f, LocalVar.VS_SpdErr);
    field_op(f, LocalVar.VS_State);
    field_op(f, LocalVar.VS_Rgn3Pitch);
    field_op(f, LocalVar.WE_Vw);
    field_op(f, LocalVar.WE_Vw_F);
    field_op(f, LocalVar.WE_VwI);
    field_op(f, LocalVar.WE_VwIdot);
    field_op(f, LocalVar.WE_Op);
    field_op(f, LocalVar.WE_Op_Last);
    field_op(f, LocalVar.VS_LastGenTrqF);
    field_op(f, LocalVar.PRC_WSE_F);
    field_op(f, LocalVar.PRC_R_Speed);
    field_op(f, LocalVar.PRC_R_Torque);
    field_op(f, LocalVar.PRC_R_Pitch);
    field_op(f, LocalVar.PRC_R_Total);
    field_op(f, LocalVar.PRC_Min_Pitch);
    field_op(f, LocalVar.PS_Min_Pitch);
    field_op(f, LocalVar.OL_Index);
    field_op(f, LocalVar.SU_Stage);
    field_op(f, LocalVar.SU_LoadStageStartTime);
    field_op(f, LocalVar.SU_RotSpeedF);
    field_op(f, LocalVar.SD_Trigger);
    field_op(f, LocalVar.SD_BlPitchF);
    field_op(f, LocalVar.SD_NacVaneF);
    field_op(f, LocalVar.SD_GenSpeedF);
    field_op(f, LocalVar.SD_Stage);
    field_op(f, LocalVar.SD_StageStartTime);
    field_op(f, LocalVar.SD_MaxPitchRate);
    field_op(f, LocalVar.SD_MaxTorqueRate);
    field_op(f, LocalVar.GenTq_SD);
    field_op(f, LocalVar.Fl_PitCom);
    field_op(f, LocalVar.NACIMU_FA_AccF);
    field_op(f, LocalVar.FA_AccF);
    field_op(f, LocalVar.FA_Hist);
    field_op(f, LocalVar.TRA_LastRefSpd);
    field_op(f, LocalVar.VS_RefSpeed);

    field_op(f, LocalVar.PtfmTDX);
    field_op(f, LocalVar.PtfmTDY);
    field_op(f, LocalVar.PtfmTDZ);
    field_op(f, LocalVar.PtfmRDX);
    field_op(f, LocalVar.PtfmRDY);
    field_op(f, LocalVar.PtfmRDZ);
    field_op(f, LocalVar.PtfmTVX);
    field_op(f, LocalVar.PtfmTVY);
    field_op(f, LocalVar.PtfmTVZ);
    field_op(f, LocalVar.PtfmRVX);
    field_op(f, LocalVar.PtfmRVY);
    field_op(f, LocalVar.PtfmRVZ);
    field_op(f, LocalVar.PtfmTAX);
    field_op(f, LocalVar.PtfmTAY);
    field_op(f, LocalVar.PtfmTAZ);
    field_op(f, LocalVar.PtfmRAX);
    field_op(f, LocalVar.PtfmRAY);
    field_op(f, LocalVar.PtfmRAZ);

    for (int i = 0; i < 12; i++) field_op(f, LocalVar.CC_DesiredL[i]);
    for (int i = 0; i < 12; i++) field_op(f, LocalVar.CC_ActuatedL[i]);
    for (int i = 0; i < 12; i++) field_op(f, LocalVar.CC_ActuatedDL[i]);
    for (int i = 0; i < 12; i++) field_op(f, LocalVar.StC_Input[i]);
    for (int i = 0; i < 3; i++) field_op(f, LocalVar.Flp_Angle[i]);
    for (int i = 0; i < 3; i++) field_op(f, LocalVar.RootMyb_Last[i]);

    field_op(f, LocalVar.ACC_INFILE_SIZE);
    field_op(f, LocalVar.ACC_INFILE);
    field_op(f, LocalVar.restart);

    // AWC complex angle (interleaved re/im)
    for (int i = 0; i < 3; i++) {
        field_op(f, LocalVar.AWC_complexangle_re[i]);
        field_op(f, LocalVar.AWC_complexangle_im[i]);
    }

    field_op(f, LocalVar.TiltMean);
    field_op(f, LocalVar.YawMean);
    field_op(f, LocalVar.ZMQ_ID);
    field_op(f, LocalVar.ZMQ_YawOffset);
    field_op(f, LocalVar.ZMQ_TorqueOffset);
    for (int i = 0; i < 3; i++) field_op(f, LocalVar.ZMQ_PitOffset[i]);
    field_op(f, LocalVar.ZMQ_R_Speed);
    field_op(f, LocalVar.ZMQ_R_Torque);
    field_op(f, LocalVar.ZMQ_R_Pitch);

    // --- WE nested struct ---
    field_op(f, LocalVar.WE.om_r);
    field_op(f, LocalVar.WE.v_t);
    field_op(f, LocalVar.WE.v_m);
    field_op(f, LocalVar.WE.v_h);
    field_op(f, LocalVar.WE.P);
    field_op(f, LocalVar.WE.xh);
    field_op(f, LocalVar.WE.K);

    // --- FP (FilterParameters) — vectors of per-instance state structs ---
    field_op(f, LocalVar.FP.lpf2);
    field_op(f, LocalVar.FP.lpfV);
    field_op(f, LocalVar.FP.hpf);
    field_op(f, LocalVar.FP.nfs);
    field_op(f, LocalVar.FP.nf);

    // --- piP, resP, rlP ---
    field_op(f, LocalVar.piP.pi);
    field_op(f, LocalVar.resP.res);
    field_op(f, LocalVar.rlP.rl);


}

} // anonymous namespace

#endif // RESTART_FIELDS_H
