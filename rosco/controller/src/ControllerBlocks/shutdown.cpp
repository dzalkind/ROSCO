#include "../include/vit_types.h"
#include "../include/vit_translated.h"
#include <cmath>
#include <cstring>

#include "../include/rosco_constants.h"

void Shutdown(localvariables_t* LocalVar, const ControlParameters& CntrPar, objectinstances_t* objInst, errorvariables_t* ErrVar) {

    // Initialize shutdown trigger
    if (LocalVar->iStatus == 0) {
        LocalVar->SD_Trigger = 0;
    }

    // Filter pitch signal
    static LPFilter pitchFilter;
    if (LocalVar->iStatus == 0 || LocalVar->restart) pitchFilter.init(CntrPar.SD_PitchCornerFreq, LocalVar->DT, LocalVar->BlPitchCMeas);
    LocalVar->SD_BlPitchF = pitchFilter.step(LocalVar->BlPitchCMeas);

    // Filter generator speed
    static LPFilter genSpeedFilter;
    if (LocalVar->iStatus == 0 || LocalVar->restart) genSpeedFilter.init(CntrPar.SD_GenSpdCornerFreq, LocalVar->DT, LocalVar->GenSpeed);
    LocalVar->SD_GenSpeedF = genSpeedFilter.step(LocalVar->GenSpeed);

    // Filter yaw error signal (NacVane)
    double SD_NacVane_cos = std::cos(LocalVar->NacVane * D2R);
    double SD_NacVane_sin = std::sin(LocalVar->NacVane * D2R);

    static LPFilter nacVaneCosFilter;
    if (LocalVar->iStatus == 0 || LocalVar->restart) nacVaneCosFilter.init(CntrPar.SD_YawErrorCornerFreq, LocalVar->DT, SD_NacVane_cos);
    double SD_NacVaneCosF = nacVaneCosFilter.step(SD_NacVane_cos);

    static LPFilter nacVaneSinFilter;
    if (LocalVar->iStatus == 0 || LocalVar->restart) nacVaneSinFilter.init(CntrPar.SD_YawErrorCornerFreq, LocalVar->DT, SD_NacVane_sin);
    double SD_NacVaneSinF = nacVaneSinFilter.step(SD_NacVane_sin);
    LocalVar->SD_NacVaneF = wrap_180(std::atan2(SD_NacVaneSinF, SD_NacVaneCosF) * R2D);

    // Check for shutdown conditions
    if ((LocalVar->SD_Trigger == 0) && (LocalVar->Time >= CntrPar.SD_TimeActivate)) {
        if (CntrPar.SD_EnablePitch == 1 && LocalVar->SD_BlPitchF > CntrPar.SD_MaxPit) {
            LocalVar->SD_Trigger = 1;
        }
        if (CntrPar.SD_EnableYawError == 1 && std::fabs(LocalVar->SD_NacVaneF) > CntrPar.SD_MaxYawError) {
            LocalVar->SD_Trigger = 2;
        }
        if (CntrPar.SD_EnableGenSpeed == 1 && LocalVar->SD_GenSpeedF > CntrPar.SD_MaxGenSpd) {
            LocalVar->SD_Trigger = 3;
        }
        if (CntrPar.SD_EnableTime == 1 && LocalVar->Time > CntrPar.SD_Time) {
            LocalVar->SD_Trigger = 4;
        }
    }

    // Method 1: stage depends on time
    if (CntrPar.SD_Method == 1) {
        if (LocalVar->SD_Stage == 0) {
            if (LocalVar->SD_Trigger > 0) {
                LocalVar->SD_Stage = 1;
                LocalVar->SD_StageStartTime = LocalVar->Time;
            }
            LocalVar->SD_MaxPitchRate = 0;
            LocalVar->SD_MaxTorqueRate = 0;

        } else if (LocalVar->SD_Stage <= CntrPar.SD_Stage_N) {
            // Fortran 1-based: SD_MaxPitchRate(SD_Stage) → C 0-based: [SD_Stage-1]
            LocalVar->SD_MaxPitchRate = CntrPar.SD_MaxPitchRate[LocalVar->SD_Stage - 1];
            LocalVar->SD_MaxTorqueRate = CntrPar.SD_MaxTorqueRate[LocalVar->SD_Stage - 1];

            if (LocalVar->Time >= LocalVar->SD_StageStartTime + CntrPar.SD_StageTime[LocalVar->SD_Stage - 1]) {
                LocalVar->SD_Stage = LocalVar->SD_Stage + 1;
                LocalVar->SD_StageStartTime = LocalVar->Time;
            }
        } else {
            LocalVar->SD_MaxPitchRate = CntrPar.PC_MaxRat;
            LocalVar->SD_MaxTorqueRate = CntrPar.VS_MaxRat;
        }

    // Method 2: stage depends on blade pitch
    } else if (CntrPar.SD_Method == 2) {
        if (LocalVar->SD_Stage == 0) {
            if (LocalVar->SD_Trigger > 0) {
                LocalVar->SD_Stage = 1;
            }
            LocalVar->SD_MaxPitchRate = 0;
            LocalVar->SD_MaxTorqueRate = 0;

        } else {
            // Determine stage from pitch angle
            for (int I_Stage = 1; I_Stage <= CntrPar.SD_Stage_N; I_Stage++) {
                if (LocalVar->BlPitchCMeas >= CntrPar.SD_StagePitch[I_Stage - 1]) {
                    LocalVar->SD_Stage = I_Stage + 1;
                }
            }

            if (LocalVar->SD_Stage > CntrPar.SD_Stage_N) {
                LocalVar->SD_MaxPitchRate = CntrPar.PC_MaxRat;
                LocalVar->SD_MaxTorqueRate = CntrPar.VS_MaxRat;
            } else {
                LocalVar->SD_MaxPitchRate = CntrPar.SD_MaxPitchRate[LocalVar->SD_Stage - 1];
                LocalVar->SD_MaxTorqueRate = CntrPar.SD_MaxTorqueRate[LocalVar->SD_Stage - 1];
            }
        }
    }

    // Add RoutineName to error message
    if (ErrVar->aviFAIL < 0) {
        int trimmed_len = 1024;
        while (trimmed_len > 0 && ErrVar->ErrMsg[trimmed_len - 1] == ' ') {
            trimmed_len--;
        }
        char buf[1024];
        const char prefix[] = "Shutdown:";
        int prefix_len = 9;
        std::memcpy(buf, prefix, prefix_len);
        int copy_len = trimmed_len;
        if (prefix_len + copy_len > 1024) copy_len = 1024 - prefix_len;
        std::memcpy(buf + prefix_len, ErrVar->ErrMsg, copy_len);
        int total = prefix_len + copy_len;
        if (total < 1024) std::memset(buf + total, ' ', 1024 - total);
        std::memcpy(ErrVar->ErrMsg, buf, 1024);
    }
}
