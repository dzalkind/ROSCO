#include "../include/vit_types.h"
#include "../include/rosco_types.hpp"
#include "../include/rosco_constants.h"
#include "../include/rosco_error.hpp"
#include <cmath>
#include <cstring>

void ReadAvrSWAP(float* avrSWAP, LocalVariables& LocalVar, const ControlParameters& CntrPar) {
    int32_t K;

    // Load variables from calling program (See Appendix A of Bladed User's Guide)
    LocalVar.iStatus        = static_cast<int32_t>(std::round(avrSWAP[0]));
    LocalVar.Time           = avrSWAP[1];
    LocalVar.DT             = avrSWAP[2];
    LocalVar.VS_MechGenPwr  = avrSWAP[13];
    LocalVar.VS_GenPwr      = avrSWAP[14];
    LocalVar.GenSpeed       = avrSWAP[19];
    LocalVar.RotSpeed       = avrSWAP[20];
    LocalVar.GenTqMeas      = avrSWAP[22];
    LocalVar.NacVane        = avrSWAP[23] * R2D;
    LocalVar.HorWindV       = avrSWAP[26];
    LocalVar.rootMOOP[0]    = avrSWAP[29];
    LocalVar.rootMOOP[1]    = avrSWAP[30];
    LocalVar.rootMOOP[2]    = avrSWAP[31];
    LocalVar.NacHeading     = avrSWAP[36] * R2D;
    LocalVar.FA_Acc_TT      = avrSWAP[52];
    LocalVar.SS_Acc_TT      = avrSWAP[53];
    LocalVar.NacIMU_FA_RAcc = avrSWAP[82];
    LocalVar.Azimuth        = avrSWAP[59];
    LocalVar.NumBl          = static_cast<int32_t>(std::round(avrSWAP[60]));

    if (CntrPar.Ext_Interface > 0) {
        // Platform signals
        LocalVar.PtfmTDX = avrSWAP[1000];
        LocalVar.PtfmTDY = avrSWAP[1001];
        LocalVar.PtfmTDZ = avrSWAP[1002];
        LocalVar.PtfmRDX = avrSWAP[1003];
        LocalVar.PtfmRDY = avrSWAP[1004];
        LocalVar.PtfmRDZ = avrSWAP[1005];
        LocalVar.PtfmTVX = avrSWAP[1006];
        LocalVar.PtfmTVY = avrSWAP[1007];
        LocalVar.PtfmTVZ = avrSWAP[1008];
        LocalVar.PtfmRVX = avrSWAP[1009];
        LocalVar.PtfmRVY = avrSWAP[1010];
        LocalVar.PtfmRVZ = avrSWAP[1011];
        LocalVar.PtfmTAX = avrSWAP[1012];
        LocalVar.PtfmTAY = avrSWAP[1013];
        LocalVar.PtfmTAZ = avrSWAP[1014];
        LocalVar.PtfmRAX = avrSWAP[1015];
        LocalVar.PtfmRAY = avrSWAP[1016];
        LocalVar.PtfmRAZ = avrSWAP[1017];
    }

    // Check that we haven't already loaded this dynamic library
    if (LocalVar.iStatus == 0) {
        if (LocalVar.AlreadyInitialized == 0) {
            LocalVar.AlreadyInitialized = 1;
        } else {
            throw RoscoError("ERROR: This ROSCO dynamic library has already been loaded.");
        }
    }

    // Blade pitch assignment
    if (LocalVar.iStatus == 0) {
        LocalVar.BlPitch[0] = avrSWAP[3];
        LocalVar.BlPitch[1] = avrSWAP[32];
        LocalVar.BlPitch[2] = avrSWAP[33];
    } else {
        if (CntrPar.PF_Mode == 1) {
            for (K = 1; K <= LocalVar.NumBl; K++) {
                LocalVar.BlPitch[K - 1] = LocalVar.PitComAct[K - 1] - CntrPar.PF_Offsets[K - 1];
            }
        } else {
            LocalVar.BlPitch[0] = LocalVar.PitComAct[0];
            LocalVar.BlPitch[1] = LocalVar.PitComAct[1];
            LocalVar.BlPitch[2] = LocalVar.PitComAct[2];
        }
    }

    LocalVar.BlPitchCMeas = (1.0 / static_cast<double>(LocalVar.NumBl)) * (LocalVar.BlPitch[0] + LocalVar.BlPitch[1] + LocalVar.BlPitch[2]);

    if (LocalVar.iStatus == 0) {
        LocalVar.restart = 1;
    } else {
        LocalVar.restart = 0;
    }

    // FA_Acc_TT is in the non-rotating tower-top frame, convert to nacelle frame
    LocalVar.FA_Acc_Nac = LocalVar.FA_Acc_TT * std::cos(LocalVar.NacHeading * D2R) + LocalVar.SS_Acc_TT * std::sin(LocalVar.NacHeading * D2R);

    // Increment timestep counter
    if (LocalVar.iStatus == 0 && LocalVar.Time == 0.0) {
        LocalVar.n_DT = 0;
    } else {
        LocalVar.n_DT = LocalVar.n_DT + 1;
    }
}
