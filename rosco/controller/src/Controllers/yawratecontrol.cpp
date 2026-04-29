#include "../include/vit_types.h"
#include "../include/vit_translated.h"
#include <cmath>
#include "../include/rosco_constants.h"

void YawRateControl(float* avrSWAP, const ControlParameters& CntrPar, LocalVariables& LocalVar, debugvariables_t* DebugVar, errorvariables_t* ErrVar) {
    // YawRateControl: yaw rate control using yaw drive
    //   Y_ControlMode = 1: active yaw rate control

    // SAVE variables — persist across calls
    static double NacVaneOffset = 0.0;
    static int YawState = 0;
    static double NacHeadingError = 0.0;
    static int Tidx = 0;

    if (CntrPar.Y_ControlMode == 1) {
        // Compass wind direction in degrees
        LocalVar.WindDir = wrap_180(LocalVar.NacHeading + LocalVar.NacVane);

        // Initialize
        if (LocalVar.iStatus == 0) {
            YawState = 0;
            Tidx = 1;  // Fortran 1-indexed, used as-is in debug output
        }

        // Compute/apply offset
        if (CntrPar.ZMQ_Mode == 1) {
            NacVaneOffset = LocalVar.ZMQ_YawOffset;
        } else {
            NacVaneOffset = CntrPar.Y_MErrSet;
        }

        // Update filtered wind direction
        double WindDirPlusOffset = wrap_180(LocalVar.WindDir + NacVaneOffset);
        double WDpO_cos = cos(WindDirPlusOffset * D2R);
        double WDpO_sin = sin(WindDirPlusOffset * D2R);

        static LPFilter windDirCosFilter;
        if (LocalVar.iStatus == 0) windDirCosFilter.init(CntrPar.F_YawErr, LocalVar.DT, WDpO_cos);
        double WindDirPlusOffsetCosF = windDirCosFilter.step(WDpO_cos);

        static LPFilter windDirSinFilter;
        if (LocalVar.iStatus == 0) windDirSinFilter.init(CntrPar.F_YawErr, LocalVar.DT, WDpO_sin);
        double WindDirPlusOffsetSinF = windDirSinFilter.step(WDpO_sin);
        double NacHeadingTarget = wrap_180(atan2(WindDirPlusOffsetSinF, WindDirPlusOffsetCosF) * R2D);

        // Yaw error
        NacHeadingError = wrap_180(NacHeadingTarget - LocalVar.NacHeading);

        // Check for deadband
        double deadband;
        if (LocalVar.WE_Vw_F <= CntrPar.Y_uSwitch) {
            deadband = CntrPar.Y_ErrThresh[0];  // Fortran(1) → C[0]
        } else {
            deadband = CntrPar.Y_ErrThresh[1];  // Fortran(2) → C[1]
        }

        // Yaw state machine
        double YawRateCom;
        if (YawState == 1) {
            // Yawing right
            if (NacHeadingError <= 0.0) {
                YawRateCom = 0.0;
                YawState = 0;
            } else {
                YawRateCom = CntrPar.Y_Rate;
                YawState = 1;
            }
        } else if (YawState == -1) {
            // Yawing left
            if (NacHeadingError >= 0.0) {
                YawRateCom = 0.0;
                YawState = 0;
            } else {
                YawRateCom = -CntrPar.Y_Rate;
                YawState = -1;
            }
        } else {
            // Stopped — initiate yaw if outside deadband
            if (NacHeadingError > deadband) {
                YawState = 1;
            }
            if (NacHeadingError < -deadband) {
                YawState = -1;
            }
            YawRateCom = 0.0;
        }

        // Output yaw rate command in rad/s (Fortran avrSWAP(48) → C [47])
        avrSWAP[47] = YawRateCom * D2R;

        // Open loop yaw rate control override
        if ((CntrPar.OL_Mode > 0) && (CntrPar.Ind_YawRate > 0)) {
            if (LocalVar.Time >= CntrPar.OL_Breakpoints[0]) {
                avrSWAP[47] = interp1d(CntrPar.OL_Breakpoints,
                                       CntrPar.OL_YawRate,
                                       LocalVar.OL_Index, ErrVar);
            }
        }

        // Save for debug
        DebugVar->YawRateCom = YawRateCom;
        DebugVar->NacHeadingTarget = NacHeadingTarget;
        DebugVar->NacVaneOffset = NacVaneOffset;
        DebugVar->YawState = YawState;
        DebugVar->Yaw_Err = NacHeadingError;
    }
}
