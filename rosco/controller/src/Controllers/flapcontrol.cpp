#include "../include/vit_types.h"
#include "../include/vit_translated.h"

#include "../include/rosco_constants.h"
#include "../ControlElements/picontroller.hpp"
#include "../ControlElements/piicontroller.hpp"

void FlapControl(float* avrSWAP, const ControlParameters& CntrPar, LocalVariables& LocalVar) {
    // FlapControl: blade flap angle control
    //   Flp_Mode = 1: steady-state flap angle
    //   Flp_Mode = 2: PII flap control (proportional-integral-integral)
    //   Flp_Mode = 3: cyclic (1P) flap control via Coleman transform

    if (CntrPar.Flp_Mode > 0) {
        static PIIController flpPII[3];
        if (LocalVar.iStatus == 0) {
            // Initialization
            LocalVar.RootMyb_Last[0] = 0.0 - LocalVar.rootMOOP[0];
            LocalVar.RootMyb_Last[1] = 0.0 - LocalVar.rootMOOP[1];
            LocalVar.RootMyb_Last[2] = 0.0 - LocalVar.rootMOOP[2];

            // Initial flap angle
            LocalVar.Flp_Angle[0] = CntrPar.Flp_Angle;
            LocalVar.Flp_Angle[1] = CntrPar.Flp_Angle;
            LocalVar.Flp_Angle[2] = CntrPar.Flp_Angle;

            // Initialize PII controller (Flp_Mode==2)
            if (CntrPar.Flp_Mode == 2) {
                // Note: K is uninitialized in Fortran source (ROSCO bug, fixed in fork)
                // The init call uses K which was set to loop counter from prior init
                // In the golden fixture K=0 (C index), matching Fortran K=1 (first iter)
                int K = 0;
                flpPII[K].init(0.0);
                LocalVar.Flp_Angle[K] = 0.0;
            }

        } else if (CntrPar.Flp_Mode == 1) {
            // Steady flap angle — no change
            // LocalVar.Flp_Angle[i] retains its value

        } else if (CntrPar.Flp_Mode == 2) {
            // PII flap control
            for (int K = 0; K < LocalVar.NumBl; K++) {
                if (LocalVar.restart) {
                    flpPII[K].init(0.0);
                    LocalVar.Flp_Angle[K] = 0.0;
                } else {
                    LocalVar.Flp_Angle[K] = flpPII[K].step(
                        -LocalVar.rootMOOPF[K],
                        0.0 - LocalVar.Flp_Angle[K],
                        CntrPar.Flp_Kp, CntrPar.Flp_Ki, 0.05,
                        -CntrPar.Flp_MaxPit, CntrPar.Flp_MaxPit, LocalVar.DT);
                }
                // Saturation limits, convert to degrees
                LocalVar.Flp_Angle[K] = saturate(
                    LocalVar.Flp_Angle[K],
                    -CntrPar.Flp_MaxPit, CntrPar.Flp_MaxPit) * R2D;
            }

        } else if (CntrPar.Flp_Mode == 3) {
            // Cyclic flap control via Coleman transform
            double axisTilt_1P, axisYaw_1P;
            ColemanTransform(LocalVar.rootMOOPF, LocalVar.Azimuth, 1,
                               &axisTilt_1P, &axisYaw_1P);

            // PI control on tilt and yaw axes
            static PIController flpTiltPI;
            static PIController flpYawPI;
            double Flp_axisTilt_1P, Flp_axisYaw_1P;
            if (LocalVar.iStatus == 0 || LocalVar.restart) {
                flpTiltPI.init(0.0);
                flpYawPI.init(0.0);
                Flp_axisTilt_1P = 0.0;
                Flp_axisYaw_1P = 0.0;
            } else {
                Flp_axisTilt_1P = flpTiltPI.step(axisTilt_1P, CntrPar.Flp_Kp, CntrPar.Flp_Ki,
                    -CntrPar.Flp_MaxPit, CntrPar.Flp_MaxPit, LocalVar.DT);
                Flp_axisYaw_1P = flpYawPI.step(axisYaw_1P, CntrPar.Flp_Kp, CntrPar.Flp_Ki,
                    -CntrPar.Flp_MaxPit, CntrPar.Flp_MaxPit, LocalVar.DT);
            }

            // Inverse Coleman transform back to blade coordinates
            ColemanTransformInverse(Flp_axisTilt_1P, Flp_axisYaw_1P,
                                      LocalVar.Azimuth, 1, 0.0,
                                      LocalVar.Flp_Angle);
        }

        // Send flap pitch commands to avrSWAP (Fortran indices 120,121,122 → C 119,120,121)
        avrSWAP[119] = LocalVar.Flp_Angle[0];
        avrSWAP[120] = LocalVar.Flp_Angle[1];
        avrSWAP[121] = LocalVar.Flp_Angle[2];
    }
}
