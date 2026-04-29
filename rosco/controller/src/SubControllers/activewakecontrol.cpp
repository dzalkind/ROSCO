#include "../include/vit_types.h"
#include "../include/vit_translated.h"
#include <cmath>
#include "../include/rosco_constants.h"
#include "../ControlElements/picontroller.hpp"
#include "../ControlElements/rescontroller.hpp"

void ActiveWakeControl(const ControlParameters& CntrPar, LocalVariables& LocalVar, debugvariables_t* DebugVar) {
    // ActiveWakeControl: active wake mixing via individual pitch
    //   AWC_Mode 1: SNL complex-number approach
    //   AWC_Mode 2: Open-loop Coleman transform
    //   AWC_Mode 3: Closed-loop PI
    //   AWC_Mode 4: Closed-loop PR (proportional-resonant)
    //   AWC_Mode 5: Strouhal transformation closed-loop

    static const double phi1 = 0.0;
    static const double phi2 = 2.0 / 3.0 * PI;
    static const double phi3 = 4.0 / 3.0 * PI;

    double AWC_angle[3] = {0, 0, 0};
    double AWC_TiltYaw[2] = {0.0, 0.0};
    double Error[2] = {0.0, 0.0};
    double FixedFrameM[2] = {0.0, 0.0};
    double StartTime = 0.0;

    if (CntrPar.AWC_Mode == 1) {
        // SNL complex number approach
        LocalVar.AWC_complexangle_re[0] = 0.0;
        LocalVar.AWC_complexangle_re[1] = 0.0;
        LocalVar.AWC_complexangle_re[2] = 0.0;
        LocalVar.AWC_complexangle_im[0] = 0.0;
        LocalVar.AWC_complexangle_im[1] = 0.0;
        LocalVar.AWC_complexangle_im[2] = 0.0;

        for (int Imode = 0; Imode < CntrPar.AWC_NumModes; Imode++) {
            double clockang = CntrPar.AWC_clockangle[Imode] * PI / 180.0;
            double omega = CntrPar.AWC_freq[Imode] * PI * 2.0;
            AWC_angle[0] = omega * LocalVar.Time - CntrPar.AWC_n[Imode] * (LocalVar.Azimuth + phi1 + clockang);
            AWC_angle[1] = omega * LocalVar.Time - CntrPar.AWC_n[Imode] * (LocalVar.Azimuth + phi2 + clockang);
            AWC_angle[2] = omega * LocalVar.Time - CntrPar.AWC_n[Imode] * (LocalVar.Azimuth + phi3 + clockang);

            double amp = CntrPar.AWC_amp[Imode] * PI / 180.0;
            for (int K = 0; K < LocalVar.NumBl; K++) {
                LocalVar.AWC_complexangle_re[K] += amp * cos(AWC_angle[K]);
                LocalVar.AWC_complexangle_im[K] += amp * sin(AWC_angle[K]);
            }
        }

        for (int K = 0; K < LocalVar.NumBl; K++) {
            LocalVar.PitCom[K] += LocalVar.AWC_complexangle_re[K];
        }

    } else if (CntrPar.AWC_Mode == 2) {
        // Open-loop Coleman transform
        AWC_TiltYaw[0] = 0.0;
        AWC_TiltYaw[1] = 0.0;
        for (int Imode = 0; Imode < CntrPar.AWC_NumModes; Imode++) {
            AWC_TiltYaw[Imode] = D2R * CntrPar.AWC_amp[Imode] *
                sin(LocalVar.Time * 2.0 * PI * CntrPar.AWC_freq[Imode] +
                    CntrPar.AWC_clockangle[Imode] * D2R);
            if (CntrPar.AWC_NumModes == 1) {
                AWC_TiltYaw[1] = D2R * CntrPar.AWC_amp[0] *
                    sin(LocalVar.Time * 2.0 * PI * CntrPar.AWC_freq[0] +
                        2.0 * CntrPar.AWC_clockangle[0] * D2R);
            }

            ColemanTransformInverse(AWC_TiltYaw[0], AWC_TiltYaw[1],
                                       LocalVar.Azimuth, CntrPar.AWC_harmonic[Imode],
                                       CntrPar.AWC_phaseoffset * D2R, AWC_angle);
        }

        for (int K = 0; K < LocalVar.NumBl; K++) {
            LocalVar.PitCom[K] += AWC_angle[K];
        }

        // Debug variables
        DebugVar->axisTilt_2P = AWC_TiltYaw[0];
        DebugVar->axisYaw_2P = AWC_TiltYaw[1];
        ColemanTransform(LocalVar.BlPitch, LocalVar.Azimuth,
                           CntrPar.AWC_harmonic[0], &AWC_TiltYaw[0], &AWC_TiltYaw[1]);
        DebugVar->axisTilt_1P = AWC_TiltYaw[0];
        DebugVar->axisYaw_1P = AWC_TiltYaw[1];

    } else if ((CntrPar.AWC_Mode == 3) || (CntrPar.AWC_Mode == 4)) {
        // Closed-loop PI (mode 3) or PR (mode 4)

        ColemanTransform(LocalVar.rootMOOPF, LocalVar.Azimuth,
                           CntrPar.AWC_harmonic[0], &FixedFrameM[0], &FixedFrameM[1]);

        if (CntrPar.AWC_harmonic[0] == 0) {
            LocalVar.TiltMean += FixedFrameM[0];
            StartTime = 1.0 / CntrPar.AWC_freq[0];
        }

        if (LocalVar.Time > StartTime) {
            for (int Imode = 0; Imode < CntrPar.AWC_NumModes; Imode++) {
                Error[Imode] = CntrPar.AWC_amp[Imode] *
                    sin(LocalVar.Time * 2.0 * PI * CntrPar.AWC_freq[Imode] +
                        CntrPar.AWC_clockangle[Imode] * D2R) +
                    (FixedFrameM[Imode] - LocalVar.TiltMean / (LocalVar.n_DT + 1));

                if (CntrPar.AWC_Mode == 4) {
                    static ResController awcResCtrl[2];
                    if (LocalVar.iStatus == 0 || LocalVar.restart) {
                        awcResCtrl[Imode].init();
                        AWC_TiltYaw[Imode] = 0.0;
                    } else {
                        AWC_TiltYaw[Imode] = awcResCtrl[Imode].step(Error[Imode], CntrPar.AWC_CntrGains[0], CntrPar.AWC_CntrGains[1],
                            CntrPar.AWC_freq[Imode], CntrPar.PC_MinPit, CntrPar.PC_MaxPit, LocalVar.DT);
                    }
                } else {
                    static PIController awcPI[2];
                    if (LocalVar.iStatus == 0 || LocalVar.restart) {
                        awcPI[Imode].init(0.0);
                        AWC_TiltYaw[Imode] = 0.0;
                    } else {
                        AWC_TiltYaw[Imode] = awcPI[Imode].step(Error[Imode], CntrPar.AWC_CntrGains[0], CntrPar.AWC_CntrGains[1],
                            CntrPar.PC_MinPit, CntrPar.PC_MaxPit, LocalVar.DT);
                    }
                }
            }
        }

        ColemanTransformInverse(AWC_TiltYaw[0], AWC_TiltYaw[1],
                                   LocalVar.Azimuth, CntrPar.AWC_harmonic[0],
                                   CntrPar.AWC_phaseoffset * D2R, AWC_angle);

        for (int K = 0; K < LocalVar.NumBl; K++) {
            if (CntrPar.AWC_harmonic[0] == 0) {
                LocalVar.PitCom[K] += AWC_TiltYaw[0];
            } else {
                LocalVar.PitCom[K] += AWC_angle[K];
            }
        }

        // Debug variables
        DebugVar->axisTilt_1P = AWC_TiltYaw[0];
        DebugVar->axisYaw_1P = -FixedFrameM[0] + LocalVar.TiltMean / (LocalVar.n_DT + 1);
        DebugVar->axisTilt_2P = CntrPar.AWC_amp[0] *
            sin(LocalVar.Time * 2.0 * PI * CntrPar.AWC_freq[0] +
                CntrPar.AWC_clockangle[0] * D2R);
        DebugVar->axisYaw_2P = Error[0];

    } else if (CntrPar.AWC_Mode == 5) {
        // Strouhal transformation closed-loop

        double StrAzimuth = wrap_360(360.0 * LocalVar.Time * CntrPar.AWC_freq[0]) * D2R;
        double clockangle_yaw = (CntrPar.AWC_NumModes > 1)
            ? CntrPar.AWC_clockangle[1] : CntrPar.AWC_clockangle[0];

        ColemanTransform(LocalVar.rootMOOPF, LocalVar.Azimuth,
                           CntrPar.AWC_harmonic[0], &FixedFrameM[0], &FixedFrameM[1]);

        // Accumulate mean moments
        LocalVar.TiltMean += FixedFrameM[0];
        LocalVar.YawMean += FixedFrameM[1];

        // Error with zero-mean moments
        Error[0] = CntrPar.AWC_amp[0] +
            sin(StrAzimuth + CntrPar.AWC_clockangle[0] * D2R) *
                (FixedFrameM[0] - LocalVar.TiltMean / (LocalVar.n_DT + 1)) +
            sin(StrAzimuth + clockangle_yaw * D2R) *
                (FixedFrameM[1] - LocalVar.YawMean / (LocalVar.n_DT + 1));

        // PI control (after one period)
        if (LocalVar.Time > 1.0 / CntrPar.AWC_freq[0]) {
            static PIController awcStrPI;
            if (LocalVar.iStatus == 0 || LocalVar.restart) {
                awcStrPI.init(0.0);
                AWC_TiltYaw[0] = 0.0;
            } else {
                AWC_TiltYaw[0] = awcStrPI.step(Error[0], CntrPar.AWC_CntrGains[0], CntrPar.AWC_CntrGains[1],
                    CntrPar.PC_MinPit, CntrPar.PC_MaxPit, LocalVar.DT);
            }
        }

        // Inverse Strouhal + Coleman transform
        double tiltSig = sin(StrAzimuth + CntrPar.AWC_clockangle[0] * D2R) * AWC_TiltYaw[0];
        double yawSig = sin(StrAzimuth + clockangle_yaw * D2R) * AWC_TiltYaw[0];
        ColemanTransformInverse(tiltSig, yawSig,
                                   LocalVar.Azimuth, CntrPar.AWC_harmonic[0],
                                   CntrPar.AWC_phaseoffset * D2R, AWC_angle);

        for (int K = 0; K < LocalVar.NumBl; K++) {
            LocalVar.PitCom[K] += AWC_angle[K];
        }

        // Debug variables
        DebugVar->axisTilt_1P = tiltSig;
        DebugVar->axisYaw_1P = -FixedFrameM[0] + LocalVar.TiltMean / (LocalVar.n_DT + 1);
        DebugVar->axisTilt_2P = CntrPar.AWC_amp[0] *
            sin(StrAzimuth + CntrPar.AWC_clockangle[0] * D2R);
        DebugVar->axisYaw_2P = Error[0];
    }
}
