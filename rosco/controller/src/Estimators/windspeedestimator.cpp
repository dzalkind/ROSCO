#include "../include/vit_types.h"
#include "../include/rosco_objects.hpp"
#include "../include/vit_translated.h"
#include <cmath>
#include <cstring>
#include <cstdio>
#include <limits>

#include "../include/rosco_constants.h"

// Fortran P(i,j) = C P[j-1][i-1] (column-major to row-major mapping)
#define WE_P(i,j)  LocalVar.WE.P[(j)-1][(i)-1]
#define WE_xh(i)   LocalVar.WE.xh[(i)-1][0]
#define WE_K(i)    LocalVar.WE.K[(i)-1][0]

void WindSpeedEstimator(LocalVariables& LocalVar, const ControlParameters& CntrPar,
                        objectinstances_t* objInst, const PerformanceData& PerfData,
                        debugvariables_t* DebugVar, errorvariables_t* ErrVar) {

    double WE_Inp_Pitch, WE_Inp_Torque, WE_Inp_Speed, Max_Op_Pitch;
    double eps = std::numeric_limits<double>::epsilon();

    // Saturate inputs to WSE
    // Rotor speed
    if (LocalVar.RotSpeedF < 0.25 * CntrPar.VS_MinOMSpd / CntrPar.WE_GearboxRatio) {
        WE_Inp_Speed = 0.25 * CntrPar.VS_MinOMSpd / CntrPar.WE_GearboxRatio + eps;
    } else {
        WE_Inp_Speed = LocalVar.RotSpeedF;
    }

    // Blade pitch
    if (CntrPar.WE_Mode > 0) {
        // PerfData%Beta_vec(SIZE(PerfData%Beta_vec)) — last element, 0-based
        Max_Op_Pitch = PerfData.Beta_vec.back() * D2R;
    } else {
        Max_Op_Pitch = 0.0;
    }
    WE_Inp_Pitch = saturate(LocalVar.BlPitchCMeas, CntrPar.PC_MinPit, Max_Op_Pitch);

    // Gen torque
    if (LocalVar.VS_LastGenTrqF < 0.0001 * CntrPar.VS_RtTq) {
        WE_Inp_Torque = 0.0001 * CntrPar.VS_RtTq;
    } else {
        WE_Inp_Torque = LocalVar.VS_LastGenTrqF;
    }

    // Check operational range
    LocalVar.WE_Op_Last = LocalVar.WE_Op;
    if (std::fabs(WE_Inp_Pitch - LocalVar.BlPitchCMeas) > 0) {
        LocalVar.WE_Op = 0;
    } else if (std::fabs(WE_Inp_Torque - LocalVar.VS_LastGenTrqF) > 0) {
        LocalVar.WE_Op = 0;
    } else if (std::fabs(WE_Inp_Speed - LocalVar.RotSpeedF) > 0) {
        LocalVar.WE_Op = 0;
    } else {
        LocalVar.WE_Op = 1;
    }

    // Restart flag for WSE
    LocalVar.RestartWSE = LocalVar.iStatus;

    if (CntrPar.WE_Mode > 0) {
        if (LocalVar.WE_Op == 0 && LocalVar.WE_Op_Last == 1) {
            // Print warning (matches Fortran PRINT behavior)
            fprintf(stderr, "\n***************************************************************************************************************************************\n"
                "ROSCO Warning: The wind speed estimator is used, but an input (pitch, rotor speed, or torque) has left the bounds of normal operation.\n"
                "The filtered hub-height wind speed will be used instead.\n"
                "***************************************************************************************************************************************\n");
            LocalVar.RestartWSE = 0;
        }
        if (LocalVar.WE_Op == 1 && LocalVar.WE_Op_Last == 0) {
            LocalVar.RestartWSE = 0;
        }
    }

    // Filter hub height wind speed; initialize to WE_Vw on first WSE call or restart
    static LPFilter horWindFilter;
    if (LocalVar.RestartWSE == 0 || LocalVar.restart) horWindFilter.init(CntrPar.F_WECornerFreq / 10.0, LocalVar.DT, LocalVar.WE_Vw);
    LocalVar.HorWindV_F = std::cos(LocalVar.NacVaneF * D2R) * horWindFilter.step(LocalVar.HorWindV);

    // Debug inputs
    DebugVar->WE_b = WE_Inp_Pitch;
    DebugVar->WE_w = WE_Inp_Speed;
    DebugVar->WE_t = WE_Inp_Torque;

    // ---- Define wind speed estimate ----
    double Tau_r = 0.0, Cp_op = 0.0, lambda = 0.0;

    // Inversion and Invariance Filter
    if (CntrPar.WE_Mode == 1 && LocalVar.WE_Op > 0) {
        Tau_r = AeroDynTorque(LocalVar.RotSpeedF, LocalVar.BlPitchCMeas,
                                LocalVar.WE_Vw, CntrPar.WE_BladeRadius, CntrPar.WE_RhoAir,
                                PerfData, ErrVar);

        LocalVar.WE_VwIdot = CntrPar.WE_Gamma / CntrPar.WE_Jtot *
            (LocalVar.VS_LastGenTrq * CntrPar.WE_GearboxRatio - Tau_r);
        LocalVar.WE_VwI = LocalVar.WE_VwI + LocalVar.WE_VwIdot * LocalVar.DT;
        LocalVar.WE_Vw = LocalVar.WE_VwI + CntrPar.WE_Gamma * LocalVar.RotSpeedF;

    // Extended Kalman Filter (EKF)
    } else if (CntrPar.WE_Mode == 2 && LocalVar.WE_Op > 0) {
        double L = 6.0 * CntrPar.WE_BladeRadius;
        double Ti = 0.18;
        double R_m = 0.02;

        // H = [1, 0, 0] (1x3)
        double H[3] = {1.0, 0.0, 0.0};

        // Initialize F, Q to zero (3x3, column-major flat)
        double F[9] = {0};
        double Q[9] = {0};
        // Column-major access: F(i,j) = F[(j-1)*3 + (i-1)]
        #define FM(i,j) F[((j)-1)*3 + ((i)-1)]
        #define QM(i,j) Q[((j)-1)*3 + ((i)-1)]

        if (LocalVar.RestartWSE == 0) {
            // Initialize
            LocalVar.WE.om_r = WE_Inp_Speed;
            LocalVar.WE.v_t = 0.0;
            LocalVar.WE.v_m = LocalVar.HorWindV_F > 3.0 ? LocalVar.HorWindV_F : 3.0;
            LocalVar.WE.v_h = LocalVar.HorWindV_F > 3.0 ? LocalVar.HorWindV_F : 3.0;
            LocalVar.WE_Vw = LocalVar.WE.v_m + LocalVar.WE.v_t;
            lambda = WE_Inp_Speed * CntrPar.WE_BladeRadius / LocalVar.WE.v_h;

            // xh = [om_r, v_t, v_m]^T (column-major 3x1)
            WE_xh(1) = LocalVar.WE.om_r;
            WE_xh(2) = LocalVar.WE.v_t;
            WE_xh(3) = LocalVar.WE.v_m;

            // P = diag(0.01, 0.01, 1.0)
            for (int i = 1; i <= 3; i++)
                for (int j = 1; j <= 3; j++)
                    WE_P(i,j) = 0.0;
            WE_P(1,1) = 0.01;
            WE_P(2,2) = 0.01;
            WE_P(3,3) = 1.0;

            // K = [0,0,0]^T
            WE_K(1) = 0.0; WE_K(2) = 0.0; WE_K(3) = 0.0;

            Cp_op = 0.25;

        } else {
            // Find estimated operating Cp and system pole
            double A_op = interp1d(CntrPar.WE_FOPoles_v,
                                   CntrPar.WE_FOPoles,
                                   LocalVar.WE.v_h, ErrVar);

            lambda = (WE_Inp_Speed > eps ? WE_Inp_Speed : eps) * CntrPar.WE_BladeRadius / LocalVar.WE.v_h;
            Cp_op = interp2d(PerfData.Beta_vec.data(), (int)PerfData.Beta_vec.size(),
                             PerfData.TSR_vec.data(),  (int)PerfData.TSR_vec.size(),
                             PerfData.Cp_mat.data(),   (int)PerfData.TSR_vec.size(), (int)PerfData.Beta_vec.size(),
                             WE_Inp_Pitch * R2D, lambda, ErrVar);
            Cp_op = Cp_op > 0.0 ? Cp_op : 0.0;

            // Update Jacobian F
            FM(1,1) = A_op;
            double F12 = 1.0 / (2.0 * CntrPar.WE_Jtot) * CntrPar.WE_RhoAir * PI *
                         (CntrPar.WE_BladeRadius * CntrPar.WE_BladeRadius) *
                         1.0 / LocalVar.WE.om_r * 3.0 * Cp_op *
                         (LocalVar.WE.v_h * LocalVar.WE.v_h);
            FM(1,2) = F12;
            FM(1,3) = F12;
            FM(2,2) = -PI * LocalVar.WE.v_m / (2.0 * L);
            FM(2,3) = -PI * LocalVar.WE.v_t / (2.0 * L);

            // Update process noise Q
            QM(1,1) = 0.00001;
            QM(2,2) = (PI * std::pow(LocalVar.WE.v_m, 3.0) * (Ti * Ti)) / L;
            QM(3,3) = (2.0 * 2.0) / 600.0;

            // Prediction update
            Tau_r = AeroDynTorque(WE_Inp_Speed, WE_Inp_Pitch,
                                  LocalVar.WE_Vw, CntrPar.WE_BladeRadius, CntrPar.WE_RhoAir,
                                  PerfData, ErrVar);
            double a = PI * LocalVar.WE.v_m / (2.0 * L);
            double dxh[3];
            dxh[0] = 1.0 / CntrPar.WE_Jtot * (Tau_r - CntrPar.WE_GearboxRatio * WE_Inp_Torque);
            dxh[1] = -a * LocalVar.WE.v_t;
            dxh[2] = 0.0;

            // EKF update — state prediction, P prediction, measurement update
            // Uses explicit element-by-element MATMUL with left-to-right
            // accumulation and temporaries to avoid aliasing (P appears on
            // both sides of the P prediction equation).

            // State prediction: xh = xh + DT * dxh
            WE_xh(1) += LocalVar.DT * dxh[0];
            WE_xh(2) += LocalVar.DT * dxh[1];
            WE_xh(3) += LocalVar.DT * dxh[2];

            // P prediction: P_new = P + DT*(F*P + P*F^T + Q - K*R_m*K^T)
            // Compute into temporary to avoid aliasing (P is read and written)
            {
                double P_new[3][3];
                for (int i = 1; i <= 3; i++) {
                    for (int j = 1; j <= 3; j++) {
                        // F*P element (i,j)
                        double FP_ij = FM(i,1)*WE_P(1,j) + FM(i,2)*WE_P(2,j) + FM(i,3)*WE_P(3,j);
                        // P*F^T element (i,j) = sum_k P(i,k)*F(j,k)
                        double PFt_ij = WE_P(i,1)*FM(j,1) + WE_P(i,2)*FM(j,2) + WE_P(i,3)*FM(j,3);
                        // K*R_m*K^T element (i,j) = K(i)*R_m*K(j)
                        double KRKt_ij = WE_K(i) * R_m * WE_K(j);
                        P_new[j-1][i-1] = WE_P(i,j) + LocalVar.DT * (FP_ij + PFt_ij + QM(i,j) - KRKt_ij);
                    }
                }
                // Copy back
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        LocalVar.WE.P[i][j] = P_new[i][j];
            }

            // Measurement update
            // H = [1, 0, 0], so H*P*H^T = P(1,1), P*H^T = P(:,1)
            double S = WE_P(1,1) + R_m;
            WE_K(1) = WE_P(1,1) / S;
            WE_K(2) = WE_P(2,1) / S;
            WE_K(3) = WE_P(3,1) / S;

            // xh = xh + K*(WE_Inp_Speed - om_r)
            double innov = WE_Inp_Speed - LocalVar.WE.om_r;
            WE_xh(1) += WE_K(1) * innov;
            WE_xh(2) += WE_K(2) * innov;
            WE_xh(3) += WE_K(3) * innov;

            // P = (I - K*H) * P  where K*H is rank-1: (K*H)(i,j) = K(i)*H(j) = K(i) if j==1, else 0
            {
                double P_new[3][3];
                for (int i = 1; i <= 3; i++) {
                    for (int j = 1; j <= 3; j++) {
                        // (I - K*H)(i,k) = delta(i,k) - K(i)*H(k)
                        // = delta(i,k) - K(i) if k==1, else delta(i,k)
                        // So (I-KH)*P (i,j) = P(i,j) - K(i)*P(1,j)
                        P_new[j-1][i-1] = WE_P(i,j) - WE_K(i) * WE_P(1,j);
                    }
                }
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        LocalVar.WE.P[i][j] = P_new[i][j];
            }

            // Extract state estimates
            LocalVar.WE.om_r = WE_xh(1) > eps ? WE_xh(1) : eps;
            LocalVar.WE.v_t = WE_xh(2);
            LocalVar.WE.v_m = WE_xh(3);
            LocalVar.WE.v_h = LocalVar.WE.v_t + LocalVar.WE.v_m;
            LocalVar.WE_Vw = LocalVar.WE.v_m + LocalVar.WE.v_t;

            if (std::isnan(LocalVar.WE.v_h)) {
                LocalVar.WE.om_r = WE_Inp_Speed;
                LocalVar.WE.v_t = 0.0;
                LocalVar.WE.v_m = LocalVar.HorWindV;
                LocalVar.WE.v_h = LocalVar.HorWindV;
                LocalVar.WE_Vw = LocalVar.WE.v_m + LocalVar.WE.v_t;
            }
        }

        #undef FM
        #undef QM

        // Debug outputs
        DebugVar->WE_Cp = Cp_op;
        DebugVar->WE_Vm = LocalVar.WE.v_m;
        DebugVar->WE_Vt = LocalVar.WE.v_t;
        DebugVar->WE_lambda = lambda;

    } else {
        // Use filtered hub-height wind speed
        LocalVar.WE_Vw = LocalVar.HorWindV_F;
    }

    DebugVar->WE_Vw = LocalVar.WE_Vw;

    // Diagnostic: pitch trace
    {
        static int wse_call = 0;
        static FILE* diag = fopen("/tmp/wse_diag.txt", "w");
        wse_call++;
        if (diag) {
            fprintf(diag, "%8d %25.17E %25.17E %25.17E %25.17E\n",
                wse_call, LocalVar.WE_Vw,
                LocalVar.BlPitchCMeas, WE_Inp_Pitch, WE_Inp_Pitch * R2D);
            fflush(diag);
        }
    }

    // Add RoutineName to error message
    if (ErrVar->aviFAIL < 0) {
        int trimmed_len = 1024;
        while (trimmed_len > 0 && ErrVar->ErrMsg[trimmed_len - 1] == ' ') trimmed_len--;
        char buf[1024];
        const char prefix[] = "WindSpeedEstimator:";
        int prefix_len = 19;
        std::memcpy(buf, prefix, prefix_len);
        int copy_len = trimmed_len;
        if (prefix_len + copy_len > 1024) copy_len = 1024 - prefix_len;
        std::memcpy(buf + prefix_len, ErrVar->ErrMsg, copy_len);
        int total = prefix_len + copy_len;
        if (total < 1024) std::memset(buf + total, ' ', 1024 - total);
        std::memcpy(ErrVar->ErrMsg, buf, 1024);
    }
}

#undef WE_P
#undef WE_xh
#undef WE_K
