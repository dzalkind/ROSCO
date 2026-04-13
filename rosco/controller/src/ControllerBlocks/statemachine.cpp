#include "../include/vit_types.h"
#include "../include/rosco_types.hpp"
#include "../include/rosco_constants.h"

void StateMachine(const ControlParameters& CntrPar, localvariables_t* LocalVar) {

    // Initialize State machine if first call
    if (LocalVar->iStatus == 0) {

        if (LocalVar->PitCom[0] >= LocalVar->VS_Rgn3Pitch) { // We are in region 3
            LocalVar->PC_State = PC_State_Enabled;
            if (CntrPar.VS_ConstPower == VS_Mode_ConstPwr) { // Constant power tracking
                LocalVar->VS_State = VS_State_Region_3_ConstPwr;
            } else { // Constant torque tracking
                LocalVar->VS_State = VS_State_Region_3_ConstTrq;
            }
        } else { // We are in Region 2
            LocalVar->VS_State = VS_State_Region_2;
            LocalVar->PC_State = PC_State_Disabled;
        }

    // Operational States
    } else {
        // --- Pitch controller state machine ---
        if (CntrPar.PC_ControlMode == 1) {
            LocalVar->PC_State = PC_State_Enabled;
        } else {
            LocalVar->PC_State = PC_State_Disabled;
        }

        // --- Torque control state machine ---
        if (LocalVar->BlPitchCMeas >= LocalVar->VS_Rgn3Pitch) {
            if (CntrPar.VS_ConstPower == VS_Mode_ConstPwr) { // Region 3
                LocalVar->VS_State = VS_State_Region_3_ConstPwr; // Constant power tracking
            } else {
                LocalVar->VS_State = VS_State_Region_3_ConstTrq; // Constant torque tracking
            }
        } else {
            if (LocalVar->GenArTq >= CntrPar.VS_MaxOMTq * 1.01) { // Region 2 1/2
                if (CntrPar.VS_FBP == VS_FBP_Variable_Pitch) {
                    LocalVar->VS_State = VS_State_Region_2_5;
                } else {
                    LocalVar->VS_State = VS_State_Region_3_FBP; // Region 3 - fixed blade pitch
                }
            } else if ((LocalVar->GenSpeedF < CntrPar.VS_RefSpd) &&
                       (LocalVar->GenBrTq >= CntrPar.VS_MinOMTq)) { // Region 2
                LocalVar->VS_State = VS_State_Region_2;
            } else if (LocalVar->GenBrTq < CntrPar.VS_MinOMTq) { // Region 1 1/2
                LocalVar->VS_State = VS_State_Region_1_5;
            } else { // Error state, Debug
                LocalVar->VS_State = VS_State_Error;
            }
        }
    }
}
