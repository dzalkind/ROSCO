#include "../include/vit_types.h"
#include "../include/vit_translated.h"
#include "../include/rosco_constants.h"

void PowerControlSetpoints(const ControlParameters& CntrPar, LocalVariables& LocalVar, objectinstances_t* objInst, debugvariables_t* DebugVar, errorvariables_t* ErrVar) {

    // Set up power control
    if (CntrPar.PRC_Mode == 2) { // Using power reference control
        if (CntrPar.PRC_Comm == PRC_Comm_Constant) { // Constant, from DISCON
            LocalVar.PRC_R_Speed = CntrPar.PRC_R_Speed;
            LocalVar.PRC_R_Torque = CntrPar.PRC_R_Torque;
            LocalVar.PRC_R_Pitch = CntrPar.PRC_R_Pitch;

        } else if (CntrPar.PRC_Comm == PRC_Comm_OpenLoop) { // Open loop

            ArrayView ol_bp = CntrPar.OL_Breakpoints;
            if (CntrPar.Ind_R_Speed > 0) {
                LocalVar.PRC_R_Speed  = interp1d(ol_bp, CntrPar.OL_R_Speed,  LocalVar.OL_Index, ErrVar);
            } else {
                LocalVar.PRC_R_Speed = 1.0;
            }

            if (CntrPar.Ind_R_Torque > 0) {
                LocalVar.PRC_R_Torque = interp1d(ol_bp, CntrPar.OL_R_Torque, LocalVar.OL_Index, ErrVar);
            } else {
                LocalVar.PRC_R_Torque = 1.0;
            }

            if (CntrPar.Ind_R_Pitch > 0) {
                LocalVar.PRC_R_Pitch  = interp1d(ol_bp, CntrPar.OL_R_Pitch,  LocalVar.OL_Index, ErrVar);
            } else {
                LocalVar.PRC_R_Pitch = 1.0;
            }

        } else if (CntrPar.PRC_Comm == PRC_Comm_ZMQ) { // ZeroMQ
            LocalVar.PRC_R_Speed = LocalVar.ZMQ_R_Speed;
            LocalVar.PRC_R_Torque = LocalVar.ZMQ_R_Torque;
            LocalVar.PRC_R_Pitch = LocalVar.ZMQ_R_Pitch;
        }

        // Set min pitch for power control
        LocalVar.PRC_Min_Pitch = interp1d(CntrPar.PRC_R_Table,
                                           CntrPar.PRC_Pitch_Table,
                                           LocalVar.PRC_R_Pitch, ErrVar);

    } else {
        LocalVar.PRC_R_Speed = 1.0;
        LocalVar.PRC_R_Torque = 1.0;
        LocalVar.PRC_R_Pitch = 1.0;
        LocalVar.PRC_Min_Pitch = CntrPar.PC_FinePit;
    }
}
