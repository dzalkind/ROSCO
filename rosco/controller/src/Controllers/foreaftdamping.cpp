#include "../include/vit_types.h"
#include "../include/vit_translated.h"
#include <algorithm>

void ForeAftDamping(const ControlParameters& CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst) {
    // Fore-aft damping controller: reduces tower vibrations using pitch

    // PI controller on fore-aft acceleration (high-pass filtered)
    LocalVar->FA_AccHPFI = PIController(
        LocalVar->FA_AccHPF,        // error: filtered fore-aft acceleration
        0.0,                         // kp: no proportional gain
        CntrPar.FA_KI,             // ki: integral gain (from view struct)
        -CntrPar.FA_IntSat,        // minValue: negative saturation
        CntrPar.FA_IntSat,         // maxValue: positive saturation
        LocalVar->DT,               // time step
        0.0,                         // I0: initial integrator value
        &LocalVar->piP,             // PI parameters struct
        (LocalVar->restart != 0),    // reset flag
        &objInst->instPI             // instance counter
    );

    // Store the fore-aft pitch contribution for all blades
    for (int K = 0; K < LocalVar->NumBl; K++) {
        LocalVar->FA_PitCom[K] = LocalVar->FA_AccHPFI;
    }
}
